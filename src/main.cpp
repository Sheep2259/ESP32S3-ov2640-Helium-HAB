#include <Arduino.h>
#include <LittleFS.h>
#include <TinyGPSPlus.h>

#include <GPS.h>
#include <camutils.h>
#include <geofence.h>
#include <helium_jpeg.h>
#include <pin_defs.h>
#include <radio.h>
#include <wspr_tx.h>

namespace {
constexpr unsigned long kLoRaTxIntervalMs = 60000UL;
// An 80 KB image takes about 6.7 hours at one 200-byte packet per minute.
// Eight hours prevents an in-coverage queue from growing without bound.
constexpr unsigned long kImageIntervalMs = 8UL * 60UL * 60UL * 1000UL;
constexpr unsigned long kFirstCaptureGpsWaitMs = 3UL * 60UL * 1000UL;
constexpr uint8_t kImageProgressCheckpointPackets = 16;

// WSPR remains physically inhibited until the authorised base tone and
// declared radiated power are supplied. Do not guess these values: band choice
// and airborne operation depend on the applicable administration and route.
constexpr bool kWsprEnabled = true;
constexpr char kWsprCallsign[] = "M7CWV";
// Direct-RF lowest tone near the conventional 18.1046 MHz USB dial frequency
// plus a 1.5 kHz WSPR audio offset. Calibrate the Si5351 before flight.
constexpr uint64_t kWsprBaseFrequencyCentiHz = 1810610000ULL;
constexpr int8_t kWsprPowerDbm = 0;
char wsprGrid[5] = "AA00";

const wspr::Config kWsprConfig(
    kWsprCallsign, wsprGrid, kWsprBaseFrequencyCentiHz, kWsprPowerDbm,
    2,  // Base WSPR cadence; the application gates inside regions to 2 hours.
    25, 0);

float lat = 0;
float lng = 0;
float age_s = 3600;
float hdop = 99;
float alt = 0;
float speed_kmh = 0;
float course_deg = 0;
float previousAltitude = 0;
int16_t verticalSpeedCms = 0;
unsigned long previousAltitudeMs = 0;

uint16_t year = 0;
uint8_t month = 0;
uint8_t day = 0;
uint8_t hour = 0;
uint8_t minute = 0;
uint8_t second = 0;
uint8_t centisecond = 0;
uint8_t sats = 0;

bool littleFsError = false;
bool gpsError = true;
bool encoderError = false;
bool cameraCaptureError = false;
bool cameraInitError = false;
bool cameraCaptureInProgress = false;
bool wsprInitialised = false;
bool loraTxError = false;
bool capturedThisBoot = false;

unsigned long lastTxTime = 0;
unsigned long lastImageTime = 0;
uint8_t packetsSinceImageCheckpoint = 0;

helium_jpeg::HeliumJPEG imageEncoder;
wspr::Transmitter wsprTransmitter(kWsprConfig, []() {
  return !cameraCaptureInProgress;
});

int activeSlot = -1;
bool imageEncoderReady = false;

bool cameraI2cIsIdle() { return !cameraCaptureInProgress; }

bool updateWsprGridFromGps() {
  if (gpsError || lat < -90.0f || lat >= 90.0f || lng < -180.0f ||
      lng >= 180.0f) return false;

  const float longitude = lng + 180.0f;
  const float latitude = lat + 90.0f;
  wsprGrid[0] = static_cast<char>('A' + static_cast<int>(longitude / 20.0f));
  wsprGrid[1] = static_cast<char>('A' + static_cast<int>(latitude / 10.0f));
  wsprGrid[2] = static_cast<char>('0' + static_cast<int>(longitude / 2.0f) % 10);
  wsprGrid[3] = static_cast<char>('0' + static_cast<int>(latitude) % 10);
  wsprGrid[4] = '\0';
  return true;
}

void updateGps() {
  while (Serial2.available()) {
    if (!gps.encode(Serial2.read())) continue;
    UpdateGPSInfo(lat, lng, age_s, year, month, day, hour, minute, second,
                  centisecond, alt, speed_kmh, course_deg, sats, hdop);

    if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
      const unsigned long now = millis();
      if (previousAltitudeMs != 0) {
        const float elapsedSeconds = (now - previousAltitudeMs) / 1000.0f;
        if (elapsedSeconds > 0.1f) {
          verticalSpeedCms = static_cast<int16_t>(constrain(
              (alt - previousAltitude) * 100.0f / elapsedSeconds,
              -32768.0f, 32767.0f));
        }
      }
      previousAltitude = alt;
      previousAltitudeMs = now;
    }
  }

  if (gps.location.isValid()) age_s = gps.location.age() / 1000.0f;
  gpsError = !GPSPositionFresh();
  if (gpsError) {
    GEOFENCE_inhibit();
  } else {
    GEOFENCE_position(lat, lng);
  }
}

// Howard Hinnant's civil-date conversion, shifted to the Unix epoch.
int64_t daysFromCivil(int yearValue, unsigned monthValue, unsigned dayValue) {
  yearValue -= monthValue <= 2;
  const int era = (yearValue >= 0 ? yearValue : yearValue - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(yearValue - era * 400);
  const unsigned doy =
      (153 * (monthValue + (monthValue > 2 ? -3 : 9)) + 2) / 5 +
      dayValue - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return static_cast<int64_t>(era) * 146097 + static_cast<int>(doe) - 719468;
}

uint32_t gpsUnixTime() {
  if (!GPSTimeFresh() || year < 1970 || month < 1 || month > 12 || day < 1 ||
      day > 31) return 0;
  const int64_t epoch = daysFromCivil(year, month, day) * 86400LL +
                        hour * 3600UL + minute * 60UL + second;
  return epoch > 0 && epoch <= UINT32_MAX ? static_cast<uint32_t>(epoch) : 0;
}

helium_jpeg::HeliumTelemetry captureTelemetry() {
  helium_jpeg::HeliumTelemetry value = {};
  value.latitude = static_cast<int32_t>(lat * 1000000.0f);
  value.longitude = static_cast<int32_t>(lng * 1000000.0f);
  value.altitude = static_cast<uint16_t>(constrain(alt, 0.0f, 65535.0f));
  value.speed = static_cast<uint16_t>(
      constrain(speed_kmh * (100000.0f / 3600.0f), 0.0f, 65535.0f));
  value.heading = static_cast<uint16_t>(
      constrain(course_deg * 100.0f, 0.0f, 35999.0f));
  value.v_speed = verticalSpeedCms;
  value.hdop = static_cast<uint8_t>(constrain(hdop * 10.0f, 0.0f, 255.0f));
  value.satellites = sats;
  value.fix_type = gpsError ? 0 : 3;
  value.timestamp = gpsUnixTime();
  value.uptime = millis() / 1000UL;
  value.esp_temp = static_cast<int8_t>(constrain(temperatureRead(), -128.0f, 127.0f));
  value.free_heap_kb = ESP.getFreeHeap() / 1024U;
  value.status_flags =
      (gpsError ? helium_jpeg::STATUS_FLAG_GPS_INVALID : 0) |
      (littleFsError ? helium_jpeg::STATUS_FLAG_FS_ERROR : 0) |
      ((cameraInitError || cameraCaptureError)
           ? helium_jpeg::STATUS_FLAG_CAMERA_ERROR
           : 0) |
      (encoderError ? helium_jpeg::STATUS_FLAG_ENCODE_FAILED : 0) |
      (loraTxError ? helium_jpeg::STATUS_FLAG_LORA_TX_FAILURE : 0);
  // battery_mv and environmental fields stay zero until their actual board
  // pins/sensors are specified; fabricated readings are worse than missing.
  return value;
}

bool prepareImageEncoder(int slot) {
  if (imageEncoderReady && activeSlot == slot) return true;
  if (slot < 0 || slot >= static_cast<int>(IMAGE_SLOT_COUNT)) return false;

  const uint16_t imageId = imageIds[slot];
  char filename[16];
  imageFilename(imageId, filename, sizeof(filename));
  File image = LittleFS.open(filename, FILE_READ);
  if (!image) {
    discardImageSlot(slot);
    return false;
  }

  const int totalPackets = imageEncoder.begin(image, imageId);
  image.close();
  helium_jpeg::HeliumTelemetry telemetry = {};
  if (totalPackets <= 0 || !readImageTelemetry(imageId, telemetry)) {
    discardImageSlot(slot);
    return false;
  }

  if (savedImages[slot] > totalPackets) {
    savedImages[slot] = static_cast<uint16_t>(totalPackets);
    persistImageProgress();
  }
  imageEncoder.setTelemetry(telemetry);
  const uint16_t sent = static_cast<uint16_t>(totalPackets) - savedImages[slot];
  if (!imageEncoder.skipPackets(sent)) {
    discardImageSlot(slot);
    return false;
  }

  activeSlot = slot;
  imageEncoderReady = true;
  return true;
}

bool transmitNextImagePacket() {
  const int slot = imageEncoderReady ? activeSlot : oldestStoredImage();
  if (slot < 0 || !prepareImageEncoder(slot)) return false;

  helium_jpeg::HeliumPacket packet;
  if (!imageEncoder.getNextPacket(packet) ||
      !transmitHelium(packet.data, sizeof(packet.data))) return false;

  if (savedImages[slot] > 0) --savedImages[slot];
  if (savedImages[slot] == 0) {
    // The completion helper orders the telemetry removal and queue commit so
    // every possible nightly power-loss point can be reconciled on next boot.
    completeImageSlot(slot);
    imageEncoderReady = false;
    activeSlot = -1;
    packetsSinceImageCheckpoint = 0;
  } else if (++packetsSinceImageCheckpoint >= kImageProgressCheckpointPackets) {
    persistImageProgress();
    packetsSinceImageCheckpoint = 0;
  }
  return true;
}

void captureImage() {
  lastImageTime = millis();
  capturedThisBoot = true;
  if (littleFsError) {
    cameraCaptureError = true;
    return;
  }
  if (!imageStoreCanAcceptCapture()) {
    Serial.println("Image capture skipped: archival store is full.");
    return;
  }
  cameraCaptureInProgress = true;
  resetCamera();
  cameraInitError = StartCamera() != ESP_OK;
  cameraCaptureError = cameraInitError ||
                       savePhoto(captureTelemetry()) != ESP_OK;
  stopCamera();
  cameraCaptureInProgress = false;
}

void initialiseWspr() {
  if (!kWsprEnabled || !cameraI2cIsIdle()) return;
  const wspr::Result result = wsprTransmitter.begin();
  wsprInitialised = result == wspr::Result::Ok;
  if (!wsprInitialised) {
    Serial.printf("WSPR disabled: initialisation error %u\n",
                  static_cast<unsigned>(result));
  }
}

bool maybeTransmitWspr() {
  if (!kWsprEnabled || !wsprInitialised || gpsError || !GPSTimeFresh() ||
      !updateWsprGridFromGps()) return false;

  const wspr::UtcTime utc = {minute, second, centisecond};
  const bool insideHeliumRegion = GEOFENCE_region != HeliumRegion::None;
  if (insideHeliumRegion && (hour % 2U != 0 || minute != 0)) return false;
  if (!wsprTransmitter.due(utc)) return false;
  Serial.printf("WSPR %s %s at %02u:%02u:%02u UTC\n", kWsprCallsign,
                wsprGrid, hour, minute, second);
  const wspr::Result result = wsprTransmitter.transmitBlocking(utc);
  if (result != wspr::Result::Ok) {
    Serial.printf("WSPR transmission error %u\n", static_cast<unsigned>(result));
  }
  // A WSPR frame blocks GPS parsing for about 111 seconds. Force RF inhibit
  // until the next loop has parsed a genuinely fresh position and time.
  gpsError = true;
  GEOFENCE_inhibit();
  return true;
}

void maybeCaptureImage() {
  if (!capturedThisBoot) {
    if (!gpsError || millis() >= kFirstCaptureGpsWaitMs) captureImage();
    return;
  }
  if (millis() - lastImageTime >= kImageIntervalMs) captureImage();
}

void maybeTransmitImagePacket() {
  if (!lorawanCanTransmit() || oldestStoredImage() < 0 ||
      millis() - lastTxTime < kLoRaTxIntervalMs) return;

  lastTxTime = millis();
  if (transmitNextImagePacket()) {
    encoderError = false;
    loraTxError = false;
  } else {
    encoderError = true;
    loraTxError = true;
  }
}
}  // namespace

void setup() {
  // Board2 LED1 is wired from GPIO2 (ESP module physical pin 38) directly to
  // GND without an external series resistor. The manufactured board was
  // bench-tested with GPIO2 kept as an input and its internal pull-up used as
  // the current-limited source. Do not drive GPIO2 push-pull high unless an
  // external current limiter is added. U0TXD is separate, on physical pin 37.
  Serial.begin(115200);
  delay(1000);

  littleFsError = !initialiseImageStore();
  stopCamera();
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, -1);
  GEOFENCE_inhibit();
  initLoRaWAN();
  initialiseWspr();
}

void loop() {
  updateGps();
  if (maybeTransmitWspr()) {
    delay(5);
    return;
  }
  serviceLoRaWAN(GEOFENCE_region);
  maybeCaptureImage();
  maybeTransmitImagePacket();
  delay(5);
}
