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
constexpr unsigned long kImageIntervalMs = 5UL * 60UL * 60UL * 1000UL;

// --- WSPR configuration -----------------------------------------------------
// Set WSPR_ENABLED to true only after filling every value below with values
// that you are authorised to transmit. base_frequency_centi_hz is Hz * 100.
constexpr bool kWsprEnabled = false;
constexpr char kWsprCallsign[] = "N0CALL";  // Replace with your callsign.
constexpr uint64_t kWsprBaseFrequencyCentiHz = 0;  // Replace; no band default.
constexpr int8_t kWsprPowerDbm = 0;  // Declared radiated power, not Si5351 drive.
char wsprGrid[5] = "AA00";  // Updated from a valid GPS position before transmit.

const wspr::Config kWsprConfig(
    kWsprCallsign,
    wsprGrid,
    kWsprBaseFrequencyCentiHz,
    kWsprPowerDbm,
    60,  // One transmission per hour, at hh:00:02 UTC.
    25,
    0);

float lat = 0;
float lng = 0;
float age_s = 3600;
float hdop = 26;
float alt = 0;
float speed_kmh = 0;
float course_deg = 0;

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

unsigned long lastTxTime = 0;
unsigned long lastImageTime = 0;

helium_jpeg::HeliumJPEG imageEncoder;
wspr::Transmitter wsprTransmitter(kWsprConfig, []() {
  return !cameraCaptureInProgress;
});

int activeSlot = -1;
bool imageEncoderReady = false;

bool cameraI2cIsIdle() {
  return !cameraCaptureInProgress;
}

bool updateWsprGridFromGps() {
  if (gpsError || !gps.location.isValid() || lat < -90.0f || lat >= 90.0f ||
      lng < -180.0f || lng >= 180.0f) {
    return false;
  }

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
  while (Serial2.available() && gps.encode(Serial2.read())) {
    UpdateGPSInfo(lat, lng, age_s, year, month, day, hour, minute, second,
                  centisecond, alt, speed_kmh, course_deg, sats, hdop);
    gpsError = hdop >= 20;
  }
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
  value.hdop = static_cast<uint8_t>(constrain(hdop * 10.0f, 0.0f, 255.0f));
  value.satellites = sats;
  value.fix_type = gpsError ? 0 : 3;
  value.uptime = millis() / 1000UL;
  value.free_heap_kb = ESP.getFreeHeap() / 1024U;
  value.status_flags = (gpsError ? helium_jpeg::STATUS_FLAG_GPS_INVALID : 0) |
                       (littleFsError ? helium_jpeg::STATUS_FLAG_FS_ERROR : 0) |
                       (cameraInitError ? helium_jpeg::STATUS_FLAG_CAMERA_ERROR : 0);
  return value;
}

void persistImageProgress() {
  prefs.putBytes("remain", savedImages, sizeof(savedImages));
  prefs.putBytes("image_ids", imageIds, sizeof(imageIds));
  prefs.putUShort("next_image_id", nextImageId);
}

bool prepareImageEncoder(int slot) {
  if (imageEncoderReady && activeSlot == slot) return true;

  const uint16_t imageId = imageIds[slot];
  char filename[16];
  imageFilename(imageId, filename, sizeof(filename));

  File image = LittleFS.open(filename, FILE_READ);
  if (!image) return false;

  const int totalPackets = imageEncoder.begin(image, imageId);
  image.close();

  helium_jpeg::HeliumTelemetry telemetry = {};
  if (totalPackets <= 0 || !readImageTelemetry(imageId, telemetry)) return false;

  imageEncoder.setTelemetry(telemetry);
  const uint16_t sent = totalPackets > savedImages[slot]
                            ? totalPackets - savedImages[slot]
                            : 0;
  if (!imageEncoder.skipPackets(sent)) return false;

  activeSlot = slot;
  imageEncoderReady = true;
  return true;
}

bool transmitNextImagePacket() {
  const int slot = imageEncoderReady ? activeSlot : oldestStoredImage();
  if (slot < 0 || !prepareImageEncoder(slot)) return false;

  helium_jpeg::HeliumPacket packet;
  if (!imageEncoder.getNextPacket(packet) ||
      !transmitHelium(packet.data, sizeof(packet.data))) {
    return false;
  }

  if (savedImages[slot] > 0) --savedImages[slot];
  if (savedImages[slot] == 0) {
    char jpegName[16];
    char telemetryName[16];
    imageFilename(imageIds[slot], jpegName, sizeof(jpegName));
    telemetryFilename(imageIds[slot], telemetryName, sizeof(telemetryName));
    LittleFS.remove(jpegName);
    LittleFS.remove(telemetryName);
    imageEncoderReady = false;
    activeSlot = -1;
  }

  persistImageProgress();
  return true;
}

void captureImage() {
  // A full store must not cause continuous recapture attempts.
  lastImageTime = millis();
  cameraCaptureInProgress = true;
  cameraCaptureError = savePhoto(captureTelemetry()) != ESP_OK;
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

void maybeTransmitWspr() {
  if (!kWsprEnabled || !wsprInitialised || gpsError || !gps.time.isValid() ||
      !updateWsprGridFromGps()) {
    return;
  }

  const wspr::UtcTime utc = {minute, second, centisecond};
  if (!wsprTransmitter.due(utc)) return;

  Serial.printf("WSPR %s %s at %02u:%02u:%02u UTC\n", kWsprCallsign,
                wsprGrid, hour, minute, second);
  const wspr::Result result = wsprTransmitter.transmitBlocking(utc);
  if (result != wspr::Result::Ok) {
    Serial.printf("WSPR transmission error %u\n", static_cast<unsigned>(result));
  }
}

void maybeCaptureImage() {
  if (oldestStoredImage() < 0 || millis() - lastImageTime >= kImageIntervalMs) {
    captureImage();
  }
}

void maybeTransmitImagePacket() {
  if (millis() - lastTxTime < kLoRaTxIntervalMs) return;

  lastTxTime = millis();
  GEOFENCE_position(lat, lng);
  if (oldestStoredImage() >= 0 && !transmitNextImagePacket()) {
    encoderError = true;
  }
}

void initialiseImageStore() {
  prefs.begin("img_data", false);
  const bool progressMissing =
      prefs.getBytes("remain", savedImages, sizeof(savedImages)) != sizeof(savedImages) ||
      prefs.getBytes("image_ids", imageIds, sizeof(imageIds)) != sizeof(imageIds);
  if (progressMissing) {
    memset(savedImages, 0, sizeof(savedImages));
    memset(imageIds, 0, sizeof(imageIds));
    persistImageProgress();
  }
  nextImageId = prefs.getUShort("next_image_id", 0);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  resetCamera();
  cameraInitError = StartCamera() != ESP_OK;
  initialiseImageStore();

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, -1);
  littleFsError = !LittleFS.begin(true);
  GEOFENCE_position(lat, lng);
  initLoRaWAN();
  initialiseWspr();
}

void loop() {
  updateGps();
  maybeTransmitWspr();
  maybeCaptureImage();
  maybeTransmitImagePacket();
}
