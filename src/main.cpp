#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <esp_task_wdt.h>

#include <GPS.h>
#include <camera.h>
#include <geofence.h>
#include <helium_jpeg.h>
#include <mission_config.h>
#include <mission_diagnostics.h>
#include <pin_defs.h>
#include <radio.h>
#include <serial_packet_test.h>
#include <image_store.h>
#include <wspr.h>

namespace {
constexpr int kBoardLed = 2;

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
bool loraTxError = false;
bool watchdogReady = false;
bool imageTransferMode = false;
bool previousNetworkReachable = false;
uint32_t imageModeStartCycle = UINT32_MAX;
bool captureAttempted = false;
unsigned long lastCaptureAttemptMs = 0;

helium_jpeg::HeliumJPEG imageEncoder;
int activeImageSlot = -1;
bool imageEncoderReady = false;

void feedWatchdog() {
  if (watchdogReady) esp_task_wdt_reset();
}

wspr::MissionRadio wsprRadio(
    []() { return !cameraCaptureInProgress; }, feedWatchdog);

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
      day > 31) {
    return 0;
  }
  const int64_t epoch = daysFromCivil(year, month, day) * 86400LL +
                        hour * 3600UL + minute * 60UL + second;
  return epoch > 0 && epoch <= UINT32_MAX ? static_cast<uint32_t>(epoch) : 0;
}

uint16_t currentStatusFlags() {
  const MissionDiagnostics diagnostics = missionDiagnostics();
  return (gpsError ? helium_jpeg::STATUS_FLAG_GPS_INVALID : 0) |
         (littleFsError ? helium_jpeg::STATUS_FLAG_FS_ERROR : 0) |
         ((cameraInitError || cameraCaptureError)
              ? helium_jpeg::STATUS_FLAG_CAMERA_ERROR
              : 0) |
         (encoderError ? helium_jpeg::STATUS_FLAG_ENCODE_FAILED : 0) |
         (loraTxError ? helium_jpeg::STATUS_FLAG_LORA_TX_FAILURE : 0) |
         ((diagnostics.resets != 0)
              ? helium_jpeg::STATUS_FLAG_UNEXPECTED_REBOOT
              : 0);
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
  value.esp_temp =
      static_cast<int8_t>(constrain(temperatureRead(), -128.0f, 127.0f));
  value.free_heap_kb = ESP.getFreeHeap() / 1024U;
  value.status_flags = currentStatusFlags();
  return value;
}

bool prepareImageEncoder(int slot) {
  if (imageEncoderReady && activeImageSlot == slot) return true;
  if (!image_store::prepareEncoder(slot, imageEncoder)) return false;
  activeImageSlot = slot;
  imageEncoderReady = true;
  return true;
}

bool transmitNextImagePacket() {
  const int slot = imageEncoderReady ? activeImageSlot
                                     : image_store::oldestImage();
  if (slot < 0 || !prepareImageEncoder(slot)) return false;
  helium_jpeg::HeliumPacket packet;
  if (!imageEncoder.getNextPacket(packet)) return false;
  serial_packet_test::mirrorLoRaPacket(packet.data, packet.length);
  if (!transmitHelium(packet.data, packet.length)) {
    imageEncoderReady = false;
    activeImageSlot = -1;
    return false;
  }
  if (image_store::packetSent(slot)) {
    imageEncoderReady = false;
    activeImageSlot = -1;
  }
  return true;
}

void captureImage() {
  if (littleFsError || !image_store::canAcceptCapture()) {
    if (!littleFsError) {
      Serial.println("Image capture skipped: archival store is full.");
    }
    return;
  }

  cameraCaptureInProgress = true;
  const unsigned long captureStarted = millis();
  camera::powerOn();
  cameraInitError = camera::begin() != ESP_OK;
  camera_fb_t* frame = cameraInitError ? nullptr : camera::captureJpeg();
  if (frame == nullptr) {
    cameraCaptureError = true;
  } else {
    helium_jpeg::HeliumTelemetry telemetry = captureTelemetry();
    telemetry.capture_ms = static_cast<uint16_t>(
        min(static_cast<unsigned long>(UINT16_MAX), millis() - captureStarted));
    cameraCaptureError = image_store::save(*frame, telemetry) != ESP_OK;
    if (!cameraCaptureError) serial_packet_test::imageStored();
    camera::release(frame);
  }
  camera::powerOff();
  cameraCaptureInProgress = false;
  Serial.println(cameraCaptureError ? "[camera] Capture failed."
                                    : "[camera] Image stored.");
}

wspr::MissionData currentWsprData(uint32_t nowUtc) {
  wspr::MissionData data = {};
  data.unixTime = nowUtc;
  data.latitude = lat;
  data.longitude = lng;
  data.altitudeMetres = alt;
  data.speedKmh = speed_kmh;
  data.hdop = hdop;
  data.statusFlags = currentStatusFlags();
  data.remainingImagePackets = image_store::remainingPacketCount();
  data.storedImages = image_store::imageCount();
  data.satellites = sats;
  data.hour = hour;
  data.minute = minute;
  data.second = second;
  data.centisecond = centisecond;
  return data;
}

void updateImageTransferMode(uint32_t nowUtc) {
  const bool reachable = lorawanNetworkReachable();
  if (!reachable || nowUtc == 0 || image_store::oldestImage() < 0) {
    imageTransferMode = false;
    imageModeStartCycle = UINT32_MAX;
  } else if (!imageTransferMode &&
             (!previousNetworkReachable || imageModeStartCycle == UINT32_MAX)) {
    // Change modes only after the current 10-minute U4B sequence completes.
    imageModeStartCycle = nowUtc / 600UL + 1UL;
  }
  if (reachable && imageModeStartCycle != UINT32_MAX &&
      nowUtc / 600UL >= imageModeStartCycle) {
    imageTransferMode = true;
    imageModeStartCycle = UINT32_MAX;
  }
  previousNetworkReachable = reachable;
}

bool maybeTransmitScheduledRf(uint32_t nowUtc) {
  if (gpsError || !GPSTimeFresh()) return false;
  if (!wsprRadio.transmitIfDue(currentWsprData(nowUtc), imageTransferMode)) {
    return false;
  }
  gpsError = true;
  GEOFENCE_inhibit();
  return true;
}

void maybeCaptureImage(uint32_t nowUtc) {
  // The final U4B slot is intentionally silent. Capture there so camera I2C
  // work cannot make one of the four scheduled WSPR messages miss its slot.
  if (littleFsError || gpsError || nowUtc == 0 || minute % 10U < 8U) return;
  const uint32_t lastCapture = image_store::lastCaptureUtc();
  if (lastCapture != 0 &&
      (nowUtc < lastCapture ||
       nowUtc - lastCapture < HAB_IMAGE_INTERVAL_SECONDS)) {
    return;
  }
  if (!image_store::canAcceptCapture()) return;
  if (captureAttempted &&
      millis() - lastCaptureAttemptMs < HAB_CAPTURE_RETRY_MS) {
    return;
  }
  captureAttempted = true;
  lastCaptureAttemptMs = millis();
  captureImage();
}

void maybeTransmitImagePacket() {
  if (!imageTransferMode || !lorawanUplinkDue() ||
      image_store::oldestImage() < 0) {
    return;
  }
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
  camera::powerOff();
  pinMode(kBoardLed, INPUT);

  Serial.begin(115200);
  delay(HAB_STARTUP_DELAY_MS);
  Serial.println("[boot] HAB firmware starting.");
  initialiseMissionDiagnostics();
  if (esp_task_wdt_init(HAB_WATCHDOG_TIMEOUT_SECONDS, true) == ESP_OK &&
      esp_task_wdt_add(nullptr) == ESP_OK) {
    watchdogReady = true;
    Serial.println("[watchdog] Ready.");
  } else {
    Serial.println("[watchdog] Initialisation failed.");
  }

#if HAB_TEMP_CAMERA_DIAGNOSTICS
  camera::runConnectionDiagnostics();
#endif

  littleFsError = !image_store::begin();
  Serial.printf("[storage] %s; %u queued image(s).\n",
                littleFsError ? "unavailable" : "ready",
                image_store::imageCount());
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, -1);
  GEOFENCE_inhibit();
  initLoRaWAN();
  wsprRadio.begin();
  serial_packet_test::begin();
  Serial.println("[boot] Setup complete; waiting for GPS.");
}

void loop() {
  feedWatchdog();
  updateGps();
  const uint32_t nowUtc = gpsUnixTime();
  updateImageTransferMode(nowUtc);
  if (maybeTransmitScheduledRf(nowUtc)) {
    delay(5);
    return;
  }
  serviceLoRaWAN(GEOFENCE_region);
  updateImageTransferMode(nowUtc);
  maybeCaptureImage(nowUtc);
  maybeTransmitImagePacket();
  const serial_packet_test::FlightStatus serialStatus = {
      millis() / 1000UL,
      nowUtc,
      !gpsError,
      GPSTimeFresh(),
      lat,
      lng,
      alt,
      hdop,
      sats,
      static_cast<uint8_t>(GEOFENCE_region),
      GEOFENCE_no_tx,
      lorawanNetworkReachable(),
      imageTransferMode,
      image_store::imageCount(),
      image_store::remainingPacketCount(),
      currentStatusFlags(),
  };
  serial_packet_test::service(serialStatus, feedWatchdog);
  delay(5);
}
