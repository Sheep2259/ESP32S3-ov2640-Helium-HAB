#include <Arduino.h>
#include <LittleFS.h>
#include <TinyGPSPlus.h>

#include <GPS.h>
#include <camutils.h>
#include <geofence.h>
#include <helium_jpeg.h>
#include <pin_defs.h>
#include <quality.h>
#include <radio.h>

// The network and local regulations still determine the final permissible
// uplink rate. This is intentionally conservative for 210-byte EU868 DR5.
constexpr unsigned long TX_INTERVAL = 60000UL;
constexpr unsigned long IMG_INTERVAL = 14400000UL;
constexpr unsigned long PREFS_INTERVAL = 600000UL;
constexpr unsigned TELEMETRY_PACKET_INTERVAL = 20;

uint32_t counter = 0;
float lat = 0.0f, lng = 0.0f, age_s = 3600.0f, hdop = 26.0f;
float alt = 0.0f, speed_kmh = 0.0f, course_deg = 0.0f;
uint16_t year = 0;
uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 9, centisecond = 0, sats = 0;

bool littleFsError = false;
bool prefsError = false;
bool gpsError = true;
bool encoderError = false;
bool cameraCaptureError = false;
bool cameraInitError = false;

unsigned long lastTxTime = 0;
unsigned long lastImageTime = 0;
unsigned long lastPrefsUpdateTime = 0;
unsigned long lastTransitionCaptureTime = 0;
unsigned quality = 0;
unsigned lastQuality = 0;
char timestampChars[30];

helium_jpeg::HeliumJPEG imageEncoder;
int activeImage = -1;
bool imageEncoderReady = false;

helium_jpeg::HeliumTelemetry makeHeliumTelemetry() {
  helium_jpeg::HeliumTelemetry telemetry = {};
  telemetry.latitude = (int32_t)(lat * 1000000.0f);
  telemetry.longitude = (int32_t)(lng * 1000000.0f);
  telemetry.altitude = (uint16_t)constrain(alt, 0.0f, 65535.0f);
  telemetry.speed = (uint16_t)constrain(speed_kmh * (100000.0f / 3600.0f), 0.0f, 65535.0f);
  telemetry.heading = (uint16_t)constrain(course_deg * 100.0f, 0.0f, 35999.0f);
  telemetry.hdop = (uint8_t)constrain(hdop * 10.0f, 0.0f, 255.0f);
  telemetry.satellites = sats;
  telemetry.fix_type = gpsError ? 0 : 3;
  telemetry.uptime = millis() / 1000UL;
  telemetry.free_heap_kb = ESP.getFreeHeap() / 1024U;
  telemetry.status_flags =
      (gpsError ? helium_jpeg::STATUS_FLAG_GPS_INVALID : 0) |
      ((cameraInitError || cameraCaptureError) ? helium_jpeg::STATUS_FLAG_CAMERA_ERROR : 0) |
      (littleFsError ? helium_jpeg::STATUS_FLAG_FS_ERROR : 0) |
      (encoderError ? helium_jpeg::STATUS_FLAG_ENCODE_FAILED : 0);
  return telemetry;
}

bool prepareImageEncoder(int imageNumber) {
  if (imageEncoderReady && activeImage == imageNumber) return true;

  char filename[12];
  snprintf(filename, sizeof(filename), "/%d.jpg", imageNumber);
  File imageFile = LittleFS.open(filename, FILE_READ);
  if (!imageFile) {
    Serial.printf("Cannot open image %s\n", filename);
    return false;
  }

  // Slot plus incrementing version makes the ID stable throughout an image.
  const uint16_t imageId = ((uint16_t)imageVersion[imageNumber] << 4) | imageNumber;
  const int packetCount = imageEncoder.begin(imageFile, imageId);
  imageFile.close();
  if (packetCount <= 0) {
    Serial.printf("HeliumJPEG setup failed for %s: %s\n", filename, imageEncoder.getError());
    return false;
  }

  imageEncoder.setTelemetry(makeHeliumTelemetry());
  activeImage = imageNumber;
  imageEncoderReady = true;
  savedImages[imageNumber] = packetCount;
  return true;
}

bool transmitNextImagePacket() {
  const int imageNumber = imageEncoderReady ? activeImage : IMGnToTX(savedImages);
  if (imageNumber < 0 || !prepareImageEncoder(imageNumber)) return false;

  helium_jpeg::HeliumPacket packet;
  if (!imageEncoder.getNextPacket(packet)) {
    imageEncoderReady = false;
    activeImage = -1;
    return false;
  }
  if (!transmitHelium(packet.data, sizeof(packet.data))) return false;

  if (savedImages[imageNumber] > 0) --savedImages[imageNumber];
  if (savedImages[imageNumber] == 0) {
    char filename[12];
    snprintf(filename, sizeof(filename), "/%d.jpg", imageNumber);
    LittleFS.remove(filename);
    imageEncoderReady = false;
    activeImage = -1;
  }
  return true;
}

bool captureAndSchedule(unsigned newQuality) {
  snprintf(timestampChars, sizeof(timestampChars), "%u/%u/%u/%u/%u",
           month, day, hour, minute, second);
  if (savePhoto(newQuality, lat, lng, alt, timestampChars) != ESP_OK) {
    lastImageTime = millis() - (IMG_INTERVAL * 31) / 32;
    cameraCaptureError = true;
    return false;
  }

  cameraCaptureError = false;
  lastImageTime = millis() - (newQuality == 1 ? IMG_INTERVAL / 2 : 0);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  resetCamera();
  bool cameraReady = false;
  for (int retries = 0; retries < 20 && !cameraReady; ++retries) {
    if (StartCamera() == ESP_OK) {
      cameraReady = true;
      for (int frame = 0; frame < 5; ++frame) {
        camera_fb_t* warmup = esp_camera_fb_get();
        if (warmup) esp_camera_fb_return(warmup);
        delay(100);
      }
    } else {
      esp_camera_deinit();
      resetCamera();
      delay(1000);
    }
  }
  cameraInitError = !cameraReady;

  prefs.begin("img_data", false);
  if (prefs.getBytes("remain", savedImages, sizeof(savedImages)) == 0) {
    memset(savedImages, 0, sizeof(savedImages));
    prefs.putBytes("remain", savedImages, sizeof(savedImages));
    prefsError = true;
  }
  if (prefs.getBytes("version", imageVersion, sizeof(imageVersion)) == 0) {
    memset(imageVersion, 0, sizeof(imageVersion));
    prefs.putBytes("version", imageVersion, sizeof(imageVersion));
    prefsError = true;
  }

  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, -1);
  if (!LittleFS.begin(true)) {
    littleFsError = true;
    Serial.println("LittleFS mount failed");
  }

  GEOFENCE_position(lat, lng);  // Retained as the global transmit inhibit.
  initLoRaWAN();
}

void loop() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      UpdateGPSInfo(lat, lng, age_s, year, month, day, hour, minute, second,
                    centisecond, alt, speed_kmh, course_deg, sats, hdop);
      gpsError = hdop >= 20;
      quality = locationQuality(lat, lng);
      if (quality == 0) lastQuality = 0;
    }
  }

  if (millis() - lastPrefsUpdateTime >= PREFS_INTERVAL) {
    prefs.putBytes("remain", savedImages, sizeof(savedImages));
    lastPrefsUpdateTime = millis();
  }

  if (millis() - lastImageTime >= IMG_INTERVAL) {
    captureAndSchedule(locationQuality(lat, lng));
  }
  if (lastQuality == 0 && quality == 1 && millis() > 120000UL && hdop < 10 &&
      millis() - lastTransitionCaptureTime > 600000UL) {
    if (captureAndSchedule(1)) {
      lastTransitionCaptureTime = millis();
      lastQuality = 1;
    }
  }
  if (millis() > 60000UL && IMGnToTX(savedImages) == -1) {
    captureAndSchedule(locationQuality(lat, lng));
  }

  if (millis() - lastTxTime < TX_INTERVAL) return;
  lastTxTime = millis();
  GEOFENCE_position(lat, lng);

  if (counter % TELEMETRY_PACKET_INTERVAL == 0) {
    // Short human-readable status uplink; image metadata carries the full
    // fixed-width HeliumTelemetry structure.
    char status[72];
    snprintf(status, sizeof(status), "T%u/%u/%u A%.0f S%u H%.1f N%u",
             hour, minute, second, alt, sats, hdop, counter);
    if (!transmitHelium((const uint8_t*)status, strlen(status))) encoderError = true;
  } else if (IMGnToTX(savedImages) >= 0) {
    if (!transmitNextImagePacket()) encoderError = true;
  }
  ++counter;
}
