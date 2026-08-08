#include <Arduino.h>
#include <LittleFS.h>
#include <TinyGPSPlus.h>
#include <GPS.h>
#include <camutils.h>
#include <geofence.h>
#include <helium_jpeg.h>
#include <pin_defs.h>
#include <radio.h>

constexpr unsigned long TX_INTERVAL = 60000UL;
constexpr unsigned long IMAGE_INTERVAL = 5UL * 60UL * 60UL * 1000UL;
float lat = 0, lng = 0, age_s = 3600, hdop = 26, alt = 0, speed_kmh = 0, course_deg = 0;
uint16_t year = 0;
uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0, centisecond = 0, sats = 0;
bool littleFsError = false, gpsError = true, encoderError = false, cameraCaptureError = false, cameraInitError = false;
unsigned long lastTxTime = 0, lastImageTime = 0;
helium_jpeg::HeliumJPEG imageEncoder;
int activeSlot = -1;
bool imageEncoderReady = false;

helium_jpeg::HeliumTelemetry captureTelemetry() {
  helium_jpeg::HeliumTelemetry value = {};
  value.latitude = (int32_t)(lat * 1000000.0f); value.longitude = (int32_t)(lng * 1000000.0f);
  value.altitude = (uint16_t)constrain(alt, 0.0f, 65535.0f);
  value.speed = (uint16_t)constrain(speed_kmh * (100000.0f / 3600.0f), 0.0f, 65535.0f);
  value.heading = (uint16_t)constrain(course_deg * 100.0f, 0.0f, 35999.0f);
  value.hdop = (uint8_t)constrain(hdop * 10.0f, 0.0f, 255.0f); value.satellites = sats;
  value.fix_type = gpsError ? 0 : 3; value.uptime = millis() / 1000UL;
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
  const uint16_t imageId = imageIds[slot]; char filename[16]; imageFilename(imageId, filename, sizeof(filename));
  File image = LittleFS.open(filename, FILE_READ); if (!image) return false;
  const int totalPackets = imageEncoder.begin(image, imageId); image.close();
  helium_jpeg::HeliumTelemetry telemetry = {};
  if (totalPackets <= 0 || !readImageTelemetry(imageId, telemetry)) return false;
  imageEncoder.setTelemetry(telemetry);
  const uint16_t sent = totalPackets > savedImages[slot] ? totalPackets - savedImages[slot] : 0;
  if (!imageEncoder.skipPackets(sent)) return false;
  activeSlot = slot; imageEncoderReady = true; return true;
}

bool transmitNextImagePacket() {
  const int slot = imageEncoderReady ? activeSlot : oldestStoredImage();
  if (slot < 0 || !prepareImageEncoder(slot)) return false;
  helium_jpeg::HeliumPacket packet;
  if (!imageEncoder.getNextPacket(packet) || !transmitHelium(packet.data, sizeof(packet.data))) return false;
  if (savedImages[slot] > 0) --savedImages[slot];
  if (savedImages[slot] == 0) {
    char jpegName[16], telemetryName[16]; imageFilename(imageIds[slot], jpegName, sizeof(jpegName));
    telemetryFilename(imageIds[slot], telemetryName, sizeof(telemetryName));
    LittleFS.remove(jpegName); LittleFS.remove(telemetryName); imageEncoderReady = false; activeSlot = -1;
  }
  persistImageProgress(); return true;
}

void captureImage() {
  lastImageTime = millis();  // A full store must not cause continuous recapture attempts.
  if (savePhoto(captureTelemetry()) == ESP_OK) cameraCaptureError = false;
  else cameraCaptureError = true;
}

void setup() {
  Serial.begin(115200); 
  delay(1000); 
  resetCamera(); 
  cameraInitError = StartCamera() != ESP_OK;

  prefs.begin("img_data", false);
  if (prefs.getBytes("remain", savedImages, sizeof(savedImages)) != sizeof(savedImages) ||
      prefs.getBytes("image_ids", imageIds, sizeof(imageIds)) != sizeof(imageIds)) {
    memset(savedImages, 0, sizeof(savedImages)); memset(imageIds, 0, sizeof(imageIds)); persistImageProgress();
  }
  nextImageId = prefs.getUShort("next_image_id", 0);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, -1);
  if (!LittleFS.begin(true)) littleFsError = true;
  GEOFENCE_position(lat, lng); 
  initLoRaWAN();
}

void loop() {
  while (Serial2.available() && gps.encode(Serial2.read())) {
    UpdateGPSInfo(lat, lng, age_s, year, month, day, hour, minute, second, centisecond, alt, speed_kmh, course_deg, sats, hdop);
    gpsError = hdop >= 20;
  }
  if (oldestStoredImage() < 0 || millis() - lastImageTime >= IMAGE_INTERVAL) captureImage();
  if (millis() - lastTxTime < TX_INTERVAL) return;
  lastTxTime = millis(); GEOFENCE_position(lat, lng);
  if (oldestStoredImage() >= 0 && !transmitNextImagePacket()) encoderError = true;
}
