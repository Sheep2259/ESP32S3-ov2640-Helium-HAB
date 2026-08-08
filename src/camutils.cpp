#include <LittleFS.h>
#include <Preferences.h>
#include <camutils.h>
#include <pin_defs.h>

Preferences prefs;
uint16_t savedImages[IMAGE_SLOT_COUNT] = {};
uint16_t imageIds[IMAGE_SLOT_COUNT] = {};
uint16_t nextImageId = 0;

void imageFilename(uint16_t imageId, char* output, size_t outputSize) {
  snprintf(output, outputSize, "/%u.jpg", imageId);
}

void telemetryFilename(uint16_t imageId, char* output, size_t outputSize) {
  snprintf(output, outputSize, "/%u.tlm", imageId);
}

esp_err_t StartCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_D0; config.pin_d1 = CAM_D1; config.pin_d2 = CAM_D2; config.pin_d3 = CAM_D3;
  config.pin_d4 = CAM_D4; config.pin_d5 = CAM_D5; config.pin_d6 = CAM_D6; config.pin_d7 = CAM_D7;
  config.pin_xclk = CAM_XCLK; config.pin_pclk = CAM_PCLK; config.pin_vsync = CAM_VSYNC; config.pin_href = CAM_HREF;
  config.pin_sccb_sda = CAM_SDA; config.pin_sccb_scl = CAM_SCL;
  config.pin_pwdn = CAM_PWDN; config.pin_reset = CAM_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 8;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&config);
}

void resetCamera() {
  pinMode(CAM_LDO_EN, OUTPUT);
  digitalWrite(CAM_LDO_EN, HIGH);
  pinMode(CAM_PWDN, OUTPUT);
  digitalWrite(CAM_PWDN, HIGH);
  delay(500);
  digitalWrite(CAM_PWDN, LOW);
  delay(500);
}

camera_fb_t* captureJpeg() {
  for (int attempt = 0; attempt < 3; ++attempt) {
    camera_fb_t* frame = esp_camera_fb_get();
    if (frame && frame->format == PIXFORMAT_JPEG) return frame;
    if (frame) esp_camera_fb_return(frame);
    delay(200);
  }
  return nullptr;
}

bool validateJpegBuffer(const uint8_t* buf, size_t len) {
  if (len < 4 || buf[0] != 0xFF || buf[1] != 0xD8) return false;
  for (size_t i = (len > 32 ? len - 32 : 0); i + 1 < len; ++i) {
    if (buf[i] == 0xFF && buf[i + 1] == 0xD9) return true;
  }
  return false;
}

bool validateJpegFile(const char* filename) {
  File file = LittleFS.open(filename, FILE_READ);
  if (!file || file.size() < 4) return false;
  uint8_t start[2];
  if (file.read(start, 2) != 2 || start[0] != 0xFF || start[1] != 0xD8) return false;
  const size_t from = file.size() > 256 ? file.size() - 256 : 2;
  file.seek(from);
  uint8_t previous = 0;
  while (file.available()) {
    uint8_t value = file.read();
    if (previous == 0xFF && value == 0xD9) return true;
    previous = value;
  }
  return false;
}

bool readImageTelemetry(uint16_t imageId, helium_jpeg::HeliumTelemetry& telemetry) {
  char filename[16];
  telemetryFilename(imageId, filename, sizeof(filename));
  File file = LittleFS.open(filename, FILE_READ);
  if (!file || file.size() != sizeof(telemetry)) return false;
  return file.read((uint8_t*)&telemetry, sizeof(telemetry)) == sizeof(telemetry);
}

int oldestStoredImage() {
  int selected = -1;
  for (size_t i = 0; i < IMAGE_SLOT_COUNT; ++i) {
    if (savedImages[i] == 0) continue;
    if (selected < 0 || (int16_t)(imageIds[i] - imageIds[selected]) < 0) selected = i;
  }
  return selected;
}

esp_err_t savePhoto(helium_jpeg::HeliumTelemetry telemetry) {
  int slot = -1;
  for (size_t i = 0; i < IMAGE_SLOT_COUNT; ++i) if (savedImages[i] == 0) { slot = (int)i; break; }
  if (slot < 0) return ESP_FAIL;

  const uint32_t captureStart = millis();
  camera_fb_t* frame = captureJpeg();
  if (!frame || !validateJpegBuffer(frame->buf, frame->len)) {
    if (frame) esp_camera_fb_return(frame);
    return ESP_FAIL;
  }

  const size_t jpegSize = frame->len;
  const size_t freeBytes = LittleFS.totalBytes() - LittleFS.usedBytes();
  if (jpegSize > MAX_JPEG_BYTES || jpegSize + sizeof(telemetry) + IMAGE_STORE_RESERVE_BYTES > freeBytes) {
    Serial.printf("Image not stored: %u bytes, %u bytes free\n", (unsigned)jpegSize, (unsigned)freeBytes);
    esp_camera_fb_return(frame);
    return ESP_FAIL;
  }
  const uint16_t imageId = nextImageId++;
  char jpegName[16], telemetryName[16];
  imageFilename(imageId, jpegName, sizeof(jpegName));
  telemetryFilename(imageId, telemetryName, sizeof(telemetryName));
  File jpeg = LittleFS.open(jpegName, FILE_WRITE);
  if (!jpeg) { esp_camera_fb_return(frame); return ESP_FAIL; }
  const size_t written = jpeg.write(frame->buf, jpegSize);
  jpeg.close();
  esp_camera_fb_return(frame);
  if (written != jpegSize || !validateJpegFile(jpegName)) {
    LittleFS.remove(jpegName);
    return ESP_FAIL;
  }

  telemetry.jpeg_file_size = jpegSize;
  telemetry.capture_ms = (uint16_t)min((uint32_t)65535, (uint32_t)(millis() - captureStart));
  File telemetryFile = LittleFS.open(telemetryName, FILE_WRITE);
  if (!telemetryFile || telemetryFile.write((const uint8_t*)&telemetry, sizeof(telemetry)) != sizeof(telemetry)) {
    if (telemetryFile) telemetryFile.close();
    LittleFS.remove(jpegName);
    LittleFS.remove(telemetryName);
    return ESP_FAIL;
  }
  telemetryFile.close();

  File packetFile = LittleFS.open(jpegName, FILE_READ);
  helium_jpeg::HeliumJPEG packetizer;
  const int count = packetFile ? packetizer.begin(packetFile, imageId) : -1;
  if (packetFile) packetFile.close();
  if (count <= 0) {
    LittleFS.remove(jpegName); LittleFS.remove(telemetryName);
    return ESP_FAIL;
  }
  imageIds[slot] = imageId;
  savedImages[slot] = (uint16_t)count;
  prefs.putBytes("remain", savedImages, sizeof(savedImages));
  prefs.putBytes("image_ids", imageIds, sizeof(imageIds));
  prefs.putUShort("next_image_id", nextImageId);
  return ESP_OK;
}
