#include <LittleFS.h>
#include <Preferences.h>

#include <cstdlib>
#include <cstring>

#include <camutils.h>
#include <pin_defs.h>

Preferences prefs;
uint16_t savedImages[IMAGE_SLOT_COUNT] = {};
uint16_t imageIds[IMAGE_SLOT_COUNT] = {};
uint16_t nextImageId = 0;

namespace {
bool cameraDriverStarted = false;

void removeImageFiles(uint16_t imageId) {
  char jpegName[16];
  char telemetryName[16];
  imageFilename(imageId, jpegName, sizeof(jpegName));
  telemetryFilename(imageId, telemetryName, sizeof(telemetryName));
  LittleFS.remove(jpegName);
  LittleFS.remove(telemetryName);
}

bool isTracked(uint16_t imageId) {
  for (size_t i = 0; i < IMAGE_SLOT_COUNT; ++i) {
    if (savedImages[i] > 0 && imageIds[i] == imageId) return true;
  }
  return false;
}

int freeSlot() {
  for (size_t i = 0; i < IMAGE_SLOT_COUNT; ++i) {
    if (savedImages[i] == 0) return static_cast<int>(i);
  }
  return -1;
}

int oldestStoredImageExcept(int excludedSlot) {
  int selected = -1;
  for (size_t i = 0; i < IMAGE_SLOT_COUNT; ++i) {
    if (savedImages[i] == 0 || static_cast<int>(i) == excludedSlot) continue;
    if (selected < 0 ||
        static_cast<int16_t>(imageIds[i] - imageIds[selected]) < 0) {
      selected = static_cast<int>(i);
    }
  }
  return selected;
}

bool parseImageId(const char* path, const char* extension, uint16_t& imageId) {
  if (path == nullptr) return false;
  const char* name = path[0] == '/' ? path + 1 : path;
  char* end = nullptr;
  const unsigned long parsed = strtoul(name, &end, 10);
  if (end == name || parsed > UINT16_MAX || strcmp(end, extension) != 0) return false;
  imageId = static_cast<uint16_t>(parsed);
  return true;
}

int packetCountForImage(uint16_t imageId) {
  char jpegName[16];
  imageFilename(imageId, jpegName, sizeof(jpegName));
  helium_jpeg::HeliumTelemetry telemetry = {};
  if (!validateJpegFile(jpegName) || !readImageTelemetry(imageId, telemetry)) return -1;

  File image = LittleFS.open(jpegName, FILE_READ);
  if (!image) return -1;
  helium_jpeg::HeliumJPEG packetizer;
  const int count = packetizer.begin(image, imageId);
  image.close();
  return count > 0 && count <= UINT16_MAX ? count : -1;
}

bool imageFilesValid(uint16_t imageId) {
  char jpegName[16];
  imageFilename(imageId, jpegName, sizeof(jpegName));
  helium_jpeg::HeliumTelemetry telemetry = {};
  return validateJpegFile(jpegName) && readImageTelemetry(imageId, telemetry);
}

void reconcileImageStore() {
  bool changed = false;
  for (size_t i = 0; i < IMAGE_SLOT_COUNT; ++i) {
    if (savedImages[i] == 0) continue;
    // Keep boot energy bounded even with a full multi-month store. A complete
    // Huffman walk is deferred until this image becomes the transmit head.
    if (!imageFilesValid(imageIds[i])) {
      Serial.printf("Dropping corrupt image queue entry %u.\n", imageIds[i]);
      removeImageFiles(imageIds[i]);
      savedImages[i] = 0;
      imageIds[i] = 0;
      changed = true;
    }
  }

  // Recover a fully written image whose power failed before its queue entry
  // reached NVS. Invalid and unpaired files are removed below.
  uint16_t jpegCandidates[IMAGE_SLOT_COUNT] = {};
  size_t candidateCount = 0;
  File root = LittleFS.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      uint16_t imageId = 0;
      if (!entry.isDirectory() && candidateCount < IMAGE_SLOT_COUNT &&
          parseImageId(entry.name(), ".jpg", imageId)) {
        jpegCandidates[candidateCount++] = imageId;
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }

  for (size_t i = 0; i < candidateCount; ++i) {
    const uint16_t imageId = jpegCandidates[i];
    if (isTracked(imageId)) continue;
    const int count = packetCountForImage(imageId);
    const int slot = freeSlot();
    if (count > 0 && slot >= 0) {
      imageIds[slot] = imageId;
      savedImages[slot] = static_cast<uint16_t>(count);
      Serial.printf("Recovered image %u after interrupted commit.\n", imageId);
      changed = true;
    } else {
      removeImageFiles(imageId);
    }
  }

  // Remove telemetry files that have no valid tracked JPEG. They can be left
  // by a reset in the middle of the two-file capture commit.
  uint16_t telemetryCandidates[IMAGE_SLOT_COUNT] = {};
  size_t telemetryCount = 0;
  root = LittleFS.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      uint16_t imageId = 0;
      if (!entry.isDirectory() && telemetryCount < IMAGE_SLOT_COUNT &&
          parseImageId(entry.name(), ".tlm", imageId)) {
        telemetryCandidates[telemetryCount++] = imageId;
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }
  for (size_t i = 0; i < telemetryCount; ++i) {
    if (!isTracked(telemetryCandidates[i])) {
      char telemetryName[16];
      telemetryFilename(telemetryCandidates[i], telemetryName,
                        sizeof(telemetryName));
      LittleFS.remove(telemetryName);
    }
  }

  while (isTracked(nextImageId)) ++nextImageId;
  if (changed) persistImageProgress();
}

}  // namespace

void imageFilename(uint16_t imageId, char* output, size_t outputSize) {
  snprintf(output, outputSize, "/%u.jpg", imageId);
}

void telemetryFilename(uint16_t imageId, char* output, size_t outputSize) {
  snprintf(output, outputSize, "/%u.tlm", imageId);
}

void persistImageProgress() {
  const bool ok =
      prefs.putBytes("remain", savedImages, sizeof(savedImages)) ==
          sizeof(savedImages) &&
      prefs.putBytes("image_ids", imageIds, sizeof(imageIds)) ==
          sizeof(imageIds) &&
      prefs.putUShort("next_image_id", nextImageId) == sizeof(nextImageId);
  if (!ok) Serial.println("Failed to persist image queue progress.");
}

bool initialiseImageStore() {
  if (!prefs.begin("img_data", false)) return false;

  bool formatted = false;
  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS mount failed; formatting the image partition.");
    if (!LittleFS.format() || !LittleFS.begin(false)) return false;
    formatted = true;
  }

  const bool progressMissing = formatted ||
      prefs.getBytes("remain", savedImages, sizeof(savedImages)) != sizeof(savedImages) ||
      prefs.getBytes("image_ids", imageIds, sizeof(imageIds)) != sizeof(imageIds);
  if (progressMissing) {
    memset(savedImages, 0, sizeof(savedImages));
    memset(imageIds, 0, sizeof(imageIds));
    nextImageId = 0;
    persistImageProgress();
  } else {
    nextImageId = prefs.getUShort("next_image_id", 0);
  }
  reconcileImageStore();
  return true;
}

bool imageStoreCanAcceptCapture() {
  return freeSlot() >= 0 &&
         LittleFS.totalBytes() - LittleFS.usedBytes() >=
             MAX_JPEG_BYTES + sizeof(helium_jpeg::HeliumTelemetry) +
                 IMAGE_STORE_RESERVE_BYTES;
}

void discardImageSlot(int slot) {
  if (slot < 0 || slot >= static_cast<int>(IMAGE_SLOT_COUNT)) return;
  const uint16_t imageId = imageIds[slot];
  savedImages[slot] = 0;
  imageIds[slot] = 0;
  persistImageProgress();
  removeImageFiles(imageId);
}

void completeImageSlot(int slot) {
  if (slot < 0 || slot >= static_cast<int>(IMAGE_SLOT_COUNT)) return;
  const uint16_t imageId = imageIds[slot];
  char telemetryName[16];
  char jpegName[16];
  telemetryFilename(imageId, telemetryName, sizeof(telemetryName));
  imageFilename(imageId, jpegName, sizeof(jpegName));

  // Remove telemetry first. If power fails at any later point, recovery sees
  // either a queued image missing telemetry or an unpaired JPEG and safely
  // removes it instead of retransmitting or blocking the queue forever.
  LittleFS.remove(telemetryName);
  savedImages[slot] = 0;
  imageIds[slot] = 0;
  persistImageProgress();
  LittleFS.remove(jpegName);
}

esp_err_t StartCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_D0;
  config.pin_d1 = CAM_D1;
  config.pin_d2 = CAM_D2;
  config.pin_d3 = CAM_D3;
  config.pin_d4 = CAM_D4;
  config.pin_d5 = CAM_D5;
  config.pin_d6 = CAM_D6;
  config.pin_d7 = CAM_D7;
  config.pin_xclk = CAM_XCLK;
  config.pin_pclk = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC;
  config.pin_href = CAM_HREF;
  config.pin_sccb_sda = CAM_SDA;
  config.pin_sccb_scl = CAM_SCL;
  config.pin_pwdn = CAM_PWDN;
  config.pin_reset = CAM_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 8;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;
  const esp_err_t result = esp_camera_init(&config);
  cameraDriverStarted = result == ESP_OK;
  return result;
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

void stopCamera() {
  if (cameraDriverStarted) {
    esp_camera_deinit();
    cameraDriverStarted = false;
  }
  pinMode(CAM_PWDN, OUTPUT);
  digitalWrite(CAM_PWDN, HIGH);
  pinMode(CAM_LDO_EN, OUTPUT);
  digitalWrite(CAM_LDO_EN, LOW);
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
    const uint8_t value = file.read();
    if (previous == 0xFF && value == 0xD9) return true;
    previous = value;
  }
  return false;
}

bool readImageTelemetry(uint16_t imageId,
                        helium_jpeg::HeliumTelemetry& telemetry) {
  char filename[16];
  telemetryFilename(imageId, filename, sizeof(filename));
  File file = LittleFS.open(filename, FILE_READ);
  if (!file || file.size() != sizeof(telemetry)) return false;
  return file.read(reinterpret_cast<uint8_t*>(&telemetry), sizeof(telemetry)) ==
         sizeof(telemetry);
}

int oldestStoredImage() { return oldestStoredImageExcept(-1); }

esp_err_t savePhoto(helium_jpeg::HeliumTelemetry telemetry) {
  const uint32_t captureStart = millis();
  camera_fb_t* frame = captureJpeg();
  if (!frame || !validateJpegBuffer(frame->buf, frame->len)) {
    if (frame) esp_camera_fb_return(frame);
    return ESP_FAIL;
  }

  const size_t jpegSize = frame->len;
  if (jpegSize > MAX_JPEG_BYTES) {
    Serial.printf("Image not stored: %u bytes exceeds the limit.\n",
                  static_cast<unsigned>(jpegSize));
    esp_camera_fb_return(frame);
    return ESP_FAIL;
  }

  int slot = freeSlot();
  if (slot < 0) {
    Serial.println("Image not stored: archive has no free slots.");
    esp_camera_fb_return(frame);
    return ESP_FAIL;
  }

  const size_t required = jpegSize + sizeof(telemetry) + IMAGE_STORE_RESERVE_BYTES;
  if (required > LittleFS.totalBytes() - LittleFS.usedBytes()) {
    Serial.println("Image not stored: archive is full.");
    esp_camera_fb_return(frame);
    return ESP_FAIL;
  }

  const uint16_t imageId = nextImageId++;
  char jpegName[16];
  char telemetryName[16];
  imageFilename(imageId, jpegName, sizeof(jpegName));
  telemetryFilename(imageId, telemetryName, sizeof(telemetryName));
  File jpeg = LittleFS.open(jpegName, FILE_WRITE);
  if (!jpeg) {
    esp_camera_fb_return(frame);
    return ESP_FAIL;
  }
  const size_t written = jpeg.write(frame->buf, jpegSize);
  jpeg.close();
  esp_camera_fb_return(frame);
  if (written != jpegSize || !validateJpegFile(jpegName)) {
    LittleFS.remove(jpegName);
    return ESP_FAIL;
  }

  telemetry.jpeg_file_size = jpegSize;
  telemetry.capture_ms = static_cast<uint16_t>(
      min(static_cast<unsigned long>(65535), millis() - captureStart));
  File telemetryFile = LittleFS.open(telemetryName, FILE_WRITE);
  if (!telemetryFile ||
      telemetryFile.write(reinterpret_cast<const uint8_t*>(&telemetry),
                          sizeof(telemetry)) != sizeof(telemetry)) {
    if (telemetryFile) telemetryFile.close();
    removeImageFiles(imageId);
    return ESP_FAIL;
  }
  telemetryFile.close();

  const int count = packetCountForImage(imageId);
  if (count <= 0) {
    removeImageFiles(imageId);
    return ESP_FAIL;
  }
  imageIds[slot] = imageId;
  savedImages[slot] = static_cast<uint16_t>(count);
  persistImageProgress();
  return ESP_OK;
}
