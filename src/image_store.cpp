#include "image_store.h"

#include <LittleFS.h>
#include <Preferences.h>

#include <cstddef>
#include <cstdlib>
#include <cstring>

#include <camera.h>
#include <mission_diagnostics.h>

namespace image_store {
namespace {
constexpr size_t kReserveBytes = 2048U;
constexpr uint8_t kProgressCheckpointPackets = 16;
constexpr uint32_t kQueueMagic = 0x48514232UL;  // "HQB2"
constexpr uint16_t kQueueVersion = 2;
constexpr char kLittleFsBasePath[] = "/littlefs";
constexpr char kLittleFsPartitionLabel[] = "littlefs";
constexpr uint8_t kLittleFsMaxOpenFiles = 10;

Preferences preferences;
uint16_t remainingPackets[kSlotCount] = {};
uint16_t imageIds[kSlotCount] = {};
uint16_t nextImageId = 0;
uint32_t lastCapture = 0;
uint32_t queueGeneration = 0;
uint8_t packetsSinceCheckpoint = 0;

struct QueueRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t reserved;
  uint32_t generation;
  uint16_t remaining[kSlotCount];
  uint16_t ids[kSlotCount];
  uint16_t nextId;
  uint16_t reserved2;
  uint32_t crc;
};

void imageFilename(uint16_t imageId, char* output, size_t outputSize) {
  snprintf(output, outputSize, "/%u.jpg", imageId);
}

void telemetryFilename(uint16_t imageId, char* output, size_t outputSize) {
  snprintf(output, outputSize, "/%u.tlm", imageId);
}

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1) ^ (0xEDB88320UL &
                          static_cast<uint32_t>(-static_cast<int32_t>(crc & 1U)));
    }
  }
  return ~crc;
}

bool validJpegFile(const char* filename) {
  File file = LittleFS.open(filename, FILE_READ);
  if (!file || file.size() < 4) return false;
  uint8_t start[2];
  if (file.read(start, 2) != 2 || start[0] != 0xFF || start[1] != 0xD8) {
    return false;
  }
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

bool readTelemetry(uint16_t imageId,
                   helium_jpeg::HeliumTelemetry& telemetry) {
  char filename[16];
  telemetryFilename(imageId, filename, sizeof(filename));
  File file = LittleFS.open(filename, FILE_READ);
  if (!file || file.size() != sizeof(telemetry)) return false;
  return file.read(reinterpret_cast<uint8_t*>(&telemetry), sizeof(telemetry)) ==
         sizeof(telemetry);
}

void removeFiles(uint16_t imageId) {
  char jpegName[16];
  char telemetryName[16];
  imageFilename(imageId, jpegName, sizeof(jpegName));
  telemetryFilename(imageId, telemetryName, sizeof(telemetryName));
  LittleFS.remove(jpegName);
  LittleFS.remove(telemetryName);
}

bool isTracked(uint16_t imageId) {
  for (size_t i = 0; i < kSlotCount; ++i) {
    if (remainingPackets[i] > 0 && imageIds[i] == imageId) return true;
  }
  return false;
}

int freeSlot() {
  for (size_t i = 0; i < kSlotCount; ++i) {
    if (remainingPackets[i] == 0) return static_cast<int>(i);
  }
  return -1;
}

int oldestImageExcept(int excludedSlot) {
  int selected = -1;
  for (size_t i = 0; i < kSlotCount; ++i) {
    if (remainingPackets[i] == 0 || static_cast<int>(i) == excludedSlot) {
      continue;
    }
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
  if (end == name || parsed > UINT16_MAX || strcmp(end, extension) != 0) {
    return false;
  }
  imageId = static_cast<uint16_t>(parsed);
  return true;
}

int packetCountForImage(uint16_t imageId) {
  char jpegName[16];
  imageFilename(imageId, jpegName, sizeof(jpegName));
  helium_jpeg::HeliumTelemetry telemetry = {};
  if (!validJpegFile(jpegName) || !readTelemetry(imageId, telemetry)) return -1;

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
  return validJpegFile(jpegName) && readTelemetry(imageId, telemetry);
}

void persistProgress() {
  QueueRecord record = {};
  record.magic = kQueueMagic;
  record.version = kQueueVersion;
  record.generation = ++queueGeneration;
  memcpy(record.remaining, remainingPackets, sizeof(remainingPackets));
  memcpy(record.ids, imageIds, sizeof(imageIds));
  record.nextId = nextImageId;
  record.crc = crc32(reinterpret_cast<const uint8_t*>(&record),
                     offsetof(QueueRecord, crc));
  if (preferences.putBytes("queue_v2", &record, sizeof(record)) !=
      sizeof(record)) {
    Serial.println("Failed to persist atomic image queue progress.");
    recordStorageFault();
  }
}

bool loadQueueRecord() {
  QueueRecord record = {};
  if (preferences.getBytesLength("queue_v2") != sizeof(record) ||
      preferences.getBytes("queue_v2", &record, sizeof(record)) !=
          sizeof(record) ||
      record.magic != kQueueMagic || record.version != kQueueVersion ||
      record.crc != crc32(reinterpret_cast<const uint8_t*>(&record),
                          offsetof(QueueRecord, crc))) {
    return false;
  }
  memcpy(remainingPackets, record.remaining, sizeof(remainingPackets));
  memcpy(imageIds, record.ids, sizeof(imageIds));
  nextImageId = record.nextId;
  queueGeneration = record.generation;
  return true;
}

void discardSlot(int slot) {
  if (slot < 0 || slot >= static_cast<int>(kSlotCount)) return;
  const uint16_t imageId = imageIds[slot];
  remainingPackets[slot] = 0;
  imageIds[slot] = 0;
  persistProgress();
  removeFiles(imageId);
}

void completeSlot(int slot) {
  if (slot < 0 || slot >= static_cast<int>(kSlotCount)) return;
  const uint16_t imageId = imageIds[slot];
  char telemetryName[16];
  char jpegName[16];
  telemetryFilename(imageId, telemetryName, sizeof(telemetryName));
  imageFilename(imageId, jpegName, sizeof(jpegName));

  // Deleting telemetry first makes an interrupted cleanup unambiguously
  // recoverable on the next boot.
  LittleFS.remove(telemetryName);
  remainingPackets[slot] = 0;
  imageIds[slot] = 0;
  persistProgress();
  LittleFS.remove(jpegName);
}

void reconcile() {
  bool changed = false;
  for (size_t i = 0; i < kSlotCount; ++i) {
    if (remainingPackets[i] == 0) continue;
    if (!imageFilesValid(imageIds[i])) {
      Serial.printf("Dropping corrupt image queue entry %u.\n", imageIds[i]);
      removeFiles(imageIds[i]);
      remainingPackets[i] = 0;
      imageIds[i] = 0;
      changed = true;
    }
  }

  uint16_t jpegCandidates[kSlotCount] = {};
  size_t candidateCount = 0;
  File root = LittleFS.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      uint16_t imageId = 0;
      if (!entry.isDirectory() && candidateCount < kSlotCount &&
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
      remainingPackets[slot] = static_cast<uint16_t>(count);
      Serial.printf("Recovered image %u after interrupted commit.\n", imageId);
      changed = true;
    } else {
      removeFiles(imageId);
    }
  }

  uint16_t telemetryCandidates[kSlotCount] = {};
  size_t telemetryCount = 0;
  root = LittleFS.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      uint16_t imageId = 0;
      if (!entry.isDirectory() && telemetryCount < kSlotCount &&
          parseImageId(entry.name(), ".tlm", imageId)) {
        telemetryCandidates[telemetryCount++] = imageId;
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }
  for (size_t i = 0; i < telemetryCount; ++i) {
    if (isTracked(telemetryCandidates[i])) continue;
    char telemetryName[16];
    telemetryFilename(telemetryCandidates[i], telemetryName,
                      sizeof(telemetryName));
    LittleFS.remove(telemetryName);
    changed = true;
  }

  while (isTracked(nextImageId)) ++nextImageId;
  for (size_t i = 0; i < kSlotCount; ++i) {
    if (remainingPackets[i] == 0) continue;
    helium_jpeg::HeliumTelemetry telemetry = {};
    if (readTelemetry(imageIds[i], telemetry) &&
        telemetry.timestamp > lastCapture) {
      lastCapture = telemetry.timestamp;
    }
  }
  if (changed) {
    recordStorageRepair();
    persistProgress();
  }
  if (lastCapture != 0) preferences.putUInt("last_capture", lastCapture);
}

}  // namespace

bool begin() {
  if (!preferences.begin("img_data", false)) return false;

  bool mounted = false;
  for (uint8_t attempt = 0; attempt < 3 && !mounted; ++attempt) {
    mounted = LittleFS.begin(false, kLittleFsBasePath, kLittleFsMaxOpenFiles,
                             kLittleFsPartitionLabel);
    if (!mounted) {
      LittleFS.end();
      delay(50);
    }
  }
  if (!mounted) {
    // Count one failed mount episode, rather than each bounded retry.
    recordStorageFault();
    Serial.println(
        "LittleFS mount failed after bounded retries; formatting filesystem.");
    if (!LittleFS.format() ||
        !LittleFS.begin(false, kLittleFsBasePath, kLittleFsMaxOpenFiles,
                        kLittleFsPartitionLabel)) {
      LittleFS.end();
      Serial.println("LittleFS format/recovery failed; storage unavailable.");
      return false;
    }

    // Formatting removes every JPEG and telemetry file. Preserve the rolling
    // ID from a valid NVS queue when possible, but clear queue membership and
    // capture time so erased files cannot block or masquerade as live images.
    const bool queueLoaded = loadQueueRecord();
    const uint16_t preservedNextImageId =
        queueLoaded ? nextImageId : preferences.getUShort("next_image_id", 0);
    memset(remainingPackets, 0, sizeof(remainingPackets));
    memset(imageIds, 0, sizeof(imageIds));
    nextImageId = preservedNextImageId;
    lastCapture = 0;
    packetsSinceCheckpoint = 0;
    if (preferences.putUInt("last_capture", 0) != sizeof(lastCapture)) {
      recordStorageFault();
      Serial.println("Failed to clear capture time after filesystem format.");
    }
    persistProgress();
    recordStorageRepair();
    Serial.println(
        "LittleFS recovery complete; previous stored images were erased.");
    return true;
  }

  lastCapture = preferences.getUInt("last_capture", 0);
  const bool queueV2Present = preferences.getBytesLength("queue_v2") != 0;
  bool loaded = loadQueueRecord();
  if (!loaded && !queueV2Present &&
      preferences.getBytes("remain", remainingPackets,
                           sizeof(remainingPackets)) == sizeof(remainingPackets) &&
      preferences.getBytes("image_ids", imageIds, sizeof(imageIds)) ==
          sizeof(imageIds)) {
    nextImageId = preferences.getUShort("next_image_id", 0);
    loaded = true;
    Serial.println("Migrating legacy image queue record.");
  }
  if (!loaded) {
    if (queueV2Present) recordStorageRepair();
    memset(remainingPackets, 0, sizeof(remainingPackets));
    memset(imageIds, 0, sizeof(imageIds));
    nextImageId = 0;
    persistProgress();
  } else if (queueGeneration == 0) {
    persistProgress();
  }
  reconcile();
  return true;
}

uint32_t lastCaptureUtc() { return lastCapture; }

uint8_t imageCount() {
  uint16_t count = 0;
  for (size_t i = 0; i < kSlotCount; ++i) {
    if (remainingPackets[i] != 0) ++count;
  }
  return count > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(count);
}

uint16_t remainingPacketCount() {
  uint32_t count = 0;
  for (size_t i = 0; i < kSlotCount; ++i) count += remainingPackets[i];
  return count > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(count);
}

bool canAcceptCapture() {
  return freeSlot() >= 0 &&
         LittleFS.totalBytes() - LittleFS.usedBytes() >=
             kMaxJpegBytes + sizeof(helium_jpeg::HeliumTelemetry) +
                 kReserveBytes;
}

int oldestImage() { return oldestImageExcept(-1); }

int newestImage() {
  int selected = -1;
  for (size_t i = 0; i < kSlotCount; ++i) {
    if (remainingPackets[i] == 0) continue;
    if (selected < 0 ||
        static_cast<int16_t>(imageIds[i] - imageIds[selected]) > 0) {
      selected = static_cast<int>(i);
    }
  }
  return selected;
}

bool prepareEncoder(int slot, helium_jpeg::HeliumJPEG& encoder) {
  if (slot < 0 || slot >= static_cast<int>(kSlotCount) ||
      remainingPackets[slot] == 0) {
    return false;
  }
  const uint16_t imageId = imageIds[slot];
  char filename[16];
  imageFilename(imageId, filename, sizeof(filename));
  File image = LittleFS.open(filename, FILE_READ);
  if (!image) {
    discardSlot(slot);
    return false;
  }
  const int totalPackets = encoder.begin(image, imageId);
  image.close();
  helium_jpeg::HeliumTelemetry telemetry = {};
  if (totalPackets <= 0 || !readTelemetry(imageId, telemetry)) {
    discardSlot(slot);
    return false;
  }
  if (remainingPackets[slot] > totalPackets) {
    remainingPackets[slot] = static_cast<uint16_t>(totalPackets);
    persistProgress();
  }
  encoder.setTelemetry(telemetry);
  const uint16_t sent = static_cast<uint16_t>(totalPackets) -
                        remainingPackets[slot];
  if (!encoder.skipPackets(sent)) {
    discardSlot(slot);
    return false;
  }
  return true;
}

bool packetSent(int slot) {
  if (slot < 0 || slot >= static_cast<int>(kSlotCount) ||
      remainingPackets[slot] == 0) {
    return false;
  }
  if (--remainingPackets[slot] == 0) {
    completeSlot(slot);
    packetsSinceCheckpoint = 0;
    return true;
  }
  if (++packetsSinceCheckpoint >= kProgressCheckpointPackets) {
    persistProgress();
    packetsSinceCheckpoint = 0;
  }
  return false;
}

esp_err_t save(const camera_fb_t& frame,
               helium_jpeg::HeliumTelemetry telemetry) {
  if (telemetry.timestamp == 0) return ESP_ERR_INVALID_ARG;
  if (!camera::validJpeg(frame.buf, frame.len)) return ESP_FAIL;
  if (frame.len > kMaxJpegBytes) {
    Serial.printf("Image not stored: %u bytes exceeds the limit.\n",
                  static_cast<unsigned>(frame.len));
    return ESP_FAIL;
  }

  const int slot = freeSlot();
  if (slot < 0) {
    Serial.println("Image not stored: archive has no free slots.");
    return ESP_FAIL;
  }
  const size_t required = frame.len + sizeof(telemetry) + kReserveBytes;
  if (required > LittleFS.totalBytes() - LittleFS.usedBytes()) {
    Serial.println("Image not stored: archive is full.");
    return ESP_FAIL;
  }

  const uint16_t imageId = nextImageId++;
  char jpegName[16];
  char telemetryName[16];
  imageFilename(imageId, jpegName, sizeof(jpegName));
  telemetryFilename(imageId, telemetryName, sizeof(telemetryName));
  File jpeg = LittleFS.open(jpegName, FILE_WRITE);
  if (!jpeg) {
    recordStorageFault();
    return ESP_FAIL;
  }
  const size_t written = jpeg.write(frame.buf, frame.len);
  jpeg.close();
  if (written != frame.len || !validJpegFile(jpegName)) {
    LittleFS.remove(jpegName);
    recordStorageFault();
    return ESP_FAIL;
  }

  telemetry.jpeg_file_size = frame.len;
  File telemetryFile = LittleFS.open(telemetryName, FILE_WRITE);
  if (!telemetryFile) {
    removeFiles(imageId);
    recordStorageFault();
    return ESP_FAIL;
  }
  const size_t telemetryWritten = telemetryFile.write(
      reinterpret_cast<const uint8_t*>(&telemetry), sizeof(telemetry));
  telemetryFile.close();
  if (telemetryWritten != sizeof(telemetry)) {
    removeFiles(imageId);
    recordStorageFault();
    return ESP_FAIL;
  }

  const int count = packetCountForImage(imageId);
  if (count <= 0) {
    removeFiles(imageId);
    return ESP_FAIL;
  }
  if (preferences.putUInt("last_capture", telemetry.timestamp) !=
      sizeof(telemetry.timestamp)) {
    removeFiles(imageId);
    recordStorageFault();
    return ESP_FAIL;
  }
  lastCapture = telemetry.timestamp;
  imageIds[slot] = imageId;
  remainingPackets[slot] = static_cast<uint16_t>(count);
  persistProgress();
  return ESP_OK;
}

}  // namespace image_store
