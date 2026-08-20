#include "serial_packet_test.h"

#include <helium_jpeg.h>

#include <image_store.h>
#include <mission_config.h>

namespace serial_packet_test {
namespace {
#if HAB_SERIAL_PACKET_TEST_MODE
helium_jpeg::HeliumJPEG* shadowEncoder = nullptr;
int pendingSlot = -1;
uint16_t shadowSequence = 0;
uint16_t loraSequence = 0;
uint16_t shadowImageId = 0;
uint16_t shadowPackets = 0;
unsigned long lastPacketMs = 0;
unsigned long lastStatusMs = 0;
bool statusPrinted = false;

uint16_t readU16Be(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) << 8 | data[1];
}

uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFFU;
  while (length--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void printHex(const uint8_t* data, size_t length) {
  static constexpr char digits[] = "0123456789ABCDEF";
  for (size_t index = 0; index < length; ++index) {
    const char encoded[] = {digits[data[index] >> 4],
                            digits[data[index] & 0x0FU]};
    Serial.write(reinterpret_cast<const uint8_t*>(encoded), sizeof(encoded));
  }
}

void emitPacket(const char* source, uint16_t sequence, const uint8_t* data,
                size_t length) {
  Serial.printf("HAB_PACKET %s %u %04X ", source, sequence,
                crc16Ccitt(data, length));
  printHex(data, length);
  Serial.println();
}

void stopShadow(const char* reason) {
  if (shadowEncoder != nullptr) {
    Serial.printf("HAB_IMAGE_END source=shadow image=%u packets=%u reason=%s\n",
                  shadowImageId, shadowPackets, reason);
    delete shadowEncoder;
    shadowEncoder = nullptr;
  }
  shadowImageId = 0;
  shadowPackets = 0;
}

bool startShadow() {
  if (pendingSlot < 0) return false;
  stopShadow("replaced");
  shadowEncoder = new helium_jpeg::HeliumJPEG();
  if (shadowEncoder == nullptr ||
      !image_store::prepareEncoder(pendingSlot, *shadowEncoder)) {
    Serial.println("HAB_IMAGE_ERROR source=shadow reason=prepare_failed");
    delete shadowEncoder;
    shadowEncoder = nullptr;
    pendingSlot = -1;
    return false;
  }
  Serial.printf("HAB_IMAGE_READY source=shadow total=%d data=%d\n",
                shadowEncoder->getPacketCount(),
                shadowEncoder->getDataPacketCount());
  pendingSlot = -1;
  lastPacketMs = millis() - HAB_SERIAL_TEST_PACKET_INTERVAL_MS;
  return true;
}

void emitStatus(const FlightStatus& status) {
  Serial.printf(
      "HAB_STATUS uptime=%u utc=%u gps=%u time=%u lat=%.6f lon=%.6f "
      "alt=%.1f hdop=%.1f sats=%u region=%u geofence_inhibit=%u "
      "lorawan=%u image_mode=%u images=%u packets=%u flags=0x%04X\n",
      status.uptimeSeconds, status.unixTime, status.gpsPositionFresh ? 1U : 0U,
      status.gpsTimeFresh ? 1U : 0U, status.latitude, status.longitude,
      status.altitudeMetres, status.hdop, status.satellites, status.region,
      status.geofenceInhibited ? 1U : 0U,
      status.lorawanReachable ? 1U : 0U,
      status.imageTransferMode ? 1U : 0U, status.storedImages,
      status.remainingImagePackets, status.statusFlags);
}
#endif
}  // namespace

void begin() {
#if HAB_SERIAL_PACKET_TEST_MODE
  Serial.println(
      "[serial-test] TEMPORARY passive observer enabled; serial input is ignored.");
  pendingSlot = image_store::newestImage();
  if (pendingSlot >= 0) {
    Serial.printf("HAB_IMAGE_QUEUED source=shadow slot=%d reason=boot\n",
                  pendingSlot);
  }
#endif
}

void imageStored() {
#if HAB_SERIAL_PACKET_TEST_MODE
  pendingSlot = image_store::newestImage();
  Serial.printf("HAB_IMAGE_QUEUED source=shadow slot=%d\n", pendingSlot);
#endif
}

void service(const FlightStatus& status, ServiceCallback serviceCallback) {
#if HAB_SERIAL_PACKET_TEST_MODE
  const unsigned long now = millis();
  if (!statusPrinted || now - lastStatusMs >= HAB_SERIAL_TEST_STATUS_INTERVAL_MS) {
    statusPrinted = true;
    lastStatusMs = now;
    emitStatus(status);
  }

  if (shadowEncoder == nullptr && !startShadow()) return;
  if (now - lastPacketMs < HAB_SERIAL_TEST_PACKET_INTERVAL_MS) return;
  lastPacketMs = now;

  helium_jpeg::HeliumPacket packet;
  if (!shadowEncoder->getNextPacket(packet)) {
    stopShadow("complete");
    return;
  }
  const uint16_t imageId = readU16Be(packet.data);
  if (shadowPackets == 0) {
    shadowImageId = imageId;
    Serial.printf("HAB_IMAGE_BEGIN source=shadow image=%u\n", shadowImageId);
  }
  emitPacket("shadow", shadowSequence++, packet.data, packet.length);
  ++shadowPackets;
  if (serviceCallback != nullptr) serviceCallback();
#else
  (void)status;
  (void)serviceCallback;
#endif
}

void mirrorLoRaPacket(const uint8_t* data, size_t length) {
#if HAB_SERIAL_PACKET_TEST_MODE
  if (data == nullptr || length == 0) return;
  emitPacket("lora", loraSequence++, data, length);
#else
  (void)data;
  (void)length;
#endif
}

}  // namespace serial_packet_test
