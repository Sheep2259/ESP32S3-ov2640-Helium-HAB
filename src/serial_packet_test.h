#pragma once

#include <Arduino.h>

namespace serial_packet_test {

struct FlightStatus {
  uint32_t uptimeSeconds;
  uint32_t unixTime;
  bool gpsPositionFresh;
  bool gpsTimeFresh;
  float latitude;
  float longitude;
  float altitudeMetres;
  float hdop;
  uint8_t satellites;
  uint8_t region;
  bool geofenceInhibited;
  bool lorawanReachable;
  bool imageTransferMode;
  uint8_t storedImages;
  uint16_t remainingImagePackets;
  uint16_t statusFlags;
};

using ServiceCallback = void (*)();

// Starts the passive observer. No serial input or test command is accepted.
void begin();

// Called only after the normal mission capture has committed a new queue image.
void imageStored();

// Periodically reports flight state and emits at most one shadow packet.
void service(const FlightStatus& status, ServiceCallback serviceCallback);

// Mirrors a packet prepared for a real LoRaWAN attempt without changing it.
void mirrorLoRaPacket(const uint8_t* data, size_t length);

}  // namespace serial_packet_test
