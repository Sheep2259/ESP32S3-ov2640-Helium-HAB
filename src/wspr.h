#pragma once

#include <Arduino.h>

namespace wspr {

using BusIdleCallback = bool (*)();
using ServiceCallback = void (*)();

struct MissionData {
  uint32_t unixTime;
  float latitude;
  float longitude;
  float altitudeMetres;
  float speedKmh;
  float hdop;
  uint16_t statusFlags;
  uint16_t remainingImagePackets;
  uint8_t storedImages;
  uint8_t satellites;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t centisecond;
};

// Owns the Si5351, WSPR encoding, custom telemetry encoding, and mission
// transmit schedule. The implementation stays private to wspr.cpp.
class MissionRadio {
 public:
  MissionRadio(BusIdleCallback busIdle, ServiceCallback service);
  ~MissionRadio();

  MissionRadio(const MissionRadio&) = delete;
  MissionRadio& operator=(const MissionRadio&) = delete;

  void begin();
  // A fresh GPS snapshot is required to start a sequence. Once its regular
  // WSPR frame starts, the remaining U4B frames use that snapshot and a
  // monotonic schedule so temporary GPS loss cannot break the sequence.
  bool transmitIfDue(const MissionData& data, bool imageTransferMode,
                     bool gpsFresh);
  bool sequenceActive() const;
  bool transmissionFaulted() const;

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace wspr
