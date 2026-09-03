#pragma once

#include <stddef.h>
#include <stdint.h>

namespace wspr {
namespace telemetry {

constexpr uint64_t kCtOpaqueValueCount = 38951228160ULL;

// Matching WSPR TV ct_dec schemas (line breaks added only for readability):
//   ct,s:2_129:0:1,4096:0:16,32:0:1,64:0:0.1,32:0:1
//   ct,s:3,600:2:0:ts_16:0:1,32768:0:1,32768:0:1
//   ct,s:3,600:2:1:ts_128:0:1,128:0:1,128:0:1,128:0:1,128:0:1
// The temporal filters select diagnostics A/B from the preceding regular
// message timestamp without spending an on-air frame-type field.

struct ChannelDetails {
  bool valid;
  uint16_t number;
  char id1;
  char id3;
  uint8_t startMinute;
  uint8_t lane;
  uint32_t frequencyHz;
};

struct Type1Message {
  char callsign[7];
  char grid[5];
  uint8_t powerDbm;
};

// Resolve a numeric U4B channel using the published 17 m channel map.
ChannelDetails channel17m(uint16_t channel);

// Convert a position into a six-character Maidenhead locator.
bool maidenhead6(double latitude, double longitude, char locator[7]);

// Encode the standard U4B Basic Telemetry message which follows the regular
// callsign/grid message. voltageCentivolts is a real reading or the documented
// fixed sentinel when the hardware has no voltage monitor.
bool encodeBasic(const ChannelDetails& channel, const char locator6[7],
                 int32_t altitudeMetres, int32_t temperatureC,
                 uint16_t voltageCentivolts, double speedKnots,
                 bool gpsValid, Type1Message& message);

// Encode a WSPR TV U4B Custom Telemetry opaque value. slot is the relative
// U4B transmission slot (2-4), not the absolute UTC minute.
bool encodeCustom(const ChannelDetails& channel, uint8_t slot,
                  uint64_t opaqueValue, Type1Message& message);

// Slot 2, extracted least-significant field first with radices:
//   129, 4096, 32, 64, 32
// Remaining packets are encoded as a non-zero-preserving upper bound in
// multiples of 16. faultSummary contains the five mission fault bits.
uint64_t packMissionState(uint8_t storedImages, uint16_t remainingPackets,
                          uint8_t satellites, float hdop,
                          uint8_t faultSummary);

// Slot 3 diagnostics A: reset reason, boots, and short solar boots, extracted
// with radices 16, 32768, 32768.
uint64_t packDiagnosticsA(uint8_t lastResetReason, uint32_t boots,
                          uint32_t shortBoots);

// Slot 3 diagnostics B: brownouts, watchdogs, consecutive failed joins, WSPR
// failures, and storage faults, extracted with five radix-128 fields.
uint64_t packDiagnosticsB(uint32_t brownouts, uint32_t watchdogs,
                          uint32_t consecutiveFailedJoins,
                          uint32_t wsprFailures,
                          uint32_t storageFaults);

// Known-answer and boundary checks used to inhibit RF if the codec is broken.
bool selfTest();

}  // namespace telemetry
}  // namespace wspr
