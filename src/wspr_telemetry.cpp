#include "wspr_telemetry.h"

#include <cmath>
#include <cstring>

namespace wspr {
namespace telemetry {
namespace {

constexpr uint8_t kPowerDbmValues[19] = {
    0,  3,  7,  10, 13, 17, 20, 23, 27, 30,
    33, 37, 40, 43, 47, 50, 53, 57, 60};
constexpr uint8_t k17mMinutes[5] = {2, 4, 6, 8, 0};
constexpr uint16_t k17mLaneOffsetsHz[4] = {20, 60, 140, 180};
constexpr uint32_t k17mChannelWindowLowHz = 18106000UL;

template <typename T>
T clampValue(T value, T low, T high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

void encodeCallValue(uint32_t value, char id1, char id3,
                     char callsign[7]) {
  const uint8_t id6 = value % 26U;
  value /= 26U;
  const uint8_t id5 = value % 26U;
  value /= 26U;
  const uint8_t id4 = value % 26U;
  value /= 26U;
  const uint8_t id2 = value % 36U;

  callsign[0] = id1;
  callsign[1] = id2 < 10U ? static_cast<char>('0' + id2)
                          : static_cast<char>('A' + id2 - 10U);
  callsign[2] = id3;
  callsign[3] = static_cast<char>('A' + id4);
  callsign[4] = static_cast<char>('A' + id5);
  callsign[5] = static_cast<char>('A' + id6);
  callsign[6] = '\0';
}

void encodeGridPowerValue(uint32_t value, char grid[5], uint8_t& powerDbm) {
  const uint8_t power = value % 19U;
  value /= 19U;
  const uint8_t g4 = value % 10U;
  value /= 10U;
  const uint8_t g3 = value % 10U;
  value /= 10U;
  const uint8_t g2 = value % 18U;
  value /= 18U;
  const uint8_t g1 = value % 18U;

  grid[0] = static_cast<char>('A' + g1);
  grid[1] = static_cast<char>('A' + g2);
  grid[2] = static_cast<char>('0' + g3);
  grid[3] = static_cast<char>('0' + g4);
  grid[4] = '\0';
  powerDbm = kPowerDbmValues[power];
}

uint32_t quantizeNearest(double value, double low, double step) {
  return static_cast<uint32_t>(std::floor(((value - low) / step) + 0.5));
}

uint32_t saturate(uint32_t value, uint32_t maximum) {
  return value > maximum ? maximum : value;
}

bool decodeCustomForTest(const Type1Message& message, uint8_t& slot,
                         uint64_t& opaqueValue) {
  uint32_t callValue = message.callsign[1] <= '9'
                           ? static_cast<uint32_t>(message.callsign[1] - '0')
                           : static_cast<uint32_t>(message.callsign[1] - 'A' + 10);
  callValue = callValue * 26U +
              static_cast<uint32_t>(message.callsign[3] - 'A');
  callValue = callValue * 26U +
              static_cast<uint32_t>(message.callsign[4] - 'A');
  callValue = callValue * 26U +
              static_cast<uint32_t>(message.callsign[5] - 'A');

  uint8_t power = 0;
  while (power < 19U && kPowerDbmValues[power] != message.powerDbm) ++power;
  if (power == 19U) return false;
  uint32_t gridPowerValue = static_cast<uint32_t>(message.grid[0] - 'A');
  gridPowerValue = gridPowerValue * 18U +
                   static_cast<uint32_t>(message.grid[1] - 'A');
  gridPowerValue = gridPowerValue * 10U +
                   static_cast<uint32_t>(message.grid[2] - '0');
  gridPowerValue = gridPowerValue * 10U +
                   static_cast<uint32_t>(message.grid[3] - '0');
  gridPowerValue = gridPowerValue * 19U + power;

  const uint64_t bigNumber =
      static_cast<uint64_t>(callValue) * 615600ULL + gridPowerValue;
  if ((bigNumber & 1ULL) != 0ULL) return false;
  uint64_t value = bigNumber >> 1U;
  value = (value / 320ULL) * 320ULL + (value % 64ULL) * 5ULL +
          ((value / 64ULL) % 5ULL);
  slot = static_cast<uint8_t>(value % 5ULL);
  opaqueValue = value / 5ULL;
  return true;
}

}  // namespace

ChannelDetails channel17m(uint16_t channel) {
  ChannelDetails result = {};
  if (channel > 599U) return result;

  const uint8_t column = channel / 20U;
  const uint8_t row = channel % 20U;
  result.valid = true;
  result.number = channel;
  result.id1 = column < 10U ? '0' : (column < 20U ? '1' : 'Q');
  result.id3 = static_cast<char>('0' + (column % 10U));
  result.startMinute = k17mMinutes[row % 5U];
  result.lane = row / 5U + 1U;
  result.frequencyHz =
      k17mChannelWindowLowHz + k17mLaneOffsetsHz[row / 5U];
  return result;
}

bool maidenhead6(double latitude, double longitude, char locator[7]) {
  if (locator == nullptr || !std::isfinite(latitude) ||
      !std::isfinite(longitude) || latitude < -90.0 || latitude >= 90.0 ||
      longitude < -180.0 || longitude >= 180.0) {
    return false;
  }

  double lon = longitude + 180.0;
  double lat = latitude + 90.0;
  const int fieldLon = static_cast<int>(lon / 20.0);
  const int fieldLat = static_cast<int>(lat / 10.0);
  lon -= fieldLon * 20.0;
  lat -= fieldLat * 10.0;
  const int squareLon = static_cast<int>(lon / 2.0);
  const int squareLat = static_cast<int>(lat);
  lon -= squareLon * 2.0;
  lat -= squareLat;
  const int subLon = clampValue(static_cast<int>(lon * 12.0), 0, 23);
  const int subLat = clampValue(static_cast<int>(lat * 24.0), 0, 23);

  locator[0] = static_cast<char>('A' + fieldLon);
  locator[1] = static_cast<char>('A' + fieldLat);
  locator[2] = static_cast<char>('0' + squareLon);
  locator[3] = static_cast<char>('0' + squareLat);
  locator[4] = static_cast<char>('A' + subLon);
  locator[5] = static_cast<char>('A' + subLat);
  locator[6] = '\0';
  return true;
}

bool encodeBasic(const ChannelDetails& channel, const char locator6[7],
                 int32_t altitudeMetres, int32_t temperatureC,
                 uint16_t voltageCentivolts, double speedKnots,
                 bool gpsValid, Type1Message& message) {
  if (!channel.valid || locator6 == nullptr || strlen(locator6) != 6U ||
      locator6[4] < 'A' || locator6[4] > 'X' || locator6[5] < 'A' ||
      locator6[5] > 'X' || !std::isfinite(speedKnots)) {
    return false;
  }

  altitudeMetres = clampValue<int32_t>(altitudeMetres, 0, 21340);
  temperatureC = clampValue<int32_t>(temperatureC, -50, 39);
  voltageCentivolts =
      clampValue<uint16_t>(voltageCentivolts, 300U, 495U);
  speedKnots = clampValue<double>(speedKnots, 0.0, 82.0);

  uint32_t callValue = static_cast<uint32_t>(locator6[4] - 'A');
  callValue = callValue * 24U + static_cast<uint32_t>(locator6[5] - 'A');
  callValue = callValue * 1068U +
              quantizeNearest(altitudeMetres, 0.0, 20.0);
  encodeCallValue(callValue, channel.id1, channel.id3, message.callsign);

  const uint8_t temperature = static_cast<uint8_t>(temperatureC + 50);
  const uint8_t voltage = static_cast<uint8_t>(
      (quantizeNearest(voltageCentivolts, 300.0, 5.0) + 20U) % 40U);
  const uint8_t speed = static_cast<uint8_t>(
      quantizeNearest(speedKnots, 0.0, 2.0));
  uint32_t gridPowerValue = temperature;
  gridPowerValue = gridPowerValue * 40U + voltage;
  gridPowerValue = gridPowerValue * 42U + speed;
  gridPowerValue = gridPowerValue * 2U + (gpsValid ? 1U : 0U);
  gridPowerValue = gridPowerValue * 2U + 1U;
  encodeGridPowerValue(gridPowerValue, message.grid, message.powerDbm);
  return true;
}

bool encodeCustom(const ChannelDetails& channel, uint8_t slot,
                  uint64_t opaqueValue, Type1Message& message) {
  if (!channel.valid || slot < 2U || slot > 4U ||
      opaqueValue >= kCtOpaqueValueCount) {
    return false;
  }

  // HdrSlot is radix 5 and HdrTelemetryType is the least-significant bit.
  uint64_t bigNumber = (opaqueValue * 5ULL + slot) << 1U;

  // WSPR TV CT compatibility transform. Temporarily remove the type bit,
  // move HdrSlot into the legacy wire position, then restore the type bit.
  uint64_t value = bigNumber >> 1U;
  value = (value / 320ULL) * 320ULL + (value % 5ULL) * 64ULL +
          ((value / 5ULL) % 64ULL);
  bigNumber = value << 1U;

  const uint32_t callValue = static_cast<uint32_t>(bigNumber / 615600ULL);
  const uint32_t gridPowerValue =
      static_cast<uint32_t>(bigNumber % 615600ULL);
  encodeCallValue(callValue, channel.id1, channel.id3, message.callsign);
  encodeGridPowerValue(gridPowerValue, message.grid, message.powerDbm);
  return true;
}

uint64_t packMissionState(uint8_t storedImages, uint16_t remainingPackets,
                          uint8_t satellites, float hdop,
                          uint8_t faultSummary) {
  const uint64_t images = storedImages > 128U ? 128U : storedImages;
  const uint64_t packets = remainingPackets == 0U
                               ? 0U
                               : saturate((remainingPackets + 15U) / 16U,
                                          4095U);
  const uint64_t sats = satellites > 31U ? 31U : satellites;
  const uint64_t hdopTenths = !std::isfinite(hdop) || hdop <= 0.0f
                                  ? 0U
                                  : saturate(static_cast<uint32_t>(
                                                 std::floor(hdop * 10.0f + 0.5f)),
                                             63U);
  const uint64_t faults = faultSummary & 0x1FU;

  // Pack in reverse display order so StoredImages is the first field that a
  // WSPR TV mixed-radix extractor reads after the CT header.
  uint64_t value = faults;
  value = value * 64ULL + hdopTenths;
  value = value * 32ULL + sats;
  value = value * 4096ULL + packets;
  value = value * 129ULL + images;
  return value;
}

uint64_t packDiagnosticsA(uint8_t lastResetReason, uint32_t boots,
                          uint32_t resets) {
  uint64_t value = saturate(resets, 32767U);
  value = value * 32768ULL + saturate(boots, 32767U);
  value = value * 16ULL + (lastResetReason & 0x0FU);
  return value;
}

uint64_t packDiagnosticsB(uint32_t brownouts, uint32_t watchdogs,
                          uint32_t failedJoins, uint32_t storageRepairs,
                          uint32_t storageFaults) {
  uint64_t value = saturate(storageFaults, 127U);
  value = value * 128ULL + saturate(storageRepairs, 127U);
  value = value * 128ULL + saturate(failedJoins, 127U);
  value = value * 128ULL + saturate(watchdogs, 127U);
  value = value * 128ULL + saturate(brownouts, 127U);
  return value;
}

bool selfTest() {
  const ChannelDetails testChannel = {true, 0, 'Q', '5', 0, 1, 18106020UL};
  Type1Message message = {};
  if (!encodeBasic(testChannel, "AA00JM", 5120, -5, 325, 25.0, true,
                   message)) {
    return false;
  }
  if (strcmp(message.callsign, "QD5WPK") != 0 ||
      strcmp(message.grid, "IR39") != 0 || message.powerDbm != 47U) {
    return false;
  }

  const ChannelDetails channel = channel17m(599U);
  if (!channel.valid || channel.id1 != 'Q' || channel.id3 != '9' ||
      channel.startMinute != 0U || channel.lane != 4U ||
      channel.frequencyHz != 18106180UL) {
    return false;
  }

  if (packMissionState(128U, 65535U, 31U, 99.0f, 0xFFU) >=
          kCtOpaqueValueCount ||
      packDiagnosticsA(0xFFU, UINT32_MAX, UINT32_MAX) >=
          kCtOpaqueValueCount ||
      packDiagnosticsB(UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX,
                       UINT32_MAX) >= kCtOpaqueValueCount) {
    return false;
  }

  const uint64_t values[] = {
      0ULL,
      packMissionState(17U, 1234U, 9U, 1.3f, 0x12U),
      packDiagnosticsA(9U, 456U, 23U),
      packDiagnosticsB(1U, 2U, 3U, 4U, 5U),
      kCtOpaqueValueCount - 1ULL,
  };
  for (uint8_t testSlot = 2U; testSlot <= 4U; ++testSlot) {
    for (const uint64_t expected : values) {
      Type1Message custom = {};
      uint8_t decodedSlot = 0;
      uint64_t decodedValue = 0;
      if (!encodeCustom(channel, testSlot, expected, custom) ||
          !decodeCustomForTest(custom, decodedSlot, decodedValue) ||
          decodedSlot != testSlot || decodedValue != expected) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace telemetry
}  // namespace wspr
