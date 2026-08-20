#include "wspr.h"

#include <JTEncode.h>
#include <Wire.h>
#include <esp_timer.h>
#include <si5351.h>

#include <cstring>

#include "mission_config.h"
#include "mission_diagnostics.h"
#include "pin_defs.h"

namespace wspr {
namespace {
constexpr size_t kFrameSymbols = 162;
constexpr uint8_t kDiagnosticTelemetryPeriod = 10;
constexpr uint64_t kToneSpacingCentiHz = 146;
constexpr uint64_t kSymbolNumeratorUs = 8192ULL * 1000000ULL;
constexpr uint32_t kSymbolDenominator = 12000;

struct UtcTime {
  uint8_t minute;
  uint8_t second;
  uint8_t centisecond;
};

struct Config {
  const char* callsign;
  const char* grid;
  uint64_t baseFrequencyCentiHz;
  int8_t powerDbm;
  uint8_t intervalMinutes;
  uint8_t startWindowCentiseconds;
  int32_t correctionPpb;
  uint32_t referenceFrequencyHz;
};

enum class Result : uint8_t {
  Ok,
  NotDue,
  InvalidConfig,
  I2cBusy,
  Si5351NotFound,
  SynthesizerError,
};

class Transmitter {
 public:
  Transmitter(const Config& config, BusIdleCallback busIdle,
              ServiceCallback service)
      : config_(config), busIdle_(busIdle), service_(service) {}

  Result begin();
  bool due(const UtcTime& utc) const;
  Result transmit(const UtcTime& utc);
  Result transmitSymbols(const uint8_t* symbols, size_t count);

 private:
  bool configValid() const;
  bool busIdle() const { return busIdle_ == nullptr || busIdle_(); }
  void stop();

  Config config_;
  BusIdleCallback busIdle_;
  ServiceCallback service_;
  Si5351 si5351_;
  bool ready_ = false;
};

struct PositionTelemetry {
  uint32_t unixTime;
  int32_t latitudeMicrodegrees;
  int32_t longitudeMicrodegrees;
  int16_t altitudeMetres;
  uint8_t storedImages;
  uint16_t remainingPackets;
  int8_t espTemperatureC;
  uint8_t satellites;
  uint8_t hdopTenths;
  uint16_t statusFlags;
};

constexpr uint8_t kSync[16] = {0, 3, 1, 2, 3, 0, 2, 1,
                               1, 2, 0, 3, 2, 1, 3, 0};
constexpr size_t kPayloadBytes = 32;

bool upperAlpha(char value) { return value >= 'A' && value <= 'Z'; }
bool digit(char value) { return value >= '0' && value <= '9'; }

void waitUntil(int64_t deadlineUs) {
  while (true) {
    const int64_t remainingUs = deadlineUs - esp_timer_get_time();
    if (remainingUs <= 0) return;
    if (remainingUs > 2000) {
      delay(1);
    } else {
      yield();
    }
  }
}

void put16(uint8_t*& output, uint16_t value) {
  *output++ = static_cast<uint8_t>(value >> 8);
  *output++ = static_cast<uint8_t>(value);
}

void put32(uint8_t*& output, uint32_t value) {
  *output++ = static_cast<uint8_t>(value >> 24);
  *output++ = static_cast<uint8_t>(value >> 16);
  *output++ = static_cast<uint8_t>(value >> 8);
  *output++ = static_cast<uint8_t>(value);
}

uint16_t saturate16(uint32_t value) {
  return value > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(value);
}

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= static_cast<uint16_t>(*data++) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint8_t* beginPayload(uint8_t payload[kPayloadBytes], uint8_t type,
                      uint16_t sequence, uint32_t unixTime) {
  memset(payload, 0, kPayloadBytes);
  uint8_t* output = payload;
  *output++ = 'H';
  *output++ = 'T';
  *output++ = 1;
  *output++ = type;
  put16(output, sequence);
  put32(output, unixTime);
  return output;
}

void payloadToSymbols(uint8_t payload[kPayloadBytes],
                      uint8_t symbols[kFrameSymbols]) {
  const uint16_t checksum = crc16(payload, kPayloadBytes - 2);
  payload[kPayloadBytes - 2] = static_cast<uint8_t>(checksum >> 8);
  payload[kPayloadBytes - 1] = static_cast<uint8_t>(checksum);
  memcpy(symbols, kSync, sizeof(kSync));
  size_t symbol = sizeof(kSync);
  for (size_t byte = 0; byte < kPayloadBytes; ++byte) {
    for (int shift = 6; shift >= 0; shift -= 2) {
      symbols[symbol++] = (payload[byte] >> shift) & 0x03U;
    }
  }
  while (symbol < kFrameSymbols) {
    symbols[symbol] = kSync[(symbol - sizeof(kSync)) % sizeof(kSync)];
    ++symbol;
  }
}

void encodePosition(const PositionTelemetry& value, uint16_t sequence,
                    uint8_t symbols[kFrameSymbols]) {
  uint8_t payload[kPayloadBytes];
  uint8_t* output = beginPayload(payload, 1, sequence, value.unixTime);
  put32(output, static_cast<uint32_t>(value.latitudeMicrodegrees));
  put32(output, static_cast<uint32_t>(value.longitudeMicrodegrees));
  put16(output, static_cast<uint16_t>(value.altitudeMetres));
  *output++ = value.storedImages;
  put16(output, value.remainingPackets);
  *output++ = static_cast<uint8_t>(value.espTemperatureC);
  *output++ = value.satellites;
  *output++ = value.hdopTenths;
  put16(output, value.statusFlags);
  payloadToSymbols(payload, symbols);
}

void encodeDiagnostics(const MissionDiagnostics& value, uint32_t unixTime,
                       uint16_t sequence,
                       uint8_t symbols[kFrameSymbols]) {
  uint8_t payload[kPayloadBytes];
  uint8_t* output = beginPayload(payload, 2, sequence, unixTime);
  *output++ = value.lastResetReason;
  put16(output, saturate16(value.boots));
  put16(output, saturate16(value.resets));
  put16(output, saturate16(value.brownouts));
  put16(output, saturate16(value.watchdogs));
  put16(output, saturate16(value.failedJoins));
  put16(output, saturate16(value.storageRepairs));
  put16(output, saturate16(value.storageFaults));
  payloadToSymbols(payload, symbols);
}

bool Transmitter::configValid() const {
  return config_.callsign != nullptr && config_.grid != nullptr &&
         config_.baseFrequencyCentiHz >= 400000ULL &&
         config_.baseFrequencyCentiHz <= 22500000000ULL &&
         config_.powerDbm >= 0 && config_.powerDbm <= 60 &&
         config_.intervalMinutes != 0 && !(config_.intervalMinutes & 1U) &&
         config_.startWindowCentiseconds <= 99 &&
         config_.referenceFrequencyHz >= 10000000UL &&
         config_.referenceFrequencyHz <= 40000000UL &&
         strlen(config_.grid) == 4 && upperAlpha(config_.grid[0]) &&
         upperAlpha(config_.grid[1]) && digit(config_.grid[2]) &&
         digit(config_.grid[3]) && strlen(config_.callsign) > 0 &&
         strlen(config_.callsign) <= 11;
}

Result Transmitter::begin() {
  if (ready_) stop();
  ready_ = false;
  if (!configValid()) return Result::InvalidConfig;
  if (!busIdle()) return Result::I2cBusy;
  if (!Wire.setPins(I2C_SDA, I2C_SCL)) return Result::I2cBusy;
  if (!si5351_.init(SI5351_CRYSTAL_LOAD_8PF, config_.referenceFrequencyHz,
                    config_.correctionPpb)) {
    return Result::Si5351NotFound;
  }
  si5351_.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  si5351_.set_clock_disable(SI5351_CLK0, SI5351_CLK_DISABLE_LOW);
  si5351_.output_enable(SI5351_CLK0, 0);
  ready_ = true;
  return Result::Ok;
}

bool Transmitter::due(const UtcTime& utc) const {
  return ready_ && utc.minute < 60 && utc.second == 2 &&
         utc.centisecond <= config_.startWindowCentiseconds &&
         utc.minute % config_.intervalMinutes == 0;
}

Result Transmitter::transmit(const UtcTime& utc) {
  if (!ready_) return Result::Si5351NotFound;
  if (!due(utc)) return Result::NotDue;
  uint8_t symbols[kFrameSymbols];
  JTEncode encoder;
  encoder.wspr_encode(config_.callsign, config_.grid, config_.powerDbm,
                      symbols);
  return transmitSymbols(symbols, kFrameSymbols);
}

Result Transmitter::transmitSymbols(const uint8_t* symbols, size_t count) {
  if (!ready_) return Result::Si5351NotFound;
  if (symbols == nullptr || count != kFrameSymbols) return Result::InvalidConfig;
  if (!busIdle()) return Result::I2cBusy;

  const int64_t startedUs = esp_timer_get_time();
  Result result = Result::Ok;
  for (uint8_t symbol = 0; symbol < kFrameSymbols; ++symbol) {
    if (!busIdle()) {
      result = Result::I2cBusy;
      break;
    }
    if (symbols[symbol] > 3) {
      result = Result::InvalidConfig;
      break;
    }
    const uint64_t frequency = config_.baseFrequencyCentiHz +
        static_cast<uint64_t>(symbols[symbol]) * kToneSpacingCentiHz;
    if (si5351_.set_freq(frequency, SI5351_CLK0) != 0) {
      result = Result::SynthesizerError;
      break;
    }
    waitUntil(startedUs +
              static_cast<int64_t>((static_cast<uint64_t>(symbol) + 1U) *
                                   kSymbolNumeratorUs / kSymbolDenominator));
    if (service_ != nullptr) service_();
  }
  stop();
  return result;
}

void Transmitter::stop() {
  if (ready_) si5351_.output_enable(SI5351_CLK0, 0);
}

}  // namespace

struct MissionRadio::Impl {
  char grid[5] = {'A', 'A', '0', '0', '\0'};
  Config config;
  Transmitter transmitter;
  bool enabled = false;
  uint16_t sequence = 0;
  uint8_t framesSinceDiagnostic = 0;

  Impl(BusIdleCallback busIdle, ServiceCallback service)
      : config{HAB_WSPR_CALLSIGN,
               grid,
               HAB_WSPR_BASE_FREQUENCY_CENTIHZ,
               HAB_WSPR_POWER_DBM,
               2,
               25,
               HAB_SI5351_CORRECTION_PPB,
               HAB_SI5351_REFERENCE_HZ},
        transmitter(config, busIdle, service) {}

  bool updateGrid(float latitude, float longitude) {
    if (latitude < -90.0f || latitude >= 90.0f || longitude < -180.0f ||
        longitude >= 180.0f) {
      return false;
    }
    longitude += 180.0f;
    latitude += 90.0f;
    grid[0] = static_cast<char>('A' + static_cast<int>(longitude / 20.0f));
    grid[1] = static_cast<char>('A' + static_cast<int>(latitude / 10.0f));
    grid[2] = static_cast<char>('0' + static_cast<int>(longitude / 2.0f) % 10);
    grid[3] = static_cast<char>('0' + static_cast<int>(latitude) % 10);
    return true;
  }

  bool transmitStandard(const MissionData& data) {
    if (!enabled || !updateGrid(data.latitude, data.longitude)) return false;
    const UtcTime utc = {data.minute, data.second, data.centisecond};
    if (!transmitter.due(utc)) return false;
    Serial.printf("WSPR %s %s at %02u:%02u:%02u UTC\n", HAB_WSPR_CALLSIGN,
                  grid, data.hour, data.minute, data.second);
    const Result result = transmitter.transmit(utc);
    if (result != Result::Ok) {
      Serial.printf("WSPR transmission error %u\n",
                    static_cast<unsigned>(result));
    }
    return true;
  }

  bool transmitTelemetry(const MissionData& data) {
    if (!enabled || data.second != 2 || data.centisecond > 25) return false;
    uint8_t symbols[kFrameSymbols];
    if (++framesSinceDiagnostic >= kDiagnosticTelemetryPeriod) {
      encodeDiagnostics(missionDiagnostics(), data.unixTime, sequence++, symbols);
      framesSinceDiagnostic = 0;
      Serial.println("Custom diagnostic RF frame.");
    } else {
      PositionTelemetry position = {};
      position.unixTime = data.unixTime;
      position.latitudeMicrodegrees =
          static_cast<int32_t>(data.latitude * 1000000.0f);
      position.longitudeMicrodegrees =
          static_cast<int32_t>(data.longitude * 1000000.0f);
      position.altitudeMetres = static_cast<int16_t>(constrain(
          data.altitudeMetres, static_cast<float>(INT16_MIN),
          static_cast<float>(INT16_MAX)));
      position.storedImages = data.storedImages;
      position.remainingPackets = data.remainingImagePackets;
      position.espTemperatureC =
          static_cast<int8_t>(constrain(temperatureRead(), -128.0f, 127.0f));
      position.satellites = data.satellites;
      position.hdopTenths =
          static_cast<uint8_t>(constrain(data.hdop * 10.0f, 0.0f, 255.0f));
      position.statusFlags = data.statusFlags;
      encodePosition(position, sequence++, symbols);
      Serial.println("Custom position RF frame.");
    }
    const Result result = transmitter.transmitSymbols(symbols, kFrameSymbols);
    if (result != Result::Ok) {
      Serial.printf("Custom RF transmission error %u\n",
                    static_cast<unsigned>(result));
    }
    return true;
  }
};

MissionRadio::MissionRadio(BusIdleCallback busIdle, ServiceCallback service)
    : impl_(new Impl(busIdle, service)) {}

MissionRadio::~MissionRadio() { delete impl_; }

void MissionRadio::begin() {
  if (HAB_WSPR_AUTHORISED == 0) {
    Serial.println(
        "WSPR/custom RF inhibited: authorised calibrated settings absent.");
    return;
  }
  const Result result = impl_->transmitter.begin();
  impl_->enabled = result == Result::Ok;
  if (!impl_->enabled) {
    Serial.printf("WSPR disabled: initialisation error %u\n",
                  static_cast<unsigned>(result));
  }
}

bool MissionRadio::transmitIfDue(const MissionData& data,
                                 bool imageTransferMode) {
  if (!impl_->enabled || data.unixTime == 0) return false;
  if (imageTransferMode) {
    return data.hour % 2U == 0 && data.minute == 0 &&
           impl_->transmitStandard(data);
  }
  const uint8_t minuteInCycle = data.minute % 6U;
  if (minuteInCycle == 2U) return impl_->transmitStandard(data);
  if (minuteInCycle == 4U) return impl_->transmitTelemetry(data);
  return false;
}

}  // namespace wspr
