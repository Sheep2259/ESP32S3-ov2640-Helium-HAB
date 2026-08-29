#include "wspr.h"

#include <JTEncode.h>
#include <Wire.h>
#include <esp_timer.h>
#include <si5351.h>

#include <cstring>

#include <helium_jpeg.h>

#include "mission_config.h"
#include "mission_diagnostics.h"
#include "pin_defs.h"
#include "wspr_telemetry.h"

namespace wspr {
namespace {
constexpr size_t kFrameSymbols = 162;
constexpr uint64_t kToneSpacingCentiHz = 146;
constexpr uint64_t kSymbolNumeratorUs = 8192ULL * 1000000ULL;
constexpr uint32_t kSymbolDenominator = 12000;
constexpr uint8_t kBasicSlot = 1;
constexpr uint8_t kMissionStateSlot = 2;
constexpr uint8_t kDiagnosticsSlot = 3;

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
  Result transmitMessage(const UtcTime& utc, const char* callsign,
                         const char* grid, int8_t powerDbm);
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

uint8_t compactFaultSummary(uint16_t flags) {
  return ((flags & helium_jpeg::STATUS_FLAG_CAMERA_ERROR) ? (1U << 0) : 0U) |
         ((flags & helium_jpeg::STATUS_FLAG_FS_ERROR) ? (1U << 1) : 0U) |
         ((flags & helium_jpeg::STATUS_FLAG_ENCODE_FAILED) ? (1U << 2) : 0U) |
         ((flags & helium_jpeg::STATUS_FLAG_UNEXPECTED_REBOOT) ? (1U << 3)
                                                               : 0U) |
         ((flags & helium_jpeg::STATUS_FLAG_LORA_TX_FAILURE) ? (1U << 4) : 0U);
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
  return transmitMessage(utc, config_.callsign, config_.grid,
                         config_.powerDbm);
}

Result Transmitter::transmitMessage(const UtcTime& utc, const char* callsign,
                                    const char* grid, int8_t powerDbm) {
  if (!ready_) return Result::Si5351NotFound;
  if (!due(utc)) return Result::NotDue;
  if (callsign == nullptr || grid == nullptr || powerDbm < 0 ||
      powerDbm > 60) {
    return Result::InvalidConfig;
  }
  uint8_t symbols[kFrameSymbols];
  JTEncode encoder;
  encoder.wspr_encode(callsign, grid, powerDbm, symbols);
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
    if (symbol == 0U) si5351_.output_enable(SI5351_CLK0, 1);
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
  char locator6[7] = {'A', 'A', '0', '0', 'A', 'A', '\0'};
  telemetry::ChannelDetails channel;
  Config config;
  Transmitter transmitter;
  bool enabled = false;
  uint32_t activeCycle = UINT32_MAX;
  uint8_t nextSlot = 0;

  Impl(BusIdleCallback busIdle, ServiceCallback service)
      : channel(telemetry::channel17m(HAB_WSPR_U4B_CHANNEL)),
        config{HAB_WSPR_CALLSIGN,
               grid,
               HAB_WSPR_BASE_FREQUENCY_CENTIHZ,
               HAB_WSPR_POWER_DBM,
               2,
               25,
               HAB_SI5351_CORRECTION_PPB,
               HAB_SI5351_REFERENCE_HZ},
        transmitter(config, busIdle, service) {}

  bool settingsValid() const {
    return channel.valid && telemetry::selfTest() &&
           static_cast<uint64_t>(channel.frequencyHz) * 100ULL ==
               HAB_WSPR_BASE_FREQUENCY_CENTIHZ &&
           HAB_U4B_VOLTAGE_CENTIVOLTS >= 300 &&
           HAB_U4B_VOLTAGE_CENTIVOLTS <= 495;
  }

  bool updateLocator(float latitude, float longitude) {
    if (!telemetry::maidenhead6(latitude, longitude, locator6)) {
      return false;
    }
    memcpy(grid, locator6, 4U);
    grid[4] = '\0';
    return true;
  }

  bool sendEncoded(const MissionData& data, const char* label,
                   const telemetry::Type1Message& message) {
    const UtcTime utc = {data.minute, data.second, data.centisecond};
    if (!transmitter.due(utc)) return false;
    Serial.printf("U4B %s: %s %s %u at %02u:%02u:%02u UTC\n", label,
                  message.callsign, message.grid, message.powerDbm, data.hour,
                  data.minute, data.second);
    const Result result = transmitter.transmitMessage(
        utc, message.callsign, message.grid, message.powerDbm);
    if (result != Result::Ok) {
      Serial.printf("U4B %s transmission error %u\n", label,
                    static_cast<unsigned>(result));
    }
    return true;
  }

  bool transmitStandard(const MissionData& data) {
    if (!enabled || !updateLocator(data.latitude, data.longitude)) return false;
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

  bool transmitBasic(const MissionData& data) {
    if (!enabled || !updateLocator(data.latitude, data.longitude)) return false;
    telemetry::Type1Message message = {};
    const int32_t temperatureC = static_cast<int32_t>(temperatureRead());
    if (!telemetry::encodeBasic(
            channel, locator6, static_cast<int32_t>(data.altitudeMetres),
            temperatureC, HAB_U4B_VOLTAGE_CENTIVOLTS,
            static_cast<double>(data.speedKmh) / 1.852, true, message)) {
      return false;
    }
    return sendEncoded(data, "basic", message);
  }

  bool transmitMissionState(const MissionData& data) {
    if (!enabled) return false;
    const uint64_t opaque = telemetry::packMissionState(
        data.storedImages, data.remainingImagePackets, data.satellites,
        data.hdop, compactFaultSummary(data.statusFlags));
    telemetry::Type1Message message = {};
    if (!telemetry::encodeCustom(channel, kMissionStateSlot, opaque,
                                 message)) {
      return false;
    }
    return sendEncoded(data, "CT mission", message);
  }

  bool transmitDiagnostics(const MissionData& data) {
    if (!enabled) return false;
    const MissionDiagnostics diagnostics = missionDiagnostics();
    const bool sendA = ((data.unixTime / 600UL) & 1U) == 0U;
    const uint64_t opaque =
        sendA ? telemetry::packDiagnosticsA(diagnostics.lastResetReason,
                                            diagnostics.boots,
                                            diagnostics.resets)
              : telemetry::packDiagnosticsB(
                    diagnostics.brownouts, diagnostics.watchdogs,
                    diagnostics.failedJoins, diagnostics.storageRepairs,
                    diagnostics.storageFaults);
    telemetry::Type1Message message = {};
    if (!telemetry::encodeCustom(channel, kDiagnosticsSlot, opaque, message)) {
      return false;
    }
    return sendEncoded(data, sendA ? "CT diagnostics A" : "CT diagnostics B",
                       message);
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
  if (!impl_->settingsValid()) {
    Serial.println(
        "WSPR disabled: U4B channel, frequency, voltage sentinel, or codec "
        "configuration is invalid.");
    return;
  }
  const Result result = impl_->transmitter.begin();
  impl_->enabled = result == Result::Ok;
  if (impl_->enabled) {
    Serial.printf("U4B channel %u, id %c%c, start minute %u, lane %u.\n",
                  impl_->channel.number, impl_->channel.id1, impl_->channel.id3,
                  impl_->channel.startMinute, impl_->channel.lane);
  }
  if (!impl_->enabled) {
    Serial.printf("WSPR disabled: initialisation error %u\n",
                  static_cast<unsigned>(result));
  }
}

bool MissionRadio::transmitIfDue(const MissionData& data,
                                 bool imageTransferMode) {
  if (!impl_->enabled || data.unixTime == 0) return false;
  if (imageTransferMode) {
    return data.hour % 2U == 0 && data.minute == impl_->channel.startMinute &&
           impl_->transmitStandard(data);
  }
  const uint8_t minuteInCycle = static_cast<uint8_t>(
      (data.minute + 10U - impl_->channel.startMinute) % 10U);
  const uint32_t absoluteMinute = data.unixTime / 60UL;
  const uint32_t cycle =
      (absoluteMinute + 10UL - impl_->channel.startMinute) / 10UL;
  if (minuteInCycle == 0U) {
    const bool sent = impl_->transmitStandard(data);
    if (sent) {
      impl_->activeCycle = cycle;
      impl_->nextSlot = kBasicSlot;
    }
    return sent;
  }
  // Do not emit orphan U4B telemetry after a mid-cycle boot or missed frame.
  if (impl_->activeCycle != cycle) return false;
  if (minuteInCycle == kBasicSlot * 2U && impl_->nextSlot == kBasicSlot) {
    const bool sent = impl_->transmitBasic(data);
    if (sent) impl_->nextSlot = kMissionStateSlot;
    return sent;
  }
  if (minuteInCycle == kMissionStateSlot * 2U) {
    if (impl_->nextSlot != kMissionStateSlot) return false;
    const bool sent = impl_->transmitMissionState(data);
    if (sent) impl_->nextSlot = kDiagnosticsSlot;
    return sent;
  }
  if (minuteInCycle == kDiagnosticsSlot * 2U) {
    if (impl_->nextSlot != kDiagnosticsSlot) return false;
    const bool sent = impl_->transmitDiagnostics(data);
    if (sent) impl_->nextSlot = kDiagnosticsSlot + 1U;
    return sent;
  }
  return false;
}

}  // namespace wspr
