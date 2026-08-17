#include "wspr_tx.h"

#include <JTEncode.h>
#include <Wire.h>
#include <esp_timer.h>
#include <si5351.h>

#include "pin_defs.h"

namespace wspr {
namespace {
constexpr uint8_t kSymbolCount = 162;
constexpr uint64_t kToneSpacingCentiHz = 146;  // 1.46 Hz, WSPR nominal spacing.
constexpr uint64_t kSymbolNumeratorUs = 8192ULL * 1000000ULL;
constexpr uint32_t kSymbolDenominator = 12000;

bool isUpperAlpha(const char c) { return c >= 'A' && c <= 'Z'; }
bool isDigit(const char c) { return c >= '0' && c <= '9'; }

void waitUntil(const int64_t deadline_us) {
  while (true) {
    const int64_t remaining_us = deadline_us - esp_timer_get_time();
    if (remaining_us <= 0) return;
    if (remaining_us > 2000) delay(1);
    else yield();
  }
}
}  // namespace

Transmitter::Transmitter(const Config& config, BusIdleCallback bus_idle)
    : config_(config), bus_idle_(bus_idle) {}

bool Transmitter::busIdle() const { return bus_idle_ == nullptr || bus_idle_(); }

bool Transmitter::configValid() const {
  if (config_.callsign == nullptr || config_.grid == nullptr ||
      config_.base_frequency_centi_hz < 400000ULL ||
      config_.base_frequency_centi_hz > 22500000000ULL ||
      config_.power_dbm < 0 || config_.power_dbm > 60 ||
      config_.tx_interval_minutes == 0 || (config_.tx_interval_minutes & 1U) ||
      config_.start_window_centiseconds > 99) return false;

  // Type-1 WSPR uses a four-character locator. Keep validation intentionally
  // strict so accidental dynamic GPS strings cannot be transmitted.
  return strlen(config_.grid) == 4 && isUpperAlpha(config_.grid[0]) &&
      isUpperAlpha(config_.grid[1]) && isDigit(config_.grid[2]) &&
      isDigit(config_.grid[3]) && strlen(config_.callsign) > 0 &&
      strlen(config_.callsign) <= 11;
}

Result Transmitter::begin() {
  stop();
  ready_ = false;
  if (!configValid()) return Result::InvalidConfig;
  if (!busIdle()) return Result::I2cBusy;

  // Si5351Arduino calls Wire.begin() internally. setPins() ensures that its
  // default begin uses this board's shared camera/Si5351 bus.
  if (!Wire.setPins(I2C_SDA, I2C_SCL)) return Result::I2cBusy;
  if (si5351_ == nullptr) si5351_ = new Si5351();
  if (!si5351_->init(SI5351_CRYSTAL_LOAD_8PF, 0, config_.si5351_correction_ppb)) {
    return Result::Si5351NotFound;
  }
  si5351_->drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  si5351_->set_clock_disable(SI5351_CLK0, SI5351_CLK_DISABLE_LOW);
  si5351_->output_enable(SI5351_CLK0, 0);
  ready_ = true;
  return Result::Ok;
}

bool Transmitter::due(const UtcTime& utc) const {
  return ready_ && utc.minute < 60 && utc.second == 2 &&
      utc.centisecond <= config_.start_window_centiseconds &&
      (utc.minute % config_.tx_interval_minutes) == 0;
}

Result Transmitter::transmitBlocking(const UtcTime& utc) {
  if (!ready_) return Result::Si5351NotFound;
  if (!due(utc)) return Result::NotDue;
  if (!busIdle()) return Result::I2cBusy;

  uint8_t symbols[kSymbolCount];
  JTEncode encoder;
  encoder.wspr_encode(config_.callsign, config_.grid, config_.power_dbm, symbols);

  // set_freq() takes centi-Hz. It enables CLK0 on the first update, so the
  // first programmed frequency is already a valid WSPR tone.
  const int64_t started_us = esp_timer_get_time();
  Result result = Result::Ok;
  for (uint8_t symbol = 0; symbol < kSymbolCount; ++symbol) {
    if (!busIdle()) { result = Result::I2cBusy; break; }
    const uint64_t frequency = config_.base_frequency_centi_hz +
        static_cast<uint64_t>(symbols[symbol]) * kToneSpacingCentiHz;
    if (si5351_->set_freq(frequency, SI5351_CLK0) != 0) {
      result = Result::SynthesizerError;
      break;
    }
    const int64_t next_deadline_us = started_us +
        static_cast<int64_t>((static_cast<uint64_t>(symbol) + 1U) *
                             kSymbolNumeratorUs / kSymbolDenominator);
    waitUntil(next_deadline_us);
  }
  stop();
  return result;
}

void Transmitter::stop() {
  if (si5351_ != nullptr) si5351_->output_enable(SI5351_CLK0, 0);
}

}  // namespace wspr
