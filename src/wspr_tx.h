#pragma once

#include <Arduino.h>

class Si5351;

// WSPR is an amateur-radio transmission.  Select only a frequency, power,
// callsign and locator that you are authorised to use, and use a suitable
// low-pass filter / antenna matching network after the Si5351 output.
namespace wspr {

struct UtcTime {
  uint8_t minute;
  uint8_t second;
  uint8_t centisecond;
};

struct Config {
  Config(const char* callsign_value, const char* grid_value,
         uint64_t base_frequency_value, int8_t power_value,
         uint8_t interval_minutes = 60, uint8_t start_window = 25,
         int32_t correction_ppb = 0)
      : callsign(callsign_value),
        grid(grid_value),
        base_frequency_centi_hz(base_frequency_value),
        power_dbm(power_value),
        tx_interval_minutes(interval_minutes),
        start_window_centiseconds(start_window),
        si5351_correction_ppb(correction_ppb) {}

  const char* callsign;                 // Standard WSPR Type-1 callsign.
  const char* grid;                     // Four-character Maidenhead locator, e.g. IO91.
  uint64_t base_frequency_centi_hz;     // Lowest WSPR tone; no band default is assumed.
  int8_t power_dbm;                     // Declared RF power, 0..60 dBm.
  uint8_t tx_interval_minutes = 60;     // Must be even; 60 sends at hh:00:02.
  uint8_t start_window_centiseconds = 25;
  int32_t si5351_correction_ppb = 0;    // Calibrate against a known reference if possible.
};

enum class Result : uint8_t {
  Ok,
  NotDue,
  InvalidConfig,
  I2cBusy,
  Si5351NotFound,
  SynthesizerError,
};

// Return true only while no camera capture/SCCB transaction can occur.  The
// Si5351 is on the camera's shared GPIO 4/5 I2C bus on this board.
using BusIdleCallback = bool (*)();

class Transmitter {
 public:
  explicit Transmitter(const Config& config, BusIdleCallback bus_idle = nullptr);

  // Call once while the camera is idle.  It configures CLK0, but keeps it off.
  Result begin();

  // True at a valid WSPR boundary: hh:mm:02 where minute is an interval
  // multiple.  WSPR slots are always even UTC minutes.
  bool due(const UtcTime& utc) const;

  // Blocking 110.6 second transmission. Call immediately after due() is true;
  // it checks the bus before every I2C frequency update and always disables CLK0
  // before returning.
  Result transmitBlocking(const UtcTime& utc);

  // Emergency / shutdown operation. Safe to call repeatedly.
  void stop();
  bool ready() const { return ready_; }

 private:
  bool configValid() const;
  bool busIdle() const;

  Config config_;
  BusIdleCallback bus_idle_;
  ::Si5351* si5351_ = nullptr;
  bool ready_ = false;
};

}  // namespace wspr
