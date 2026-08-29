#pragma once

#include <Arduino.h>

// Compile-time mission settings. The transmit gates default to the safe/off
// state. Override the macros with -D... build flags only after the associated
// RF settings, antenna, route, licence, and regional policy have been reviewed.

// TEMPORARY PASSIVE BENCH MODE. 1 periodically prints flight state and creates
// a non-destructive, regularly paced UART copy of packets from images captured
// by the normal mission scheduler. It also mirrors real LoRaWAN attempts. It
// does not trigger hardware, enable RF, or advance/delete the mission queue.
// Restore this to 0 after testing because packet dumps can delay the flight loop.
#ifndef HAB_SERIAL_PACKET_TEST_MODE
#define HAB_SERIAL_PACKET_TEST_MODE 1
#endif

// TEMPORARY CAMERA CONNECTION DIAGNOSTICS. 1 runs a read-only SCCB test once
// during setup: it power-cycles the camera, checks whether SDA/SCL are stuck,
// probes the OV2640 and Si5351 addresses, and reads the OV2640 identity at both
// 20 MHz and 16 MHz XCLK. It does not capture/store an image or enable RF.
// Restore this to 0 after the camera fault has been diagnosed.
#ifndef HAB_TEMP_CAMERA_DIAGNOSTICS
#define HAB_TEMP_CAMERA_DIAGNOSTICS 0
#endif

// 0: do not initialise the Si5351 and never produce WSPR/custom RF on CLK0.
// 1: allow Si5351 initialisation and scheduled RF frames, but only when the
// frequency below is valid and fresh GPS position/time are available.
#ifndef HAB_WSPR_AUTHORISED
#define HAB_WSPR_AUTHORISED 0
#endif

// Lowest WSPR tone in 0.01 Hz units; the other tones are up to 4.38 Hz higher.
// Zero is deliberately invalid, so authorising WSPR without also supplying a
// reviewed frequency still leaves this transmitter disabled at startup.
#ifndef HAB_WSPR_BASE_FREQUENCY_CENTIHZ
#define HAB_WSPR_BASE_FREQUENCY_CENTIHZ 1810606000ULL
#endif

// Power value announced inside a standard WSPR message. This does NOT adjust
// the physical RF output power; output level is determined by the Si5351 drive
// setting and external RF chain. Use a legal WSPR dBm value matching measured
// radiated power so receiving stations decode the correct report.
#ifndef HAB_WSPR_POWER_DBM
#define HAB_WSPR_POWER_DBM 10
#endif

// Numeric U4B channel used to identify and associate the regular, Basic, and
// Custom Telemetry WSPR spots. Channel 599 is Q9, starts at minute 0 on 17 m,
// and uses lane 4 at 18.106180 MHz. Check current channel activity before
// flight and change both this value and the base frequency when required.
#ifndef HAB_WSPR_U4B_CHANNEL
#define HAB_WSPR_U4B_CHANNEL 429
#endif

// U4B Basic Telemetry requires a voltage field. This board has no voltage
// monitor, so transmit the bottom-of-range 3.00 V sentinel. It must not be
// interpreted as a measurement.
#ifndef HAB_U4B_VOLTAGE_CENTIVOLTS
#define HAB_U4B_VOLTAGE_CENTIVOLTS 300
#endif

// Measured Si5351 frequency correction in parts per billion. Leaving this at
// zero applies no calibration; an inaccurate value shifts all generated tones.
#ifndef HAB_SI5351_CORRECTION_PPB
#define HAB_SI5351_CORRECTION_PPB 0
#endif

// 0: initialise the SX1262 only, then inhibit every LoRaWAN join and uplink.
// 1: permit GPS-geofenced regional OTAA joins and image uplinks when that
// region also has valid credentials. It does not bypass the GPS/geofence,
// credential, duty-cycle, dwell-time, or network-evidence checks.
#ifndef HAB_LORAWAN_POLICY_REVIEWED
#define HAB_LORAWAN_POLICY_REVIEWED 1
#endif

// Callsign encoded in standard WSPR frames; change it to the station legally
// responsible for transmissions before authorising WSPR.
constexpr char HAB_WSPR_CALLSIGN[] = "M7CWV";

// Nominal Si5351 crystal/reference frequency used to calculate every RF tone.
// This must match the fitted oscillator; correction is applied separately.
constexpr uint32_t HAB_SI5351_REFERENCE_HZ = 26000000UL;

// Minimum UTC interval between stored photos. A due capture additionally waits
// for a fresh GPS fix, free archival space, and the first two minutes of a
// six-minute mission cycle. With no prior image, the first eligible fix can
// trigger a capture immediately. 8UL * 60UL * 60UL;
constexpr uint32_t HAB_IMAGE_INTERVAL_SECONDS = 10UL * 60UL;

// Minimum delay before retrying after an eligible camera capture attempt that
// did not produce a stored image. Successful captures use the longer interval
// above instead.
constexpr uint32_t HAB_CAPTURE_RETRY_MS = 4UL * 60UL * 1000UL;

// How long a successful OTAA exchange/recent network response is accepted as
// proof of coverage. When it expires, the saved session is cleared and fresh
// OTAA evidence is required before further image packets are sent.
constexpr uint32_t HAB_NETWORK_EVIDENCE_TIMEOUT_MS = 2UL * 60UL * 60UL * 1000UL;

// Reset the ESP32 if the main task goes this many seconds without servicing the
// watchdog. The reset is recorded in persistent mission diagnostics.
constexpr uint32_t HAB_WATCHDOG_TIMEOUT_SECONDS = 180UL;

// Quiet time after UART0 starts and before any diagnostics or mission hardware
// initialisation. This gives a PC serial monitor time to open after reset.
constexpr uint32_t HAB_STARTUP_DELAY_MS = 5000UL;

// Temporary passive-test output cadence. One variable-length image packet is
// copied to UART per interval; flight state is printed at the slower interval.
constexpr uint32_t HAB_SERIAL_TEST_PACKET_INTERVAL_MS = 1000UL;
constexpr uint32_t HAB_SERIAL_TEST_STATUS_INTERVAL_MS = 5000UL;
