#pragma once

#include <RadioLib.h>

// The balloon changes frequency plan only after a fresh GPS position enters
// one of the configured geofences. Use a different DevEUI for each plan so
// DevNonce and network-session state remain independent at the LNS.
constexpr uint8_t LORAWAN_EU_DATA_RATE = 5;
constexpr uint8_t LORAWAN_US_DATA_RATE = 4;
constexpr uint8_t LORAWAN_US_SUB_BAND = 2;
constexpr uint8_t LORAWAN_APP_PORT = 1;
constexpr unsigned long LORAWAN_JOIN_RETRY_MS = 15UL * 60UL * 1000UL;
// While image data is flowing, request explicit network evidence at this
// cadence. The LinkCheckReq is piggybacked on the next ordinary image uplink.
constexpr unsigned long LORAWAN_LINK_CHECK_INTERVAL_MS =
    10UL * 60UL * 1000UL;
// A missed periodic check freezes image progress immediately. Keep the active
// session for a small, bounded set of MAC-only probes so each new uplink can be
// heard by a different gateway before falling back to a fresh OTAA exchange.
constexpr uint8_t LORAWAN_LINK_CHECK_RECOVERY_ATTEMPTS = 3;
constexpr unsigned long LORAWAN_LINK_CHECK_RECOVERY_INTERVAL_MS =
    2UL * 60UL * 1000UL;
// These are provisional hardware-capable settings, not a declaration that
// they are lawful for the final antenna/route. HAB_LORAWAN_POLICY_REVIEWED
// remains the flight transmit gate until the regional review is complete.
constexpr int8_t LORAWAN_EU_TX_POWER_DBM = 14;
constexpr int8_t LORAWAN_US_TX_POWER_DBM = 20;
constexpr unsigned long LORAWAN_EU_MIN_UPLINK_INTERVAL_MS = 1000UL;
constexpr unsigned long LORAWAN_US_MIN_UPLINK_INTERVAL_MS = 1000UL;
constexpr uint32_t LORAWAN_US_DWELL_TIME_MS = 400UL;

// Set the corresponding *_LORAWAN_1_1 flag true only when that LNS device
// profile is explicitly LoRaWAN 1.1. Most hosted profiles are LoRaWAN 1.0.x.
#if __has_include("lorawan_secrets.h")
#include "lorawan_secrets.h"
#else
constexpr bool EU_LORAWAN_1_1 = false;
constexpr uint64_t EU_LORAWAN_JOIN_EUI = 0;
constexpr uint64_t EU_LORAWAN_DEV_EUI = 0;
constexpr uint8_t EU_LORAWAN_NWK_KEY[16] = {};
constexpr uint8_t EU_LORAWAN_APP_KEY[16] = {};

constexpr bool US_LORAWAN_1_1 = false;
constexpr uint64_t US_LORAWAN_JOIN_EUI = 0;
constexpr uint64_t US_LORAWAN_DEV_EUI = 0;
constexpr uint8_t US_LORAWAN_NWK_KEY[16] = {};
constexpr uint8_t US_LORAWAN_APP_KEY[16] = {};
#endif

inline bool keyConfigured(const uint8_t* key) {
  for (uint8_t i = 0; i < 16; ++i) {
    if (key[i] != 0) return true;
  }
  return false;
}

inline bool euLorawanCredentialsConfigured() {
  return EU_LORAWAN_DEV_EUI != 0 && keyConfigured(EU_LORAWAN_APP_KEY) &&
         (!EU_LORAWAN_1_1 || keyConfigured(EU_LORAWAN_NWK_KEY));
}

inline bool usLorawanCredentialsConfigured() {
  return US_LORAWAN_DEV_EUI != 0 && keyConfigured(US_LORAWAN_APP_KEY) &&
         (!US_LORAWAN_1_1 || keyConfigured(US_LORAWAN_NWK_KEY));
}
