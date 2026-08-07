#pragma once

#include <RadioLib.h>

// Helium Console must be configured for the same LoRaWAN region. UK flights
// normally use EU868; change this to the region used by your Helium device.
const LoRaWANBand_t* const LORAWAN_REGION = &EU868;
constexpr uint8_t LORAWAN_SUB_BAND = 0;  // US915/AU915 use the assigned 1-based sub-band.
constexpr uint8_t LORAWAN_DATA_RATE = 5; // EU868 DR5 supports the 210-byte HeliumJPEG packets.
constexpr uint8_t LORAWAN_APP_PORT = 1;

// Put real OTAA credentials in include/lorawan_secrets.h (which should not be
// committed). Copy the four declarations below exactly, with real values.
// Leaving these placeholders disables joining and keeps the firmware safe to
// build and flash before provisioning.
#if __has_include("lorawan_secrets.h")
#include "lorawan_secrets.h"
#else
constexpr uint64_t LORAWAN_JOIN_EUI = 0;
constexpr uint64_t LORAWAN_DEV_EUI = 0;
constexpr uint8_t LORAWAN_NWK_KEY[16] = {};
constexpr uint8_t LORAWAN_APP_KEY[16] = {};
#endif

inline bool lorawanCredentialsConfigured() {
    if (LORAWAN_DEV_EUI == 0) return false;
    for (uint8_t i = 0; i < 16; ++i) {
        if (LORAWAN_NWK_KEY[i] != 0 || LORAWAN_APP_KEY[i] != 0) return true;
    }
    return false;
}
