#pragma once

#include <cstddef>
#include <cstdint>

#include "geofence.h"

// Starts the SX1262. Joining is deferred until serviceLoRaWAN() receives a
// region selected from a fresh GPS position.
bool initLoRaWAN();
void serviceLoRaWAN(HeliumRegion region);
bool lorawanCanTransmit();
bool lorawanNetworkReachable();
bool lorawanUplinkDue();

// Sends one binary, ordinarily unconfirmed LoRaWAN uplink on the configured
// application port. Every ten minutes the next uplink carries a same-data-rate
// LinkCheckReq; false is returned unless that checkpoint receives LinkCheckAns,
// allowing the caller to retain the packet. A miss freezes image progress while
// bounded same-DR MAC-only probes reuse the session before OTAA discovery. The
// geofence remains the final transmit inhibit.
bool transmitHelium(const uint8_t* payload, size_t length);
