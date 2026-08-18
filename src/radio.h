#pragma once

#include <cstddef>
#include <cstdint>

#include "geofence.h"

// Starts the SX1262. Joining is deferred until serviceLoRaWAN() receives a
// region selected from a fresh GPS position.
bool initLoRaWAN();
void serviceLoRaWAN(HeliumRegion region);
bool lorawanCanTransmit();

// Sends one binary LoRaWAN uplink on the configured application port.
// The geofence remains the final transmit inhibit.
bool transmitHelium(const uint8_t* payload, size_t length);
