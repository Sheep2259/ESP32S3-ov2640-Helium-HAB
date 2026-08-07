#pragma once

#include <cstddef>
#include <cstdint>

// Starts the SX1262 and joins the configured Helium LoRaWAN network.
// Returns false when credentials are not configured or a join fails.
bool initLoRaWAN();

// Sends one binary LoRaWAN uplink on the configured application port.
// The geofence remains the final transmit inhibit.
bool transmitHelium(const uint8_t* payload, size_t length);
