#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

#include "geofence.h"
#include "lorawan_config.h"
#include "pin_defs.h"
#include "radio.h"

namespace {
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
LoRaWANNode node(&radio, LORAWAN_REGION, LORAWAN_SUB_BAND);
bool lorawanReady = false;

void printRadioError(const char* action, int16_t state) {
  Serial.printf("[LoRaWAN] %s failed: %d\n", action, state);
}
}

bool initLoRaWAN() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);

  const int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    printRadioError("SX1262 init", state);
    return false;
  }

  // The E22's RX front-end is externally enabled; TX is selected by DIO2 on
  // this board. RadioLib drives RXEN automatically for Rx windows.
  radio.setRfSwitchPins(LORA_RXEN, RADIOLIB_NC);

  if (!lorawanCredentialsConfigured()) {
    Serial.println("[LoRaWAN] No OTAA credentials: uplinks disabled.");
    return false;
  }

  int16_t joinState = node.beginOTAA(LORAWAN_JOIN_EUI, LORAWAN_DEV_EUI,
                                     LORAWAN_NWK_KEY, LORAWAN_APP_KEY);
  if (joinState != RADIOLIB_ERR_NONE) {
    printRadioError("OTAA configuration", joinState);
    return false;
  }

  node.setADR(false);  // Fixed DR is required for the 210-byte packet budget.
  node.setDatarate(LORAWAN_DATA_RATE);
  joinState = node.activateOTAA();
  if (joinState != RADIOLIB_LORAWAN_NEW_SESSION &&
      joinState != RADIOLIB_LORAWAN_SESSION_RESTORED) {
    printRadioError("OTAA join", joinState);
    return false;
  }

  lorawanReady = true;
  Serial.println("[LoRaWAN] Joined and ready for Helium uplinks.");
  return true;
}

bool transmitHelium(const uint8_t* payload, size_t length) {
  if (GEOFENCE_no_tx || !lorawanReady) return false;

  const int16_t state = node.sendReceive(payload, length, LORAWAN_APP_PORT);
  if (state < RADIOLIB_ERR_NONE) {
    printRadioError("uplink", state);
    return false;
  }
  return true;
}
