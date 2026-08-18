#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>

#include "geofence.h"
#include "lorawan_config.h"
#include "pin_defs.h"
#include "radio.h"

namespace {
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
LoRaWANNode euNode(&radio, &EU868, 0);
LoRaWANNode usNode(&radio, &US915, LORAWAN_US_SUB_BAND);
Preferences radioPrefs;

struct RegionRuntime {
  HeliumRegion region;
  const char* name;
  const char* nonceKey;
  LoRaWANNode* node;
  bool credentialsConfigured;
  bool initialised;
  bool firstJoinAttempt;
  unsigned long lastJoinAttemptMs;
  uint8_t dataRate;
};

RegionRuntime euRuntime = {HeliumRegion::Europe, "EU868", "eu_nonce", &euNode,
                           false, false, true, 0, LORAWAN_EU_DATA_RATE};
RegionRuntime usRuntime = {HeliumRegion::Americas, "US915", "us_nonce", &usNode,
                           false, false, true, 0, LORAWAN_US_DATA_RATE};

bool radioReady = false;
bool lorawanReady = false;
RegionRuntime* activeRuntime = nullptr;

void printRadioError(const char* region, const char* action, int16_t state) {
  Serial.printf("[LoRaWAN/%s] %s failed: %d\n", region, action, state);
}

void restoreNonces(RegionRuntime& runtime) {
  uint8_t buffer[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  if (radioPrefs.getBytesLength(runtime.nonceKey) != sizeof(buffer)) return;
  if (radioPrefs.getBytes(runtime.nonceKey, buffer, sizeof(buffer)) != sizeof(buffer)) return;
  const int16_t state = runtime.node->setBufferNonces(buffer);
  if (state != RADIOLIB_ERR_NONE) printRadioError(runtime.name, "nonce restore", state);
}

void persistNonces(RegionRuntime& runtime) {
  const uint8_t* buffer = runtime.node->getBufferNonces();
  if (buffer == nullptr ||
      radioPrefs.putBytes(runtime.nonceKey, buffer,
                          RADIOLIB_LORAWAN_NONCES_BUF_SIZE) !=
          RADIOLIB_LORAWAN_NONCES_BUF_SIZE) {
    Serial.printf("[LoRaWAN/%s] Failed to persist OTAA nonces.\n", runtime.name);
  }
}

void initialiseRuntime(RegionRuntime& runtime, uint64_t joinEui,
                       uint64_t devEui, const uint8_t* nwkKey,
                       const uint8_t* appKey, bool lorawan11) {
  if (!runtime.credentialsConfigured) return;
  const int16_t state = runtime.node->beginOTAA(
      joinEui, devEui, lorawan11 ? nwkKey : nullptr, appKey);
  if (state != RADIOLIB_ERR_NONE) {
    printRadioError(runtime.name, "OTAA configuration", state);
    runtime.credentialsConfigured = false;
    return;
  }
  restoreNonces(runtime);
  runtime.initialised = true;
}

RegionRuntime* runtimeFor(HeliumRegion region) {
  if (region == HeliumRegion::Europe) return &euRuntime;
  if (region == HeliumRegion::Americas) return &usRuntime;
  return nullptr;
}
}  // namespace

bool initLoRaWAN() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  const int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    printRadioError("radio", "SX1262 init", state);
    return false;
  }

  // The E22 RX front-end uses RXEN; DIO2 controls its TX path on this PCB.
  radio.setRfSwitchPins(LORA_RXEN, RADIOLIB_NC);
  if (!radioPrefs.begin("lorawan", false)) {
    Serial.println("[LoRaWAN] NVS unavailable; joins inhibited to prevent DevNonce reuse.");
    radioReady = true;
    return true;
  }

  euRuntime.credentialsConfigured = euLorawanCredentialsConfigured();
  usRuntime.credentialsConfigured = usLorawanCredentialsConfigured();
  initialiseRuntime(euRuntime, EU_LORAWAN_JOIN_EUI, EU_LORAWAN_DEV_EUI,
                    EU_LORAWAN_NWK_KEY, EU_LORAWAN_APP_KEY, EU_LORAWAN_1_1);
  initialiseRuntime(usRuntime, US_LORAWAN_JOIN_EUI, US_LORAWAN_DEV_EUI,
                    US_LORAWAN_NWK_KEY, US_LORAWAN_APP_KEY, US_LORAWAN_1_1);

  if (!euRuntime.initialised) Serial.println("[LoRaWAN/EU868] Credentials not configured.");
  if (!usRuntime.initialised) Serial.println("[LoRaWAN/US915] Credentials not configured.");
  radioReady = true;
  return true;
}

void serviceLoRaWAN(HeliumRegion region) {
  if (!radioReady || GEOFENCE_no_tx) {
    activeRuntime = nullptr;
    lorawanReady = false;
    return;
  }

  RegionRuntime* target = runtimeFor(region);
  if (target == nullptr || !target->initialised) {
    activeRuntime = target;
    lorawanReady = false;
    return;
  }

  if (activeRuntime != target) {
    activeRuntime = target;
    lorawanReady = target->node->isActivated();
    Serial.printf("[LoRaWAN] GPS selected %s.\n", target->name);
  }
  if (lorawanReady) return;

  const unsigned long now = millis();
  if (!target->firstJoinAttempt &&
      now - target->lastJoinAttemptMs < LORAWAN_JOIN_RETRY_MS) return;

  target->firstJoinAttempt = false;
  target->lastJoinAttemptMs = now;
  Serial.printf("[LoRaWAN/%s] Attempting OTAA join.\n", target->name);
  const int16_t state = target->node->activateOTAA();
  // DevNonce advances even when no JoinAccept arrives, so persist after every
  // transmitted attempt rather than only after success.
  persistNonces(*target);
  if (state != RADIOLIB_LORAWAN_NEW_SESSION && state != RADIOLIB_ERR_NONE) {
    printRadioError(target->name, "OTAA join", state);
    return;
  }

  target->node->setADR(false);
  const int16_t drState = target->node->setDatarate(target->dataRate);
  if (drState != RADIOLIB_ERR_NONE) {
    printRadioError(target->name, "data-rate selection", drState);
    return;
  }
  lorawanReady = true;
  Serial.printf("[LoRaWAN/%s] Joined and ready.\n", target->name);
}

bool lorawanCanTransmit() {
  return radioReady && lorawanReady && activeRuntime != nullptr &&
         activeRuntime->region == GEOFENCE_region && !GEOFENCE_no_tx;
}

bool transmitHelium(const uint8_t* payload, size_t length) {
  if (!lorawanCanTransmit() || payload == nullptr || length == 0) return false;
  const int16_t state =
      activeRuntime->node->sendReceive(payload, length, LORAWAN_APP_PORT);
  if (state < RADIOLIB_ERR_NONE) {
    printRadioError(activeRuntime->name, "uplink", state);
    return false;
  }
  return true;
}
