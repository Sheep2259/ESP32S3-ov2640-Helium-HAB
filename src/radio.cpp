#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>

#include "geofence.h"
#include "lorawan_config.h"
#include "mission_config.h"
#include "mission_diagnostics.h"
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
  int8_t txPowerDbm;
  unsigned long minimumUplinkIntervalMs;
  bool evidenceValid;
  unsigned long lastEvidenceMs;
  unsigned long lastLinkCheckMs;
  bool linkCheckSuspect;
  uint8_t recoveryProbeAttempts;
  unsigned long lastRecoveryProbeMs;
  bool uplinkSent;
  unsigned long lastUplinkMs;
};

RegionRuntime euRuntime = {HeliumRegion::Europe, "EU868", "eu_nonce", &euNode,
                           false, false, true, 0, LORAWAN_EU_DATA_RATE,
                           LORAWAN_EU_TX_POWER_DBM,
                           LORAWAN_EU_MIN_UPLINK_INTERVAL_MS,
                           false, 0, 0, false, 0, 0, false, 0};
RegionRuntime usRuntime = {HeliumRegion::Americas, "US915", "us_nonce", &usNode,
                           false, false, true, 0, LORAWAN_US_DATA_RATE,
                           LORAWAN_US_TX_POWER_DBM,
                           LORAWAN_US_MIN_UPLINK_INTERVAL_MS,
                           false, 0, 0, false, 0, 0, false, 0};

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

bool evidenceFresh(const RegionRuntime& runtime) {
  return runtime.evidenceValid &&
         millis() - runtime.lastEvidenceMs <= HAB_NETWORK_EVIDENCE_TIMEOUT_MS;
}

bool runtimeUplinkDue(const RegionRuntime& runtime) {
  if (runtime.uplinkSent &&
      millis() - runtime.lastUplinkMs < runtime.minimumUplinkIntervalMs) {
    return false;
  }
  return runtime.node->timeUntilUplink() == 0;
}

bool forceConfiguredUplinkDatarate(RegionRuntime& runtime,
                                   const char* purpose) {
  const int16_t state = runtime.node->setDatarate(runtime.dataRate);
  if (state == RADIOLIB_ERR_NONE) return true;
  printRadioError(runtime.name, purpose, state);
  return false;
}

void beginLinkCheckRecovery(RegionRuntime& runtime) {
  runtime.evidenceValid = false;
  if (runtime.linkCheckSuspect) return;
  runtime.linkCheckSuspect = true;
  runtime.recoveryProbeAttempts = 0;
  runtime.lastRecoveryProbeMs = millis();
  Serial.printf(
      "[LoRaWAN/%s] LinkCheck answer missed; retaining the image packet and "
      "starting %u bounded same-DR recovery probes.\n",
      runtime.name, LORAWAN_LINK_CHECK_RECOVERY_ATTEMPTS);
}

void completeLinkCheck(RegionRuntime& runtime, uint8_t marginDb,
                       uint8_t gatewayCount, uint8_t actualDataRate) {
  runtime.linkCheckSuspect = false;
  runtime.recoveryProbeAttempts = 0;
  runtime.lastLinkCheckMs = millis();
  runtime.evidenceValid = true;
  runtime.lastEvidenceMs = runtime.lastLinkCheckMs;
  Serial.printf(
      "[LoRaWAN/%s] LinkCheck passed at uplink DR%u: margin=%u dB "
      "gateways=%u.\n",
      runtime.name, actualDataRate, marginDb, gatewayCount);
}

void returnToCoverageDiscovery(RegionRuntime& runtime) {
  Serial.printf(
      "[LoRaWAN/%s] No LinkCheck answer after %u recovery probes; clearing "
      "the session and returning to OTAA discovery.\n",
      runtime.name, runtime.recoveryProbeAttempts);
  runtime.node->clearSession();
  runtime.evidenceValid = false;
  runtime.linkCheckSuspect = false;
  runtime.recoveryProbeAttempts = 0;
  runtime.firstJoinAttempt = true;
  lorawanReady = false;
}

void serviceLinkCheckRecovery(RegionRuntime& runtime, unsigned long now) {
  if (!runtime.linkCheckSuspect) return;
  if (!runtime.node->isActivated()) {
    Serial.printf(
        "[LoRaWAN/%s] Session became inactive during LinkCheck recovery; "
        "returning to OTAA discovery.\n",
        runtime.name);
    runtime.evidenceValid = false;
    runtime.linkCheckSuspect = false;
    runtime.recoveryProbeAttempts = 0;
    runtime.firstJoinAttempt = true;
    lorawanReady = false;
    return;
  }
  if (runtime.recoveryProbeAttempts >=
      LORAWAN_LINK_CHECK_RECOVERY_ATTEMPTS) {
    returnToCoverageDiscovery(runtime);
    return;
  }
  if (now - runtime.lastRecoveryProbeMs <
          LORAWAN_LINK_CHECK_RECOVERY_INTERVAL_MS ||
      !runtimeUplinkDue(runtime)) {
    return;
  }

  runtime.lastRecoveryProbeMs = now;
  ++runtime.recoveryProbeAttempts;
  if (!forceConfiguredUplinkDatarate(runtime,
                                     "recovery-probe data-rate selection")) {
    if (runtime.recoveryProbeAttempts >=
        LORAWAN_LINK_CHECK_RECOVERY_ATTEMPTS) {
      returnToCoverageDiscovery(runtime);
    }
    return;
  }

  const int16_t requestState = runtime.node->sendMacCommandReq(
      RADIOLIB_LORAWAN_MAC_LINK_CHECK);
  if (requestState != RADIOLIB_ERR_NONE) {
    printRadioError(runtime.name, "recovery LinkCheck request", requestState);
    if (runtime.recoveryProbeAttempts >=
        LORAWAN_LINK_CHECK_RECOVERY_ATTEMPTS) {
      returnToCoverageDiscovery(runtime);
    }
    return;
  }

  Serial.printf(
      "[LoRaWAN/%s] Recovery LinkCheck probe %u/%u requested at DR%u.\n",
      runtime.name, runtime.recoveryProbeAttempts,
      LORAWAN_LINK_CHECK_RECOVERY_ATTEMPTS, runtime.dataRate);
  LoRaWANEvent_t uplinkEvent = {};
  const int16_t state = runtime.node->sendReceive(
      nullptr, 0, RADIOLIB_LORAWAN_FPORT_MAC_COMMAND, false, &uplinkEvent);
  runtime.uplinkSent = true;
  runtime.lastUplinkMs = millis();
  if (state < RADIOLIB_ERR_NONE) {
    printRadioError(runtime.name, "recovery LinkCheck uplink", state);
  } else if (uplinkEvent.datarate != runtime.dataRate) {
    Serial.printf(
        "[LoRaWAN/%s] Recovery probe used unexpected DR%u instead of DR%u; "
        "not accepting it as coverage evidence.\n",
        runtime.name, uplinkEvent.datarate, runtime.dataRate);
  } else if (state > 0) {
    uint8_t marginDb = 0;
    uint8_t gatewayCount = 0;
    const int16_t answerState =
        runtime.node->getMacLinkCheckAns(&marginDb, &gatewayCount);
    if (answerState == RADIOLIB_ERR_NONE) {
      completeLinkCheck(runtime, marginDb, gatewayCount,
                        uplinkEvent.datarate);
      return;
    }
  }

  Serial.printf("[LoRaWAN/%s] Recovery probe %u received no LinkCheckAns.\n",
                runtime.name, runtime.recoveryProbeAttempts);
  if (runtime.recoveryProbeAttempts >=
      LORAWAN_LINK_CHECK_RECOVERY_ATTEMPTS) {
    returnToCoverageDiscovery(runtime);
  }
}

bool applyRegionalPolicy(RegionRuntime& runtime) {
  runtime.node->setADR(false);
  int16_t state = runtime.node->setDatarate(runtime.dataRate);
  if (state != RADIOLIB_ERR_NONE) {
    printRadioError(runtime.name, "data-rate selection", state);
    return false;
  }
  state = runtime.node->setTxPower(runtime.txPowerDbm);
  if (state != RADIOLIB_ERR_NONE) {
    printRadioError(runtime.name, "TX-power selection", state);
    return false;
  }
  if (runtime.region == HeliumRegion::Europe) {
    runtime.node->setDutyCycle(true);
    runtime.node->setDwellTime(false);
  } else {
    runtime.node->setDutyCycle(false);
    runtime.node->setDwellTime(true, LORAWAN_US_DWELL_TIME_MS);
  }
  return true;
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

#if !HAB_LORAWAN_POLICY_REVIEWED
  Serial.println("[LoRaWAN] RF policy is not reviewed; all joins/transmits are inhibited.");
  radioReady = true;
  return true;
#endif

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

  const unsigned long now = millis();
  if (lorawanReady && target->linkCheckSuspect) {
    serviceLinkCheckRecovery(*target, now);
    return;
  }
  if (lorawanReady && evidenceFresh(*target)) return;
  if (lorawanReady) {
    // An activated session alone is not evidence of present coverage.  Once
    // evidence expires, perform a fresh OTAA exchange at the normal join retry
    // cadence rather than continuing blind image uplinks.
    if (now - target->lastJoinAttemptMs < LORAWAN_JOIN_RETRY_MS) return;
    target->node->clearSession();
    lorawanReady = false;
    target->evidenceValid = false;
  }
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
    recordFailedJoin();
    return;
  }

  if (!applyRegionalPolicy(*target)) return;
  lorawanReady = true;
  target->evidenceValid = true;
  target->lastEvidenceMs = millis();
  target->linkCheckSuspect = false;
  target->recoveryProbeAttempts = 0;
  // A successful join is fresh bidirectional evidence, so the first periodic
  // LinkCheck is due one full interval after the JoinAccept.
  target->lastLinkCheckMs = target->lastEvidenceMs;
  Serial.printf("[LoRaWAN/%s] Joined and ready.\n", target->name);
}

bool lorawanCanTransmit() {
  return radioReady && lorawanReady && activeRuntime != nullptr &&
         activeRuntime->region == GEOFENCE_region && !GEOFENCE_no_tx;
}

bool lorawanNetworkReachable() {
  return lorawanCanTransmit() && !activeRuntime->linkCheckSuspect &&
         evidenceFresh(*activeRuntime);
}

bool lorawanUplinkDue() {
  if (!lorawanNetworkReachable()) return false;
  return runtimeUplinkDue(*activeRuntime);
}

bool transmitHelium(const uint8_t* payload, size_t length) {
  if (!lorawanUplinkDue() || payload == nullptr || length == 0) return false;

  if (!forceConfiguredUplinkDatarate(*activeRuntime,
                                     "uplink data-rate selection")) {
    return false;
  }

  const unsigned long now = millis();
  bool linkCheckRequested = false;
  const bool linkCheckDue =
      now - activeRuntime->lastLinkCheckMs >=
      LORAWAN_LINK_CHECK_INTERVAL_MS;

  // LinkCheckReq occupies one FOpts byte. Delay it by one packet if other MAC
  // commands leave no payload room; the current 210-byte image packets have
  // ample room at the configured EU DR5 and US DR4 limits.
  if (linkCheckDue &&
      length + 1U <= activeRuntime->node->getMaxPayloadLen()) {
    const int16_t requestState = activeRuntime->node->sendMacCommandReq(
        RADIOLIB_LORAWAN_MAC_LINK_CHECK);
    if (requestState == RADIOLIB_ERR_NONE) {
      linkCheckRequested = true;
      Serial.printf(
          "[LoRaWAN/%s] Requesting image LinkCheck checkpoint at DR%u.\n",
          activeRuntime->name, activeRuntime->dataRate);
    } else if (requestState == RADIOLIB_ERR_COMMAND_QUEUE_FULL) {
      Serial.printf(
          "[LoRaWAN/%s] LinkCheck delayed while queued MAC commands are sent.\n",
          activeRuntime->name);
    } else {
      printRadioError(activeRuntime->name, "LinkCheck request", requestState);
      beginLinkCheckRecovery(*activeRuntime);
      return false;
    }
  }

  if (length > activeRuntime->node->getMaxPayloadLen()) return false;
  LoRaWANEvent_t uplinkEvent = {};
  const int16_t state = activeRuntime->node->sendReceive(
      payload, length, LORAWAN_APP_PORT, false, &uplinkEvent);
  activeRuntime->uplinkSent = true;
  activeRuntime->lastUplinkMs = millis();
  if (state < RADIOLIB_ERR_NONE) {
    printRadioError(activeRuntime->name, "uplink", state);
    if (linkCheckRequested) beginLinkCheckRecovery(*activeRuntime);
    return false;
  }
  if (uplinkEvent.datarate != activeRuntime->dataRate) {
    Serial.printf(
        "[LoRaWAN/%s] Uplink used unexpected DR%u instead of DR%u; retaining "
        "the image packet.\n",
        activeRuntime->name, uplinkEvent.datarate, activeRuntime->dataRate);
    if (linkCheckRequested) beginLinkCheckRecovery(*activeRuntime);
    return false;
  }
  if (state > 0) {
    activeRuntime->evidenceValid = true;
    activeRuntime->lastEvidenceMs = millis();
  }

  if (linkCheckRequested) {
    uint8_t marginDb = 0;
    uint8_t gatewayCount = 0;
    const int16_t answerState =
        state > 0
            ? activeRuntime->node->getMacLinkCheckAns(&marginDb, &gatewayCount)
            : RADIOLIB_ERR_UNKNOWN;
    if (answerState != RADIOLIB_ERR_NONE) {
      beginLinkCheckRecovery(*activeRuntime);
      return false;
    }
    completeLinkCheck(*activeRuntime, marginDb, gatewayCount,
                      uplinkEvent.datarate);
  }
  return true;
}
