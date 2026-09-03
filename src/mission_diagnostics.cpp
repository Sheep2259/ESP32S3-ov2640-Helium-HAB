#include "mission_diagnostics.h"

#include <Preferences.h>
#include <esp_system.h>

#include "mission_config.h"

namespace {
Preferences diagnosticPrefs;
MissionDiagnostics values = {};
bool ready = false;
bool healthyMarkerWritten = false;

void persist(const char* key, uint32_t value) {
  if (ready && diagnosticPrefs.putUInt(key, value) != sizeof(value)) {
    Serial.printf("[diagnostics] Failed to persist %s.\n", key);
  }
}

uint32_t increment(const char* key, uint32_t& value) {
  if (value != UINT32_MAX) ++value;
  persist(key, value);
  return value;
}

bool isWatchdogReset(esp_reset_reason_t reason) {
  return reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT ||
         reason == ESP_RST_WDT;
}

bool isAbnormalReset(esp_reset_reason_t reason) {
  // Power loss and brownout are normal lifecycle events for the solar supply.
  // A software restart is abnormal because this firmware never requests one.
  return reason == ESP_RST_SW || reason == ESP_RST_PANIC ||
         isWatchdogReset(reason);
}
}  // namespace

bool initialiseMissionDiagnostics() {
  if (!diagnosticPrefs.begin("mission", false)) return false;
  ready = true;
  values.boots = diagnosticPrefs.getUInt("boots", 0);
  values.shortBoots = diagnosticPrefs.getUInt("short_boot", 0);
  values.brownouts = diagnosticPrefs.getUInt("brownouts", 0);
  values.watchdogs = diagnosticPrefs.getUInt("watchdogs", 0);
  values.consecutiveFailedJoins = diagnosticPrefs.getUInt("join_streak", 0);
  values.wsprFailures = diagnosticPrefs.getUInt("wspr_fail", 0);
  values.storageRepairs = diagnosticPrefs.getUInt("fs_repair", 0);
  values.storageFaults = diagnosticPrefs.getUInt("fs_fault", 0);

  // Missing marker means an upgrade or first boot, not a short prior boot.
  if (values.boots != 0 &&
      diagnosticPrefs.getBool("boot_healthy", true) == false) {
    increment("short_boot", values.shortBoots);
  }
  healthyMarkerWritten =
      diagnosticPrefs.putBool("boot_healthy", false) == sizeof(uint8_t);

  const esp_reset_reason_t reason = esp_reset_reason();
  values.lastResetReason = static_cast<uint8_t>(reason);
  values.abnormalReset = isAbnormalReset(reason);
  values.storageFaultThisBoot = false;
  increment("boots", values.boots);
  if (reason == ESP_RST_BROWNOUT) increment("brownouts", values.brownouts);
  if (isWatchdogReset(reason)) increment("watchdogs", values.watchdogs);
  diagnosticPrefs.putUChar("last_reset", values.lastResetReason);
  return true;
}

void serviceMissionDiagnostics(uint32_t uptimeSeconds) {
  if (!ready || !healthyMarkerWritten ||
      uptimeSeconds < HAB_HEALTHY_BOOT_SECONDS) {
    return;
  }
  // At most one additional NVS write is performed on a healthy boot.
  healthyMarkerWritten = false;
  if (diagnosticPrefs.putBool("boot_healthy", true) != sizeof(uint8_t)) {
    Serial.println("[diagnostics] Failed to mark boot healthy.");
  }
}

MissionDiagnostics missionDiagnostics() { return values; }

void recordFailedJoin() {
  increment("join_streak", values.consecutiveFailedJoins);
}

void recordSuccessfulJoin() {
  if (values.consecutiveFailedJoins == 0) return;
  values.consecutiveFailedJoins = 0;
  persist("join_streak", 0);
}

void recordWsprFailure() { increment("wspr_fail", values.wsprFailures); }
void recordStorageRepair() { increment("fs_repair", values.storageRepairs); }
void recordStorageFault() {
  values.storageFaultThisBoot = true;
  increment("fs_fault", values.storageFaults);
}
