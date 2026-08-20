#include "mission_diagnostics.h"

#include <Preferences.h>
#include <esp_system.h>

namespace {
Preferences diagnosticPrefs;
MissionDiagnostics values = {};
bool ready = false;

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
}  // namespace

bool initialiseMissionDiagnostics() {
  if (!diagnosticPrefs.begin("mission", false)) return false;
  ready = true;
  values.boots = diagnosticPrefs.getUInt("boots", 0);
  values.resets = diagnosticPrefs.getUInt("resets", 0);
  values.brownouts = diagnosticPrefs.getUInt("brownouts", 0);
  values.watchdogs = diagnosticPrefs.getUInt("watchdogs", 0);
  values.failedJoins = diagnosticPrefs.getUInt("join_fail", 0);
  values.storageRepairs = diagnosticPrefs.getUInt("fs_repair", 0);
  values.storageFaults = diagnosticPrefs.getUInt("fs_fault", 0);

  const esp_reset_reason_t reason = esp_reset_reason();
  values.lastResetReason = static_cast<uint8_t>(reason);
  increment("boots", values.boots);
  if (reason != ESP_RST_POWERON && reason != ESP_RST_UNKNOWN) {
    increment("resets", values.resets);
  }
  if (reason == ESP_RST_BROWNOUT) increment("brownouts", values.brownouts);
  if (isWatchdogReset(reason)) increment("watchdogs", values.watchdogs);
  diagnosticPrefs.putUChar("last_reset", values.lastResetReason);
  return true;
}

MissionDiagnostics missionDiagnostics() { return values; }

void recordFailedJoin() { increment("join_fail", values.failedJoins); }
void recordStorageRepair() { increment("fs_repair", values.storageRepairs); }
void recordStorageFault() { increment("fs_fault", values.storageFaults); }

