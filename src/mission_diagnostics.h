#pragma once

#include <Arduino.h>

struct MissionDiagnostics {
  uint32_t boots;
  uint32_t shortBoots;
  uint32_t brownouts;
  uint32_t watchdogs;
  uint32_t consecutiveFailedJoins;
  uint32_t wsprFailures;
  uint32_t storageRepairs;
  uint32_t storageFaults;
  uint8_t lastResetReason;
  bool abnormalReset;
  bool storageFaultThisBoot;
};

bool initialiseMissionDiagnostics();
void serviceMissionDiagnostics(uint32_t uptimeSeconds);
MissionDiagnostics missionDiagnostics();
void recordFailedJoin();
void recordSuccessfulJoin();
void recordWsprFailure();
void recordStorageRepair();
void recordStorageFault();
