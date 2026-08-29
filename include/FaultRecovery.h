#pragma once

#include <math.h>
#include <stdint.h>

enum class FaultClearResult : uint8_t {
  Cleared = 0,
  AlreadyClear,
  EncoderUnavailable,
  DriverUnavailable,
  InvalidLimits,
  PositionOutsideLimits
};

struct FaultRecoverySnapshot {
  bool faultActive = false;
  bool encoderValid = false;
  bool driverValid = false;
  bool limitsValid = false;
  float positionDeg = 0.0f;
  float minDeg = 0.0f;
  float maxDeg = 0.0f;
};

inline FaultClearResult evaluateFaultRecovery(
    const FaultRecoverySnapshot& snapshot)
{
  if (!snapshot.encoderValid || !isfinite(snapshot.positionDeg)) {
    return FaultClearResult::EncoderUnavailable;
  }
  if (!snapshot.driverValid) {
    return FaultClearResult::DriverUnavailable;
  }
  if (!snapshot.limitsValid || !isfinite(snapshot.minDeg) ||
      !isfinite(snapshot.maxDeg) || snapshot.maxDeg <= snapshot.minDeg) {
    return FaultClearResult::InvalidLimits;
  }
  if (snapshot.positionDeg < snapshot.minDeg ||
      snapshot.positionDeg > snapshot.maxDeg) {
    return FaultClearResult::PositionOutsideLimits;
  }
  return snapshot.faultActive
      ? FaultClearResult::Cleared
      : FaultClearResult::AlreadyClear;
}

inline const char* faultClearResultName(FaultClearResult result)
{
  switch (result) {
    case FaultClearResult::Cleared: return "cleared";
    case FaultClearResult::AlreadyClear: return "already clear";
    case FaultClearResult::EncoderUnavailable: return "encoder unavailable";
    case FaultClearResult::DriverUnavailable: return "driver unavailable";
    case FaultClearResult::InvalidLimits: return "invalid joint limits";
    case FaultClearResult::PositionOutsideLimits:
      return "position outside joint limits";
  }
  return "unknown";
}
