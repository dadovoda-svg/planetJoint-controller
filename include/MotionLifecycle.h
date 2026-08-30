#pragma once

#include <stdint.h>

enum class MotionLifecycleEvent : uint8_t {
  None = 0,
  CommandCompletedNoHold,
  CommandCompletedHold,
  ServoCorrectionStarted,
  ServoCorrectionCompleted
};

enum class PositionLifecycleExternalState : uint8_t {
  Moving = 0,
  Holding,
  Fault
};

struct MotionCommandExternalStatus {
  bool busy = false;
  bool done = true;
  bool fault = false;
};

struct MotionLifecycleState {
  bool positionCommandActive = false;
  bool servoHoldActive = false;
  bool forcedServoHold = false;
  bool servoCorrectionArmed = false;
  bool servoCorrectionActive = false;
  uint32_t lastServoCorrectionDebugMs = 0;

  void clear() {
    positionCommandActive = false;
    servoHoldActive = false;
    forcedServoHold = false;
    servoCorrectionArmed = false;
    servoCorrectionActive = false;
    lastServoCorrectionDebugMs = 0;
  }

  void beginPositionCommand() {
    positionCommandActive = true;
    servoHoldActive = false;
    forcedServoHold = false;
    servoCorrectionArmed = false;
    servoCorrectionActive = false;
    lastServoCorrectionDebugMs = 0;
  }

  void disableServoHold() {
    servoHoldActive = false;
    forcedServoHold = false;
    servoCorrectionArmed = false;
    servoCorrectionActive = false;
    lastServoCorrectionDebugMs = 0;
  }

  void beginForcedServoHold() {
    positionCommandActive = false;
    servoHoldActive = true;
    forcedServoHold = true;
    servoCorrectionArmed = true;
    servoCorrectionActive = false;
    lastServoCorrectionDebugMs = 0;
  }
};

inline bool positionMotionCommandBusy(const MotionLifecycleState& state)
{
  return state.positionCommandActive;
}

inline bool positionModeReportsMoving(const MotionLifecycleState& state)
{
  return state.positionCommandActive;
}

inline PositionLifecycleExternalState positionLifecycleExternalState(
    const MotionLifecycleState& state,
    bool fault)
{
  if (fault) {
    return PositionLifecycleExternalState::Fault;
  }

  return positionModeReportsMoving(state)
      ? PositionLifecycleExternalState::Moving
      : PositionLifecycleExternalState::Holding;
}

inline MotionCommandExternalStatus motionCommandExternalStatus(
    const MotionLifecycleState& state,
    bool otherOperationBusy,
    bool fault)
{
  MotionCommandExternalStatus status;
  status.busy =
      !fault && (positionMotionCommandBusy(state) || otherOperationBusy);
  status.done = !status.busy && !fault;
  status.fault = fault;
  return status;
}

inline MotionLifecycleEvent updateMotionLifecycle(
    MotionLifecycleState& state,
    bool settled,
    bool servoHoldEnabled,
    bool inDeadband)
{
  if (state.positionCommandActive && settled) {
    state.positionCommandActive = false;
    state.servoCorrectionActive = false;
    state.lastServoCorrectionDebugMs = 0;

    if (servoHoldEnabled) {
      state.servoHoldActive = true;
      state.servoCorrectionArmed = inDeadband;
      return MotionLifecycleEvent::CommandCompletedHold;
    }

    state.servoHoldActive = false;
    state.servoCorrectionArmed = false;
    return MotionLifecycleEvent::CommandCompletedNoHold;
  }

  if (state.positionCommandActive ||
      !state.servoHoldActive ||
      !servoHoldEnabled) {
    return MotionLifecycleEvent::None;
  }

  if (!state.servoCorrectionArmed) {
    if (inDeadband) {
      state.servoCorrectionArmed = true;
    }
    return MotionLifecycleEvent::None;
  }

  if (!state.servoCorrectionActive && !inDeadband) {
    state.servoCorrectionActive = true;
    state.lastServoCorrectionDebugMs = 0;
    return MotionLifecycleEvent::ServoCorrectionStarted;
  }

  if (state.servoCorrectionActive && inDeadband) {
    state.servoCorrectionActive = false;
    state.lastServoCorrectionDebugMs = 0;
    return MotionLifecycleEvent::ServoCorrectionCompleted;
  }

  return MotionLifecycleEvent::None;
}
