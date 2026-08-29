#include "JointPlanner.h"

#include <math.h>

namespace {

float clampMagnitude(float value, float minimum, float maximum)
{
  const float magnitude = fabsf(value);
  if (magnitude < minimum) return minimum;
  if (magnitude > maximum) return maximum;
  return magnitude;
}

} // namespace

const char* jointMoveResultName(JointMoveResult result)
{
  switch (result) {
    case JointMoveResult::Accepted: return "accepted";
    case JointMoveResult::BlendAccepted: return "blend accepted";
    case JointMoveResult::SafeReplan: return "safe replan";
    case JointMoveResult::Stopped: return "stopped";
    case JointMoveResult::NotReferenced: return "not referenced";
    case JointMoveResult::CalibrationActive: return "calibration active";
    case JointMoveResult::FaultActive: return "fault active";
    case JointMoveResult::EncoderUnavailable: return "encoder unavailable";
    case JointMoveResult::InvalidCommand: return "invalid command";
    case JointMoveResult::InvalidLimits: return "invalid limits";
    case JointMoveResult::DurationInfeasible: return "duration infeasible";
    case JointMoveResult::DriverError: return "driver error";
  }
  return "unknown";
}

JointPlanner::JointPlanner(JointPlannerRuntime& runtime, const JointPlannerConfig& config)
  : _runtime(runtime), _config(config)
{
}

JointMoveOutcome JointPlanner::stop()
{
  _runtime.stopMotion();
  JointMoveCommand empty;
  return makeOutcome(JointMoveResult::Stopped, empty);
}

JointMoveResult JointPlanner::validateAndPrepare(const JointMoveCommand& cmd,
                                                 PreparedMove& prepared) const
{
  if (!_runtime.isReferenced()) return JointMoveResult::NotReferenced;
  if (_runtime.motionMode() == MotionMode::CALIBRATION) return JointMoveResult::CalibrationActive;
  if (_runtime.motionMode() == MotionMode::FAULT) return JointMoveResult::FaultActive;
  if (!_runtime.encoderReady()) return JointMoveResult::EncoderUnavailable;

  if (!isfinite(cmd.targetDeg) || !isfinite(cmd.vmaxDegS) ||
      !isfinite(cmd.amaxDegS2) || !isfinite(cmd.outMaxDegS) ||
      cmd.vmaxDegS <= 0.0f || cmd.amaxDegS2 <= 0.0f) {
    return JointMoveResult::InvalidCommand;
  }

  if (!isfinite(_config.minVmaxDegS) || !isfinite(_config.maxVmaxDegS) ||
      !isfinite(_config.minAmaxDegS2) || !isfinite(_config.maxAmaxDegS2) ||
      !isfinite(_config.blendMinRefVelDegS) ||
      _config.minVmaxDegS <= 0.0f || _config.maxVmaxDegS < _config.minVmaxDegS ||
      _config.minAmaxDegS2 <= 0.0f || _config.maxAmaxDegS2 < _config.minAmaxDegS2 ||
      _config.blendMinRefVelDegS < 0.0f) {
    return JointMoveResult::InvalidLimits;
  }

  prepared.command = cmd;
  prepared.command.vmaxDegS = clampMagnitude(cmd.vmaxDegS,
                                              _config.minVmaxDegS,
                                              _config.maxVmaxDegS);
  prepared.command.amaxDegS2 = clampMagnitude(cmd.amaxDegS2,
                                               _config.minAmaxDegS2,
                                               _config.maxAmaxDegS2);
  prepared.command.outMaxDegS = cmd.outMaxDegS > 0.0f
    ? cmd.outMaxDegS
    : prepared.command.vmaxDegS * 1.25f;

  float clippedTargetDeg = cmd.targetDeg;
  if (!_runtime.clipTarget(cmd.targetDeg, clippedTargetDeg) || !isfinite(clippedTargetDeg)) {
    return JointMoveResult::InvalidLimits;
  }

  prepared.command.targetDeg = clippedTargetDeg;
  const float epsilon = 0.005f;
  if (clippedTargetDeg < cmd.targetDeg - epsilon) {
    prepared.targetAdjustment = JointTargetAdjustment::ClippedToMax;
  } else if (clippedTargetDeg > cmd.targetDeg + epsilon) {
    prepared.targetAdjustment = JointTargetAdjustment::ClippedToMin;
  }

  return JointMoveResult::Accepted;
}

JointMoveOutcome JointPlanner::makeOutcome(JointMoveResult result,
                                           const JointMoveCommand& requested,
                                           const PreparedMove* prepared) const
{
  JointMoveOutcome outcome;
  outcome.result = result;
  outcome.requestedTargetDeg = requested.targetDeg;
  outcome.targetDeg = prepared ? prepared->command.targetDeg : requested.targetDeg;
  outcome.targetAdjustment = prepared
    ? prepared->targetAdjustment
    : JointTargetAdjustment::None;
  return outcome;
}

JointMoveOutcome JointPlanner::executeMove(const JointMoveCommand& cmd,
                                           RetargetMode mode,
                                           float fixedDurationS)
{
  PreparedMove prepared;
  const JointMoveResult validation = validateAndPrepare(cmd, prepared);
  if (validation != JointMoveResult::Accepted) {
    return makeOutcome(validation, cmd);
  }

  const bool alreadyPositioning = _runtime.positionCommandActive();
  const bool blendRequested = mode == RetargetMode::BlendIfSafe;
  const bool fixedDuration = fixedDurationS > 0.0f;
  if ((fixedDuration && !isfinite(fixedDurationS)) ||
      (!fixedDuration && fixedDurationS != 0.0f)) {
    return makeOutcome(JointMoveResult::InvalidCommand, cmd, &prepared);
  }

  if (!blendRequested) {
    if (_runtime.motionMode() != MotionMode::IDLE) {
      _runtime.stopMotion();
    }
  } else if (!alreadyPositioning && _runtime.motionMode() != MotionMode::IDLE) {
    _runtime.stopMotion();
  }

  if (!alreadyPositioning || !_runtime.driverEnabled()) {
    if (!_runtime.ensureDriverEnabled()) {
      _runtime.latchDriverFault();
      return makeOutcome(JointMoveResult::DriverError, cmd, &prepared);
    }
  }

  _runtime.configureController(prepared.command.vmaxDegS,
                               prepared.command.amaxDegS2,
                               prepared.command.outMaxDegS,
                               !alreadyPositioning);

  const float currentDeg = _runtime.currentPositionDeg();
  bool fullBlend = false;

  if (blendRequested && alreadyPositioning) {
    const float refPos = _runtime.controllerRefPositionDeg();
    const float refVel = _runtime.controllerRefVelocityDegS();
    const float distance = prepared.command.targetDeg - refPos;
    const bool movingReference = fabsf(refVel) > _config.blendMinRefVelDegS;
    const bool targetAhead = !movingReference || distance * refVel >= 0.0f;

    const bool blended = targetAhead &&
      (fixedDuration
        ? _runtime.blendControllerTargetTimed(
            prepared.command.targetDeg, fixedDurationS)
        : _runtime.blendControllerTarget(prepared.command.targetDeg));
    if (blended) {
      fullBlend = true;
    } else {
      const bool restarted = !fixedDuration ||
        _runtime.restartControllerTimed(
          currentDeg, prepared.command.targetDeg, fixedDurationS);
      if (!restarted) {
        return makeOutcome(
          JointMoveResult::DurationInfeasible, cmd, &prepared);
      }
      if (!fixedDuration) {
        _runtime.restartController(currentDeg, prepared.command.targetDeg);
      }
    }
  } else {
    if (fixedDuration) {
      if (!_runtime.restartControllerTimed(
            currentDeg, prepared.command.targetDeg, fixedDurationS)) {
        return makeOutcome(
          JointMoveResult::DurationInfeasible, cmd, &prepared);
      }
    } else {
      _runtime.restartController(currentDeg, prepared.command.targetDeg);
    }
  }

  _runtime.beginPositionMotion(prepared.command.targetDeg);

  const JointMoveResult result = fullBlend
    ? JointMoveResult::BlendAccepted
    : (blendRequested ? JointMoveResult::SafeReplan : JointMoveResult::Accepted);

  return makeOutcome(result, cmd, &prepared);
}

JointMoveOutcome JointPlanner::moveTo(const JointMoveCommand& cmd)
{
  return executeMove(cmd, RetargetMode::Restart);
}

JointMoveOutcome JointPlanner::moveTo(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetDeg;
  cmd.vmaxDegS = vmaxDegS;
  cmd.amaxDegS2 = amaxDegS2;
  return moveTo(cmd);
}

JointMoveOutcome JointPlanner::moveToBlended(const JointMoveCommand& cmd)
{
  return executeMove(cmd, RetargetMode::BlendIfSafe);
}

JointMoveOutcome JointPlanner::moveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetDeg;
  cmd.vmaxDegS = vmaxDegS;
  cmd.amaxDegS2 = amaxDegS2;
  return moveToBlended(cmd);
}

JointMoveOutcome JointPlanner::moveToBlendedTimed(
    const JointMoveCommand& cmd,
    float durationS)
{
  return executeMove(cmd, RetargetMode::BlendIfSafe, durationS);
}

bool JointPlanner::minimumBlendedDuration(const JointMoveCommand& cmd,
                                          float& durationS) const
{
  PreparedMove prepared;
  if (validateAndPrepare(cmd, prepared) != JointMoveResult::Accepted) {
    return false;
  }
  return _runtime.minimumCoordinatedDuration(
    prepared.command.targetDeg,
    prepared.command.vmaxDegS,
    prepared.command.amaxDegS2,
    durationS);
}
