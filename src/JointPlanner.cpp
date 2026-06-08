#include "JointPlanner.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "SCurvePosVelController.h"
#include "Tmc2209Driver.h"
#include "led_status.h"
#include "logger.h"

// The first refactoring step keeps hardware ownership in main.cpp and moves
// planner/move orchestration here. This keeps the tested bring-up sequence
// unchanged while making the planner independently extensible.
extern SCurvePosVelController jointCtrl;
extern Tmc2209Driver tmc;
extern JointPlanner planner;

extern bool encoderOk;
extern bool tmcReady;
extern uint32_t lastServoUs;
extern float servoTargetZeroedDeg;
extern MotionMode motionMode;

extern bool ensureDriverEnabled();
extern void stopMotion();
extern float jointGetPositionDeg();
extern float readParamMinOrDefault(const char* key, float fallback, float minValue);
extern void printServoStatus();
extern bool jointClipTargetToLimits(float requestedDeg, float& clippedDeg);

static constexpr float SERVO_VMAX_DEG_S       = 2.0f;
static constexpr float SERVO_AMAX_DEG_S2      = 6.0f;
static constexpr float SERVO_SCURVE_TIME_S    = 0.150f;
static constexpr float SERVO_OUTPUT_MAX_DEG_S = 2.5f;

static constexpr float PLANNER_HARD_VMAX_LIMIT_DEG_S = 12.0f;
static constexpr float PLANNER_HARD_AMAX_LIMIT_DEG_S2 = 100.0f;

JointPlanner::JointPlanner(HardwareSerial& plannerSerial)
: _serial(plannerSerial)
{
}

void JointPlanner::begin(uint32_t baud)
{
  _serial.begin(baud);
  resetInput();
  LOG_NFO("Planner serial initialized\r\n");
}

void JointPlanner::update()
{
  while (_serial.available() > 0) {
    processInputChar(static_cast<char>(_serial.read()));
  }
}

void JointPlanner::resetInput()
{
  _rxLen = 0;
  _rxLine[0] = '\0';
}

void JointPlanner::processInputChar(char c)
{
  // Accept Linux, Windows and old Mac line endings. Multiple terminators are harmless.
  if (c == '\n' || c == '\r') {
    if (_rxLen > 0) {
      _rxLine[_rxLen] = '\0';
      handleLine(_rxLine);
      resetInput();
    }
    return;
  }

  if (_rxLen >= sizeof(_rxLine) - 1) {
    _serial.println("ERR planner line too long");
    resetInput();
    return;
  }

  _rxLine[_rxLen++] = c;
}

bool JointPlanner::parseFloatToken(const char* token, float& out)
{
  if (token == nullptr || *token == '\0') {
    return false;
  }

  char* endPtr = nullptr;
  out = strtof(token, &endPtr);
  return (endPtr != token && *endPtr == '\0' && isfinite(out));
}

void JointPlanner::printHelp()
{
  _serial.println("OK planner commands:");
  _serial.println("  move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]");
  _serial.println("  moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]");
  _serial.println("  stop");
  _serial.println("  status");
  _serial.println("  help");
}

void JointPlanner::handleLine(char* line)
{
  char* argv[6] = {nullptr};
  int argc = 0;

  char* token = strtok(line, " \t");
  while (token != nullptr && argc < 6) {
    argv[argc++] = token;
    token = strtok(nullptr, " \t");
  }

  if (argc == 0) {
    return;
  }

  if (strcmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(argv[0], "stop") == 0) {
    _serial.println(stop() ? "OK stopped" : "ERR stop failed");
    return;
  }

  if (strcmp(argv[0], "status") == 0) {
    printServoStatus();
    _serial.println("OK status printed on USB console");
    return;
  }

  const bool blended = (strcmp(argv[0], "moveb") == 0);
  const bool normal = (strcmp(argv[0], "move") == 0);

  if (!normal && !blended) {
    _serial.println("ERR unknown planner command");
    return;
  }

  if (argc != 4 && argc != 5) {
    _serial.println(blended
      ? "ERR usage: moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]"
      : "ERR usage: move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]");
    return;
  }

  JointMoveCommand cmd;

  if (!parseFloatToken(argv[1], cmd.targetDeg) ||
      !parseFloatToken(argv[2], cmd.vmaxDegS) ||
      !parseFloatToken(argv[3], cmd.amaxDegS2)) {
    _serial.println("ERR invalid numeric argument");
    return;
  }

  if (argc == 5 && !parseFloatToken(argv[4], cmd.sCurveTimeS)) {
    _serial.println("ERR invalid sct_s");
    return;
  }

  const bool ok = blended ? moveToBlended(cmd) : moveTo(cmd);
  _serial.println(ok ? "OK move accepted" : "ERR move rejected");
}

bool JointPlanner::stop()
{
  stopMotion();
  return true;
}

bool JointPlanner::moveTo(const JointMoveCommand& cmd)
{
  if (motionMode == MotionMode::CALIBRATION) {
    Serial.println("[ERR] Calibration in progress");
    return false;
  }

  if (motionMode == MotionMode::FAULT) {
    Serial.println("[ERR] Motion fault latched; use stop/reboot after checking hardware");
    return false;
  }

  if (!encoderOk) {
    Serial.println("[ERR] Encoder not OK");
    return false;
  }

  if (!isfinite(cmd.targetDeg) || !isfinite(cmd.vmaxDegS) || !isfinite(cmd.amaxDegS2)) {
    Serial.println("[ERR] Invalid move command: non-finite value");
    return false;
  }

  if (cmd.vmaxDegS <= 0.0f || cmd.amaxDegS2 <= 0.0f) {
    Serial.println("[ERR] Invalid move command: vmax and amax must be > 0");
    return false;
  }

  const float safeVmax = constrain(fabsf(cmd.vmaxDegS), 0.01f, PLANNER_HARD_VMAX_LIMIT_DEG_S);
  const float safeAmax = constrain(fabsf(cmd.amaxDegS2), 0.01f, PLANNER_HARD_AMAX_LIMIT_DEG_S2);

  float clippedTargetDeg = cmd.targetDeg;
  if (!jointClipTargetToLimits(cmd.targetDeg, clippedTargetDeg)) {
    Serial.println("[ERR] Invalid joint limit configuration");
    return false;
  }

  // If another motion/test mode is active, stop it first.
  // If we are already IDLE but the driver is intentionally holding position,
  // do not briefly disable/re-enable the bridge before the next move.
  if (motionMode != MotionMode::IDLE) {
    stopMotion();
  }

  if (!ensureDriverEnabled()) {
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    return false;
  }

  jointCtrl.clearFault();
  jointCtrl.setLimits(safeVmax, safeAmax);

  if (isfinite(cmd.sCurveTimeS) && cmd.sCurveTimeS > 0.0f) {
    jointCtrl.setSCurveTime(cmd.sCurveTimeS);
  }

  if (isfinite(cmd.outMaxDegS) && cmd.outMaxDegS > 0.0f) {
    jointCtrl.setOutputMax(cmd.outMaxDegS);
  } else {
    jointCtrl.setOutputMax(safeVmax * 1.25f);
  }

  // IMPORTANT: controller coordinates are zeroed joint degrees.
  const float currentZeroedDeg = jointGetPositionDeg();
  servoTargetZeroedDeg = clippedTargetDeg;
  jointCtrl.reset(currentZeroedDeg);
  jointCtrl.setTarget(clippedTargetDeg);

  lastServoUs = micros();
  motionMode = MotionMode::POSITION;

  LOG_NFO("MOVE target=%.3f deg requested=%.3f deg current=%.3f deg vmax=%.3f deg/s amax=%.3f deg/s2\r\n",
                clippedTargetDeg, cmd.targetDeg, currentZeroedDeg, safeVmax, safeAmax);
  return true;
}

bool JointPlanner::moveTo(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetDeg;
  cmd.vmaxDegS = vmaxDegS;
  cmd.amaxDegS2 = amaxDegS2;
  return moveTo(cmd);
}

bool JointPlanner::moveTo(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetDeg;
  cmd.vmaxDegS = vmaxDegS;
  cmd.amaxDegS2 = amaxDegS2;
  cmd.sCurveTimeS = sCurveTimeS;
  return moveTo(cmd);
}

bool JointPlanner::moveToBlended(const JointMoveCommand& cmd)
{
  if (motionMode == MotionMode::CALIBRATION) {
    Serial.println("[ERR] Calibration in progress");
    return false;
  }

  if (motionMode == MotionMode::FAULT) {
    Serial.println("[ERR] Motion fault latched; use stop/reboot after checking hardware");
    return false;
  }

  if (!encoderOk) {
    Serial.println("[ERR] Encoder not OK");
    return false;
  }

  if (!isfinite(cmd.targetDeg) || !isfinite(cmd.vmaxDegS) || !isfinite(cmd.amaxDegS2)) {
    Serial.println("[ERR] Invalid blended move command: non-finite value");
    return false;
  }

  if (cmd.vmaxDegS <= 0.0f || cmd.amaxDegS2 <= 0.0f) {
    Serial.println("[ERR] Invalid blended move command: vmax and amax must be > 0");
    return false;
  }

  const float safeVmax = constrain(fabsf(cmd.vmaxDegS), 0.01f, PLANNER_HARD_VMAX_LIMIT_DEG_S);
  const float safeAmax = constrain(fabsf(cmd.amaxDegS2), 0.01f, PLANNER_HARD_AMAX_LIMIT_DEG_S2);

  float clippedTargetDeg = cmd.targetDeg;
  if (!jointClipTargetToLimits(cmd.targetDeg, clippedTargetDeg)) {
    Serial.println("[ERR] Invalid joint limit configuration");
    return false;
  }

  const bool alreadyPositioning = (motionMode == MotionMode::POSITION);
  const bool driverAlreadyEnabled = (tmc.status() == Tmc2209Driver::Status::Enabled);

  // If another manual/test mode is running, stop it and start the blended move
  // from the current zeroed position. If we are already in POSITION mode, do
  // NOT call stopMotion(), because it intentionally disables the driver and
  // resets the internal reference velocity. If we are IDLE but holding the
  // motor enabled at target, keep the bridge enabled for a clean restart.
  if (!alreadyPositioning && motionMode != MotionMode::IDLE) {
    stopMotion();
  }

  // If we are already positioning and the TMC is already enabled, this is a
  // pure re-target operation: do not arm the power stage again.
  if (!alreadyPositioning || !driverAlreadyEnabled) {
    if (!ensureDriverEnabled()) {
      motionMode = MotionMode::FAULT;
      wsSetState(LedState::FAULT);
      return false;
    }
  }

  if (!alreadyPositioning) {
    jointCtrl.clearFault();
  }

  jointCtrl.setLimits(safeVmax, safeAmax);

  if (isfinite(cmd.sCurveTimeS) && cmd.sCurveTimeS > 0.0f) {
    jointCtrl.setSCurveTime(cmd.sCurveTimeS);
  }

  if (isfinite(cmd.outMaxDegS) && cmd.outMaxDegS > 0.0f) {
    jointCtrl.setOutputMax(cmd.outMaxDegS);
  } else {
    jointCtrl.setOutputMax(safeVmax * 1.25f);
  }

  const float currentZeroedDeg = jointGetPositionDeg();
  servoTargetZeroedDeg = clippedTargetDeg;

  const float refPos = jointCtrl.refPos();
  const float refVel = jointCtrl.refVel();
  const float distToNewTarget = clippedTargetDeg - refPos;

  static constexpr float BLEND_MIN_REF_VEL_DEG_S = 0.02f;

  const bool movingReference = fabsf(refVel) > BLEND_MIN_REF_VEL_DEG_S;
  const bool targetAheadOfVelocity =
    !movingReference || ((distToNewTarget * refVel) >= 0.0f);

  bool fullBlend = false;

  if (alreadyPositioning && targetAheadOfVelocity) {
    // Good case: the new target is compatible with the current reference
    // velocity. Keep ref_pos/ref_vel/ref_acc and only update the target.
    jointCtrl.setTargetBlended(clippedTargetDeg);
    fullBlend = true;
  } else {
    // First segment, or reversal / ambiguous target.
    // Safe fallback: replan from the current measured zeroed position.
    // This intentionally drops the internal reference velocity to zero instead
    // of carrying a velocity that would push the joint further away.
    jointCtrl.reset(currentZeroedDeg);
    jointCtrl.setTarget(clippedTargetDeg);
    fullBlend = false;
  }

  lastServoUs = micros();
  motionMode = MotionMode::POSITION;

  LOG_NFO("MOVEB target=%.3f deg requested=%.3f deg current=%.3f deg vmax=%.3f deg/s amax=%.3f deg/s2 mode=%s\r\n",
                clippedTargetDeg,
                cmd.targetDeg,
                currentZeroedDeg,
                safeVmax,
                safeAmax,
                fullBlend ? "blend" : "safe-replan");
  return true;
}

bool JointPlanner::moveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetDeg;
  cmd.vmaxDegS = vmaxDegS;
  cmd.amaxDegS2 = amaxDegS2;
  return moveToBlended(cmd);
}

bool JointPlanner::moveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetDeg;
  cmd.vmaxDegS = vmaxDegS;
  cmd.amaxDegS2 = amaxDegS2;
  cmd.sCurveTimeS = sCurveTimeS;
  return moveToBlended(cmd);
}

bool moveJointToDeg(float targetZeroedDeg)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetZeroedDeg;
  cmd.vmaxDegS = readParamMinOrDefault("vmax", SERVO_VMAX_DEG_S, 0.001f);
  cmd.amaxDegS2 = readParamMinOrDefault("amax", SERVO_AMAX_DEG_S2, 0.001f);
  cmd.sCurveTimeS = readParamMinOrDefault("sct", SERVO_SCURVE_TIME_S, 0.001f);
  cmd.outMaxDegS = readParamMinOrDefault("outmax", SERVO_OUTPUT_MAX_DEG_S, 0.0f);
  return planner.moveTo(cmd);
}

bool jointMoveTo(const JointMoveCommand& cmd)
{
  return planner.moveTo(cmd);
}

bool jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  return planner.moveTo(targetDeg, vmaxDegS, amaxDegS2);
}

bool jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS)
{
  return planner.moveTo(targetDeg, vmaxDegS, amaxDegS2, sCurveTimeS);
}

bool jointMoveToBlended(const JointMoveCommand& cmd)
{
  return planner.moveToBlended(cmd);
}

bool jointMoveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  return planner.moveToBlended(targetDeg, vmaxDegS, amaxDegS2);
}

bool jointMoveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS)
{
  return planner.moveToBlended(targetDeg, vmaxDegS, amaxDegS2, sCurveTimeS);
}

bool jointStop()
{
  return planner.stop();
}
