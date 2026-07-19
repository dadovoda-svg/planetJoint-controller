#pragma once

#include <stdint.h>

// Motion state shared with the firmware runtime.
enum class MotionMode : uint8_t {
  IDLE,
  VELOCITY_TEST,
  STEP_TEST,
  POSITION,
  CALIBRATION,
  PARK,
  FAULT
};

struct JointMoveCommand {
  // Real joint degrees, relative to the logical zero.
  float targetDeg = 0.0f;
  float vmaxDegS = 0.0f;
  float amaxDegS2 = 0.0f;
  float sCurveTimeS = 0.0f; // <= 0 keeps the current controller value
  float outMaxDegS = 0.0f;  // <= 0 selects 1.25 * vmax
};

struct JointPlannerConfig {
  float minVmaxDegS = 0.0f;
  float maxVmaxDegS = 0.0f;
  float minAmaxDegS2 = 0.0f;
  float maxAmaxDegS2 = 0.0f;
  float blendMinRefVelDegS = 0.0f;
};

enum class JointMoveResult : uint8_t {
  Accepted,
  BlendAccepted,
  SafeReplan,
  Stopped,
  NotReferenced,
  CalibrationActive,
  FaultActive,
  EncoderUnavailable,
  InvalidCommand,
  InvalidLimits,
  DriverError
};

enum class JointTargetAdjustment : uint8_t {
  None,
  ClippedToMin,
  ClippedToMax
};

struct JointMoveOutcome {
  JointMoveResult result = JointMoveResult::InvalidCommand;
  JointTargetAdjustment targetAdjustment = JointTargetAdjustment::None;
  float requestedTargetDeg = 0.0f;
  float targetDeg = 0.0f;

  bool accepted() const {
    return result == JointMoveResult::Accepted ||
           result == JointMoveResult::BlendAccepted ||
           result == JointMoveResult::SafeReplan ||
           result == JointMoveResult::Stopped;
  }
};

const char* jointMoveResultName(JointMoveResult result);

class JointPlannerRuntime {
public:
  virtual ~JointPlannerRuntime() = default;

  virtual bool isReferenced() const = 0;
  virtual MotionMode motionMode() const = 0;
  virtual bool encoderReady() const = 0;
  virtual bool driverEnabled() const = 0;

  virtual void stopMotion() = 0;
  virtual bool ensureDriverEnabled() = 0;
  virtual void latchDriverFault() = 0;

  virtual bool clipTarget(float requestedDeg, float& clippedDeg) const = 0;
  virtual float currentPositionDeg() const = 0;
  virtual float controllerRefPositionDeg() const = 0;
  virtual float controllerRefVelocityDegS() const = 0;

  virtual void configureController(float vmaxDegS,
                                   float amaxDegS2,
                                   float sCurveTimeS,
                                   float outMaxDegS,
                                   bool clearFault) = 0;
  virtual void restartController(float currentDeg, float targetDeg) = 0;
  virtual void blendControllerTarget(float targetDeg) = 0;
  virtual void beginPositionMotion(float targetDeg) = 0;
};

class JointPlanner {
public:
  JointPlanner(JointPlannerRuntime& runtime, const JointPlannerConfig& config);

  JointMoveOutcome stop();
  JointMoveOutcome moveTo(const JointMoveCommand& cmd);
  JointMoveOutcome moveTo(float targetDeg, float vmaxDegS, float amaxDegS2);
  JointMoveOutcome moveTo(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS);

  JointMoveOutcome moveToBlended(const JointMoveCommand& cmd);
  JointMoveOutcome moveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2);
  JointMoveOutcome moveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS);

private:
  enum class RetargetMode : uint8_t { Restart, BlendIfSafe };

  struct PreparedMove {
    JointMoveCommand command;
    JointTargetAdjustment targetAdjustment = JointTargetAdjustment::None;
  };

  JointMoveOutcome executeMove(const JointMoveCommand& cmd, RetargetMode mode);
  JointMoveResult validateAndPrepare(const JointMoveCommand& cmd, PreparedMove& prepared) const;
  JointMoveOutcome makeOutcome(JointMoveResult result,
                               const JointMoveCommand& requested,
                               const PreparedMove* prepared = nullptr) const;

  JointPlannerRuntime& _runtime;
  JointPlannerConfig _config;
};
