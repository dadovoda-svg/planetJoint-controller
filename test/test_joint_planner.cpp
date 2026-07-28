#include "JointPlanner.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); \
    ++failures; \
  } \
} while (0)

class FakeRuntime final : public JointPlannerRuntime {
public:
  bool referenced = true;
  MotionMode mode = MotionMode::IDLE;
  bool encoderOk = true;
  bool driverIsEnabled = false;
  bool enableSucceeds = true;
  bool limitsValid = true;
  float minTarget = -170.0f;
  float maxTarget = 170.0f;
  float position = 0.0f;
  float refPosition = 0.0f;
  float refVelocity = 0.0f;
  bool blendSucceeds = true;

  int stopCalls = 0;
  int enableCalls = 0;
  int faultCalls = 0;
  int configureCalls = 0;
  int restartCalls = 0;
  int blendCalls = 0;
  int beginCalls = 0;
  bool lastClearFault = false;
  float configuredVmax = 0.0f;
  float configuredAmax = 0.0f;
  float configuredOutMax = 0.0f;
  float startedTarget = 0.0f;

  bool isReferenced() const override { return referenced; }
  MotionMode motionMode() const override { return mode; }
  bool encoderReady() const override { return encoderOk; }
  bool driverEnabled() const override { return driverIsEnabled; }

  void stopMotion() override {
    ++stopCalls;
    mode = MotionMode::IDLE;
    driverIsEnabled = false;
  }

  bool ensureDriverEnabled() override {
    ++enableCalls;
    if (enableSucceeds) driverIsEnabled = true;
    return enableSucceeds;
  }

  void latchDriverFault() override {
    ++faultCalls;
    mode = MotionMode::FAULT;
  }

  bool clipTarget(float requestedDeg, float& clippedDeg) const override {
    if (!limitsValid) return false;
    clippedDeg = requestedDeg < minTarget ? minTarget
               : requestedDeg > maxTarget ? maxTarget
               : requestedDeg;
    return true;
  }

  float currentPositionDeg() const override { return position; }
  float controllerRefPositionDeg() const override { return refPosition; }
  float controllerRefVelocityDegS() const override { return refVelocity; }

  void configureController(float vmaxDegS,
                           float amaxDegS2,
                           float outMaxDegS,
                           bool clearFault) override {
    ++configureCalls;
    configuredVmax = vmaxDegS;
    configuredAmax = amaxDegS2;
    configuredOutMax = outMaxDegS;
    lastClearFault = clearFault;
  }

  void restartController(float currentDeg, float targetDeg) override {
    ++restartCalls;
    position = currentDeg;
    refPosition = currentDeg;
    startedTarget = targetDeg;
    refVelocity = 0.0f;
  }

  bool blendControllerTarget(float targetDeg) override {
    ++blendCalls;
    if (blendSucceeds) {
      startedTarget = targetDeg;
    }
    return blendSucceeds;
  }

  void beginPositionMotion(float targetDeg) override {
    ++beginCalls;
    startedTarget = targetDeg;
    mode = MotionMode::POSITION;
  }
};

JointPlannerConfig testConfig()
{
  JointPlannerConfig config;
  config.minVmaxDegS = 0.01f;
  config.maxVmaxDegS = 240.0f;
  config.minAmaxDegS2 = 0.01f;
  config.maxAmaxDegS2 = 650.0f;
  config.blendMinRefVelDegS = 0.02f;
  return config;
}

JointMoveCommand command(float target = 20.0f, float vmax = 10.0f, float amax = 30.0f)
{
  JointMoveCommand cmd;
  cmd.targetDeg = target;
  cmd.vmaxDegS = vmax;
  cmd.amaxDegS2 = amax;
  return cmd;
}

void testInvalidNumbersAndZeroLimits()
{
  FakeRuntime runtime;
  JointPlanner planner(runtime, testConfig());

  JointMoveCommand cmd = command();
  cmd.targetDeg = std::numeric_limits<float>::quiet_NaN();
  CHECK(planner.moveTo(cmd).result == JointMoveResult::InvalidCommand);

  cmd = command();
  cmd.vmaxDegS = std::numeric_limits<float>::infinity();
  CHECK(planner.moveTo(cmd).result == JointMoveResult::InvalidCommand);

  cmd = command();
  cmd.vmaxDegS = 0.0f;
  CHECK(planner.moveTo(cmd).result == JointMoveResult::InvalidCommand);
  CHECK(runtime.enableCalls == 0);
}

void testStateRejections()
{
  FakeRuntime runtime;
  JointPlanner planner(runtime, testConfig());

  runtime.referenced = false;
  CHECK(planner.moveTo(command()).result == JointMoveResult::NotReferenced);

  runtime.referenced = true;
  runtime.mode = MotionMode::CALIBRATION;
  CHECK(planner.moveTo(command()).result == JointMoveResult::CalibrationActive);

  runtime.mode = MotionMode::FAULT;
  CHECK(planner.moveTo(command()).result == JointMoveResult::FaultActive);

  runtime.mode = MotionMode::IDLE;
  runtime.encoderOk = false;
  CHECK(planner.moveTo(command()).result == JointMoveResult::EncoderUnavailable);
}

void testClippingAndSafetyClamps()
{
  FakeRuntime runtime;
  JointPlanner planner(runtime, testConfig());
  JointMoveCommand cmd = command(200.0f, 500.0f, 900.0f);

  const JointMoveOutcome outcome = planner.moveTo(cmd);
  CHECK(outcome.result == JointMoveResult::Accepted);
  CHECK(outcome.targetAdjustment == JointTargetAdjustment::ClippedToMax);
  CHECK(std::fabs(outcome.targetDeg - 170.0f) < 0.001f);
  CHECK(std::fabs(runtime.configuredVmax - 240.0f) < 0.001f);
  CHECK(std::fabs(runtime.configuredAmax - 650.0f) < 0.001f);
  CHECK(std::fabs(runtime.configuredOutMax - 300.0f) < 0.001f);
  CHECK(runtime.restartCalls == 1);
  CHECK(runtime.beginCalls == 1);

  FakeRuntime badLimits;
  badLimits.limitsValid = false;
  JointPlanner badPlanner(badLimits, testConfig());
  CHECK(badPlanner.moveTo(command()).result == JointMoveResult::InvalidLimits);

  JointPlannerConfig invalidConfig = testConfig();
  invalidConfig.maxVmaxDegS = 0.0f;
  FakeRuntime invalidConfigRuntime;
  JointPlanner invalidConfigPlanner(invalidConfigRuntime, invalidConfig);
  CHECK(invalidConfigPlanner.moveTo(command()).result == JointMoveResult::InvalidLimits);
}

void testBlendAheadKeepsReference()
{
  FakeRuntime runtime;
  runtime.mode = MotionMode::POSITION;
  runtime.driverIsEnabled = true;
  runtime.position = 4.0f;
  runtime.refPosition = 5.0f;
  runtime.refVelocity = 2.0f;
  JointPlanner planner(runtime, testConfig());

  const JointMoveOutcome outcome = planner.moveToBlended(command(20.0f));
  CHECK(outcome.result == JointMoveResult::BlendAccepted);
  CHECK(runtime.blendCalls == 1);
  CHECK(runtime.restartCalls == 0);
  CHECK(runtime.enableCalls == 0);
  CHECK(!runtime.lastClearFault);
}

void testBlendReverseUsesSafeReplan()
{
  FakeRuntime runtime;
  runtime.mode = MotionMode::POSITION;
  runtime.driverIsEnabled = true;
  runtime.position = 4.0f;
  runtime.refPosition = 5.0f;
  runtime.refVelocity = 2.0f;
  JointPlanner planner(runtime, testConfig());

  const JointMoveOutcome outcome = planner.moveToBlended(command(-20.0f));
  CHECK(outcome.result == JointMoveResult::SafeReplan);
  CHECK(runtime.blendCalls == 0);
  CHECK(runtime.restartCalls == 1);
  CHECK(std::fabs(runtime.refPosition - 4.0f) < 0.001f);
}

void testRejectedBlendUsesSafeReplan()
{
  FakeRuntime runtime;
  runtime.mode = MotionMode::POSITION;
  runtime.driverIsEnabled = true;
  runtime.position = 4.0f;
  runtime.refPosition = 5.0f;
  runtime.refVelocity = 2.0f;
  runtime.blendSucceeds = false;
  JointPlanner planner(runtime, testConfig());

  const JointMoveOutcome outcome = planner.moveToBlended(command(20.0f));
  CHECK(outcome.result == JointMoveResult::SafeReplan);
  CHECK(runtime.blendCalls == 1);
  CHECK(runtime.restartCalls == 1);
  CHECK(std::fabs(runtime.refPosition - 4.0f) < 0.001f);
}

void testDriverFailureLatchesFault()
{
  FakeRuntime runtime;
  runtime.enableSucceeds = false;
  JointPlanner planner(runtime, testConfig());

  const JointMoveOutcome outcome = planner.moveTo(command());
  CHECK(outcome.result == JointMoveResult::DriverError);
  CHECK(runtime.enableCalls == 1);
  CHECK(runtime.faultCalls == 1);
  CHECK(runtime.configureCalls == 0);
  CHECK(runtime.beginCalls == 0);
}

void testStop()
{
  FakeRuntime runtime;
  JointPlanner planner(runtime, testConfig());
  const JointMoveOutcome outcome = planner.stop();
  CHECK(outcome.result == JointMoveResult::Stopped);
  CHECK(outcome.accepted());
  CHECK(runtime.stopCalls == 1);
}

} // namespace

int main()
{
  testInvalidNumbersAndZeroLimits();
  testStateRejections();
  testClippingAndSafetyClamps();
  testBlendAheadKeepsReference();
  testBlendReverseUsesSafeReplan();
  testRejectedBlendUsesSafeReplan();
  testDriverFailureLatchesFault();
  testStop();

  if (failures != 0) {
    std::printf("joint planner tests failed: %d\n", failures);
    return 1;
  }

  std::printf("joint planner tests passed\n");
  return 0;
}
