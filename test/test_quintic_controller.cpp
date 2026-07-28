#include "SCurvePosVelController.h"

#include <cmath>
#include <cstdio>
#include <initializer_list>

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); \
    ++failures; \
  } \
} while (0)

constexpr float EPS = 1.0e-4f;

void configure(SCurvePosVelController& controller,
               float vmax,
               float amax)
{
  controller.setLimits(vmax, amax);
  controller.setOutputMax(vmax * 1.25f);
  controller.setGains(0.8f, 0.0f, 0.02f, 1.0f);
  controller.setIntegratorLimit(0.0f);
  controller.setTolerances(0.01f, 0.02f);
  controller.setVelocityFilterTau(0.0f);
  controller.disableDeadband();
  controller.setPositionLimits(-500.0f, 500.0f, 0.5f, 1.0f);
}

void followUntilComplete(SCurvePosVelController& controller,
                         float dt,
                         float direction,
                         float velocity_limit,
                         float acceleration_limit)
{
  float measured = controller.refPos();
  float previous = controller.refPos();
  int guard = 0;

  while (controller.trajectoryActive() && guard++ < 200000) {
    measured = controller.refPos();
    (void)controller.update(measured, dt);

    CHECK(std::isfinite(controller.refPos()));
    CHECK(std::isfinite(controller.refVel()));
    CHECK(std::isfinite(controller.refAcc()));
    const float directed_step = direction * (controller.refPos() - previous);
    if (directed_step < -2.0e-4f) {
      std::printf("non-monotonic step prev=%.9f current=%.9f target=%.9f dt=%.9f\n",
                  previous,
                  controller.refPos(),
                  controller.target(),
                  dt);
    }
    CHECK(directed_step >= -2.0e-4f);
    CHECK(std::fabs(controller.refVel()) <= velocity_limit + 0.02f);
    CHECK(std::fabs(controller.refAcc()) <= acceleration_limit + 0.05f);
    previous = controller.refPos();
  }

  CHECK(guard < 200000);
}

void testPositiveAndNegativeRestToRest()
{
  for (const float target : {25.0f, -25.0f}) {
    SCurvePosVelController controller;
    configure(controller, 8.0f, 20.0f);
    controller.reset(0.0f);
    controller.setTarget(target);

    CHECK(controller.trajectoryActive());
    CHECK(std::fabs(controller.refPos()) < EPS);
    CHECK(std::fabs(controller.refVel()) < EPS);
    CHECK(std::fabs(controller.refAcc()) < EPS);
    CHECK(!controller.isSettled());

    const float dt = controller.trajectoryDuration() / 1200.0f;
    followUntilComplete(controller,
                        dt,
                        target > 0.0f ? 1.0f : -1.0f,
                        8.0f,
                        20.0f);

    CHECK(!controller.trajectoryActive());
    CHECK(std::fabs(controller.refPos() - target) < EPS);
    CHECK(std::fabs(controller.refVel()) < EPS);
    CHECK(std::fabs(controller.refAcc()) < EPS);
  }
}

void testLimitsAndShortMove()
{
  const float distances[] = {0.001f, 0.05f, 1.0f, 40.0f, -0.001f, -17.0f};
  for (float target : distances) {
    SCurvePosVelController controller;
    configure(controller, 3.5f, 9.0f);
    controller.reset(0.0f);
    controller.setTarget(target);
    CHECK(!controller.fault());

    const float dt = fmaxf(controller.trajectoryDuration() / 1000.0f, 1.0e-6f);
    followUntilComplete(controller,
                        dt,
                        target >= 0.0f ? 1.0f : -1.0f,
                        3.5f,
                        9.0f);
    CHECK(std::fabs(controller.refPos() - target) < EPS);
  }
}

void testNoTerminalSnap()
{
  SCurvePosVelController controller;
  configure(controller, 5.0f, 12.0f);
  controller.reset(0.0f);
  controller.setTarget(10.0f);

  const float dt = controller.trajectoryDuration() / 5000.0f;
  float previous_position = controller.refPos();
  float previous_velocity = 0.0f;
  float previous_acceleration = 0.0f;

  while (controller.trajectoryActive()) {
    previous_position = controller.refPos();
    previous_velocity = controller.refVel();
    previous_acceleration = controller.refAcc();
    (void)controller.update(previous_position, dt);
  }

  CHECK(std::fabs(10.0f - previous_position) < 2.0e-4f);
  CHECK(std::fabs(previous_velocity) < 0.01f);
  CHECK(std::fabs(previous_acceleration) < 0.05f);
}

void testForwardAndReverseBlend()
{
  SCurvePosVelController controller;
  configure(controller, 6.0f, 15.0f);
  controller.reset(0.0f);
  controller.setTarget(20.0f);

  const float dt = controller.trajectoryDuration() / 1000.0f;
  for (int i = 0; i < 250; ++i) {
    (void)controller.update(controller.refPos(), dt);
  }

  const float p0 = controller.refPos();
  const float v0 = controller.refVel();
  const float a0 = controller.refAcc();
  const float old_elapsed = controller.trajectoryElapsed();

  CHECK(v0 > 0.0f);
  CHECK(controller.setTargetBlended(30.0f));
  CHECK(std::fabs(controller.refPos() - p0) < EPS);
  CHECK(std::fabs(controller.refVel() - v0) < EPS);
  CHECK(std::fabs(controller.refAcc() - a0) < EPS);

  followUntilComplete(controller,
                      controller.trajectoryDuration() / 1500.0f,
                      1.0f,
                      fmaxf(6.0f, std::fabs(v0)),
                      fmaxf(15.0f, std::fabs(a0)));
  CHECK(std::fabs(controller.refPos() - 30.0f) < EPS);

  controller.reset(0.0f);
  controller.setTarget(20.0f);
  for (int i = 0; i < 250; ++i) {
    (void)controller.update(controller.refPos(), dt);
  }

  const float reverse_p0 = controller.refPos();
  const float reverse_v0 = controller.refVel();
  const float reverse_a0 = controller.refAcc();
  const float reverse_elapsed = controller.trajectoryElapsed();
  CHECK(!controller.setTargetBlended(-10.0f));
  CHECK(std::fabs(controller.refPos() - reverse_p0) < EPS);
  CHECK(std::fabs(controller.refVel() - reverse_v0) < EPS);
  CHECK(std::fabs(controller.refAcc() - reverse_a0) < EPS);
  CHECK(std::fabs(controller.trajectoryElapsed() - reverse_elapsed) < EPS);
  CHECK(controller.target() == 20.0f);
  CHECK(old_elapsed > 0.0f);
}

void testInfeasibleBlendAndActiveLimitUpdate()
{
  SCurvePosVelController controller;
  configure(controller, 10.0f, 30.0f);
  controller.reset(0.0f);
  controller.setTarget(50.0f);

  const float dt = controller.trajectoryDuration() / 1000.0f;
  for (int i = 0; i < 500; ++i) {
    (void)controller.update(controller.refPos(), dt);
  }

  const float before_p = controller.refPos();
  const float before_v = controller.refVel();
  const float before_a = controller.refAcc();
  CHECK(!controller.setTargetBlended(before_p + 1.0e-4f));
  CHECK(std::fabs(controller.refPos() - before_p) < EPS);
  CHECK(std::fabs(controller.refVel() - before_v) < EPS);
  CHECK(std::fabs(controller.refAcc() - before_a) < EPS);

  controller.setLimits(8.0f, 20.0f);
  CHECK(controller.replanActiveTrajectory());
  CHECK(std::fabs(controller.refPos() - before_p) < EPS);
  CHECK(std::fabs(controller.refVel() - before_v) < EPS);
  CHECK(std::fabs(controller.refAcc() - before_a) < EPS);

  followUntilComplete(controller,
                      controller.trajectoryDuration() / 1800.0f,
                      1.0f,
                      fmaxf(8.0f, std::fabs(before_v)),
                      fmaxf(20.0f, std::fabs(before_a)));
  CHECK(std::fabs(controller.refPos() - 50.0f) < EPS);
}

void testDeadbandSettledAndFaultReset()
{
  SCurvePosVelController controller;
  configure(controller, 4.0f, 10.0f);
  controller.setDeadband(100.0f, 120.0f, 100.0f);
  controller.reset(0.0f);
  controller.setDeadband(100.0f, 120.0f, 100.0f);
  controller.setTarget(5.0f);

  const float dt = controller.trajectoryDuration() / 500.0f;
  for (int i = 0; i < 100 && controller.trajectoryActive(); ++i) {
    (void)controller.update(controller.refPos(), dt);
    CHECK(controller.trajectoryActive());
    CHECK(!controller.inDeadband());
    CHECK(!controller.isSettled());
  }

  followUntilComplete(controller, dt, 1.0f, 4.0f, 10.0f);
  CHECK(!controller.inDeadband());
  (void)controller.update(controller.refPos(), dt);
  CHECK(controller.inDeadband());
  CHECK(controller.isSettled());

  controller.reset(0.0f);
  controller.setTarget(10.0f);
  CHECK(controller.trajectoryActive());
  controller.latchEmergencyStop(controller.refPos());
  CHECK(controller.fault());
  CHECK(!controller.trajectoryActive());
  CHECK(std::fabs(controller.refVel()) < EPS);
  CHECK(std::fabs(controller.refAcc()) < EPS);

  controller.reset(2.0f);
  CHECK(!controller.fault());
  CHECK(!controller.trajectoryActive());
  CHECK(std::fabs(controller.refPos() - 2.0f) < EPS);
}

uint32_t random_state = 0x12345678u;

float randomUnit()
{
  random_state = random_state * 1664525u + 1013904223u;
  return static_cast<float>((random_state >> 8) & 0x00FFFFFFu) /
         static_cast<float>(0x01000000u);
}

void testRandomizedRestProfiles()
{
  for (int iteration = 0; iteration < 1000; ++iteration) {
    const float target_magnitude = 0.001f + randomUnit() * 150.0f;
    const float target = randomUnit() < 0.5f
      ? -target_magnitude
      : target_magnitude;
    const float vmax = 0.05f + randomUnit() * 30.0f;
    const float amax = 0.05f + randomUnit() * 80.0f;

    SCurvePosVelController controller;
    configure(controller, vmax, amax);
    controller.reset(0.0f);
    controller.setTarget(target);
    CHECK(!controller.fault());

    const float dt = fmaxf(controller.trajectoryDuration() / 300.0f, 1.0e-6f);
    followUntilComplete(controller,
                        dt,
                        target > 0.0f ? 1.0f : -1.0f,
                        vmax,
                        amax);
    CHECK(std::fabs(controller.refPos() - target) < 2.0e-4f);
  }
}

void testRandomizedForwardRetargets()
{
  int accepted = 0;

  for (int iteration = 0; iteration < 300; ++iteration) {
    const float direction = randomUnit() < 0.5f ? -1.0f : 1.0f;
    const float first_target =
      direction * (1.0f + randomUnit() * 80.0f);
    const float second_target =
      first_target + direction * (1.0f + randomUnit() * 80.0f);
    const float vmax = 0.5f + randomUnit() * 20.0f;
    const float amax = 1.0f + randomUnit() * 60.0f;

    SCurvePosVelController controller;
    configure(controller, vmax, amax);
    controller.reset(0.0f);
    controller.setTarget(first_target);

    const float first_dt = controller.trajectoryDuration() / 500.0f;
    const int advance_steps =
      75 + static_cast<int>(randomUnit() * 250.0f);
    for (int step = 0;
         step < advance_steps && controller.trajectoryActive();
         ++step) {
      (void)controller.update(controller.refPos(), first_dt);
    }

    const float p0 = controller.refPos();
    const float v0 = controller.refVel();
    const float a0 = controller.refAcc();
    const float old_target = controller.target();
    const float old_elapsed = controller.trajectoryElapsed();

    if (!controller.setTargetBlended(second_target)) {
      CHECK(std::fabs(controller.refPos() - p0) < EPS);
      CHECK(std::fabs(controller.refVel() - v0) < EPS);
      CHECK(std::fabs(controller.refAcc() - a0) < EPS);
      CHECK(std::fabs(controller.target() - old_target) < EPS);
      CHECK(std::fabs(controller.trajectoryElapsed() - old_elapsed) < EPS);
      continue;
    }

    ++accepted;
    CHECK(std::fabs(controller.refPos() - p0) < EPS);
    CHECK(std::fabs(controller.refVel() - v0) < EPS);
    CHECK(std::fabs(controller.refAcc() - a0) < EPS);

    followUntilComplete(controller,
                        controller.trajectoryDuration() / 600.0f,
                        direction,
                        fmaxf(vmax, std::fabs(v0)),
                        fmaxf(amax, std::fabs(a0)));
    CHECK(std::fabs(controller.refPos() - second_target) < 3.0e-4f);
  }

  std::printf("randomized forward blends accepted: %d/300\n", accepted);
  CHECK(accepted >= 100);
}

} // namespace

int main()
{
  testPositiveAndNegativeRestToRest();
  testLimitsAndShortMove();
  testNoTerminalSnap();
  testForwardAndReverseBlend();
  testInfeasibleBlendAndActiveLimitUpdate();
  testDeadbandSettledAndFaultReset();
  testRandomizedRestProfiles();
  testRandomizedForwardRetargets();

  if (failures != 0) {
    std::printf("quintic controller tests failed: %d\n", failures);
    return 1;
  }

  std::printf("quintic controller tests passed\n");
  return 0;
}
