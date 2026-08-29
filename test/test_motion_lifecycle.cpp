#include "MotionLifecycle.h"

#include <cstdio>

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); \
    ++failures; \
  } \
} while (0)

void testCompletionLatchWithoutHold()
{
  MotionLifecycleState state;
  state.beginPositionCommand();
  CHECK(positionMotionCommandBusy(state));

  CHECK(updateMotionLifecycle(state, false, false, false) ==
        MotionLifecycleEvent::None);
  CHECK(state.positionCommandActive);

  CHECK(updateMotionLifecycle(state, true, false, false) ==
        MotionLifecycleEvent::CommandCompletedNoHold);
  CHECK(!positionMotionCommandBusy(state));

  CHECK(updateMotionLifecycle(state, false, false, false) ==
        MotionLifecycleEvent::None);
  CHECK(updateMotionLifecycle(state, true, false, false) ==
        MotionLifecycleEvent::None);
  CHECK(!state.positionCommandActive);

  state.beginPositionCommand();
  CHECK(state.positionCommandActive);
}

void testServoCorrectionTransitions()
{
  MotionLifecycleState state;
  state.beginPositionCommand();

  CHECK(updateMotionLifecycle(state, true, true, false) ==
        MotionLifecycleEvent::CommandCompletedHold);
  CHECK(!state.positionCommandActive);
  CHECK(state.servoHoldActive);
  CHECK(!state.servoCorrectionArmed);
  CHECK(!state.servoCorrectionActive);
  CHECK(!positionModeReportsMoving(state));

  // Initial convergence before deadband is not an external correction.
  CHECK(updateMotionLifecycle(state, false, true, false) ==
        MotionLifecycleEvent::None);
  CHECK(!state.servoCorrectionActive);

  CHECK(updateMotionLifecycle(state, true, true, true) ==
        MotionLifecycleEvent::None);
  CHECK(state.servoCorrectionArmed);

  CHECK(updateMotionLifecycle(state, false, true, false) ==
        MotionLifecycleEvent::ServoCorrectionStarted);
  CHECK(state.servoCorrectionActive);
  CHECK(!positionMotionCommandBusy(state));
  CHECK(!positionModeReportsMoving(state));

  CHECK(updateMotionLifecycle(state, false, true, false) ==
        MotionLifecycleEvent::None);
  CHECK(state.servoCorrectionActive);

  CHECK(updateMotionLifecycle(state, true, true, true) ==
        MotionLifecycleEvent::ServoCorrectionCompleted);
  CHECK(!state.servoCorrectionActive);
  CHECK(state.servoCorrectionArmed);

  CHECK(updateMotionLifecycle(state, true, true, true) ==
        MotionLifecycleEvent::None);
}

void testNewCommandAndClearResetCorrectionState()
{
  MotionLifecycleState state;
  state.servoHoldActive = true;
  state.servoCorrectionArmed = true;
  state.servoCorrectionActive = true;
  state.lastServoCorrectionDebugMs = 1234;

  state.beginPositionCommand();
  CHECK(state.positionCommandActive);
  CHECK(!state.servoHoldActive);
  CHECK(!state.servoCorrectionArmed);
  CHECK(!state.servoCorrectionActive);
  CHECK(state.lastServoCorrectionDebugMs == 0);

  state.clear();
  CHECK(!state.positionCommandActive);
  CHECK(!state.servoHoldActive);
  CHECK(!state.servoCorrectionArmed);
  CHECK(!state.servoCorrectionActive);
}

void testExternalStatusSemantics()
{
  MotionLifecycleState state;
  state.beginPositionCommand();

  CHECK(positionLifecycleExternalState(state, false) ==
        PositionLifecycleExternalState::Moving);
  MotionCommandExternalStatus status =
      motionCommandExternalStatus(state, false, false);
  CHECK(status.busy);
  CHECK(!status.done);
  CHECK(!status.fault);

  CHECK(updateMotionLifecycle(state, true, true, true) ==
        MotionLifecycleEvent::CommandCompletedHold);
  CHECK(positionLifecycleExternalState(state, false) ==
        PositionLifecycleExternalState::Holding);
  status = motionCommandExternalStatus(state, false, false);
  CHECK(!status.busy);
  CHECK(status.done);
  CHECK(!status.fault);

  CHECK(updateMotionLifecycle(state, false, true, false) ==
        MotionLifecycleEvent::ServoCorrectionStarted);
  CHECK(positionLifecycleExternalState(state, false) ==
        PositionLifecycleExternalState::Holding);
  status = motionCommandExternalStatus(state, false, false);
  CHECK(!status.busy);
  CHECK(status.done);
  CHECK(!status.fault);

  state.beginPositionCommand();
  CHECK(positionLifecycleExternalState(state, true) ==
        PositionLifecycleExternalState::Fault);
  status = motionCommandExternalStatus(state, false, true);
  CHECK(!status.busy);
  CHECK(!status.done);
  CHECK(status.fault);
}

} // namespace

int main()
{
  testCompletionLatchWithoutHold();
  testServoCorrectionTransitions();
  testNewCommandAndClearResetCorrectionState();
  testExternalStatusSemantics();

  if (failures != 0) {
    std::printf("motion lifecycle tests failed: %d\n", failures);
    return 1;
  }

  std::printf("motion lifecycle tests passed\n");
  return 0;
}
