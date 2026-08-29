#include "FaultRecovery.h"
#include "JointBusProtocol.h"

#include <cstdio>

namespace {

int failures = 0;

#define CHECK(condition) do { \
  if (!(condition)) { \
    std::printf("FAIL line %d: %s\n", __LINE__, #condition); \
    ++failures; \
  } \
} while (0)

FaultRecoverySnapshot validSnapshot()
{
  FaultRecoverySnapshot snapshot;
  snapshot.faultActive = true;
  snapshot.encoderValid = true;
  snapshot.driverValid = true;
  snapshot.limitsValid = true;
  snapshot.positionDeg = 0.0f;
  snapshot.minDeg = -10.0f;
  snapshot.maxDeg = 10.0f;
  return snapshot;
}

void testSafeClearConditions()
{
  FaultRecoverySnapshot snapshot = validSnapshot();
  CHECK(evaluateFaultRecovery(snapshot) == FaultClearResult::Cleared);

  snapshot.faultActive = false;
  CHECK(evaluateFaultRecovery(snapshot) == FaultClearResult::AlreadyClear);
}

void testUnsafeConditionsAreRejected()
{
  FaultRecoverySnapshot snapshot = validSnapshot();
  snapshot.encoderValid = false;
  CHECK(evaluateFaultRecovery(snapshot) ==
        FaultClearResult::EncoderUnavailable);

  snapshot = validSnapshot();
  snapshot.driverValid = false;
  CHECK(evaluateFaultRecovery(snapshot) ==
        FaultClearResult::DriverUnavailable);

  snapshot = validSnapshot();
  snapshot.limitsValid = false;
  CHECK(evaluateFaultRecovery(snapshot) == FaultClearResult::InvalidLimits);

  snapshot = validSnapshot();
  snapshot.maxDeg = snapshot.minDeg;
  CHECK(evaluateFaultRecovery(snapshot) == FaultClearResult::InvalidLimits);

  snapshot = validSnapshot();
  snapshot.positionDeg = 10.001f;
  CHECK(evaluateFaultRecovery(snapshot) ==
        FaultClearResult::PositionOutsideLimits);

  snapshot = validSnapshot();
  snapshot.positionDeg = snapshot.minDeg;
  CHECK(evaluateFaultRecovery(snapshot) == FaultClearResult::Cleared);
  snapshot.positionDeg = snapshot.maxDeg;
  CHECK(evaluateFaultRecovery(snapshot) == FaultClearResult::Cleared);
}

void testClearFaultProtocolFrame()
{
  JointBus::Frame request;
  request.address = 3;
  request.type = JointBus::FrameType::Request;
  request.seq = 0x62;
  request.command = JointBus::Command::ClearFault;
  request.payloadLen = 0;

  uint8_t wire[JointBus::MAX_FRAME_SIZE] = {0};
  const size_t wireLength =
      JointBus::encodeFrame(request, wire, sizeof(wire));
  CHECK(wireLength == 7);

  JointBus::Parser parser;
  JointBus::Frame parsed;
  JointBus::Parser::Result result = JointBus::Parser::Result::None;
  for (size_t i = 0; i < wireLength; ++i) {
    result = parser.push(wire[i], parsed);
  }
  CHECK(result == JointBus::Parser::Result::FrameReady);
  CHECK(parsed.address == 3);
  CHECK(parsed.seq == 0x62);
  CHECK(parsed.command == JointBus::Command::ClearFault);
  CHECK(parsed.payloadLen == 0);
  CHECK(static_cast<uint8_t>(JointBus::AckCode::FaultCleared) == 0x0C);
}

} // namespace

int main()
{
  testSafeClearConditions();
  testUnsafeConditionsAreRejected();
  testClearFaultProtocolFrame();

  if (failures != 0) {
    std::printf("fault recovery tests failed: %d\n", failures);
    return 1;
  }

  std::printf("fault recovery tests passed\n");
  return 0;
}
