#include "HobbyServoMotion.h"
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

void testConfigurationAndMapping()
{
  CHECK(HobbyServoMotion::validConfig(1500, 1000, 2000));
  CHECK(!HobbyServoMotion::validConfig(900, 1000, 2000));
  CHECK(!HobbyServoMotion::validConfig(1500, 2000, 1000));
  CHECK(HobbyServoMotion::positionToPulseUs(0, 1000, 2000) == 1000);
  CHECK(HobbyServoMotion::positionToPulseUs(999, 1000, 2000) == 2000);
  CHECK(HobbyServoMotion::positionToPulseUs(500, 1000, 2000) == 1501);
  CHECK(HobbyServoMotion::PWM_RESOLUTION_BITS == 14);
  CHECK(HobbyServoMotion::pulseUsToDuty(1500) == 1229);
}

void testSpeedAndInterpolation()
{
  CHECK(HobbyServoMotion::moveDurationMs(1000, 2000, 1000, 2000, 1) == 5000);
  CHECK(HobbyServoMotion::moveDurationMs(1000, 2000, 1000, 2000, 10) == 0);
  CHECK(HobbyServoMotion::moveDurationMs(1000, 1500, 1000, 2000, 1) == 2500);
  uint32_t previousDuration = 5001;
  for (uint8_t speed = 1; speed <= 9; ++speed) {
    const uint32_t duration = HobbyServoMotion::moveDurationMs(
        1000, 2000, 1000, 2000, speed);
    CHECK(duration > 0);
    CHECK(duration < previousDuration);
    previousDuration = duration;
  }
  CHECK(HobbyServoMotion::interpolatePulseUs(1000, 2000, 2500, 5000) == 1500);
  CHECK(HobbyServoMotion::interpolatePulseUs(2000, 1000, 2500, 5000) == 1500);
  CHECK(HobbyServoMotion::interpolatePulseUs(1000, 2000, 5000, 5000) == 2000);

  // Reproduce the former loop race: now was sampled just before command
  // dispatch and is one millisecond older than startedMs.
  CHECK(HobbyServoMotion::elapsedMsSince(1000, 1001) == 0);
  CHECK(HobbyServoMotion::elapsedMsSince(1001, 1000) == 1);
  CHECK(HobbyServoMotion::elapsedMsSince(2, 0xFFFFFFFEU) == 4);
}

void testProtocolFrame()
{
  JointBus::Frame request;
  request.address = 2;
  request.type = JointBus::FrameType::Request;
  request.seq = 0x71;
  request.command = JointBus::Command::ServoMove;
  request.payloadLen = JointBus::SERVO_MOVE_PAYLOAD_SIZE;
  JointBus::putU16LE(&request.payload[0], 999);
  request.payload[2] = 10;

  uint8_t wire[JointBus::MAX_FRAME_SIZE] = {0};
  const size_t wireLength = JointBus::encodeFrame(request, wire, sizeof(wire));
  CHECK(wireLength == 10);

  JointBus::Parser parser;
  JointBus::Frame parsed;
  JointBus::Parser::Result result = JointBus::Parser::Result::None;
  for (size_t i = 0; i < wireLength; ++i) {
    result = parser.push(wire[i], parsed);
  }
  CHECK(result == JointBus::Parser::Result::FrameReady);
  CHECK(parsed.command == JointBus::Command::ServoMove);
  CHECK(JointBus::getU16LE(&parsed.payload[0]) == 999);
  CHECK(parsed.payload[2] == 10);
}

} // namespace

int main()
{
  testConfigurationAndMapping();
  testSpeedAndInterpolation();
  testProtocolFrame();

  if (failures != 0) {
    std::printf("hobby servo motion tests failed: %d\n", failures);
    return 1;
  }
  std::printf("hobby servo motion tests passed\n");
  return 0;
}
