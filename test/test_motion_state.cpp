#include "JointBusProtocol.h"

#include <cstdio>

int main()
{
  JointBus::MotionState source;
  source.activeSegmentId = 238;
  source.flags = JointBus::MSTATE_ACTIVE_VALID |
                 JointBus::MSTATE_TRAJECTORY_ACTIVE |
                 JointBus::MSTATE_SAFE_REPLAN;
  source.refPosCdeg = -2930;
  source.refVelCdegS = 573;
  source.refAccDdegS2 = -321;
  source.elapsedMs = 1234;
  source.durationMs = 98765;

  JointBus::Frame response;
  response.address = 6;
  response.type = JointBus::FrameType::Response;
  response.seq = 0x64;
  response.command = JointBus::Command::MotionStateRsp;
  response.payloadLen = JointBus::MOTION_STATE_PAYLOAD_SIZE;
  JointBus::encodeMotionStatePayload(source, response.payload);

  uint8_t wire[JointBus::MAX_FRAME_SIZE] = {0};
  const size_t wireLen = JointBus::encodeFrame(
      response, wire, sizeof(wire));
  if (wireLen != JointBus::MAX_FRAME_SIZE) return 1;

  JointBus::Parser parser;
  JointBus::Frame parsed;
  JointBus::Parser::Result result = JointBus::Parser::Result::None;
  for (size_t i = 0; i < wireLen; ++i) {
    result = parser.push(wire[i], parsed);
  }
  if (result != JointBus::Parser::Result::FrameReady) return 2;
  if (parsed.command != JointBus::Command::MotionStateRsp ||
      parsed.payloadLen != JointBus::MOTION_STATE_PAYLOAD_SIZE) return 3;

  JointBus::MotionState decoded;
  if (!JointBus::decodeMotionStatePayload(
          parsed.payload, parsed.payloadLen, decoded)) return 4;
  if (decoded.activeSegmentId != source.activeSegmentId ||
      decoded.flags != source.flags ||
      decoded.refPosCdeg != source.refPosCdeg ||
      decoded.refVelCdegS != source.refVelCdegS ||
      decoded.refAccDdegS2 != source.refAccDdegS2 ||
      decoded.elapsedMs != source.elapsedMs ||
      decoded.durationMs != source.durationMs) return 5;
  if (JointBus::decodeMotionStatePayload(
          parsed.payload, parsed.payloadLen - 1, decoded)) return 6;

  std::printf("motion state test passed\n");
  return 0;
}
