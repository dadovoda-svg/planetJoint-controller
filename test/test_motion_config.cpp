#include "JointBusProtocol.h"

#include <cmath>
#include <cstdio>

int main()
{
  JointBus::MotionConfig source;
  source.jminCdeg = -17000;
  source.jmaxCdeg = 17000;
  source.vmaxCdegS = 24000;
  source.amaxCdegS2 = 65000;

  JointBus::Frame response;
  response.address = 4;
  response.type = JointBus::FrameType::Response;
  response.seq = 0x52;
  response.command = JointBus::Command::MotionConfigRsp;
  response.payloadLen = JointBus::MOTION_CONFIG_PAYLOAD_SIZE;
  JointBus::encodeMotionConfigPayload(source, response.payload);

  uint8_t wire[JointBus::MAX_FRAME_SIZE] = {0};
  const size_t wireLen = JointBus::encodeFrame(response, wire, sizeof(wire));
  if (wireLen != 15) return 1;

  JointBus::Parser parser;
  JointBus::Frame parsed;
  JointBus::Parser::Result result = JointBus::Parser::Result::None;
  for (size_t i = 0; i < wireLen; ++i) {
    result = parser.push(wire[i], parsed);
  }

  if (result != JointBus::Parser::Result::FrameReady) return 2;
  if (parsed.address != 4 || parsed.type != JointBus::FrameType::Response) return 3;
  if (parsed.seq != 0x52 || parsed.command != JointBus::Command::MotionConfigRsp) return 4;

  JointBus::MotionConfig decoded;
  if (!JointBus::decodeMotionConfigPayload(parsed.payload, parsed.payloadLen, decoded)) return 5;
  if (decoded.jminCdeg != source.jminCdeg || decoded.jmaxCdeg != source.jmaxCdeg) return 6;
  if (decoded.vmaxCdegS != source.vmaxCdegS || decoded.amaxCdegS2 != source.amaxCdegS2) return 7;
  if (JointBus::decodeMotionConfigPayload(parsed.payload, parsed.payloadLen - 1, decoded)) return 8;

  const float jminDeg = static_cast<float>(decoded.jminCdeg) * 0.01f;
  const float amaxDegS2 = static_cast<float>(decoded.amaxCdegS2) * 0.01f;
  if (std::fabs(jminDeg + 170.0f) > 0.001f) return 9;
  if (std::fabs(amaxDegS2 - 650.0f) > 0.001f) return 10;

  JointBus::Frame request;
  request.address = 4;
  request.type = JointBus::FrameType::Request;
  request.seq = 0x53;
  request.command = JointBus::Command::MotionConfig;
  request.payloadLen = 0;

  const size_t requestLen = JointBus::encodeFrame(request, wire, sizeof(wire));
  if (requestLen != 7) return 11;
  result = JointBus::Parser::Result::None;
  for (size_t i = 0; i < requestLen; ++i) {
    result = parser.push(wire[i], parsed);
  }
  if (result != JointBus::Parser::Result::FrameReady) return 12;
  if (parsed.type != JointBus::FrameType::Request ||
      parsed.command != JointBus::Command::MotionConfig ||
      parsed.payloadLen != 0 || parsed.seq != 0x53) return 13;

  std::printf("motion config test passed\n");
  return 0;
}
