#include "JointBusProtocol.h"

#include <cstdio>

int main()
{
  JointBus::Frame start;
  start.address = JointBus::BROADCAST_ADDRESS;
  start.type = JointBus::FrameType::Request;
  start.seq = 17;
  start.command = JointBus::Command::StartSegment;
  start.payloadLen = JointBus::TIMED_START_PAYLOAD_SIZE;
  start.payload[0] = 42;
  JointBus::putU32LE(&start.payload[1], 3210);
  JointBus::putU32LE(&start.payload[5], 10000);

  uint8_t wire[JointBus::MAX_FRAME_SIZE] = {};
  const size_t wireLen = JointBus::encodeFrame(
    start, wire, sizeof(wire));
  if (wireLen == 0) return 1;

  JointBus::Parser parser;
  JointBus::Frame parsed;
  JointBus::Parser::Result parseResult =
    JointBus::Parser::Result::None;
  for (size_t index = 0; index < wireLen; ++index) {
    parseResult = parser.push(wire[index], parsed);
  }
  if (parseResult != JointBus::Parser::Result::FrameReady) return 2;
  if (parsed.payloadLen != JointBus::TIMED_START_PAYLOAD_SIZE ||
      parsed.payload[0] != 42 ||
      JointBus::getU32LE(&parsed.payload[1]) != 3210 ||
      JointBus::getU32LE(&parsed.payload[5]) != 10000) return 3;

  JointBus::SegmentTiming source;
  source.segmentId = 42;
  source.minimumDurationMs = 2973;
  uint8_t payload[JointBus::SEGMENT_TIMING_PAYLOAD_SIZE] = {};
  JointBus::encodeSegmentTimingPayload(source, payload);
  JointBus::SegmentTiming decoded;
  if (!JointBus::decodeSegmentTimingPayload(
        payload, sizeof(payload), decoded)) return 4;
  if (decoded.segmentId != source.segmentId ||
      decoded.minimumDurationMs != source.minimumDurationMs) return 5;
  if (JointBus::decodeSegmentTimingPayload(
        payload, sizeof(payload) - 1, decoded)) return 6;

  std::printf("timed segment protocol test passed\n");
  return 0;
}
