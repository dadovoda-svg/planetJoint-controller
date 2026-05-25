#pragma once

#include <Arduino.h>

// Motion state owned by the joint/planner layer.
enum class MotionMode : uint8_t {
  IDLE,
  VELOCITY_TEST,
  STEP_TEST,
  POSITION,
  CALIBRATION,
  FAULT
};

struct JointMoveCommand {
  // All fields are expressed in real joint degrees, relative to the logical zero.
  float targetDeg = 0.0f;
  float vmaxDegS = 0.0f;
  float amaxDegS2 = 0.0f;
  float sCurveTimeS = 0.0f;   // <= 0 means keep current/default parameter
  float outMaxDegS = 0.0f;    // <= 0 means auto: 1.25 * vmax
};

class JointPlanner {
public:
  explicit JointPlanner(HardwareSerial& plannerSerial);

  void begin(uint32_t baud);
  void update();

  bool stop();
  bool moveTo(const JointMoveCommand& cmd);
  bool moveTo(float targetDeg, float vmaxDegS, float amaxDegS2);
  bool moveTo(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS);

  bool moveToBlended(const JointMoveCommand& cmd);
  bool moveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2);
  bool moveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS);

private:
  void resetInput();
  void processInputChar(char c);
  void handleLine(char* line);
  void printHelp();
  bool parseFloatToken(const char* token, float& out);

  HardwareSerial& _serial;
  char _rxLine[96] = {0};
  uint8_t _rxLen = 0;
};

// Console/backward-compatible API wrappers.
bool moveJointToDeg(float targetZeroedDeg);
bool jointMoveTo(const JointMoveCommand& cmd);
bool jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2);
bool jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS);
bool jointMoveToBlended(const JointMoveCommand& cmd);
bool jointMoveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2);
bool jointMoveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS);
bool jointStop();
