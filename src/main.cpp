#include <Arduino.h>
#include <algorithm>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#include "led_status.h"
#include "encoder_selection.h"

#if MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5048A
#include "as5048a.h"
using MagneticEncoder = AS5048A;
#elif MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5600
#include "as5600.h"
using MagneticEncoder = AS5600;
#endif
#include "params.h"
#include "SerialConsole.h"
#include "Tmc2209Driver.h"
#include "SCurvePosVelController.h"
#include "JointPlanner.h"
#include "JointMotionApi.h"
#include "MotionLifecycle.h"
#include "FaultRecovery.h"
#include "HobbyServoMotion.h"
#include "logger.h"
#include "JointBusSlave.h"

// ===================== BAUDRATE =====================

static constexpr uint32_t USB_BAUD      = 115200;
#ifndef JOINTBUS_BAUD_921600
#define JOINTBUS_BAUD_921600 0
#endif
static constexpr uint32_t JOINTBUS_BAUD = JOINTBUS_BAUD_921600 ? 921600UL : 500000UL;
static constexpr size_t JOINTBUS_RX_BUFFER_SIZE = 1024;
static constexpr uint32_t TMC_BAUD      = 230400;

// ===================== PIN MAP =====================

static constexpr int PIN_ENCODER_SCK  = 12;  // AS5048A SCK = CLK
static constexpr int PIN_ENCODER_MISO = 13;  // AS5048A MISO = DOUT; AS5600 SDA
static constexpr int PIN_ENCODER_MOSI = 11;  // AS5048A MOSI = DIN; AS5600 SCL
static constexpr int PIN_ENCODER_CS   = 10;  // AS5048A CS; unused by AS5600

static constexpr uint32_t AS5600_I2C_CLOCK_HZ = 400000;
static constexpr uint8_t AS5600_I2C_ADDRESS = 0x36;

static constexpr int PIN_TMC_TX   = 7;  // ESP32 TX -> PDN_UART through 1k
static constexpr int PIN_TMC_RX   = 8;  // ESP32 RX -> PDN_UART
static constexpr int PIN_TMC_STEP = 4;
static constexpr int PIN_TMC_DIR  = 5;
static constexpr int PIN_TMC_ENN  = 6;
static constexpr int PIN_PARK_SENSOR = 3; // active LOW, external pull-up
static constexpr int PIN_RS485_RTS   = 9; // UART0 RTS -> SP3485 DE, hardware RS485 half-duplex

// ===================== MECHANICAL SCALE =====================

// Default mechanical scale.
// Runtime parameter `jrev` overrides this value:
// one full 360-degree encoder revolution corresponds to `jrev` degrees of real joint motion.
static constexpr float DEFAULT_JOINT_DEGREES_PER_ENCODER_REV = 15.6f;
static constexpr float DEFAULT_MOTOR_DIRECTION_SIGN = 1.0f;
static constexpr float DEFAULT_ENCODER_DIRECTION_SIGN = 1.0f;


// ===================== VERY CONSERVATIVE SERVO DEFAULTS =====================

// Real joint was observed to start losing steps above ~15 deg/s.
// First closed-loop tests are intentionally much lower.
static constexpr uint32_t SERVO_CONTROL_PERIOD_US = 5000; // 200 Hz
static constexpr float SERVO_VMAX_DEG_S       = 2.0f;
static constexpr float SERVO_AMAX_DEG_S2      = 6.0f;
static constexpr float SERVO_OUTPUT_MAX_DEG_S = 2.5f;

static constexpr float SERVO_KP               = 0.8f;
static constexpr float SERVO_KI               = 0.0f;
static constexpr float SERVO_KD               = 0.02f;
static constexpr float SERVO_FF_VEL           = 1.0f;
static constexpr float SERVO_I_LIMIT          = 0.0f;

static constexpr float SERVO_POS_TOL_DEG      = 0.08f;
static constexpr float SERVO_VEL_TOL_DEG_S    = 0.15f;
static constexpr float SERVO_DEADBAND_ENTER   = 0.05f;
static constexpr float SERVO_DEADBAND_EXIT    = 0.12f;
static constexpr float SERVO_DEADBAND_VEL     = 0.20f;
static constexpr float SERVO_VEL_FILTER_TAU_S = 0.050f;

// Absolute planner safety envelope. Runtime commands and persisted motion
// parameters are always constrained inside these bounds.
static constexpr float PLANNER_MIN_VMAX_DEG_S = 0.01f;
static constexpr float PLANNER_MAX_VMAX_DEG_S = 240.0f;
static constexpr float PLANNER_MIN_AMAX_DEG_S2 = 0.01f;
static constexpr float PLANNER_MAX_AMAX_DEG_S2 = 650.0f;
static constexpr float PLANNER_BLEND_MIN_REF_VEL_DEG_S = 0.02f;

// ===================== JOINT POSITION LIMIT DEFAULTS =====================

// All limits are expressed in zeroed real joint degrees.
// jmin/jmax clip requested targets; jtol is the extra measured overshoot
// allowed before the firmware latches a fault.
static constexpr float JOINT_MIN_DEG_DEFAULT = -170.0f;
static constexpr float JOINT_MAX_DEG_DEFAULT = +170.0f;
static constexpr float JOINT_LIMIT_TOL_DEFAULT = 1.0f;
static constexpr float JOINT_LIMIT_STOP_MARGIN_DEG = 0.5f;


// ===================== PARK / MULTI-TURN REFERENCE =====================

static constexpr float PARK_ACCEL_DEG_S2 = 2.0f;
static constexpr float PARK_DEFAULT_VELOCITY_DEG_S = 0.5f;
static constexpr float PARK_DEFAULT_ENCODER_ANGLE_DEG = 0.0f;
static constexpr float PARK_DEFAULT_JOINT_POSITION_DEG = 0.0f;
static constexpr float PARK_ENCODER_TOLERANCE_DEG = 0.15f;
static constexpr uint32_t PARK_RELEASE_TIMEOUT_MS = 60000;
static constexpr uint32_t PARK_SEARCH_TIMEOUT_MS = 120000;
static constexpr uint32_t PARK_ALIGN_TIMEOUT_MS = 60000;

// GPIO3 can drive a standard 50 Hz hobby servo only in absolute-encoder mode
// (pkdir=0). Persisted pulse values are expressed directly in microseconds.
static constexpr uint32_t HOBBY_SERVO_ZERO_DEFAULT_US = 1500U;
static constexpr uint32_t HOBBY_SERVO_MIN_DEFAULT_US = 1000U;
static constexpr uint32_t HOBBY_SERVO_MAX_DEFAULT_US = 2000U;

// ===================== JOINTBUS HOME COMMAND =====================

// HOME is a JointBus command that moves the already-referenced joint to the
// logical zero position. It never runs the park procedure implicitly: if this
// firmware is configured for multi-turn park mode, HOME is accepted only after
// park has completed and jointReferenced is true.
static constexpr float JOINTBUS_HOME_TARGET_DEG = 0.0f;
static constexpr float JOINTBUS_HOME_VMAX_DEG_S = 8.0f;
static constexpr float JOINTBUS_HOME_AMAX_DEG_S2 = 15.0f;

// ===================== STDEG CALIBRATION =====================

// Calibration moves the joint by a known amount and estimates the number
// of motor microsteps per real joint degree. It takes exclusive control
// of the motor until the measurement is complete.
static constexpr float STDEG_CALIBRATION_TARGET_DEG = 30.0f;
static constexpr uint16_t STDEG_CALIBRATION_STEP_HIGH_US = 4;
static constexpr uint32_t STDEG_CALIBRATION_STEP_PERIOD_US = 1000;
static constexpr uint16_t STDEG_CALIBRATION_SETTLE_MS = 300;
static constexpr uint32_t STDEG_CALIBRATION_ENCODER_SAMPLE_EVERY_STEPS = 20;
static constexpr float STDEG_CALIBRATION_MIN_DELTA_DEG = 0.25f;

// ===================== TMC CONFIG =====================

Tmc2209Driver::Pins tmcPins {
    .step   = PIN_TMC_STEP,
    .dir    = PIN_TMC_DIR,
    .enn    = PIN_TMC_ENN,
    .uartRx = PIN_TMC_RX,
    .uartTx = PIN_TMC_TX
};

Tmc2209Driver::Config tmcConfig {
    .uartAddress = 0,
    .baud = TMC_BAUD,
    .fclkHz = 12000000.0f,

    .useInternalRsense = false,
    .useStealthChop = true,
    .interpolate256 = true,
    .microstepResolution = 16,

    .irun = 10,
    .ihold = 4,
    .iholdDelay = 4,
    .tpowerDown = 20,
    .vsenseLowRange = false
};

// ===================== OBJECTS =====================

#if MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5048A
SPIClass EncoderSPI(HSPI);
MagneticEncoder encoder(EncoderSPI, PIN_ENCODER_CS);
#elif MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5600
TwoWire EncoderI2C(0);
MagneticEncoder encoder(EncoderI2C, AS5600_I2C_ADDRESS);
#endif

HardwareSerial SerialJointBus(0);
HardwareSerial SerialTMC(2);

Tmc2209Driver tmc(SerialTMC, tmcPins, tmcConfig);
SCurvePosVelController jointCtrl;

PersistentParams params;
SerialConsole console(Serial, params);

class FirmwareJointPlannerRuntime final : public JointPlannerRuntime {
public:
  bool isReferenced() const override;
  MotionMode motionMode() const override;
  bool encoderReady() const override;
  bool driverEnabled() const override;
  bool positionCommandActive() const override;
  void stopMotion() override;
  bool ensureDriverEnabled() override;
  void latchDriverFault() override;
  bool clipTarget(float requestedDeg, float& clippedDeg) const override;
  float currentPositionDeg() const override;
  float controllerRefPositionDeg() const override;
  float controllerRefVelocityDegS() const override;
  void configureController(float vmaxDegS,
                           float amaxDegS2,
                           float outMaxDegS,
                           bool clearFault) override;
  void restartController(float currentDeg, float targetDeg) override;
  bool blendControllerTarget(float targetDeg) override;
  bool restartControllerTimed(float currentDeg,
                              float targetDeg,
                              float durationS) override;
  bool blendControllerTargetTimed(float targetDeg,
                                  float durationS) override;
  bool minimumCoordinatedDuration(float targetDeg,
                                  float vmaxDegS,
                                  float amaxDegS2,
                                  float& durationS) const override;
  void beginPositionMotion(float targetDeg) override;
};

static const JointPlannerConfig plannerConfig {
  .minVmaxDegS = PLANNER_MIN_VMAX_DEG_S,
  .maxVmaxDegS = PLANNER_MAX_VMAX_DEG_S,
  .minAmaxDegS2 = PLANNER_MIN_AMAX_DEG_S2,
  .maxAmaxDegS2 = PLANNER_MAX_AMAX_DEG_S2,
  .blendMinRefVelDegS = PLANNER_BLEND_MIN_REF_VEL_DEG_S
};

static FirmwareJointPlannerRuntime plannerRuntime;
JointPlanner planner(plannerRuntime, plannerConfig);

static JointBus::SlaveHooks jointBusHooks;
static JointBus::Slave jointBus(SerialJointBus, 0, jointBusHooks, PIN_RS485_RTS);

void printJointBusStats(bool reset)
{
  if (reset) {
    jointBus.resetDiagnostics();
    Serial.println("JointBus diagnostics reset");
    return;
  }
  Serial.printf(
    "jbus addr=%u baud=%lu rxbuf=%u rx=%lu tx=%lu ignored=%lu "
    "crc=%lu length=%lu version=%lu\r\n",
    static_cast<unsigned>(jointBus.address()),
    static_cast<unsigned long>(JOINTBUS_BAUD),
    static_cast<unsigned>(JOINTBUS_RX_BUFFER_SIZE),
    static_cast<unsigned long>(jointBus.rxFrames()),
    static_cast<unsigned long>(jointBus.txFrames()),
    static_cast<unsigned long>(jointBus.ignoredFrames()),
    static_cast<unsigned long>(jointBus.crcErrors()),
    static_cast<unsigned long>(jointBus.lengthErrors()),
    static_cast<unsigned long>(jointBus.versionErrors()));
  Serial.printf(
    "broadcast total=%lu start_rx=%lu start_ok=%lu start_reject=%lu "
    "last_seq=%u last_nack=0x%02X\r\n",
    static_cast<unsigned long>(jointBus.broadcastFrames()),
    static_cast<unsigned long>(jointBus.broadcastStartFrames()),
    static_cast<unsigned long>(jointBus.broadcastStartAccepted()),
    static_cast<unsigned long>(jointBus.broadcastStartRejected()),
    static_cast<unsigned>(jointBus.lastBroadcastStartSeq()),
    static_cast<unsigned>(jointBus.lastBroadcastStartNack()));
  Serial.printf(
    "uart errors=%lu buffer_full=%lu fifo_ovf=%lu frame=%lu parity=%lu break=%lu\r\n",
    static_cast<unsigned long>(jointBus.uartErrors()),
    static_cast<unsigned long>(jointBus.uartBufferFullErrors()),
    static_cast<unsigned long>(jointBus.uartFifoOverflowErrors()),
    static_cast<unsigned long>(jointBus.uartFrameErrors()),
    static_cast<unsigned long>(jointBus.uartParityErrors()),
    static_cast<unsigned long>(jointBus.uartBreakErrors()));
}

// ===================== STATE =====================

static constexpr uint32_t ENCODER_PRINT_PERIOD_MS = 250;
static constexpr uint32_t TEST_PERIOD_MS = 1;

uint32_t lastServoUs = 0;
static uint32_t lastTestMs = 0;
static uint32_t lastEncoderPrintMs = 0;

static uint16_t encoderRaw = 0;
static float encoderDeg = 0.0f;          // encoder angle modulo 360
static float jointDeg = 0.0f;            // real unrolled joint angle
static float encoderZero = 0.0f;         // real unrolled joint zero
bool encoderOk = false;
static bool traceEnabled = false;
static bool jointReferenced = false;

enum class ParkPhase : uint8_t {
  IDLE,
  RELEASE_SENSOR,
  SEARCH_FALLING_EDGE,
  ALIGN_ENCODER
};

static ParkPhase parkPhase = ParkPhase::IDLE;
static uint32_t parkStartedMs = 0;
static uint32_t parkPhaseStartedMs = 0;
static bool parkPrevSensorActive = false;
static float parkCommandVelocityDegS = 0.0f;

enum class TraceMode : uint8_t {
  FULL = 0,
  POS_TARGET = 1,
  POS_TARGET_VEL = 2,
  PID = 3,
  SCURVE = 4
};

static TraceMode traceMode = TraceMode::FULL;

static bool testEnabled = false;
static bool testStepEnabled = false;
static bool testForward = false;
static int32_t stepNum = 0;

float servoTargetZeroedDeg = 0.0f;
static float servoLastCmdDegS = 0.0f;
static MotionLifecycleState motionLifecycle;
bool tmcReady = false;

static bool jointBusLastLimitClipped = false;
static bool jointBusRebootPending = false;
static uint32_t jointBusRebootRequestedMs = 0;
static float activeMotorDirectionSign = DEFAULT_MOTOR_DIRECTION_SIGN;

struct HobbyServoState {
  bool attached = false;
  bool moving = false;
  uint32_t zeroUs = HOBBY_SERVO_ZERO_DEFAULT_US;
  uint32_t minUs = HOBBY_SERVO_MIN_DEFAULT_US;
  uint32_t maxUs = HOBBY_SERVO_MAX_DEFAULT_US;
  uint32_t currentUs = HOBBY_SERVO_ZERO_DEFAULT_US;
  uint32_t startUs = HOBBY_SERVO_ZERO_DEFAULT_US;
  uint32_t targetUs = HOBBY_SERVO_ZERO_DEFAULT_US;
  uint32_t startedMs = 0;
  uint32_t durationMs = 0;
};

static HobbyServoState hobbyServo;

struct JointBusPreparedSegment {
  bool valid = false;
  bool hold = false;
  uint8_t segmentId = JointBus::NO_SEGMENT_ID;
  int16_t targetCdeg = 0;
  uint16_t vmaxCdegS = 0;
  uint16_t amaxCdegS2 = 0;
};

static JointBusPreparedSegment jointBusPreparedSegment;
static bool jointBusActiveSegmentValid = false;
static uint8_t jointBusActiveSegmentId = JointBus::NO_SEGMENT_ID;
static uint8_t jointBusActiveMotionFlags = 0;
static bool jointBusActiveSegmentHold = false;
static uint32_t jointBusActiveSegmentStartedMs = 0;
static uint32_t jointBusActiveSegmentDurationMs = 0;

struct JointBusScheduledStart {
  bool valid = false;
  uint8_t segmentId = JointBus::NO_SEGMENT_ID;
  uint32_t durationMs = 0;
  uint32_t deadlineUs = 0;
};

static JointBusScheduledStart jointBusScheduledStart;

MotionMode motionMode = MotionMode::IDLE;

// Forward declarations for helpers used by parameter callbacks.
float jointGetPositionDeg();
void stopMotion();
static void latchEmergencyStopFault(const char* source);
static void jointBusClearCoordinatedSegments();
static void applyZeroOffsetFromParams(bool rebaseController);
static void parkUpdate(float dt);
static void abortPark(const char* reason);
static void setupJointBusHooks();
static uint8_t readJointBusAddressFromParams();
static bool applyMotorDirectionFromParams(bool stopBeforeApply);
static bool applyEncoderDirectionFromParams(bool preserveZeroedPosition);
static bool applyHobbyServoParams();
static void hobbyServoUpdate(uint32_t nowMs);
static void clearPositionLifecycleState();
bool setMotorVelocityDegPerSecond(float degreesPerSecond);
bool startPark();
FaultClearResult clearMotionFault();
HobbyServoMotion::MoveResult moveHobbyServo(uint16_t position, uint8_t speed);

// ===================== PARAMS =====================

static void clearPositionLifecycleState()
{
  motionLifecycle.clear();
}

static bool readParamU8(const char* key, uint8_t& out)
{
  float value = 0.0f;

  if (!params.get(key, value)) {
    LOG_ERR("Parameter not found: %s\r\n", key);
    return false;
  }

  if (value < 0.0f || value > 255.0f) {
    LOG_ERR("Parameter out of uint8 range: %s=%.6f\r\n", key, value);
    return false;
  }

  out = static_cast<uint8_t>(value);
  return true;
}

static bool readParamU16(const char* key, uint16_t& out)
{
  float value = 0.0f;

  if (!params.get(key, value)) {
    LOG_ERR("Parameter not found: %s\r\n", key);
    return false;
  }

  if (value < 0.0f || value > 65535.0f) {
    LOG_ERR("Parameter out of uint16 range: %s=%.6f\r\n", key, value);
    return false;
  }

  out = static_cast<uint16_t>(value);
  return true;
}

static float readParamFloatOrDefault(const char* key, float fallback)
{
  float value = fallback;
  if (!params.get(key, value) || !isfinite(value)) {
    return fallback;
  }
  return value;
}

float readParamMinOrDefault(const char* key, float fallback, float minValue)
{
  const float value = readParamFloatOrDefault(key, fallback);

  if (!isfinite(value) || value < minValue) {
    LOG_NFO("Invalid parameter %s=%.6f, using default %.6f\r\n", key, value, fallback);
    return fallback;
  }

  return value;
}

static float stepsPerDegree()
{
  float value = readParamFloatOrDefault("stdeg", 100.0f);
  if (!isfinite(value) || value <= 0.0f) {
    value = 100.0f;
  }
  return value;
}

static bool isValidDirectionSign(float value)
{
  return isfinite(value) && (value == -1.0f || value == 1.0f);
}

static bool applyMotorDirectionFromParams(bool stopBeforeApply)
{
  float requested = DEFAULT_MOTOR_DIRECTION_SIGN;
  if (!params.get("mdir", requested) || !isValidDirectionSign(requested)) {
    params.set("mdir", activeMotorDirectionSign);
    LOG_ERR("Invalid mdir rejected: use -1 or +1; keeping %+.0f\r\n",
            activeMotorDirectionSign);
    return false;
  }

  const bool changed = requested != activeMotorDirectionSign;
  if (stopBeforeApply && changed) {
    stopMotion();
  }
  activeMotorDirectionSign = requested;
  LOG_NFO("Motor direction sign mdir=%+.0f%s\r\n",
          activeMotorDirectionSign,
          stopBeforeApply && changed ? "; motion stopped" : "");
  return true;
}

static bool motorHoldEnabled()
{
  const float value = readParamFloatOrDefault("mhold", 0.0f);
  return isfinite(value) && value >= 0.5f;
}

static bool servoHoldEnabled()
{
  const float value = readParamFloatOrDefault("shold", 0.0f);
  return isfinite(value) && value >= 0.5f;
}

static float jointDegreesPerEncoderRevolution()
{
  float value = readParamFloatOrDefault("jrev", DEFAULT_JOINT_DEGREES_PER_ENCODER_REV);

  if (!isfinite(value) || value <= 0.0f) {
    value = DEFAULT_JOINT_DEGREES_PER_ENCODER_REV;
  }

  return value;
}

static bool readJointLimitParams(float& jmin, float& jmax, float& jtol)
{
  jmin = readParamFloatOrDefault("jmin", JOINT_MIN_DEG_DEFAULT);
  jmax = readParamFloatOrDefault("jmax", JOINT_MAX_DEG_DEFAULT);
  jtol = readParamFloatOrDefault("jtol", JOINT_LIMIT_TOL_DEFAULT);

  if (!isfinite(jmin) || !isfinite(jmax) || !isfinite(jtol)) {
    LOG_ERR("Invalid joint limit parameter: non-finite value\r\n");
    return false;
  }

  if (jmax <= jmin) {
    LOG_ERR("Invalid joint limits: jmin=%.3f must be lower than jmax=%.3f\r\n", jmin, jmax);
    return false;
  }

  if (jtol < 0.0f) {
    LOG_ERR("Invalid joint limit tolerance: jtol=%.3f must be >= 0\r\n", jtol);
    return false;
  }

  return true;
}

static bool isJointPositionOutsideFaultWindow(float zeroedDeg)
{
  float jmin = 0.0f;
  float jmax = 0.0f;
  float jtol = 0.0f;

  if (!readJointLimitParams(jmin, jmax, jtol)) {
    return true;
  }

  return zeroedDeg < (jmin - jtol) || zeroedDeg > (jmax + jtol);
}

static void latchPlannerConfigurationFault(const char* reason)
{
  clearPositionLifecycleState();

  if (tmcReady) {
    tmc.stopInternalMotion();
    tmc.disableDriver(false);
  }

  // Keep the controller in a numerically safe envelope while retaining the
  // BadLimits code that JointBus exposes as PlannerError.
  jointCtrl.setPositionLimits(JOINT_MIN_DEG_DEFAULT,
                              JOINT_MAX_DEG_DEFAULT,
                              JOINT_LIMIT_STOP_MARGIN_DEG,
                              JOINT_LIMIT_TOL_DEFAULT);
  const float measured = jointGetPositionDeg();
  jointCtrl.latchPlannerFault(isfinite(measured) ? measured : 0.0f);
  motionMode = MotionMode::FAULT;
  wsSetState(LedState::FAULT);
  LOG_ERR("Planner configuration fault: %s\r\n",
          reason != nullptr ? reason : "invalid joint limits");
}

bool jointClipTargetToLimits(float requestedDeg, float& clippedDeg)
{
  float jmin = 0.0f;
  float jmax = 0.0f;
  float jtol = 0.0f;

  if (!readJointLimitParams(jmin, jmax, jtol)) {
    latchPlannerConfigurationFault("invalid jmin/jmax/jtol");
    clippedDeg = requestedDeg;
    return false;
  }

  clippedDeg = constrain(requestedDeg, jmin, jmax);

  if (clippedDeg != requestedDeg) {
    LOG_NFO("WARNING: requested target %.3f deg clipped to %.3f deg by limits [%.3f, %.3f]\r\n",
            requestedDeg, clippedDeg, jmin, jmax);
  }

  return true;
}

static bool applyJointLimitsFromParams(bool announce)
{
  float jmin = 0.0f;
  float jmax = 0.0f;
  float jtol = 0.0f;

  if (!readJointLimitParams(jmin, jmax, jtol)) {
    latchPlannerConfigurationFault("invalid jmin/jmax/jtol");
    LOG_ERR("Joint limit configuration invalid: motion fault latched\r\n");
    return false;
  }

  jointCtrl.setPositionLimits(jmin, jmax, JOINT_LIMIT_STOP_MARGIN_DEG, jtol);

  if (announce) {
    LOG_NFO("Joint limits: jmin=%.3f jmax=%.3f jtol=%.3f deg\r\n", jmin, jmax, jtol);
  }

  if (jointReferenced && encoderOk && isJointPositionOutsideFaultWindow(jointGetPositionDeg())) {
    clearPositionLifecycleState();
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    LOG_ERR("Current joint position %.3f deg is outside limits [%.3f, %.3f] with jtol=%.3f\r\n",
            jointGetPositionDeg(), jmin, jmax, jtol);
    return false;
  }

  return true;
}

static void applyEncoderScaleFromParams(bool preserveZeroedPosition)
{
  const float oldZeroed = jointGetPositionDeg();
  const float jrev = jointDegreesPerEncoderRevolution();

  encoder.setOutputDegreesPerEncoderRevolution(jrev);

  // The driver immediately recomputes its last continuous value using the new
  // scale, including the configured encoder direction sign.
  jointDeg = encoder.lastContinuousDegrees();
  encoderDeg = encoder.lastDegrees();

  if (preserveZeroedPosition) {
    encoderZero = jointDeg - oldZeroed;
    if (motionMode != MotionMode::FAULT && !jointCtrl.fault()) {
      jointCtrl.reset(oldZeroed);
      jointCtrl.setTarget(oldZeroed);
      servoTargetZeroedDeg = oldZeroed;
    }
  }

  LOG_NFO("Encoder scale: 360.000 encoder deg = %.6f joint deg\r\n",
          encoder.outputDegreesPerEncoderRevolution());
}

static bool applyEncoderDirectionFromParams(bool preserveZeroedPosition)
{
  float requested = DEFAULT_ENCODER_DIRECTION_SIGN;
  if (!params.get("edir", requested) || !isValidDirectionSign(requested)) {
    params.set("edir", encoder.directionSign());
    LOG_ERR("Invalid edir rejected: use -1 or +1; keeping %+.0f\r\n",
            encoder.directionSign());
    return false;
  }

  const float oldZeroed = jointGetPositionDeg();
  const bool changed = requested != encoder.directionSign();
  if (preserveZeroedPosition && changed) {
    stopMotion();
  }

  if (!encoder.setDirectionSign(requested)) {
    return false;
  }

  if (encoderOk) {
    jointDeg = encoder.lastContinuousDegrees();
    encoderDeg = encoder.lastDegrees();
  }

  if (preserveZeroedPosition && changed && encoderOk) {
    encoderZero = jointDeg - oldZeroed;
    params.set("zoff", encoderZero);
    if (motionMode != MotionMode::FAULT && !jointCtrl.fault()) {
      jointCtrl.reset(oldZeroed);
      jointCtrl.setTarget(oldZeroed);
      servoTargetZeroedDeg = oldZeroed;
    }
  }

  LOG_NFO("Encoder direction sign edir=%+.0f%s\r\n",
          encoder.directionSign(),
          preserveZeroedPosition && changed && encoderOk
            ? "; motion stopped, logical position preserved, zoff updated in RAM"
            : "");
  return true;
}

static void applyLogLevelFromParams(bool announce)
{
  float value = 2.0f;

  if (!params.get("loglvl", value) || !isfinite(value)) {
    value = 2.0f;
  }

  int level = static_cast<int>(value);

  if (level < 0) {
    level = 0;
  } else if (level > 3) {
    level = 3;
  }

  Logger::setLevel(static_cast<uint8_t>(level));

  if (announce) {
    LOG_NFO("loglvl=%d (0=OFF, 1=ERR, 2=NFO, 3=DBG)\r\n", level);
  }
}

static bool readHobbyServoPulseParam(const char* key, uint32_t& out)
{
  float value = 0.0f;
  if (!params.get(key, value) || !isfinite(value) || value < 1.0f ||
      value >= static_cast<float>(HobbyServoMotion::PWM_PERIOD_US) ||
      floorf(value) != value) {
    return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

static void detachHobbyServo()
{
  if (hobbyServo.attached) {
    ledcDetach(PIN_PARK_SENSOR);
  }
  hobbyServo.attached = false;
  hobbyServo.moving = false;
  pinMode(PIN_PARK_SENSOR, INPUT_PULLUP);
}

static bool writeHobbyServoPulse(uint32_t pulseUs)
{
  return hobbyServo.attached &&
         ledcWrite(PIN_PARK_SENSOR, HobbyServoMotion::pulseUsToDuty(pulseUs));
}

static bool applyHobbyServoParams()
{
  float enabledValue = 0.0f;
  const float parkDirection = readParamFloatOrDefault("pkdir", 0.0f);
  if (!params.get("servo", enabledValue) || !isfinite(enabledValue) ||
      (enabledValue != 0.0f && enabledValue != 1.0f)) {
    params.set("servo", 0.0f);
    detachHobbyServo();
    LOG_ERR("Invalid servo value rejected: use 0 or 1\r\n");
    return false;
  }

  if (enabledValue == 0.0f) {
    detachHobbyServo();
    LOG_NFO("GPIO3 hobby servo output disabled\r\n");
    return true;
  }

  if (parkDirection != 0.0f) {
    params.set("servo", 0.0f);
    detachHobbyServo();
    LOG_ERR("Hobby servo rejected: servo=1 requires pkdir=0; servo reset to 0 in RAM\r\n");
    return false;
  }

  uint32_t zeroUs = 0;
  uint32_t minUs = 0;
  uint32_t maxUs = 0;
  if (!readHobbyServoPulseParam("srvzero", zeroUs) ||
      !readHobbyServoPulseParam("srvmin", minUs) ||
      !readHobbyServoPulseParam("srvmax", maxUs) ||
      !HobbyServoMotion::validConfig(zeroUs, minUs, maxUs)) {
    params.set("servo", 0.0f);
    detachHobbyServo();
    LOG_ERR("Hobby servo rejected: require integer microseconds with 0 < srvmin <= srvzero <= srvmax < %lu; servo reset to 0 in RAM\r\n",
            static_cast<unsigned long>(HobbyServoMotion::PWM_PERIOD_US));
    return false;
  }

  const bool firstAttach = !hobbyServo.attached;
  hobbyServo.zeroUs = zeroUs;
  hobbyServo.minUs = minUs;
  hobbyServo.maxUs = maxUs;

  if (firstAttach) {
    pinMode(PIN_PARK_SENSOR, OUTPUT);
    if (!ledcAttach(PIN_PARK_SENSOR,
                    HobbyServoMotion::PWM_FREQUENCY_HZ,
                    HobbyServoMotion::PWM_RESOLUTION_BITS)) {
      params.set("servo", 0.0f);
      detachHobbyServo();
      LOG_ERR("Unable to attach hobby servo PWM to GPIO3; servo reset to 0 in RAM\r\n");
      return false;
    }
    hobbyServo.attached = true;
    hobbyServo.currentUs = zeroUs;
    hobbyServo.startUs = zeroUs;
    hobbyServo.targetUs = zeroUs;
    hobbyServo.moving = false;
  } else {
    hobbyServo.currentUs = std::min(std::max(hobbyServo.currentUs, minUs), maxUs);
    hobbyServo.startUs = hobbyServo.currentUs;
    hobbyServo.targetUs = hobbyServo.currentUs;
    hobbyServo.moving = false;
  }

  if (!writeHobbyServoPulse(hobbyServo.currentUs)) {
    params.set("servo", 0.0f);
    detachHobbyServo();
    LOG_ERR("Unable to write hobby servo PWM; servo reset to 0 in RAM\r\n");
    return false;
  }

  LOG_NFO("GPIO3 hobby servo enabled: pulse=%lu us, srvmin=%lu srvzero=%lu srvmax=%lu\r\n",
          static_cast<unsigned long>(hobbyServo.currentUs),
          static_cast<unsigned long>(minUs),
          static_cast<unsigned long>(zeroUs),
          static_cast<unsigned long>(maxUs));
  return true;
}

HobbyServoMotion::MoveResult moveHobbyServo(uint16_t position, uint8_t speed)
{
  if (position > 999U || speed < 1U || speed > 10U) {
    return HobbyServoMotion::MoveResult::InvalidArgument;
  }
  if (!hobbyServo.attached) {
    return HobbyServoMotion::MoveResult::Disabled;
  }
  if (!HobbyServoMotion::validConfig(
          hobbyServo.zeroUs, hobbyServo.minUs, hobbyServo.maxUs)) {
    return HobbyServoMotion::MoveResult::InvalidConfig;
  }

  const uint32_t targetUs = HobbyServoMotion::positionToPulseUs(
      position, hobbyServo.minUs, hobbyServo.maxUs);
  const uint32_t durationMs = HobbyServoMotion::moveDurationMs(
      hobbyServo.currentUs,
      targetUs,
      hobbyServo.minUs,
      hobbyServo.maxUs,
      speed);

  hobbyServo.startUs = hobbyServo.currentUs;
  hobbyServo.targetUs = targetUs;
  hobbyServo.startedMs = millis();
  hobbyServo.durationMs = durationMs;
  hobbyServo.moving = durationMs != 0U;

  if (!hobbyServo.moving) {
    hobbyServo.currentUs = targetUs;
    if (!writeHobbyServoPulse(targetUs)) {
      params.set("servo", 0.0f);
      detachHobbyServo();
      return HobbyServoMotion::MoveResult::PwmError;
    }
  }

  LOG_NFO("Hobby servo move accepted: position=%u speed=%u target=%lu us duration=%lu ms\r\n",
          static_cast<unsigned>(position),
          static_cast<unsigned>(speed),
          static_cast<unsigned long>(targetUs),
          static_cast<unsigned long>(durationMs));
  return HobbyServoMotion::MoveResult::Accepted;
}

static void hobbyServoUpdate(uint32_t nowMs)
{
  if (!hobbyServo.attached || !hobbyServo.moving) {
    return;
  }

  const uint32_t elapsedMs = HobbyServoMotion::elapsedMsSince(
      nowMs, hobbyServo.startedMs);
  const uint32_t pulseUs = HobbyServoMotion::interpolatePulseUs(
      hobbyServo.startUs, hobbyServo.targetUs, elapsedMs, hobbyServo.durationMs);
  if (pulseUs != hobbyServo.currentUs) {
    if (!writeHobbyServoPulse(pulseUs)) {
      LOG_ERR("Hobby servo PWM update failed; disabling GPIO3 output\r\n");
      params.set("servo", 0.0f);
      detachHobbyServo();
      return;
    }
    hobbyServo.currentUs = pulseUs;
  }
  if (elapsedMs >= hobbyServo.durationMs) {
    hobbyServo.currentUs = hobbyServo.targetUs;
    hobbyServo.moving = false;
  }
}

void paramsInit()
{
  if (!params.begin()) {
    LOG_ERR("Failed to initialize parameters storage\r\n");
    return;
  }

  // Motor / driver parameters
  params.initKey("ustep", 16.0f);
  params.initKey("irun", 10.0f);
  params.initKey("ihold", 4.0f);
  params.initKey("stdeg", 100.0f); // motor microsteps per real joint degree
  params.initKey("jrev", DEFAULT_JOINT_DEGREES_PER_ENCODER_REV); // real joint degrees per encoder revolution
  params.initKey("mdir", DEFAULT_MOTOR_DIRECTION_SIGN); // motor command direction, -1 or +1
  params.initKey("edir", DEFAULT_ENCODER_DIRECTION_SIGN); // encoder angle direction, -1 or +1
  params.initKey("loglvl", 2.0f); // 0=OFF, 1=ERR, 2=NFO, 3=DBG
  params.initKey("mhold", 0.0f); // 0=disable driver at target, 1=keep driver enabled at IHOLD
  params.initKey("shold", 0.0f); // 0=stop control at target, 1=active servo hold at target
  params.initKey("zoff", 0.0f); // persistent absolute encoder position used as logical joint zero
  params.initKey("pkdir", 0.0f); // 0=absolute encoder, +1/-1=park search direction
  params.initKey("pkvel", PARK_DEFAULT_VELOCITY_DEG_S); // park speed in joint deg/s
  params.initKey("pkenc", PARK_DEFAULT_ENCODER_ANGLE_DEG); // final encoder modulo angle in deg
  params.initKey("pkpos", PARK_DEFAULT_JOINT_POSITION_DEG); // known logical joint position at park
  params.initKey("servo", 0.0f); // 1=GPIO3 hobby servo output; allowed only with pkdir=0
  params.initKey("srvzero", static_cast<float>(HOBBY_SERVO_ZERO_DEFAULT_US)); // rest pulse, microseconds
  params.initKey("srvmin", static_cast<float>(HOBBY_SERVO_MIN_DEFAULT_US)); // minimum pulse, microseconds
  params.initKey("srvmax", static_cast<float>(HOBBY_SERVO_MAX_DEFAULT_US)); // maximum pulse, microseconds

  // Joint safety limits in zeroed real joint degrees.
  params.initKey("jmin", JOINT_MIN_DEG_DEFAULT);
  params.initKey("jmax", JOINT_MAX_DEG_DEFAULT);
  params.initKey("jtol", JOINT_LIMIT_TOL_DEFAULT); // allowed measured overshoot beyond jmin/jmax before fault

  // PID + analytic quintic trajectory controller parameters.
  // Key names are intentionally short because PersistentParams allows max 7 chars.
  params.initKey("kp", SERVO_KP);
  params.initKey("ki", SERVO_KI);
  params.initKey("kd", SERVO_KD);
  params.initKey("ffv", SERVO_FF_VEL);
  params.initKey("ilim", SERVO_I_LIMIT);

  params.initKey("vmax", SERVO_VMAX_DEG_S);
  params.initKey("amax", SERVO_AMAX_DEG_S2);
  params.initKey("outmax", SERVO_OUTPUT_MAX_DEG_S);

  params.initKey("ptol", SERVO_POS_TOL_DEG);
  params.initKey("vtol", SERVO_VEL_TOL_DEG_S);
  params.initKey("dbent", SERVO_DEADBAND_ENTER);
  params.initKey("dbext", SERVO_DEADBAND_EXIT);
  params.initKey("dbvel", SERVO_DEADBAND_VEL);
  params.initKey("vtau", SERVO_VEL_FILTER_TAU_S);

  params.initKey("addr", 0.0f); // JointBus slave address, 0..15

  params.load();
  applyLogLevelFromParams(false);
  applyZeroOffsetFromParams(false);
  applyHobbyServoParams();

  LOG_DBG("Loaded parameters:\r\n");
  for (uint8_t i = 0; i < params.count(); i++) {
    const char* key = nullptr;
    float value = 0.0f;

    if (params.getByIndex(i, key, value)) {
      LOG_DBG("%s = %.6f\r\n", key, value);
    }
  }
}

static void applyCurrentScaleFromParams()
{
  uint8_t irun = 0;
  uint8_t ihold = 0;

  if (!readParamU8("irun", irun) || !readParamU8("ihold", ihold)) {
    LOG_ERR("Current scale update skipped\r\n");
    return;
  }

  tmc.setCurrentScale(irun, ihold, tmcConfig.iholdDelay);
  LOG_NFO("Updated TMC2209 current scale: IRUN=%u, IHOLD=%u\r\n", irun, ihold);
}

static void applyMicrostepResolutionFromParams()
{
  uint16_t ustep = 0;

  if (!readParamU16("ustep", ustep)) {
    LOG_ERR("Microstep resolution update skipped\r\n");
    return;
  }

  if (!tmc.setMicrostepResolution(ustep)) {
    LOG_ERR("Invalid TMC2209 microstep resolution: %u\r\n", ustep);
    return;
  }

  LOG_NFO("Updated TMC2209 microstep resolution: USTEP=%u\r\n", ustep);
}

static void applyControllerMotionParamsFromParams(bool replanActive)
{
  const float vmax = readParamMinOrDefault("vmax", SERVO_VMAX_DEG_S, 0.001f);
  const float amax = readParamMinOrDefault("amax", SERVO_AMAX_DEG_S2, 0.001f);
  const float outmax = readParamMinOrDefault("outmax", SERVO_OUTPUT_MAX_DEG_S, 0.0f);

  jointCtrl.setLimits(vmax, amax);
  jointCtrl.setOutputMax(outmax);

  LOG_NFO("Servo motion params: vmax=%.4f amax=%.4f outmax=%.4f\r\n",
          vmax, amax, outmax);

  if (replanActive && jointCtrl.trajectoryActive() &&
      !jointCtrl.replanActiveTrajectory()) {
    LOG_ERR("Active quintic trajectory cannot satisfy updated motion limits; stopping safely\r\n");
    stopMotion();
  }
}

static void applyControllerGainsFromParams()
{
  const float kp = readParamMinOrDefault("kp", SERVO_KP, 0.0f);
  const float ki = readParamMinOrDefault("ki", SERVO_KI, 0.0f);
  const float kd = readParamMinOrDefault("kd", SERVO_KD, 0.0f);
  const float ffv = readParamMinOrDefault("ffv", SERVO_FF_VEL, 0.0f);
  const float ilim = readParamMinOrDefault("ilim", SERVO_I_LIMIT, 0.0f);

  jointCtrl.setGains(kp, ki, kd, ffv);
  jointCtrl.setIntegratorLimit(ilim);

  LOG_NFO("PID params: kp=%.4f ki=%.4f kd=%.4f ffv=%.4f ilim=%.4f\r\n",
                kp, ki, kd, ffv, ilim);
}

static void applyControllerSettlingParamsFromParams()
{
  const float ptol = readParamMinOrDefault("ptol", SERVO_POS_TOL_DEG, 0.0f);
  const float vtol = readParamMinOrDefault("vtol", SERVO_VEL_TOL_DEG_S, 0.0f);
  const float dbent = readParamMinOrDefault("dbent", SERVO_DEADBAND_ENTER, 0.0f);
  const float dbext = readParamMinOrDefault("dbext", SERVO_DEADBAND_EXIT, 0.0f);
  const float dbvel = readParamMinOrDefault("dbvel", SERVO_DEADBAND_VEL, 0.0f);
  const float vtau = readParamMinOrDefault("vtau", SERVO_VEL_FILTER_TAU_S, 0.0f);

  jointCtrl.setTolerances(ptol, vtol);
  jointCtrl.setDeadband(dbent, dbext, dbvel);
  jointCtrl.setVelocityFilterTau(vtau);

  LOG_NFO("Servo settling params: ptol=%.4f vtol=%.4f dbent=%.4f dbext=%.4f dbvel=%.4f vtau=%.4f\r\n",
                ptol, vtol, dbent, dbext, dbvel, vtau);
}

static void applyControllerParamsFromParams()
{
  applyControllerMotionParamsFromParams(false);
  applyControllerGainsFromParams();
  applyControllerSettlingParamsFromParams();
}

static void applyZeroOffsetFromParams(bool rebaseController)
{
  float value = 0.0f;
  if (!params.get("zoff", value) || !isfinite(value)) {
    value = 0.0f;
    params.set("zoff", value);
    LOG_ERR("Invalid zoff value replaced with 0.000000 deg\r\n");
  }

  encoderZero = value;

  if (rebaseController && encoderOk) {
    stopMotion();
    const float currentZeroedDeg = jointGetPositionDeg();
    if (motionMode != MotionMode::FAULT && !jointCtrl.fault()) {
      jointCtrl.reset(currentZeroedDeg);
      jointCtrl.setTarget(currentZeroedDeg);
      servoTargetZeroedDeg = currentZeroedDeg;
    }
  }

  LOG_NFO("Logical zero offset zoff=%.6f deg%s\r\n",
          encoderZero,
          rebaseController ? "; target rebased to current position" : "");
}

void applyAllParams()
{
  applyMotorDirectionFromParams(false);
  applyEncoderDirectionFromParams(false);
  applyCurrentScaleFromParams();
  applyMicrostepResolutionFromParams();
  applyControllerParamsFromParams();
  applyJointLimitsFromParams(true);
  applyZeroOffsetFromParams(encoderOk);
}

void onConsoleParamSet(const char* key)
{
  if (key == nullptr) {
    return;
  }

  if (strcmp(key, "irun") == 0 || strcmp(key, "ihold") == 0) {
    applyCurrentScaleFromParams();
    return;
  }

  if (strcmp(key, "ustep") == 0) {
    applyMicrostepResolutionFromParams();
    return;
  }

  if (strcmp(key, "kp") == 0 || strcmp(key, "ki") == 0 ||
      strcmp(key, "kd") == 0 || strcmp(key, "ffv") == 0 ||
      strcmp(key, "ilim") == 0) {
    applyControllerGainsFromParams();
    return;
  }

  if (strcmp(key, "vmax") == 0 || strcmp(key, "amax") == 0) {
    applyControllerMotionParamsFromParams(true);
    return;
  }

  if (strcmp(key, "outmax") == 0) {
    applyControllerMotionParamsFromParams(false);
    return;
  }

  if (strcmp(key, "ptol") == 0 || strcmp(key, "vtol") == 0 ||
      strcmp(key, "dbent") == 0 || strcmp(key, "dbext") == 0 ||
      strcmp(key, "dbvel") == 0 || strcmp(key, "vtau") == 0) {
    applyControllerSettlingParamsFromParams();
    return;
  }

  if (strcmp(key, "loglvl") == 0) {
    applyLogLevelFromParams(true);
    return;
  }

  if (strcmp(key, "mhold") == 0) {
    LOG_NFO("motor hold at target %s\r\n", motorHoldEnabled() ? "enabled" : "disabled");
    return;
  }

  if (strcmp(key, "mdir") == 0) {
    if (applyMotorDirectionFromParams(true)) {
      LOG_NFO("Use 'save' to persist the motor direction sign.\r\n");
    }
    return;
  }

  if (strcmp(key, "edir") == 0) {
    if (applyEncoderDirectionFromParams(true)) {
      LOG_NFO("Use 'save' to persist edir and the rebased zoff.\r\n");
    }
    return;
  }

  if (strcmp(key, "shold") == 0) {
    if (!servoHoldEnabled()) {
      const bool completedHold =
        !motionLifecycle.positionCommandActive &&
        motionLifecycle.servoHoldActive &&
        motionMode == MotionMode::POSITION;

      if (completedHold) {
        clearPositionLifecycleState();
      } else {
        motionLifecycle.disableServoHold();
      }

      if (completedHold) {
        setMotorVelocityDegPerSecond(0.0f);
        tmc.stopInternalMotion();
        servoLastCmdDegS = 0.0f;
        if (!motorHoldEnabled()) {
          tmc.disableDriver(false);
        }
        motionMode = MotionMode::IDLE;
        wsSetState(jointReferenced ? LedState::READY : LedState::BOOT);
      }
    }
    LOG_NFO("servo hold at target %s. Use 'save' to persist it.\r\n",
            servoHoldEnabled() ? "enabled" : "disabled");
    return;
  }

  if (strcmp(key, "zoff") == 0) {
    applyZeroOffsetFromParams(true);
    LOG_NFO("Use 'save' to persist the logical zero offset.\r\n");
    return;
  }

  if (strcmp(key, "servo") == 0 || strcmp(key, "srvzero") == 0 ||
      strcmp(key, "srvmin") == 0 || strcmp(key, "srvmax") == 0) {
    applyHobbyServoParams();
    LOG_NFO("Use 'save' to persist hobby servo parameters.\r\n");
    return;
  }

  if (strcmp(key, "pkdir") == 0) {
    const float directionValue = readParamFloatOrDefault("pkdir", 0.0f);
    stopMotion();
    if (directionValue == 0.0f) {
      jointReferenced = true;
      const float currentPosition = jointGetPositionDeg();
      if (motionMode != MotionMode::FAULT && !jointCtrl.fault()) {
        jointCtrl.reset(currentPosition);
        jointCtrl.setTarget(currentPosition);
        servoTargetZeroedDeg = currentPosition;
        wsSetState(LedState::READY);
      }
      LOG_NFO("pkdir=0: absolute-encoder mode enabled; park is not required. Use 'save' to persist it.\r\n");
    } else if (directionValue == 1.0f || directionValue == -1.0f) {
      jointReferenced = false;
      wsSetState(LedState::BOOT);
      LOG_NFO("pkdir=%+.0f: multi-turn park mode enabled; execute park before motion. Use 'save' to persist it.\r\n",
              directionValue);
    } else {
      LOG_ERR("Invalid pkdir: use -1, 0, or +1\r\n");
    }
    applyHobbyServoParams();
    return;
  }

  if (strcmp(key, "pkvel") == 0 || strcmp(key, "pkenc") == 0 ||
      strcmp(key, "pkpos") == 0) {
    LOG_NFO("Park parameter %s updated in RAM. Use 'save' to persist it.\r\n", key);
    return;
  }

  if (strcmp(key, "jmin") == 0 || strcmp(key, "jmax") == 0 || strcmp(key, "jtol") == 0) {
    applyJointLimitsFromParams(true);
    LOG_NFO("Use 'save' to persist joint limit changes.\r\n");
    return;
  }

  if (strcmp(key, "jrev") == 0) {
    float requested = 0.0f;
    if (!params.get("jrev", requested) || !isfinite(requested) || requested <= 0.0f) {
      params.set("jrev", encoder.outputDegreesPerEncoderRevolution());
      LOG_ERR("Invalid jrev value rejected. It must be > 0. Current jrev remains %.6f\r\n",
              encoder.outputDegreesPerEncoderRevolution());
      return;
    }

    stopMotion();
    applyEncoderScaleFromParams(true);
    LOG_NFO("jrev now %.6f joint deg / encoder rev. Use 'save' to persist it.\r\n",
            encoder.outputDegreesPerEncoderRevolution());
    return;
  }

  if (strcmp(key, "stdeg") == 0) {
    LOG_NFO("steps/degree now %.6f microsteps/deg\r\n", stepsPerDegree());
    return;
  }

  if (strcmp(key, "addr") == 0) {
    const uint8_t newAddress = readJointBusAddressFromParams();
    jointBus.setAddress(newAddress);
    LOG_NFO("JointBus address set to %u. Use 'save' to persist it.\r\n", static_cast<unsigned>(newAddress));
    return;
  }

  LOG_DBG("No runtime action defined for key: %s\r\n", key);
}

// ===================== ENCODER / TMC INIT =====================

void encoderInit()
{
#if MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5048A
  EncoderSPI.begin(PIN_ENCODER_SCK, PIN_ENCODER_MISO, PIN_ENCODER_MOSI, PIN_ENCODER_CS);
  LOG_NFO("HSPI initialized for AS5048A\r\n");
#elif MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5600
  EncoderI2C.begin(PIN_ENCODER_MISO, PIN_ENCODER_MOSI, AS5600_I2C_CLOCK_HZ);
  LOG_NFO("I2C initialized for AS5600: SDA=%d SCL=%d clock=%lu Hz address=0x%02X\r\n",
          PIN_ENCODER_MISO,
          PIN_ENCODER_MOSI,
          static_cast<unsigned long>(AS5600_I2C_CLOCK_HZ),
          AS5600_I2C_ADDRESS);
#endif

  encoder.begin();
  encoder.setOutputDegreesPerEncoderRevolution(jointDegreesPerEncoderRevolution());
  applyEncoderDirectionFromParams(false);

  LOG_NFO("Encoder scale: 360.000 encoder deg = %.6f joint deg\r\n",
          encoder.outputDegreesPerEncoderRevolution());
}

bool encoderFirstReadTest()
{
  encoderOk = encoder.readContinuousDegrees(jointDeg);
  encoderRaw = encoder.lastRaw();
  encoderDeg = encoder.lastDegrees();

  if (encoderOk) {
#if MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5048A
    const char* encoderName = "AS5048A";
#else
    const char* encoderName = "AS5600";
#endif
    LOG_NFO("%s first read raw14=%u enc_angle=%.3f deg joint_unrolled=%.3f deg\r\n",
            encoderName,
            encoderRaw,
            encoderDeg,
            jointDeg);
    return true;
  }

#if MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5048A
  LOG_ERR("AS5048A first read failed\r\n");
#else
  LOG_ERR("AS5600 first read failed\r\n");
#endif
  return false;
}

bool tmcInit()
{
  LOG_NFO("TMC2209 minimal init test\r\n");

  LOG_NFO("Starting communication-only mode...\r\n");
  if (!tmc.beginCommunicationOnly()) {
    LOG_ERR("beginCommunicationOnly() failed\r\n");
    return false;
  }

  LOG_NFO("Communication-only mode started\r\n");

  LOG_NFO("Probing TMC2209...\r\n");
  if (!tmc.probe()) {
    LOG_ERR("TMC2209 probe failed\r\n");
    return false;
  }

  LOG_NFO("TMC2209 detected\r\n");

  uint32_t ioin = 0;
  if (tmc.readIoin(ioin)) {
    LOG_DBG("IOIN = 0x%08lX\r\n", static_cast<unsigned long>(ioin));
  } else {
    LOG_ERR("unable to read IOIN\r\n");
  }

  uint32_t gstat = 0;
  if (tmc.readGstat(gstat)) {
    LOG_DBG("GSTAT = 0x%08lX\r\n", static_cast<unsigned long>(gstat));
  } else {
    LOG_ERR("unable to read GSTAT\r\n");
  }

  LOG_NFO("Configuring driver registers...\r\n");
  if (!tmc.configure()) {
    LOG_ERR("configure() failed\r\n");
    return false;
  }

  LOG_NFO("Driver configured, power stage still disabled\r\n");

  uint32_t ifcnt = 0;
  if (tmc.readIfcnt(ifcnt)) {
    LOG_DBG("IFCNT = %lu\r\n", static_cast<unsigned long>(ifcnt));
  } else {
    LOG_ERR("unable to read IFCNT\r\n");
  }

  LOG_NFO("Safe init completed\r\n");
  LOG_NFO("ENN is still HIGH: motor bridge disabled\r\n");
  return true;
}

// ===================== MOTION HELPERS =====================

static const char* motionModeName(MotionMode mode)
{
  switch (mode) {
    case MotionMode::IDLE: return "IDLE";
    case MotionMode::VELOCITY_TEST: return "VELOCITY_TEST";
    case MotionMode::STEP_TEST: return "STEP_TEST";
    case MotionMode::POSITION: return "POSITION";
    case MotionMode::CALIBRATION: return "CALIBRATION";
    case MotionMode::PARK: return "PARK";
    case MotionMode::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

float jointGetPositionDeg()
{
  // Public/planner-facing coordinate system:
  // real joint degrees, relative to the logical zero set by the `zero` command.
  return jointDeg - encoderZero;
}

bool jointIsReferenced()
{
  return jointReferenced;
}

float jointGetTargetDeg()
{
  return jointCtrl.target();
}

float jointGetErrorDeg()
{
  return jointGetTargetDeg() - jointGetPositionDeg();
}

float jointGetCommandVelocityDegS()
{
  return servoLastCmdDegS;
}

float jointGetMeasuredVelocityDegS()
{
  return jointCtrl.getLastMeasuredVel();
}

float jointGetRefPositionDeg()
{
  return jointCtrl.refPos();
}

float jointGetRefVelocityDegS()
{
  return jointCtrl.refVel();
}

static bool nonPositionOperationBusy()
{
  return motionMode == MotionMode::PARK ||
         motionMode == MotionMode::CALIBRATION ||
         motionMode == MotionMode::VELOCITY_TEST ||
         motionMode == MotionMode::STEP_TEST;
}

static bool jointCommandBusy()
{
  // External BUSY covers the latched position command plus exclusive
  // non-position operations. Completed servo hold is intentionally excluded.
  return positionMotionCommandBusy(motionLifecycle) ||
         nonPositionOperationBusy();
}

bool jointIsBusy()
{
  return jointCommandBusy();
}

bool jointIsSettled()
{
  return jointCtrl.isSettled();
}

bool jointHasFault()
{
  return jointCtrl.fault() || motionMode == MotionMode::FAULT;
}

bool setMotorVelocityDegPerSecond(float degreesPerSecond)
{
  if (!tmcReady) {
    LOG_ERR("TMC not ready\r\n");
    return false;
  }

  const float spd = stepsPerDegree();
  const float microstepsPerSecond = activeMotorDirectionSign * degreesPerSecond * spd;

  if (!tmc.runVelocityMicrostepsPerSecond(microstepsPerSecond)) {
    LOG_ERR("TMC velocity command failed\r\n");
    return false;
  }

  servoLastCmdDegS = degreesPerSecond;
  return true;
}

bool ensureDriverEnabled()
{
  if (!tmcReady) {
    LOG_ERR("TMC not ready\r\n");
    return false;
  }

  const Tmc2209Driver::Status st = tmc.status();

  if (st == Tmc2209Driver::Status::Enabled) {
    return true;
  }

  if (st != Tmc2209Driver::Status::Configured) {
    LOG_ERR("TMC is not in a configurable state\r\n");
    return false;
  }

  if (!tmc.armPowerStage()) {
    LOG_ERR("Unable to arm TMC power stage\r\n");
    return false;
  }

  if (!tmc.enableDriver()) {
    LOG_ERR("Unable to enable TMC driver\r\n");
    return false;
  }

  return true;
}

static void latchJointLimitFault(float zeroedDeg)
{
  clearPositionLifecycleState();

  float jmin = 0.0f;
  float jmax = 0.0f;
  float jtol = 0.0f;
  readJointLimitParams(jmin, jmax, jtol);

  if (tmcReady) {
    setMotorVelocityDegPerSecond(0.0f);
    tmc.stopInternalMotion();
    tmc.disableDriver(false);
  }

  motionMode = MotionMode::FAULT;
  jointCtrl.setPositionLimits(jmin, jmax, JOINT_LIMIT_STOP_MARGIN_DEG, jtol);
  jointCtrl.latchPositionLimitFault(zeroedDeg);
  wsSetState(LedState::FAULT);

  LOG_ERR("Joint limit fault: zeroed=%.3f deg outside allowed window [%.3f, %.3f] plus jtol=%.3f\r\n",
          zeroedDeg, jmin, jmax, jtol);
}

void stopMotion()
{
  const bool preserveFault =
      motionMode == MotionMode::FAULT || jointCtrl.fault();
  parkPhase = ParkPhase::IDLE;
  parkCommandVelocityDegS = 0.0f;
  testEnabled = false;
  testStepEnabled = false;
  stepNum = 0;
  servoLastCmdDegS = 0.0f;
  clearPositionLifecycleState();

  tmc.stopInternalMotion();
  tmc.disableDriver(false); // keep configured chopper state; hardware bridge disabled

  if (!preserveFault) {
    motionMode = MotionMode::IDLE;
    // The position controller always works in zeroed joint coordinates.
    jointCtrl.reset(jointGetPositionDeg());
    wsSetState(jointReferenced ? LedState::READY : LedState::BOOT);
  } else {
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
  }
}

bool FirmwareJointPlannerRuntime::isReferenced() const
{
  return jointReferenced;
}

MotionMode FirmwareJointPlannerRuntime::motionMode() const
{
  return ::motionMode;
}

bool FirmwareJointPlannerRuntime::encoderReady() const
{
  return encoderOk;
}

bool FirmwareJointPlannerRuntime::driverEnabled() const
{
  return tmc.status() == Tmc2209Driver::Status::Enabled;
}

bool FirmwareJointPlannerRuntime::positionCommandActive() const
{
  return motionLifecycle.positionCommandActive;
}

void FirmwareJointPlannerRuntime::stopMotion()
{
  ::stopMotion();
}

bool FirmwareJointPlannerRuntime::ensureDriverEnabled()
{
  return ::ensureDriverEnabled();
}

void FirmwareJointPlannerRuntime::latchDriverFault()
{
  clearPositionLifecycleState();
  ::motionMode = MotionMode::FAULT;
  wsSetState(LedState::FAULT);
}

bool FirmwareJointPlannerRuntime::clipTarget(float requestedDeg, float& clippedDeg) const
{
  return jointClipTargetToLimits(requestedDeg, clippedDeg);
}

float FirmwareJointPlannerRuntime::currentPositionDeg() const
{
  return jointGetPositionDeg();
}

float FirmwareJointPlannerRuntime::controllerRefPositionDeg() const
{
  return jointCtrl.refPos();
}

float FirmwareJointPlannerRuntime::controllerRefVelocityDegS() const
{
  return jointCtrl.refVel();
}

void FirmwareJointPlannerRuntime::configureController(float vmaxDegS,
                                                      float amaxDegS2,
                                                      float outMaxDegS,
                                                      bool clearFault)
{
  if (clearFault) {
    jointCtrl.clearFault();
  }
  jointCtrl.setLimits(vmaxDegS, amaxDegS2);
  jointCtrl.setOutputMax(outMaxDegS);
}

void FirmwareJointPlannerRuntime::restartController(float currentDeg, float targetDeg)
{
  jointCtrl.reset(currentDeg);
  jointCtrl.setTarget(targetDeg);
}

bool FirmwareJointPlannerRuntime::blendControllerTarget(float targetDeg)
{
  return jointCtrl.setTargetBlended(targetDeg);
}

bool FirmwareJointPlannerRuntime::restartControllerTimed(float currentDeg,
                                                         float targetDeg,
                                                         float durationS)
{
  return jointCtrl.setTargetFromRestWithDuration(
    currentDeg, targetDeg, durationS);
}

bool FirmwareJointPlannerRuntime::blendControllerTargetTimed(float targetDeg,
                                                             float durationS)
{
  return jointCtrl.setTargetBlendedWithDuration(targetDeg, durationS);
}

bool FirmwareJointPlannerRuntime::minimumCoordinatedDuration(
    float targetDeg,
    float vmaxDegS,
    float amaxDegS2,
    float& durationS) const
{
  return jointCtrl.minimumCoordinatedDuration(
    targetDeg,
    vmaxDegS,
    amaxDegS2,
    jointGetPositionDeg(),
    motionLifecycle.positionCommandActive,
    durationS);
}

void FirmwareJointPlannerRuntime::beginPositionMotion(float targetDeg)
{
  servoTargetZeroedDeg = targetDeg;
  lastServoUs = micros();
  motionLifecycle.beginPositionCommand();
  ::motionMode = MotionMode::POSITION;
}

JointMoveOutcome moveJointToDeg(float targetZeroedDeg)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetZeroedDeg;
  cmd.vmaxDegS = readParamMinOrDefault("vmax", SERVO_VMAX_DEG_S, 0.001f);
  cmd.amaxDegS2 = readParamMinOrDefault("amax", SERVO_AMAX_DEG_S2, 0.001f);
  cmd.outMaxDegS = readParamMinOrDefault("outmax", SERVO_OUTPUT_MAX_DEG_S, 0.0f);
  return planner.moveTo(cmd);
}

JointMoveOutcome jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  return planner.moveTo(targetDeg, vmaxDegS, amaxDegS2);
}

JointMoveOutcome jointMoveToBlended(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  return planner.moveToBlended(targetDeg, vmaxDegS, amaxDegS2);
}

JointMoveOutcome jointStop()
{
  return planner.stop();
}


static void latchEmergencyStopFault(const char* source)
{
  parkPhase = ParkPhase::IDLE;
  parkCommandVelocityDegS = 0.0f;
  testEnabled = false;
  testStepEnabled = false;
  stepNum = 0;
  servoLastCmdDegS = 0.0f;
  clearPositionLifecycleState();

  if (tmcReady) {
    setMotorVelocityDegPerSecond(0.0f);
    tmc.stopInternalMotion();
    tmc.disableDriver(false);
  }

  jointBusClearCoordinatedSegments();

  const float currentZeroedDeg = jointGetPositionDeg();
  jointCtrl.latchEmergencyStop(currentZeroedDeg);
  servoTargetZeroedDeg = currentZeroedDeg;
  motionMode = MotionMode::FAULT;
  wsSetState(LedState::FAULT);

  LOG_ERR("Emergency stop latched by %s at zeroed=%.3f deg\r\n",
          source ? source : "unknown",
          currentZeroedDeg);
}

void jointControllerInit(float currentJointDeg)
{
  applyControllerParamsFromParams();

  if (!applyJointLimitsFromParams(true)) {
    return;
  }
  jointCtrl.reset(currentJointDeg);

  LOG_NFO("Joint PID + analytic quintic S-curve controller initialized\r\n");
  LOG_NFO("Servo parameters are runtime-tunable with set <key> <value>. Keep early tests slow.\r\n");
}

static const char* traceModeName(TraceMode mode)
{
  switch (mode) {
    case TraceMode::FULL: return "FULL";
    case TraceMode::POS_TARGET: return "POS_TARGET";
    case TraceMode::POS_TARGET_VEL: return "POS_TARGET_VEL";
    case TraceMode::PID: return "PID";
    case TraceMode::SCURVE: return "SCURVE";
  }
  return "UNKNOWN";
}

static bool isValidTraceMode(uint8_t mode)
{
  return mode <= static_cast<uint8_t>(TraceMode::SCURVE);
}

bool toggleTrace()
{
  traceEnabled = !traceEnabled;
  return traceEnabled;
}

bool setTraceEnabled(bool enabled)
{
  traceEnabled = enabled;
  return traceEnabled;
}

bool setTraceMode(uint8_t mode)
{
  if (!isValidTraceMode(mode)) {
    return false;
  }

  traceMode = static_cast<TraceMode>(mode);
  traceEnabled = true;
  return true;
}

uint8_t getTraceMode()
{
  return static_cast<uint8_t>(traceMode);
}

const char* getTraceModeName()
{
  return traceModeName(traceMode);
}

bool toggleTest(float degreesPerSecond = 0.5f)
{
  if (!jointReferenced) {
    Serial.println("[ERR] Motion rejected: execute park first");
    return false;
  }
  if (motionMode == MotionMode::POSITION || motionMode == MotionMode::CALIBRATION) {
    Serial.println("[ERR] Stop position/calibration mode before velocity test");
    return false;
  }

  testEnabled = !testEnabled;

  if (testEnabled) {
    if (!ensureDriverEnabled()) {
      testEnabled = false;
      clearPositionLifecycleState();
      motionMode = MotionMode::FAULT;
      wsSetState(LedState::FAULT);
      return false;
    }

    clearPositionLifecycleState();
    motionMode = MotionMode::VELOCITY_TEST;
    wsSetState(LedState::TEST);
    setMotorVelocityDegPerSecond(degreesPerSecond);
  } else {
    stopMotion();
  }

  return testEnabled;
}

bool moveStep(float steps = 1.0f)
{
  if (!jointReferenced) {
    Serial.println("[ERR] Motion rejected: execute park first");
    return false;
  }
  if (motionMode == MotionMode::POSITION || motionMode == MotionMode::CALIBRATION) {
    Serial.println("[ERR] Stop position/calibration mode before step test");
    return false;
  }

  int32_t out = static_cast<int32_t>(steps);
  testStepEnabled = !testStepEnabled;

  if (testStepEnabled) {
    if (!ensureDriverEnabled()) {
      testStepEnabled = false;
      clearPositionLifecycleState();
      motionMode = MotionMode::FAULT;
      wsSetState(LedState::FAULT);
      return false;
    }

    clearPositionLifecycleState();
    motionMode = MotionMode::STEP_TEST;
    wsSetState(LedState::TEST);

    if (steps > 0) {
      testForward = true;
    } else {
      testForward = false;
      out = -out;
    }
    stepNum = out;
  } else {
    stopMotion();
  }

  return testStepEnabled;
}


float getStdegCalibrationDefaultTargetDeg()
{
  return STDEG_CALIBRATION_TARGET_DEG;
}

static bool updateEncoderSampleNow(float& jointDegOut)
{
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    float measured = 0.0f;

    if (encoder.readContinuousDegrees(measured)) {
      encoderOk = true;
      encoderRaw = encoder.lastRaw();
      encoderDeg = encoder.lastDegrees();
      jointDeg = measured;
      jointDegOut = measured;
      return true;
    }

    encoderOk = false;
    delay(5);
  }

  wsSetState(LedState::ENCODER_ERROR);
  return false;
}

FaultClearResult clearMotionFault()
{
  const bool faultActive = jointHasFault() ||
                           motionMode == MotionMode::FAULT ||
                           !encoderOk || !tmcReady;

  if (!faultActive) {
    return FaultClearResult::AlreadyClear;
  }

  // Fault recovery always starts from a de-energized, stationary bridge.
  clearPositionLifecycleState();
  jointBusClearCoordinatedSegments();
  if (tmcReady) {
    tmc.stopInternalMotion();
    tmc.disableDriver(false);
  }

  const Tmc2209Driver::Status driverStatus = tmc.status();
  const bool driverStateValid =
      driverStatus == Tmc2209Driver::Status::Configured ||
      driverStatus == Tmc2209Driver::Status::Enabled;
  const bool driverValid = tmcReady && driverStateValid && tmc.probe();

  float measuredAbsoluteDeg = 0.0f;
  const bool encoderValid = updateEncoderSampleNow(measuredAbsoluteDeg) &&
                            isfinite(measuredAbsoluteDeg);
  const float currentZeroedDeg = encoderValid
      ? measuredAbsoluteDeg - encoderZero
      : 0.0f;

  float jmin = 0.0f;
  float jmax = 0.0f;
  float jtol = 0.0f;
  const bool limitsValid = readJointLimitParams(jmin, jmax, jtol);

  FaultRecoverySnapshot snapshot;
  snapshot.faultActive = faultActive;
  snapshot.encoderValid = encoderValid;
  snapshot.driverValid = driverValid;
  snapshot.limitsValid = limitsValid;
  snapshot.positionDeg = currentZeroedDeg;
  snapshot.minDeg = jmin;
  snapshot.maxDeg = jmax;

  const FaultClearResult result = evaluateFaultRecovery(snapshot);
  if (result == FaultClearResult::InvalidLimits) {
    latchPlannerConfigurationFault("CLEAR_FAULT rejected invalid jmin/jmax/jtol");
    return result;
  }
  if (result != FaultClearResult::Cleared &&
      result != FaultClearResult::AlreadyClear) {
    LOG_ERR("CLEAR_FAULT rejected: %s\r\n", faultClearResultName(result));
    return result;
  }

  jointCtrl.setPositionLimits(jmin, jmax,
                              JOINT_LIMIT_STOP_MARGIN_DEG, jtol);
  jointCtrl.reset(currentZeroedDeg);
  jointCtrl.setTarget(currentZeroedDeg);
  servoTargetZeroedDeg = currentZeroedDeg;
  servoLastCmdDegS = 0.0f;
  motionMode = MotionMode::IDLE;
  jointBusLastLimitClipped = false;
  wsSetState(jointReferenced ? LedState::READY : LedState::BOOT);

  LOG_NFO("Motion fault %s at zeroed=%.3f deg; driver disabled\r\n",
          result == FaultClearResult::Cleared ? "cleared" : "already clear",
          currentZeroedDeg);
  return result;
}

static bool updateStdegParameter(float newStepsPerDegree)
{
  if (!isfinite(newStepsPerDegree) || newStepsPerDegree <= 0.0f) {
    Serial.printf("[CAL] Invalid estimated stdeg: %.6f\r\n", newStepsPerDegree);
    return false;
  }

  if (!params.set("stdeg", newStepsPerDegree)) {
    Serial.println("[CAL] Unable to update stdeg parameter in RAM");
    return false;
  }

  Serial.printf("[CAL] stdeg updated in RAM: %.6f microsteps/deg\r\n", stepsPerDegree());
  return true;
}

bool calibrateStepsPerDegree(float targetDegrees = STDEG_CALIBRATION_TARGET_DEG)
{
  if (!jointReferenced) {
    Serial.println("[CAL] Calibration rejected: execute park first");
    return false;
  }
  if (motionMode == MotionMode::CALIBRATION) {
    Serial.println("[CAL] Calibration already running");
    return false;
  }

  if (motionMode == MotionMode::FAULT) {
    Serial.println("[CAL] Motion fault latched; clear the fault before calibration");
    return false;
  }

  if (!isfinite(targetDegrees) || fabsf(targetDegrees) < 0.001f) {
    Serial.println("[CAL] Invalid calibration target angle");
    return false;
  }

  const float seedStepsPerDegree = stepsPerDegree();
  if (!isfinite(seedStepsPerDegree) || seedStepsPerDegree <= 0.0f) {
    Serial.println("[CAL] Cannot start calibration without a valid current stdeg seed");
    return false;
  }

  const int32_t requestedSteps = static_cast<int32_t>(lroundf(fabsf(targetDegrees) * seedStepsPerDegree));
  if (requestedSteps <= 0) {
    Serial.println("[CAL] Computed calibration step count is zero");
    return false;
  }

  // Exclusive control: stop every other motion mode first.
  stopMotion();
  motionMode = MotionMode::CALIBRATION;
  wsSetState(LedState::TEST);
  wsLedsUpdate();

  Serial.println("[CAL] STDEG calibration started");
  Serial.printf("[CAL] target=%.6f deg, seed=%.6f microsteps/deg, steps=%ld\r\n",
                targetDegrees,
                seedStepsPerDegree,
                static_cast<long>(requestedSteps));

  float startDeg = 0.0f;
  if (!updateEncoderSampleNow(startDeg)) {
    Serial.println("[CAL] Failed to read encoder before calibration move");
    clearPositionLifecycleState();
    motionMode = MotionMode::FAULT;
    tmc.disableDriver(false);
    wsSetState(LedState::ENCODER_ERROR);
    return false;
  }

  if (!ensureDriverEnabled()) {
    Serial.println("[CAL] Failed to enable TMC2209 power stage");
    clearPositionLifecycleState();
    motionMode = MotionMode::FAULT;
    tmc.disableDriver(false);
    wsSetState(LedState::FAULT);
    return false;
  }

  delay(50);

  // Apply the same direction convention used by velocity control.
  const bool forward = (targetDegrees * activeMotorDirectionSign) > 0.0f;

  for (int32_t i = 0; i < requestedSteps; i++) {
    tmc.step(forward, STDEG_CALIBRATION_STEP_HIGH_US);

    if (STDEG_CALIBRATION_STEP_PERIOD_US > STDEG_CALIBRATION_STEP_HIGH_US) {
      delayMicroseconds(STDEG_CALIBRATION_STEP_PERIOD_US - STDEG_CALIBRATION_STEP_HIGH_US);
    }

    if ((i % static_cast<int32_t>(STDEG_CALIBRATION_ENCODER_SAMPLE_EVERY_STEPS)) == 0) {
      float tmp = 0.0f;
      updateEncoderSampleNow(tmp);
      wsLedsUpdate();
      yield();
    }
  }

  delay(STDEG_CALIBRATION_SETTLE_MS);

  float endDeg = 0.0f;
  const bool finalReadOk = updateEncoderSampleNow(endDeg);

  tmc.stopInternalMotion();
  tmc.disableDriver(false);

  if (!finalReadOk) {
    Serial.println("[CAL] Failed to read encoder after calibration move");
    clearPositionLifecycleState();
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::ENCODER_ERROR);
    return false;
  }

  const float measuredDeltaDeg = endDeg - startDeg;
  const float absDeltaDeg = fabsf(measuredDeltaDeg);

  Serial.printf("[CAL] start=%.6f deg, end=%.6f deg, delta=%.6f deg\r\n",
                startDeg,
                endDeg,
                measuredDeltaDeg);

  if (absDeltaDeg < STDEG_CALIBRATION_MIN_DELTA_DEG) {
    Serial.println("[CAL] Measured movement too small: calibration rejected");
    clearPositionLifecycleState();
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    return false;
  }

  if ((targetDegrees > 0.0f && measuredDeltaDeg < 0.0f) ||
      (targetDegrees < 0.0f && measuredDeltaDeg > 0.0f)) {
    Serial.println("[CAL] WARNING: measured direction is opposite to requested direction");
    Serial.println("[CAL] Check mdir / DIR wiring / mechanics. Magnitude will still be applied.");
  }

  const float estimatedStepsPerDegree = static_cast<float>(requestedSteps) / absDeltaDeg;
  Serial.printf("[CAL] estimated stdeg=%.6f microsteps/deg\r\n", estimatedStepsPerDegree);

  const bool updated = updateStdegParameter(estimatedStepsPerDegree);

  if (updated) {
    Serial.println("[CAL] stdeg updated in RAM. Use 'save' to persist it in NVS.");
    motionMode = MotionMode::IDLE;
    wsSetState(LedState::READY);
    jointCtrl.reset(jointGetPositionDeg());
  } else {
    clearPositionLifecycleState();
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
  }

  return updated;
}

bool setZero()
{
  if (motionMode == MotionMode::FAULT || jointCtrl.fault()) {
    Serial.println("[ERR] Zero rejected: clear the fault first");
    return false;
  }
  if (!jointReferenced) {
    Serial.println("[ERR] Zero rejected: execute park first");
    return false;
  }
  testEnabled = false;
  testStepEnabled = false;
  stepNum = 0;
  servoLastCmdDegS = 0.0f;
  clearPositionLifecycleState();

  tmc.stopInternalMotion();
  if (!motorHoldEnabled()) {
    tmc.disableDriver(false);
  }

  // Keep the multi-turn unwrap established by park. Zero only changes the
  // persistent logical offset; it must never destroy the referenced turn count.
  float tmp = 0.0f;
  if (encoder.readContinuousDegrees(tmp)) {
    encoderOk = true;
    jointDeg = tmp;
    encoderRaw = encoder.lastRaw();
    encoderDeg = encoder.lastDegrees();
  } else {
    encoderOk = false;
    Serial.println("[ERR] Encoder read failed while setting zero");
    wsSetState(LedState::ENCODER_ERROR);
    return false;
  }

  encoderZero = jointDeg;
  params.set("zoff", encoderZero);
  servoTargetZeroedDeg = 0.0f;

  // After zeroing, the controller coordinate system is zeroed too.
  jointCtrl.reset(0.0f);
  jointCtrl.setTarget(0.0f);

  motionMode = MotionMode::IDLE;
  wsSetState(LedState::READY);
  Serial.printf("Zero set at joint_abs=%.3f deg; zoff updated in RAM; use 'save' to persist it\r\n",
                encoderZero);
  return true;
}

void printServoStatus()
{
  const float zeroed = jointGetPositionDeg();
  float jmin = 0.0f;
  float jmax = 0.0f;
  float jtol = 0.0f;
  readJointLimitParams(jmin, jmax, jtol);

  Serial.printf("mode=%s referenced=%u park_sensor=%u tmc=%u hold=%u shold=%u enc=%.3f joint=%.3f zeroed=%.3f target=%.3f ref=%.3f refv=%.3f refa=%.3f profile=%u time=%.3f/%.3f measv=%.3f cmd=%.3f stdeg=%.6f jrev=%.6f mdir=%+.0f edir=%+.0f zoff=%.6f jmin=%.3f jmax=%.3f jtol=%.3f fault=%u\r\n",
                motionModeName(motionMode),
                jointReferenced ? 1u : 0u,
                digitalRead(PIN_PARK_SENSOR) == LOW ? 1u : 0u,
                static_cast<unsigned>(tmc.status()),
                motorHoldEnabled() ? 1u : 0u,
                servoHoldEnabled() ? 1u : 0u,
                encoderDeg,
                jointDeg,
                zeroed,
                servoTargetZeroedDeg,
                jointCtrl.refPos(),
                jointCtrl.refVel(),
                jointCtrl.refAcc(),
                jointCtrl.trajectoryActive() ? 1u : 0u,
                jointCtrl.trajectoryElapsed(),
                jointCtrl.trajectoryDuration(),
                jointCtrl.getLastMeasuredVel(),
                servoLastCmdDegS,
                stepsPerDegree(),
                encoder.outputDegreesPerEncoderRevolution(),
                activeMotorDirectionSign,
                encoder.directionSign(),
                encoderZero,
                jmin,
                jmax,
                jtol,
                static_cast<unsigned>(jointCtrl.faultCode()));

  Serial.printf("pid kp=%.4f ki=%.4f kd=%.4f ffv=%.4f ilim=%.4f | motion vmax=%.4f amax=%.4f outmax=%.4f | settle ptol=%.4f vtol=%.4f dbent=%.4f dbext=%.4f dbvel=%.4f vtau=%.4f\r\n",
                readParamFloatOrDefault("kp", SERVO_KP),
                readParamFloatOrDefault("ki", SERVO_KI),
                readParamFloatOrDefault("kd", SERVO_KD),
                readParamFloatOrDefault("ffv", SERVO_FF_VEL),
                readParamFloatOrDefault("ilim", SERVO_I_LIMIT),
                readParamFloatOrDefault("vmax", SERVO_VMAX_DEG_S),
                readParamFloatOrDefault("amax", SERVO_AMAX_DEG_S2),
                readParamFloatOrDefault("outmax", SERVO_OUTPUT_MAX_DEG_S),
                readParamFloatOrDefault("ptol", SERVO_POS_TOL_DEG),
                readParamFloatOrDefault("vtol", SERVO_VEL_TOL_DEG_S),
                readParamFloatOrDefault("dbent", SERVO_DEADBAND_ENTER),
                readParamFloatOrDefault("dbext", SERVO_DEADBAND_EXIT),
                readParamFloatOrDefault("dbvel", SERVO_DEADBAND_VEL),
                readParamFloatOrDefault("vtau", SERVO_VEL_FILTER_TAU_S));

  Serial.printf("park pkdir=%.0f pkvel=%.4f pkenc=%.4f pkpos=%.4f accel=%.4f rel_to=%lu search_to=%lu align_to=%lu\r\n",
                readParamFloatOrDefault("pkdir", 0.0f),
                readParamFloatOrDefault("pkvel", PARK_DEFAULT_VELOCITY_DEG_S),
                readParamFloatOrDefault("pkenc", PARK_DEFAULT_ENCODER_ANGLE_DEG),
                readParamFloatOrDefault("pkpos", PARK_DEFAULT_JOINT_POSITION_DEG),
                PARK_ACCEL_DEG_S2,
                static_cast<unsigned long>(PARK_RELEASE_TIMEOUT_MS),
                static_cast<unsigned long>(PARK_SEARCH_TIMEOUT_MS),
                static_cast<unsigned long>(PARK_ALIGN_TIMEOUT_MS));
}


// ===================== JOINTBUS SLAVE HOOKS =====================

static int16_t jointBusDegToCdeg(float valueDeg)
{
  if (!isfinite(valueDeg)) {
    return 0;
  }

  const float clipped = constrain(valueDeg, -327.68f, 327.67f);
  return static_cast<int16_t>(lroundf(clipped * 100.0f));
}

static int16_t jointBusDegToDdeg(float valueDeg)
{
  if (!isfinite(valueDeg)) {
    return 0;
  }
  const float clipped = constrain(valueDeg, -3276.8f, 3276.7f);
  return static_cast<int16_t>(lroundf(clipped * 10.0f));
}

static uint32_t jointBusSecondsToMilliseconds(float seconds)
{
  if (!isfinite(seconds) || seconds <= 0.0f) {
    return 0;
  }
  const double milliseconds = static_cast<double>(seconds) * 1000.0;
  if (milliseconds >= static_cast<double>(UINT32_MAX)) {
    return UINT32_MAX;
  }
  return static_cast<uint32_t>(milliseconds + 0.5);
}

static bool jointBusDegToCdegChecked(float valueDeg, int16_t& valueCdeg)
{
  if (!isfinite(valueDeg) || valueDeg < -327.68f || valueDeg > 327.67f) {
    return false;
  }
  valueCdeg = static_cast<int16_t>(lroundf(valueDeg * 100.0f));
  return true;
}

static bool jointBusPositiveToCentiChecked(float value, uint16_t& valueCenti)
{
  if (!isfinite(value) || value < 0.0f || value > 655.35f) {
    return false;
  }
  valueCenti = static_cast<uint16_t>(lroundf(value * 100.0f));
  return true;
}

static float jointBusCdegToDeg(int16_t valueCdeg)
{
  return static_cast<float>(valueCdeg) * 0.01f;
}

static uint16_t jointBusCdegSToUInt(uint16_t valueCdegS)
{
  return valueCdegS;
}

static uint8_t readJointBusAddressFromParams()
{
  float value = 0.0f;
  if (!params.get("addr", value) || !isfinite(value)) {
    value = 0.0f;
  }

  if (value < 0.0f) {
    value = 0.0f;
  }
  if (value > 15.0f) {
    value = 15.0f;
  }

  return static_cast<uint8_t>(lroundf(value)) & 0x0F;
}

static bool jointBusMotionBusy()
{
  return jointCommandBusy();
}

static void jointBusClearPreparedSegment()
{
  jointBusScheduledStart = JointBusScheduledStart{};
  jointBusPreparedSegment.valid = false;
  jointBusPreparedSegment.hold = false;
  jointBusPreparedSegment.segmentId = JointBus::NO_SEGMENT_ID;
  jointBusPreparedSegment.targetCdeg = 0;
  jointBusPreparedSegment.vmaxCdegS = 0;
  jointBusPreparedSegment.amaxCdegS2 = 0;
}

static void jointBusClearCoordinatedSegments()
{
  jointBusClearPreparedSegment();
  jointBusActiveSegmentValid = false;
  jointBusActiveSegmentId = JointBus::NO_SEGMENT_ID;
  jointBusActiveMotionFlags = 0;
  jointBusActiveSegmentHold = false;
  jointBusActiveSegmentStartedMs = 0;
  jointBusActiveSegmentDurationMs = 0;
}

static void jointBusSegmentQueueUpdate()
{
  if (jointBusActiveSegmentValid && jointBusActiveSegmentHold) {
    const bool durationElapsed = jointBusActiveSegmentDurationMs == 0 ||
      static_cast<uint32_t>(millis() - jointBusActiveSegmentStartedMs) >=
        jointBusActiveSegmentDurationMs;
    if (durationElapsed && !jointBusMotionBusy()) {
      jointBusActiveSegmentValid = false;
      jointBusActiveSegmentId = JointBus::NO_SEGMENT_ID;
      jointBusActiveMotionFlags = 0;
      jointBusActiveSegmentHold = false;
      jointBusActiveSegmentStartedMs = 0;
      jointBusActiveSegmentDurationMs = 0;
    }
    return;
  }
  if (jointBusActiveSegmentValid && !jointBusMotionBusy()) {
    jointBusActiveSegmentValid = false;
    jointBusActiveSegmentId = JointBus::NO_SEGMENT_ID;
    jointBusActiveMotionFlags = 0;
  }
}

static JointBus::NackCode jointBusNackForMoveResult(JointMoveResult result)
{
  switch (result) {
    case JointMoveResult::NotReferenced:
      return JointBus::NackCode::NotHomed;
    case JointMoveResult::CalibrationActive:
      return JointBus::NackCode::Busy;
    case JointMoveResult::FaultActive:
      return JointBus::NackCode::FaultActive;
    case JointMoveResult::InvalidCommand:
      return JointBus::NackCode::BadPayload;
    case JointMoveResult::DurationInfeasible:
      return JointBus::NackCode::DurationInfeasible;
    case JointMoveResult::EncoderUnavailable:
    case JointMoveResult::InvalidLimits:
    case JointMoveResult::DriverError:
      return JointBus::NackCode::InternalError;
    default:
      return JointBus::NackCode::RejectedByState;
  }
}

static JointBus::AckCode jointBusAckForMoveOutcome(const JointMoveOutcome& outcome,
                                                   JointBus::AckCode successAck,
                                                   bool exposeRetargetMode)
{
  if (outcome.targetAdjustment == JointTargetAdjustment::ClippedToMax) {
    jointBusLastLimitClipped = true;
    return JointBus::AckCode::ClippedToMax;
  }
  if (outcome.targetAdjustment == JointTargetAdjustment::ClippedToMin) {
    jointBusLastLimitClipped = true;
    return JointBus::AckCode::ClippedToMin;
  }

  jointBusLastLimitClipped = false;
  if (exposeRetargetMode && outcome.result == JointMoveResult::BlendAccepted) {
    return JointBus::AckCode::BlendAccepted;
  }
  if (exposeRetargetMode && outcome.result == JointMoveResult::SafeReplan) {
    return JointBus::AckCode::SafeReplan;
  }
  return successAck;
}

static JointBus::CommandResult jointBusMove(void* context,
                                            int16_t targetCdeg,
                                            uint16_t vmaxCdegS,
                                            uint16_t amaxCdegS2)
{
  (void)context;

  const float targetDeg = jointBusCdegToDeg(targetCdeg);
  const float vmaxDegS = static_cast<float>(jointBusCdegSToUInt(vmaxCdegS)) * 0.01f;
  const float amaxDegS2 = static_cast<float>(jointBusCdegSToUInt(amaxCdegS2)) * 0.01f;
  const JointMoveOutcome outcome = jointMoveTo(targetDeg, vmaxDegS, amaxDegS2);
  if (!outcome.accepted()) {
    return JointBus::CommandResult::fail(jointBusNackForMoveResult(outcome.result));
  }

  jointBusClearCoordinatedSegments();
  return JointBus::CommandResult::ok(
    jointBusAckForMoveOutcome(outcome, JointBus::AckCode::Accepted, false));
}

static JointBus::CommandResult jointBusMoveB(void* context,
                                             int16_t targetCdeg,
                                             uint16_t vmaxCdegS,
                                             uint16_t amaxCdegS2)
{
  (void)context;

  const float targetDeg = jointBusCdegToDeg(targetCdeg);
  const float vmaxDegS = static_cast<float>(jointBusCdegSToUInt(vmaxCdegS)) * 0.01f;
  const float amaxDegS2 = static_cast<float>(jointBusCdegSToUInt(amaxCdegS2)) * 0.01f;
  const JointMoveOutcome outcome = jointMoveToBlended(targetDeg, vmaxDegS, amaxDegS2);
  if (!outcome.accepted()) {
    return JointBus::CommandResult::fail(jointBusNackForMoveResult(outcome.result));
  }

  jointBusClearCoordinatedSegments();
  return JointBus::CommandResult::ok(
    jointBusAckForMoveOutcome(outcome, JointBus::AckCode::Accepted, true));
}

static JointBus::CommandResult jointBusPrepareMoveB(void* context,
                                                    uint8_t segmentId,
                                                    int16_t targetCdeg,
                                                    uint16_t vmaxCdegS,
                                                    uint16_t amaxCdegS2)
{
  (void)context;

  if (segmentId == JointBus::NO_SEGMENT_ID) {
    return JointBus::CommandResult::fail(JointBus::NackCode::BadPayload);
  }
  if (jointBusPreparedSegment.valid) {
    return JointBus::CommandResult::fail(JointBus::NackCode::QueueFull,
                                         jointBusPreparedSegment.segmentId);
  }
  if (jointHasFault()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
  }
  if (!jointReferenced) {
    return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
  }
  if (!encoderOk || !tmcReady) {
    return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
  }
  if (motionMode == MotionMode::CALIBRATION ||
      motionMode == MotionMode::PARK ||
      motionMode == MotionMode::VELOCITY_TEST ||
      motionMode == MotionMode::STEP_TEST) {
    return JointBus::CommandResult::fail(JointBus::NackCode::Busy);
  }

  const float requestedDeg = jointBusCdegToDeg(targetCdeg);
  float clippedDeg = requestedDeg;
  if (!jointClipTargetToLimits(requestedDeg, clippedDeg)) {
    return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
  }

  JointBus::AckCode ack = JointBus::AckCode::SegmentPrepared;
  const float eps = 0.005f;
  if (clippedDeg < requestedDeg - eps) {
    jointBusLastLimitClipped = true;
    ack = JointBus::AckCode::ClippedToMax;
  } else if (clippedDeg > requestedDeg + eps) {
    jointBusLastLimitClipped = true;
    ack = JointBus::AckCode::ClippedToMin;
  } else {
    jointBusLastLimitClipped = false;
  }

  jointBusPreparedSegment.valid = true;
  jointBusPreparedSegment.hold = false;
  jointBusPreparedSegment.segmentId = segmentId;
  jointBusPreparedSegment.targetCdeg = jointBusDegToCdeg(clippedDeg);
  jointBusPreparedSegment.vmaxCdegS = vmaxCdegS;
  jointBusPreparedSegment.amaxCdegS2 = amaxCdegS2;

  LOG_NFO("JointBus segment prepared id=%u target=%.3f deg vmax=%.3f deg/s amax=%.3f deg/s2%s\r\n",
          static_cast<unsigned>(segmentId),
          clippedDeg,
          static_cast<float>(vmaxCdegS) * 0.01f,
          static_cast<float>(amaxCdegS2) * 0.01f,
          ack == JointBus::AckCode::SegmentPrepared ? "" : " clipped");

  return JointBus::CommandResult::ok(ack, segmentId);
}

static JointBus::CommandResult jointBusPrepareHold(void* context,
                                                   uint8_t segmentId)
{
  (void)context;
  if (segmentId == JointBus::NO_SEGMENT_ID) {
    return JointBus::CommandResult::fail(JointBus::NackCode::BadPayload);
  }
  if (jointBusPreparedSegment.valid) {
    return JointBus::CommandResult::fail(
      JointBus::NackCode::QueueFull, jointBusPreparedSegment.segmentId);
  }
  if (jointHasFault()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
  }
  if (!jointReferenced) {
    return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
  }
  if (!encoderOk || !tmcReady) {
    return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
  }
  if (motionMode == MotionMode::CALIBRATION ||
      motionMode == MotionMode::PARK ||
      motionMode == MotionMode::VELOCITY_TEST ||
      motionMode == MotionMode::STEP_TEST) {
    return JointBus::CommandResult::fail(JointBus::NackCode::Busy);
  }

  jointBusPreparedSegment = JointBusPreparedSegment{};
  jointBusPreparedSegment.valid = true;
  jointBusPreparedSegment.hold = true;
  jointBusPreparedSegment.segmentId = segmentId;
  jointBusPreparedSegment.targetCdeg =
    jointBusDegToCdeg(jointGetTargetDeg());
  LOG_NFO("JointBus HOLD prepared id=%u target=%.3f deg\r\n",
          static_cast<unsigned>(segmentId), jointGetTargetDeg());
  return JointBus::CommandResult::ok(
    JointBus::AckCode::HoldPrepared, segmentId);
}

static JointBus::CommandResult jointBusStartPreparedSegment(
    uint8_t segmentId,
    uint32_t fixedDurationMs)
{
  if (!jointBusPreparedSegment.valid) {
    return JointBus::CommandResult::fail(JointBus::NackCode::NoPreparedSegment);
  }
  if (jointBusPreparedSegment.segmentId != segmentId) {
    return JointBus::CommandResult::fail(JointBus::NackCode::SegmentMismatch,
                                         jointBusPreparedSegment.segmentId);
  }
  if (jointHasFault()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
  }
  if (!jointReferenced) {
    return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
  }

  if (jointBusPreparedSegment.hold) {
    jointBusActiveSegmentValid = true;
    jointBusActiveSegmentId = segmentId;
    jointBusActiveMotionFlags = JointBus::MSTATE_HOLD_AXIS;
    jointBusActiveSegmentHold = true;
    jointBusActiveSegmentStartedMs = millis();
    jointBusActiveSegmentDurationMs = fixedDurationMs;
    jointBusClearPreparedSegment();
    LOG_NFO("JointBus HOLD started id=%u duration=%lu ms\r\n",
            static_cast<unsigned>(segmentId),
            static_cast<unsigned long>(fixedDurationMs));
    return JointBus::CommandResult::ok(
      JointBus::AckCode::SegmentStarted, segmentId);
  }

  const float targetDeg = jointBusCdegToDeg(jointBusPreparedSegment.targetCdeg);
  const float vmaxDegS = static_cast<float>(jointBusPreparedSegment.vmaxCdegS) * 0.01f;
  const float amaxDegS2 = static_cast<float>(jointBusPreparedSegment.amaxCdegS2) * 0.01f;

  JointMoveOutcome outcome;
  if (fixedDurationMs != 0) {
    JointMoveCommand command;
    command.targetDeg = targetDeg;
    command.vmaxDegS = vmaxDegS;
    command.amaxDegS2 = amaxDegS2;
    outcome = planner.moveToBlendedTimed(
      command, static_cast<float>(fixedDurationMs) * 0.001f);
  } else {
    outcome = jointMoveToBlended(targetDeg, vmaxDegS, amaxDegS2);
  }
  if (!outcome.accepted()) {
    return JointBus::CommandResult::fail(jointBusNackForMoveResult(outcome.result));
  }

  jointBusActiveSegmentValid = true;
  jointBusActiveSegmentId = segmentId;
  jointBusActiveMotionFlags = 0;
  jointBusActiveSegmentHold = false;
  jointBusActiveSegmentStartedMs = millis();
  jointBusActiveSegmentDurationMs = fixedDurationMs;
  if (outcome.result == JointMoveResult::BlendAccepted) {
    jointBusActiveMotionFlags |= JointBus::MSTATE_BLEND_ACCEPTED;
  } else if (outcome.result == JointMoveResult::SafeReplan) {
    jointBusActiveMotionFlags |= JointBus::MSTATE_SAFE_REPLAN;
  }
  jointBusClearPreparedSegment();

  LOG_NFO("JointBus segment started id=%u target=%.3f deg vmax=%.3f deg/s amax=%.3f deg/s2 duration=%lu ms\r\n",
          static_cast<unsigned>(segmentId),
          targetDeg,
          vmaxDegS,
          amaxDegS2,
          static_cast<unsigned long>(fixedDurationMs));

  return JointBus::CommandResult::ok(JointBus::AckCode::SegmentStarted, segmentId);
}

static JointBus::CommandResult jointBusStartSegment(void* context,
                                                    uint8_t segmentId)
{
  (void)context;
  return jointBusStartPreparedSegment(segmentId, 0);
}

static JointBus::CommandResult jointBusScheduleSegment(
    void* context,
    uint8_t segmentId,
    uint32_t durationMs,
    uint32_t delayUs)
{
  (void)context;
  if (!jointBusPreparedSegment.valid) {
    return JointBus::CommandResult::fail(
      JointBus::NackCode::NoPreparedSegment);
  }
  if (jointBusPreparedSegment.segmentId != segmentId) {
    return JointBus::CommandResult::fail(
      JointBus::NackCode::SegmentMismatch,
      jointBusPreparedSegment.segmentId);
  }
  if (jointBusScheduledStart.valid) {
    return JointBus::CommandResult::fail(JointBus::NackCode::Busy);
  }
  if (durationMs == 0 ||
      delayUs < JointBus::MIN_SCHEDULE_DELAY_US ||
      delayUs > JointBus::MAX_SCHEDULE_DELAY_US) {
    return JointBus::CommandResult::fail(JointBus::NackCode::BadPayload);
  }
  if (jointHasFault()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
  }
  if (!jointReferenced) {
    return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
  }

  jointBusScheduledStart.valid = true;
  jointBusScheduledStart.segmentId = segmentId;
  jointBusScheduledStart.durationMs = durationMs;
  jointBusScheduledStart.deadlineUs = micros() + delayUs;
  LOG_DBG("JointBus segment scheduled id=%u duration=%lu ms delay=%lu us\r\n",
          static_cast<unsigned>(segmentId),
          static_cast<unsigned long>(durationMs),
          static_cast<unsigned long>(delayUs));
  return JointBus::CommandResult::ok(
    JointBus::AckCode::SegmentScheduled, segmentId);
}

static void jointBusScheduledStartUpdate()
{
  if (!jointBusScheduledStart.valid ||
      static_cast<int32_t>(micros() -
                           jointBusScheduledStart.deadlineUs) < 0) {
    return;
  }

  const uint8_t segmentId = jointBusScheduledStart.segmentId;
  const uint32_t durationMs = jointBusScheduledStart.durationMs;
  jointBusScheduledStart.valid = false;
  const JointBus::CommandResult result =
    jointBusStartPreparedSegment(segmentId, durationMs);
  if (result.accepted) {
    return;
  }

  LOG_ERR("JointBus scheduled start failed id=%u nack=0x%02X\r\n",
          static_cast<unsigned>(segmentId),
          static_cast<unsigned>(result.nack));

  // A duration can become infeasible between SEGMENT_TIMING and the delayed
  // start if the previous trajectory advances.  This is a recoverable
  // coordination miss, not a local controller fault.  Stop safely and retain
  // the prepared slot so the master can abort/re-prepare and retry it.
  if (result.nack == JointBus::NackCode::DurationInfeasible) {
    stopMotion();
    jointBusActiveSegmentValid = false;
    jointBusActiveSegmentId = JointBus::NO_SEGMENT_ID;
    jointBusActiveMotionFlags = 0;
    return;
  }

  stopMotion();
  jointCtrl.latchPlannerFault(jointGetPositionDeg());
  motionMode = MotionMode::FAULT;
  wsSetState(LedState::FAULT);
  jointBusClearCoordinatedSegments();
}

static bool jointBusSegmentTiming(void* context,
                                  uint8_t segmentId,
                                  JointBus::SegmentTiming& outTiming)
{
  (void)context;
  if (!jointBusPreparedSegment.valid ||
      jointBusPreparedSegment.segmentId != segmentId ||
      jointBusScheduledStart.valid) {
    return false;
  }

  if (jointBusPreparedSegment.hold) {
    outTiming.segmentId = segmentId;
    outTiming.minimumDurationMs = 0;
    return true;
  }

  JointMoveCommand command;
  command.targetDeg = jointBusCdegToDeg(
    jointBusPreparedSegment.targetCdeg);
  command.vmaxDegS = static_cast<float>(
    jointBusPreparedSegment.vmaxCdegS) * 0.01f;
  command.amaxDegS2 = static_cast<float>(
    jointBusPreparedSegment.amaxCdegS2) * 0.01f;
  float durationS = 0.0f;
  if (!planner.minimumBlendedDuration(command, durationS) ||
      !isfinite(durationS) || durationS < 0.0f) {
    return false;
  }

  outTiming.segmentId = segmentId;
  outTiming.minimumDurationMs = durationS <= 0.0f
    ? 0
    : static_cast<uint32_t>(ceilf(durationS * 1000.0f));
  return true;
}

static JointBus::CommandResult jointBusAbortSegment(void* context, uint8_t segmentId)
{
  (void)context;

  if (segmentId == JointBus::NO_SEGMENT_ID) {
    jointBusClearPreparedSegment();
    return JointBus::CommandResult::ok(JointBus::AckCode::SegmentAborted, segmentId);
  }

  if (jointBusPreparedSegment.valid && jointBusPreparedSegment.segmentId == segmentId) {
    jointBusClearPreparedSegment();
    return JointBus::CommandResult::ok(JointBus::AckCode::SegmentAborted, segmentId);
  }

  if (jointBusActiveSegmentValid && jointBusActiveSegmentId == segmentId) {
    // Active motion is not aborted by this command. Use STOP for an immediate
    // coordinated stop. AbortSegment only flushes the prepared next segment.
    return JointBus::CommandResult::fail(JointBus::NackCode::Busy, segmentId);
  }

  return JointBus::CommandResult::fail(JointBus::NackCode::NoPreparedSegment, segmentId);
}

static bool jointBusQueueStatus(void* context, JointBus::QueueStatus& outStatus)
{
  (void)context;
  jointBusSegmentQueueUpdate();

  outStatus.capacity = 2;
  outStatus.freePreparedSlots = jointBusPreparedSegment.valid ? 0 : 1;
  outStatus.activeSegmentId = jointBusActiveSegmentValid ? jointBusActiveSegmentId : JointBus::NO_SEGMENT_ID;
  outStatus.preparedSegmentId = jointBusPreparedSegment.valid ? jointBusPreparedSegment.segmentId : JointBus::NO_SEGMENT_ID;
  outStatus.flags = 0;

  if (jointBusActiveSegmentValid) {
    outStatus.flags |= JointBus::QQUEUE_ACTIVE_VALID;
  }
  if (jointBusPreparedSegment.valid) {
    outStatus.flags |= JointBus::QQUEUE_PREPARED_VALID;
  }
  if (jointBusPreparedSegment.valid && jointBusPreparedSegment.hold) {
    outStatus.flags |= JointBus::QQUEUE_PREPARED_HOLD;
  }
  if (jointBusActiveSegmentValid && jointBusActiveSegmentHold) {
    outStatus.flags |= JointBus::QQUEUE_ACTIVE_HOLD;
  }
  if (jointBusScheduledStart.valid) {
    outStatus.flags |= JointBus::QQUEUE_START_PENDING;
  }
  if (jointBusActiveSegmentValid && jointBusMotionBusy()) {
    outStatus.flags |= JointBus::QQUEUE_ACTIVE_BUSY;
  }

  return true;
}

static bool jointBusMotionState(void* context,
                                JointBus::MotionState& outState)
{
  (void)context;
  jointBusSegmentQueueUpdate();

  outState.activeSegmentId = jointBusActiveSegmentValid
      ? jointBusActiveSegmentId
      : JointBus::NO_SEGMENT_ID;
  outState.flags = jointBusActiveMotionFlags;
  if (jointBusActiveSegmentValid) {
    outState.flags |= JointBus::MSTATE_ACTIVE_VALID;
  }
  if (jointBusActiveSegmentValid && jointBusActiveSegmentHold) {
    outState.flags |= JointBus::MSTATE_HOLD_AXIS;
  }
  if (jointCtrl.trajectoryActive()) {
    outState.flags |= JointBus::MSTATE_TRAJECTORY_ACTIVE;
  } else if (jointBusActiveSegmentValid &&
             positionMotionCommandBusy(motionLifecycle)) {
    outState.flags |= JointBus::MSTATE_SERVO_SETTLING;
  }

  outState.refPosCdeg = jointBusDegToCdeg(jointCtrl.refPos());
  outState.refVelCdegS = jointBusDegToCdeg(jointCtrl.refVel());
  outState.refAccDdegS2 = jointBusDegToDdeg(jointCtrl.refAcc());
  if (jointBusActiveSegmentValid && jointBusActiveSegmentHold) {
    outState.elapsedMs = std::min<uint32_t>(
      static_cast<uint32_t>(millis() - jointBusActiveSegmentStartedMs),
      jointBusActiveSegmentDurationMs);
    outState.durationMs = jointBusActiveSegmentDurationMs;
  } else {
    outState.elapsedMs = jointBusSecondsToMilliseconds(
        jointCtrl.trajectoryElapsed());
    outState.durationMs = jointBusSecondsToMilliseconds(
        jointCtrl.trajectoryDuration());
  }
  return true;
}

static JointBus::CommandResult jointBusHome(void* context)
{
  (void)context;

  // HOME is intentionally not an implicit PARK. In multi-turn mode the joint
  // must already be referenced by a completed park cycle before HOME can move
  // to the logical zero position.
  if (!jointReferenced) {
    LOG_NFO("JointBus HOME rejected: execute park first\r\n");
    return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
  }

  const JointMoveOutcome outcome = jointMoveTo(JOINTBUS_HOME_TARGET_DEG,
                                               JOINTBUS_HOME_VMAX_DEG_S,
                                               JOINTBUS_HOME_AMAX_DEG_S2);
  if (!outcome.accepted()) {
    return JointBus::CommandResult::fail(jointBusNackForMoveResult(outcome.result));
  }

  LOG_NFO("JointBus HOME accepted: target=%.3f deg vmax=%.3f deg/s amax=%.3f deg/s2\r\n",
          JOINTBUS_HOME_TARGET_DEG,
          JOINTBUS_HOME_VMAX_DEG_S,
          JOINTBUS_HOME_AMAX_DEG_S2);
  jointBusClearCoordinatedSegments();
  return JointBus::CommandResult::ok(
    jointBusAckForMoveOutcome(outcome, JointBus::AckCode::Accepted, false));
}

static JointBus::CommandResult jointBusZero(void* context)
{
  (void)context;

  if (!jointReferenced) {
    return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
  }

  if (!setZero()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
  }

  jointBusClearCoordinatedSegments();
  return JointBus::CommandResult::ok(JointBus::AckCode::Accepted);
}

static JointBus::CommandResult jointBusPark(void* context)
{
  (void)context;

  if (startPark()) {
    jointBusClearCoordinatedSegments();
    return JointBus::CommandResult::ok(JointBus::AckCode::Accepted);
  }

  if (jointHasFault()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
  }
  return JointBus::CommandResult::fail(JointBus::NackCode::RejectedByState);
}

static JointBus::CommandResult jointBusStop(void* context)
{
  (void)context;
  if (!jointStop().accepted()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
  }
  jointBusClearCoordinatedSegments();
  return JointBus::CommandResult::ok(JointBus::AckCode::Accepted);
}

static JointBus::CommandResult jointBusHoldPosition(void* context)
{
  (void)context;

  if (jointHasFault()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
  }
  if (!jointReferenced) {
    return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
  }
  if (!encoderOk || !tmcReady) {
    return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
  }
  if (nonPositionOperationBusy()) {
    return JointBus::CommandResult::fail(JointBus::NackCode::Busy);
  }

  const float holdPositionDeg = jointGetPositionDeg();
  if (!ensureDriverEnabled() ||
      !setMotorVelocityDegPerSecond(0.0f)) {
    return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
  }
  tmc.stopInternalMotion();

  jointCtrl.reset(holdPositionDeg);
  jointCtrl.setTarget(holdPositionDeg);
  servoTargetZeroedDeg = holdPositionDeg;
  servoLastCmdDegS = 0.0f;
  lastServoUs = micros();
  motionLifecycle.beginForcedServoHold();
  motionMode = MotionMode::POSITION;
  jointBusClearCoordinatedSegments();
  wsSetState(LedState::READY);

  LOG_NFO("JointBus hold position zeroed=%.3f deg\r\n", holdPositionDeg);
  return JointBus::CommandResult::ok(JointBus::AckCode::Accepted);
}


static JointBus::CommandResult jointBusEmergencyStop(void* context)
{
  (void)context;
  latchEmergencyStopFault("JointBus");
  return JointBus::CommandResult::ok(JointBus::AckCode::EmergencyStopped);
}

static JointBus::CommandResult jointBusClearFault(void* context)
{
  (void)context;
  const FaultClearResult result = clearMotionFault();
  switch (result) {
    case FaultClearResult::Cleared:
      return JointBus::CommandResult::ok(JointBus::AckCode::FaultCleared);
    case FaultClearResult::AlreadyClear:
      return JointBus::CommandResult::ok(JointBus::AckCode::AlreadyDone);
    case FaultClearResult::InvalidLimits:
      return JointBus::CommandResult::fail(
          JointBus::NackCode::BadPayload,
          static_cast<uint8_t>(JointBus::JointFault::PlannerError));
    case FaultClearResult::PositionOutsideLimits:
      return JointBus::CommandResult::fail(
          JointBus::NackCode::RejectedByState,
          static_cast<uint8_t>(JointBus::JointFault::PositionLimit));
    case FaultClearResult::EncoderUnavailable:
      return JointBus::CommandResult::fail(
          JointBus::NackCode::RejectedByState,
          static_cast<uint8_t>(JointBus::JointFault::EncoderError));
    case FaultClearResult::DriverUnavailable:
      return JointBus::CommandResult::fail(
          JointBus::NackCode::RejectedByState,
          static_cast<uint8_t>(JointBus::JointFault::DriverError));
  }
  return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
}

static JointBus::CommandResult jointBusServoMove(void* context,
                                                 uint16_t position,
                                                 uint8_t speed)
{
  (void)context;
  switch (moveHobbyServo(position, speed)) {
    case HobbyServoMotion::MoveResult::Accepted:
      return JointBus::CommandResult::ok(JointBus::AckCode::Accepted);
    case HobbyServoMotion::MoveResult::InvalidArgument:
      return JointBus::CommandResult::fail(JointBus::NackCode::BadPayload);
    case HobbyServoMotion::MoveResult::Disabled:
    case HobbyServoMotion::MoveResult::InvalidConfig:
      return JointBus::CommandResult::fail(JointBus::NackCode::RejectedByState);
    case HobbyServoMotion::MoveResult::PwmError:
      return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
  }
  return JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
}

static JointBus::CommandResult jointBusReboot(void* context, uint16_t magic)
{
  (void)context;

  if (magic != JointBus::REBOOT_MAGIC) {
    return JointBus::CommandResult::fail(JointBus::NackCode::BadPayload);
  }

  jointBusRebootPending = true;
  jointBusRebootRequestedMs = millis();
  return JointBus::CommandResult::ok(JointBus::AckCode::Accepted);
}

static JointBus::JointState jointBusCurrentState()
{
  if (jointHasFault() || motionMode == MotionMode::FAULT) {
    return JointBus::JointState::Fault;
  }

  if (motionMode == MotionMode::PARK) {
    return JointBus::JointState::Parking;
  }

  if (motionMode == MotionMode::POSITION) {
    switch (positionLifecycleExternalState(motionLifecycle, false)) {
      case PositionLifecycleExternalState::Moving:
        return JointBus::JointState::Moving;
      case PositionLifecycleExternalState::Holding:
        return JointBus::JointState::Holding;
      case PositionLifecycleExternalState::Fault:
        return JointBus::JointState::Fault;
    }
  }

  if (motionMode == MotionMode::CALIBRATION ||
      motionMode == MotionMode::VELOCITY_TEST ||
      motionMode == MotionMode::STEP_TEST) {
    return JointBus::JointState::Moving;
  }

  if (!jointReferenced) {
    return JointBus::JointState::Init;
  }

  const Tmc2209Driver::Status driverStatus = tmc.status();
  if (motionLifecycle.servoHoldActive ||
      driverStatus == Tmc2209Driver::Status::Enabled) {
    return JointBus::JointState::Holding;
  }

  return JointBus::JointState::Ready;
}

static JointBus::JointFault jointBusCurrentFault()
{
  if (!encoderOk) {
    return JointBus::JointFault::EncoderError;
  }

  if (motionMode == MotionMode::FAULT || jointCtrl.fault()) {
    if (jointCtrl.faultCode() == SCurvePosVelController::FaultCode::PositionLimitExceeded) {
      return JointBus::JointFault::PositionLimit;
    }
    if (jointCtrl.faultCode() == SCurvePosVelController::FaultCode::BadLimits) {
      return JointBus::JointFault::PlannerError;
    }
    if (jointCtrl.faultCode() == SCurvePosVelController::FaultCode::EmergencyStop) {
      return JointBus::JointFault::EmergencyStop;
    }
    return JointBus::JointFault::InternalError;
  }

  if (!tmcReady) {
    return JointBus::JointFault::DriverError;
  }

  return JointBus::JointFault::None;
}

static bool jointBusStatus(void* context, JointBus::Status& outStatus)
{
  (void)context;

  outStatus.posCdeg = jointBusDegToCdeg(jointGetPositionDeg());
  outStatus.targetCdeg = jointBusDegToCdeg(jointGetTargetDeg());
  outStatus.velCdegS = jointBusDegToCdeg(jointGetMeasuredVelocityDegS());
  outStatus.state = jointBusCurrentState();
  outStatus.fault = jointBusCurrentFault();
  return true;
}

static bool jointBusQuickStatus(void* context, uint8_t& outQuickStatus)
{
  (void)context;

  uint8_t flags = 0;
  const bool fault = jointHasFault() || motionMode == MotionMode::FAULT || !encoderOk || !tmcReady;
  const MotionCommandExternalStatus commandStatus =
      motionCommandExternalStatus(
          motionLifecycle,
          nonPositionOperationBusy(),
          fault);

  if (commandStatus.busy) {
    flags |= JointBus::QSTAT_BUSY;
  }
  if (commandStatus.done) {
    flags |= JointBus::QSTAT_DONE;
  }
  if (commandStatus.fault) {
    flags |= JointBus::QSTAT_FAULT;
  }
  if (tmc.status() == Tmc2209Driver::Status::Enabled) {
    flags |= JointBus::QSTAT_ENABLED;
  }
  if (jointReferenced) {
    flags |= JointBus::QSTAT_HOMED;
  }
  if (jointBusLastLimitClipped) {
    flags |= JointBus::QSTAT_WARNING;
    flags |= JointBus::QSTAT_LIMIT_CLIPPED;
  }

  outQuickStatus = flags;
  return true;
}

static bool jointBusMotionConfig(void* context, JointBus::MotionConfig& outConfig)
{
  (void)context;

  float jmin = 0.0f;
  float jmax = 0.0f;
  float jtol = 0.0f;
  float vmax = 0.0f;
  float amax = 0.0f;

  if (!readJointLimitParams(jmin, jmax, jtol) ||
      !params.get("vmax", vmax) || !params.get("amax", amax) ||
      !isfinite(vmax) || !isfinite(amax) || vmax <= 0.0f || amax <= 0.0f) {
    return false;
  }

  return jointBusDegToCdegChecked(jmin, outConfig.jminCdeg) &&
         jointBusDegToCdegChecked(jmax, outConfig.jmaxCdeg) &&
         jointBusPositiveToCentiChecked(vmax, outConfig.vmaxCdegS) &&
         jointBusPositiveToCentiChecked(amax, outConfig.amaxCdegS2);
}

static void setupJointBusHooks()
{
  JointBus::SlaveHooks hooks;
  hooks.context = nullptr;
  hooks.move = jointBusMove;
  hooks.moveb = jointBusMoveB;
  hooks.prepareMoveB = jointBusPrepareMoveB;
  hooks.prepareHold = jointBusPrepareHold;
  hooks.startSegment = jointBusStartSegment;
  hooks.scheduleSegment = jointBusScheduleSegment;
  hooks.abortSegment = jointBusAbortSegment;
  hooks.queueStatus = jointBusQueueStatus;
  hooks.motionState = jointBusMotionState;
  hooks.segmentTiming = jointBusSegmentTiming;
  hooks.home = jointBusHome;
  hooks.zero = jointBusZero;
  hooks.park = jointBusPark;
  hooks.stop = jointBusStop;
  hooks.holdPosition = jointBusHoldPosition;
  hooks.emergencyStop = jointBusEmergencyStop;
  hooks.clearFault = jointBusClearFault;
  hooks.servoMove = jointBusServoMove;
  hooks.reboot = jointBusReboot;
  hooks.status = jointBusStatus;
  hooks.quickStatus = jointBusQuickStatus;
  hooks.motionConfig = jointBusMotionConfig;

  jointBus.setHooks(hooks);
}

// ===================== PARK REFERENCE =====================

static float normalizeEncoderAngle(float degrees)
{
  float out = fmodf(degrees, 360.0f);
  if (out < 0.0f) {
    out += 360.0f;
  }
  return out;
}

static float directedEncoderDistance(float currentDeg, float targetDeg, int direction)
{
  const float current = normalizeEncoderAngle(currentDeg);
  const float target = normalizeEncoderAngle(targetDeg);

  if (direction > 0) {
    return normalizeEncoderAngle(target - current);
  }
  return normalizeEncoderAngle(current - target);
}

static void abortPark(const char* reason)
{
  clearPositionLifecycleState();
  setMotorVelocityDegPerSecond(0.0f);
  tmc.stopInternalMotion();
  tmc.disableDriver(false);
  parkPhase = ParkPhase::IDLE;
  parkCommandVelocityDegS = 0.0f;
  jointReferenced = false;
  motionMode = MotionMode::IDLE;
  jointCtrl.reset(jointGetPositionDeg());
  jointCtrl.setTarget(jointGetPositionDeg());
  servoTargetZeroedDeg = jointGetPositionDeg();
  wsSetState(LedState::BOOT);
  LOG_ERR("Park aborted: %s\r\n", reason != nullptr ? reason : "unknown reason");
}

bool startPark()
{
  if (motionMode == MotionMode::PARK) {
    LOG_NFO("Park already running\r\n");
    return false;
  }

  if (motionMode == MotionMode::FAULT || jointCtrl.fault()) {
    LOG_ERR("Park rejected: motion fault is latched\r\n");
    return false;
  }

  const float directionValue = readParamFloatOrDefault("pkdir", 0.0f);
  const float velocity = readParamFloatOrDefault("pkvel", PARK_DEFAULT_VELOCITY_DEG_S);
  const float encoderTarget = readParamFloatOrDefault("pkenc", PARK_DEFAULT_ENCODER_ANGLE_DEG);
  const float parkPosition = readParamFloatOrDefault("pkpos", PARK_DEFAULT_JOINT_POSITION_DEG);

  if (!isfinite(directionValue) ||
      (directionValue != -1.0f && directionValue != 0.0f && directionValue != 1.0f)) {
    LOG_ERR("Park rejected: pkdir must be -1, 0, or +1\r\n");
    return false;
  }
  if (directionValue == 0.0f) {
    LOG_NFO("Park not required: pkdir=0 selects absolute-encoder mode\r\n");
    return false;
  }
  if (!isfinite(velocity) || velocity <= 0.0f) {
    LOG_ERR("Park rejected: pkvel must be > 0\r\n");
    return false;
  }
  if (!isfinite(encoderTarget) || encoderTarget < 0.0f || encoderTarget >= 360.0f) {
    LOG_ERR("Park rejected: pkenc must be in [0, 360) degrees\r\n");
    return false;
  }
  if (!isfinite(parkPosition)) {
    LOG_ERR("Park rejected: pkpos must be finite\r\n");
    return false;
  }

  stopMotion();
  if (!ensureDriverEnabled()) {
    LOG_ERR("Park rejected: unable to enable TMC power stage\r\n");
    return false;
  }

  jointReferenced = false;
  parkStartedMs = millis();
  parkPhaseStartedMs = parkStartedMs;
  parkCommandVelocityDegS = 0.0f;
  parkPrevSensorActive = digitalRead(PIN_PARK_SENSOR) == LOW;

  // If the sensor is already active, first back out in the opposite direction.
  // This guarantees that the following reference capture always uses the same
  // falling edge and avoids relying on the sensor active-window hysteresis.
  parkPhase = parkPrevSensorActive
      ? ParkPhase::RELEASE_SENSOR
      : ParkPhase::SEARCH_FALLING_EDGE;
  motionMode = MotionMode::PARK;
  lastServoUs = micros();
  wsSetState(LedState::TEST);

  LOG_NFO("Park started: direction=%+.0f velocity=%.3f joint deg/s pkenc=%.3f encoder deg pkpos=%.3f joint deg sensor=%s\r\n",
          directionValue,
          velocity,
          encoderTarget,
          parkPosition,
          parkPrevSensorActive ? "ACTIVE; releasing opposite to pkdir" : "inactive; searching falling edge");
  return true;
}

static void completePark()
{
  clearPositionLifecycleState();
  const float parkPosition = readParamFloatOrDefault("pkpos", PARK_DEFAULT_JOINT_POSITION_DEG);
  const float absoluteJointAtPark = encoderZero + parkPosition;

  setMotorVelocityDegPerSecond(0.0f);
  tmc.stopInternalMotion();
  if (!motorHoldEnabled()) {
    tmc.disableDriver(false);
  }

  encoder.setContinuousOutputDegrees(absoluteJointAtPark);
  jointDeg = encoder.lastContinuousDegrees();
  jointReferenced = true;
  parkPhase = ParkPhase::IDLE;
  parkCommandVelocityDegS = 0.0f;
  motionMode = MotionMode::IDLE;

  const float referencedPosition = jointGetPositionDeg();
  jointCtrl.clearFault();
  jointCtrl.reset(referencedPosition);
  jointCtrl.setTarget(referencedPosition);
  servoTargetZeroedDeg = referencedPosition;
  wsSetState(LedState::READY);

  LOG_NFO("Park complete: encoder=%.3f deg joint=%.6f deg zeroed=%.6f deg target rebased\r\n",
          encoderDeg,
          jointDeg,
          referencedPosition);
}

static void parkUpdate(float dt)
{
  if (parkPhase == ParkPhase::IDLE) {
    abortPark("invalid internal state");
    return;
  }

  const uint32_t nowMs = millis();
  const int searchDirection = readParamFloatOrDefault("pkdir", 0.0f) >= 0.0f ? 1 : -1;
  int motionDirection = searchDirection;
  const float parkVelocity = readParamMinOrDefault("pkvel", PARK_DEFAULT_VELOCITY_DEG_S, 0.001f);
  const bool sensorActive = digitalRead(PIN_PARK_SENSOR) == LOW;

  float requestedSpeedMagnitude = parkVelocity;

  if (parkPhase == ParkPhase::RELEASE_SENSOR) {
    motionDirection = -searchDirection;
    if (nowMs - parkPhaseStartedMs > PARK_RELEASE_TIMEOUT_MS) {
      abortPark("timeout releasing park sensor");
      return;
    }
    if (!sensorActive) {
      parkPhase = ParkPhase::SEARCH_FALLING_EDGE;
      parkPhaseStartedMs = nowMs;
      parkPrevSensorActive = false;
      parkCommandVelocityDegS = 0.0f;
      LOG_NFO("Park sensor released; searching falling edge in pkdir\r\n");
    }
  }

  if (parkPhase == ParkPhase::SEARCH_FALLING_EDGE) {
    motionDirection = searchDirection;
    if (nowMs - parkPhaseStartedMs > PARK_SEARCH_TIMEOUT_MS) {
      abortPark("timeout searching park falling edge");
      return;
    }
    if (!parkPrevSensorActive && sensorActive) {
      parkPhase = ParkPhase::ALIGN_ENCODER;
      parkPhaseStartedMs = nowMs;
      parkCommandVelocityDegS = 0.0f;
      LOG_NFO("Park falling edge detected at encoder %.3f deg; aligning to pkenc\r\n", encoderDeg);
    }
  }

  if (parkPhase == ParkPhase::ALIGN_ENCODER) {
    motionDirection = searchDirection;
    if (nowMs - parkPhaseStartedMs > PARK_ALIGN_TIMEOUT_MS) {
      abortPark("timeout aligning to park encoder angle");
      return;
    }

    const float encoderTarget = readParamFloatOrDefault("pkenc", PARK_DEFAULT_ENCODER_ANGLE_DEG);
    const float remainingEncoderDeg = directedEncoderDistance(encoderDeg, encoderTarget, searchDirection);

    if (remainingEncoderDeg <= PARK_ENCODER_TOLERANCE_DEG) {
      completePark();
      return;
    }

    // Convert the remaining modulo-encoder distance into real joint degrees.
    // This allows a fixed park acceleration to provide both ramp-up and a
    // controlled deceleration toward the precise encoder target.
    const float remainingJointDeg =
        remainingEncoderDeg * encoder.outputDegreesPerEncoderRevolution() / 360.0f;
    const float brakingSpeed = sqrtf(fmaxf(0.0f, 2.0f * PARK_ACCEL_DEG_S2 * remainingJointDeg));
    requestedSpeedMagnitude = fminf(parkVelocity, brakingSpeed);
  }

  const float targetVelocity = static_cast<float>(motionDirection) * requestedSpeedMagnitude;
  const float maxVelocityDelta = PARK_ACCEL_DEG_S2 * dt;

  if (parkCommandVelocityDegS < targetVelocity) {
    parkCommandVelocityDegS = fminf(parkCommandVelocityDegS + maxVelocityDelta, targetVelocity);
  } else if (parkCommandVelocityDegS > targetVelocity) {
    parkCommandVelocityDegS = fmaxf(parkCommandVelocityDegS - maxVelocityDelta, targetVelocity);
  }

  parkPrevSensorActive = sensorActive;

  if (!setMotorVelocityDegPerSecond(parkCommandVelocityDegS)) {
    abortPark("motor velocity command failed");
  }
}

// ===================== SERVO UPDATE =====================

void servoUpdate()
{
  const uint32_t nowUs = micros();

  if (static_cast<uint32_t>(nowUs - lastServoUs) < SERVO_CONTROL_PERIOD_US) {
    return;
  }

  const float dt = static_cast<float>(static_cast<uint32_t>(nowUs - lastServoUs)) * 1e-6f;
  lastServoUs = nowUs;

  float measured = 0.0f;
  encoderOk = encoder.readContinuousDegrees(measured);
  encoderRaw = encoder.lastRaw();
  encoderDeg = encoder.lastDegrees();

  if (!encoderOk) {
    clearPositionLifecycleState();
    setMotorVelocityDegPerSecond(0.0f);
    tmc.disableDriver(false);
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    LOG_ERR("Encoder read failed during control\r\n");
    return;
  }

  jointDeg = measured;

  if (motionMode == MotionMode::PARK) {
    parkUpdate(dt);
    return;
  }

  const float currentZeroedDegForLimits = jointGetPositionDeg();
  if (motionMode != MotionMode::FAULT && isJointPositionOutsideFaultWindow(currentZeroedDegForLimits)) {
    latchJointLimitFault(currentZeroedDegForLimits);
    return;
  }

  if (motionMode != MotionMode::POSITION) {
    return;
  }

  const float currentZeroedDeg = jointGetPositionDeg();
  const float vCmdDegS = jointCtrl.update(currentZeroedDeg, dt);

  if (!setMotorVelocityDegPerSecond(vCmdDegS)) {
    clearPositionLifecycleState();
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    return;
  }

  if (jointCtrl.fault()) {
    clearPositionLifecycleState();
    setMotorVelocityDegPerSecond(0.0f);
    tmc.disableDriver(false);
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    LOG_ERR("Joint controller fault code=%u\r\n", static_cast<unsigned>(jointCtrl.faultCode()));
    return;
  }

  const bool settledNow = jointCtrl.isSettled();
  const bool inDeadband = jointCtrl.inDeadband();
  const bool holdEnabled =
      servoHoldEnabled() || motionLifecycle.forcedServoHold;
  const MotionLifecycleEvent lifecycleEvent =
    updateMotionLifecycle(motionLifecycle,
                          settledNow,
                          holdEnabled,
                          inDeadband);

  if (lifecycleEvent == MotionLifecycleEvent::CommandCompletedHold) {
    wsSetState(LedState::READY);
    LOG_NFO("Move complete zeroed=%.3f deg servo-hold active\r\n",
            jointGetPositionDeg());
  } else if (lifecycleEvent ==
             MotionLifecycleEvent::CommandCompletedNoHold) {
    setMotorVelocityDegPerSecond(0.0f);
    tmc.stopInternalMotion();
    servoLastCmdDegS = 0.0f;

    if (!motorHoldEnabled()) {
      tmc.disableDriver(false);
    }

    motionMode = MotionMode::IDLE;
    motionLifecycle.disableServoHold();
    wsSetState(LedState::READY);
    LOG_NFO("Move complete zeroed=%.3f deg motor=%s\r\n",
            jointGetPositionDeg(),
            motorHoldEnabled() ? "hold" : "disabled");
    return;
  } else if (lifecycleEvent ==
             MotionLifecycleEvent::ServoCorrectionStarted) {
    LOG_NFO("Servo correction started error=%.3f deg cmd=%.3f deg/s\r\n",
            jointCtrl.target() - jointGetPositionDeg(),
            servoLastCmdDegS);
  } else if (lifecycleEvent ==
             MotionLifecycleEvent::ServoCorrectionCompleted) {
    LOG_NFO("Servo correction complete zeroed=%.3f deg error=%.3f deg\r\n",
            jointGetPositionDeg(),
            jointCtrl.target() - jointGetPositionDeg());
  }

  if (motionLifecycle.servoHoldActive && holdEnabled) {
    if (inDeadband) {
      setMotorVelocityDegPerSecond(0.0f);
      tmc.stopInternalMotion();
      servoLastCmdDegS = 0.0f;
    }

    if (motionLifecycle.servoCorrectionActive) {
      const uint32_t nowMs = millis();
      if (motionLifecycle.lastServoCorrectionDebugMs == 0 ||
          static_cast<uint32_t>(
            nowMs - motionLifecycle.lastServoCorrectionDebugMs) >= 250U) {
        motionLifecycle.lastServoCorrectionDebugMs = nowMs;
        LOG_DBG("Servo correction error=%.3f deg meas_vel=%.3f deg/s cmd=%.3f deg/s\r\n",
                jointCtrl.target() - jointGetPositionDeg(),
                jointGetMeasuredVelocityDegS(),
                servoLastCmdDegS);
      }
    }
  }
}

// ===================== SETUP =====================

void setup()
{
  JointBus::Slave::forceRs485Inactive(PIN_RS485_RTS);

  wsLedsInit();
  pinMode(PIN_PARK_SENSOR, INPUT_PULLUP);
  jointReferenced = false;
  wsSetState(LedState::BOOT);
  wsLedsUpdate();

  Serial.begin(USB_BAUD);
  delay(500);

  Serial.println();

  // Load and apply persistent loglvl before normal boot diagnostics. If NVS
  // initialization itself fails, the compile-time logger level reports it.
  paramsInit();

  #if MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5048A
  LOG_NFO("PlanetJoint ESP32-S3 - AS5048A analytic quintic S-curve build\r\n");
#else
  LOG_NFO("PlanetJoint ESP32-S3 - AS5600 analytic quintic S-curve build\r\n");
#endif
  LOG_NFO("USB CDC console ready\r\n");

  setupJointBusHooks();
  jointBus.setAddress(readJointBusAddressFromParams());
  SerialJointBus.setRxBufferSize(JOINTBUS_RX_BUFFER_SIZE);
  if (jointBus.begin(JOINTBUS_BAUD)) {
    jointBus.flushRx();
    LOG_NFO("JointBus slave initialized on UART0, baud=%lu, addr=%u, RTS/DE GPIO=%d, hw_rs485=%u\r\n",
            static_cast<unsigned long>(JOINTBUS_BAUD),
            static_cast<unsigned>(jointBus.address()),
            PIN_RS485_RTS,
            jointBus.hardwareRs485Enabled() ? 1u : 0u);
  } else {
    LOG_ERR("JointBus slave initialization failed\r\n");
  }

  if (!tmcInit()) {
    LOG_ERR("Failed to initialize TMC2209 driver\r\n");
    clearPositionLifecycleState();
    wsSetState(LedState::FAULT);
    motionMode = MotionMode::FAULT;
  } else {
    tmcReady = true;
    LOG_NFO("TMC2209 driver initialized\r\n");
    applyAllParams();
  }

  encoderInit();

  if (encoderFirstReadTest()) {
    // Keep the persisted logical zero. Initialize the controller and its target
    // at the actual measured position so boot never requests an unsolicited move.
    const float startupPositionDeg = jointGetPositionDeg();
    jointControllerInit(startupPositionDeg);
    jointCtrl.setTarget(startupPositionDeg);
    servoTargetZeroedDeg = startupPositionDeg;
    const float parkDirection = readParamFloatOrDefault("pkdir", 0.0f);
    if (parkDirection == 0.0f) {
      jointReferenced = true;
      wsSetState(LedState::READY);
      LOG_NFO("Startup encoder sample %.3f deg; pkdir=0 absolute-encoder mode, park is not required\r\n",
              startupPositionDeg);
    } else {
      jointReferenced = false;
      wsSetState(LedState::BOOT);
      LOG_NFO("Startup encoder sample %.3f deg; controller target initialized, but motion remains locked until park\r\n",
              startupPositionDeg);
    }
  } else {
    clearPositionLifecycleState();
    wsSetState(LedState::ENCODER_ERROR);
    motionMode = MotionMode::FAULT;
  }

  LOG_NFO("Init complete\r\n");
  if (readParamFloatOrDefault("pkdir", 0.0f) == 0.0f) {
    LOG_NFO("Absolute-encoder mode active: park command is not required.\r\n");
  } else {
    LOG_NFO("Multi-turn position is not valid after boot: execute park before any motion.\r\n");
  }
  LOG_NFO("Use zero, then save, to store a new logical zero.\r\n");
  LOG_NFO("Direction signs: mdir=%+.0f edir=%+.0f.\r\n",
          activeMotorDirectionSign, encoder.directionSign());
  LOG_NFO("Runtime params: kp ki kd ffv ilim vmax amax outmax ptol vtol dbent dbext dbvel vtau stdeg jrev mdir edir jmin jmax jtol loglvl mhold shold zoff pkdir pkvel pkenc pkpos servo srvzero srvmin srvmax addr.\r\n");
  LOG_NFO("trace toggles on/off; trace 0..4 selects output mode.\r\n");
  LOG_NFO("Example: set kp 1.0 / set vmax 3.0 / save\r\n");

  console.setParamSetCallback(onConsoleParamSet);
  console.begin("pj> ");
}

// ===================== LOOP =====================

void loop()
{
  console.update();
  jointBus.update();
  // Sample time after command dispatch. A servo command can set startedMs
  // while either update() runs, so a timestamp captured before them could
  // otherwise look older and complete the ramp immediately by unsigned wrap.
  const uint32_t now = millis();
  hobbyServoUpdate(now);
  jointBusScheduledStartUpdate();

  if (jointBusRebootPending && static_cast<uint32_t>(now - jointBusRebootRequestedMs) >= 150U) {
    Serial.flush();
    ESP.restart();
  }

  wsLedsUpdate();
  servoUpdate();
  jointBusSegmentQueueUpdate();

  if (traceEnabled && now - lastEncoderPrintMs >= ENCODER_PRINT_PERIOD_MS) {
    lastEncoderPrintMs = now;

    const float jointZeroedDeg = jointGetPositionDeg();
    const float targetDeg = jointCtrl.target();
    const float refDeg = jointCtrl.refPos();
    const float errDeg = targetDeg - jointZeroedDeg;
    const float measVel = jointCtrl.getLastMeasuredVel();
    const float refVel = jointCtrl.refVel();
    const float refAcc = jointCtrl.refAcc();
    const uint8_t settled = jointCtrl.isSettled() ? 1 : 0;
    const uint8_t profileActive = jointCtrl.trajectoryActive() ? 1 : 0;
    float traceJmin = 0.0f;
    float traceJmax = 0.0f;
    float traceJtol = 0.0f;
    readJointLimitParams(traceJmin, traceJmax, traceJtol);

    switch (traceMode) {
      case TraceMode::FULL:
        Serial.print("@enc_deg:");
        Serial.print(encoderDeg, 3);

        Serial.print(",joint_deg:");
        Serial.print(jointDeg, 3);

        Serial.print(",joint_zeroed_deg:");
        Serial.print(jointZeroedDeg, 3);

        Serial.print(",target_deg:");
        Serial.print(targetDeg, 3);

        Serial.print(",err_deg:");
        Serial.print(errDeg, 3);

        Serial.print(",ref_deg:");
        Serial.print(refDeg, 3);

        Serial.print(",ref_vel:");
        Serial.print(refVel, 3);

        Serial.print(",ref_acc:");
        Serial.print(refAcc, 3);

        Serial.print(",profile:");
        Serial.print(profileActive);

        Serial.print(",profile_elapsed:");
        Serial.print(jointCtrl.trajectoryElapsed(), 3);

        Serial.print(",profile_duration:");
        Serial.print(jointCtrl.trajectoryDuration(), 3);

        Serial.print(",meas_vel:");
        Serial.print(measVel, 3);

        Serial.print(",cmd_deg_s:");
        Serial.print(servoLastCmdDegS, 3);

        Serial.print(",settled:");
        Serial.print(settled);

        Serial.print(",stdeg:");
        Serial.print(stepsPerDegree(), 6);

        Serial.print(",jrev:");
        Serial.print(encoder.outputDegreesPerEncoderRevolution(), 6);

        Serial.print(",jmin:");
        Serial.print(traceJmin, 3);

        Serial.print(",jmax:");
        Serial.print(traceJmax, 3);

        Serial.print(",jtol:");
        Serial.print(traceJtol, 3);

        Serial.print(",mode:");
        Serial.print(motionModeName(motionMode));

        Serial.print(",ok:");
        Serial.print(encoderOk ? 1 : 0);

        Serial.print(",p:");
        Serial.print(encoder.lastParityOk() ? 1 : 0);

        Serial.print(",err:");
        Serial.println(encoder.lastErrorFlag() ? 1 : 0);
        break;

      case TraceMode::POS_TARGET:
        Serial.print("@joint_zeroed_deg:");
        Serial.print(jointZeroedDeg, 3);
        Serial.print(",target_deg:");
        Serial.println(targetDeg, 3);
        break;

      case TraceMode::POS_TARGET_VEL:
        Serial.print("@joint_zeroed_deg:");
        Serial.print(jointZeroedDeg, 3);
        Serial.print(",target_deg:");
        Serial.print(targetDeg, 3);
        Serial.print(",cmd_deg_s:");
        Serial.print(servoLastCmdDegS, 3);
        Serial.print(",meas_vel:");
        Serial.println(measVel, 3);
        break;

      case TraceMode::PID:
        Serial.print("@err_deg:");
        Serial.print(errDeg, 3);
        Serial.print(",cmd_deg_s:");
        Serial.print(servoLastCmdDegS, 3);
        Serial.print(",meas_vel:");
        Serial.println(measVel, 3);
        break;

      case TraceMode::SCURVE:
        Serial.print("@joint_zeroed_deg:");
        Serial.print(jointZeroedDeg, 3);
        Serial.print(",ref_deg:");
        Serial.print(refDeg, 3);
        Serial.print(",target_deg:");
        Serial.print(targetDeg, 3);
        Serial.print(",ref_vel:");
        Serial.print(refVel, 3);
        Serial.print(",ref_acc:");
        Serial.print(refAcc, 3);
        Serial.print(",cmd_deg_s:");
        Serial.println(servoLastCmdDegS, 3);
        break;
    }
  }

  if (testStepEnabled && now - lastTestMs >= TEST_PERIOD_MS) {
    lastTestMs = now;

    if (stepNum > 0) {
      tmc.step(testForward, 100);
      stepNum--;
    } else {
      moveStep(0);
    }
  }
}
