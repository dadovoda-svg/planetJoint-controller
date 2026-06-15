#include <Arduino.h>
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
#include "logger.h"
#include "JointBusSlave.h"

// ===================== BAUDRATE =====================

static constexpr uint32_t USB_BAUD      = 115200;
static constexpr uint32_t JOINTBUS_BAUD = 500000;
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

// Direction correction between positive controller output and real positive joint motion.
// If the first low-speed test moves away from the target, change this to -1.0f.
static constexpr float MOTOR_DIRECTION_SIGN = 1.0f;

// ===================== VERY CONSERVATIVE SERVO DEFAULTS =====================

// Real joint was observed to start losing steps above ~15 deg/s.
// First closed-loop tests are intentionally much lower.
static constexpr uint32_t SERVO_CONTROL_PERIOD_US = 5000; // 200 Hz
static constexpr float SERVO_VMAX_DEG_S       = 2.0f;
static constexpr float SERVO_AMAX_DEG_S2      = 6.0f;
static constexpr float SERVO_SCURVE_TIME_S    = 0.150f;
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
JointPlanner planner(SerialJointBus);

static JointBus::SlaveHooks jointBusHooks;
static JointBus::Slave jointBus(SerialJointBus, 0, jointBusHooks, PIN_RS485_RTS);

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
static bool servoHoldActive = false;
bool tmcReady = false;

static bool jointBusLastLimitClipped = false;
static bool jointBusRebootPending = false;
static uint32_t jointBusRebootRequestedMs = 0;

MotionMode motionMode = MotionMode::IDLE;

// Forward declarations for helpers used by parameter callbacks.
float jointGetPositionDeg();
void stopMotion();
static void applyZeroOffsetFromParams(bool rebaseController);
static void parkUpdate(float dt);
static void abortPark(const char* reason);
static void setupJointBusHooks();
static uint8_t readJointBusAddressFromParams();
bool startPark();

// ===================== PARAMS =====================

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

bool jointClipTargetToLimits(float requestedDeg, float& clippedDeg)
{
  float jmin = 0.0f;
  float jmax = 0.0f;
  float jtol = 0.0f;

  if (!readJointLimitParams(jmin, jmax, jtol)) {
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
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
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    jointCtrl.setPositionLimits(JOINT_MIN_DEG_DEFAULT,
                                JOINT_MAX_DEG_DEFAULT,
                                JOINT_LIMIT_STOP_MARGIN_DEG,
                                JOINT_LIMIT_TOL_DEFAULT);
    LOG_ERR("Joint limit configuration invalid: motion fault latched\r\n");
    return false;
  }

  jointCtrl.setPositionLimits(jmin, jmax, JOINT_LIMIT_STOP_MARGIN_DEG, jtol);

  if (announce) {
    LOG_NFO("Joint limits: jmin=%.3f jmax=%.3f jtol=%.3f deg\r\n", jmin, jmax, jtol);
  }

  if (jointReferenced && encoderOk && isJointPositionOutsideFaultWindow(jointGetPositionDeg())) {
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

  // Recompute the current absolute joint angle from the already unwrapped
  // encoder angle using the new scale. This avoids waiting for the next
  // encoder sample and keeps status/trace coherent immediately after set jrev.
  const float continuousEncoderDeg = encoder.lastContinuousEncoderDegrees();
  jointDeg = (continuousEncoderDeg * jrev) / 360.0f;

  if (preserveZeroedPosition) {
    encoderZero = jointDeg - oldZeroed;
    jointCtrl.reset(oldZeroed);
    jointCtrl.setTarget(oldZeroed);
    servoTargetZeroedDeg = oldZeroed;
  }

  LOG_NFO("Encoder scale: 360.000 encoder deg = %.6f joint deg\r\n",
          encoder.outputDegreesPerEncoderRevolution());
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
  params.initKey("loglvl", 2.0f); // 0=OFF, 1=ERR, 2=NFO, 3=DBG
  params.initKey("mhold", 0.0f); // 0=disable driver at target, 1=keep driver enabled at IHOLD
  params.initKey("shold", 0.0f); // 0=stop control at target, 1=active servo hold at target
  params.initKey("zoff", 0.0f); // persistent absolute encoder position used as logical joint zero
  params.initKey("pkdir", 0.0f); // 0=absolute encoder, +1/-1=park search direction
  params.initKey("pkvel", PARK_DEFAULT_VELOCITY_DEG_S); // park speed in joint deg/s
  params.initKey("pkenc", PARK_DEFAULT_ENCODER_ANGLE_DEG); // final encoder modulo angle in deg
  params.initKey("pkpos", PARK_DEFAULT_JOINT_POSITION_DEG); // known logical joint position at park

  // Joint safety limits in zeroed real joint degrees.
  params.initKey("jmin", JOINT_MIN_DEG_DEFAULT);
  params.initKey("jmax", JOINT_MAX_DEG_DEFAULT);
  params.initKey("jtol", JOINT_LIMIT_TOL_DEFAULT); // allowed measured overshoot beyond jmin/jmax before fault

  // PID + S-curve controller parameters.
  // Key names are intentionally short because PersistentParams allows max 6 chars.
  params.initKey("kp", SERVO_KP);
  params.initKey("ki", SERVO_KI);
  params.initKey("kd", SERVO_KD);
  params.initKey("ffv", SERVO_FF_VEL);
  params.initKey("ilim", SERVO_I_LIMIT);

  params.initKey("vmax", SERVO_VMAX_DEG_S);
  params.initKey("amax", SERVO_AMAX_DEG_S2);
  params.initKey("sct", SERVO_SCURVE_TIME_S);
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

static void applyControllerMotionParamsFromParams()
{
  const float vmax = readParamMinOrDefault("vmax", SERVO_VMAX_DEG_S, 0.001f);
  const float amax = readParamMinOrDefault("amax", SERVO_AMAX_DEG_S2, 0.001f);
  const float sct = readParamMinOrDefault("sct", SERVO_SCURVE_TIME_S, 0.001f);
  const float outmax = readParamMinOrDefault("outmax", SERVO_OUTPUT_MAX_DEG_S, 0.0f);

  jointCtrl.setLimits(vmax, amax);
  jointCtrl.setSCurveTime(sct);
  jointCtrl.setOutputMax(outmax);

  LOG_NFO("Servo motion params: vmax=%.4f amax=%.4f sct=%.4f outmax=%.4f\r\n",
                vmax, amax, sct, outmax);
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
  applyControllerMotionParamsFromParams();
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
    jointCtrl.reset(currentZeroedDeg);
    jointCtrl.setTarget(currentZeroedDeg);
    servoTargetZeroedDeg = currentZeroedDeg;
  }

  LOG_NFO("Logical zero offset zoff=%.6f deg%s\r\n",
          encoderZero,
          rebaseController ? "; target rebased to current position" : "");
}

void applyAllParams()
{
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

  if (strcmp(key, "vmax") == 0 || strcmp(key, "amax") == 0 ||
      strcmp(key, "sct") == 0 || strcmp(key, "outmax") == 0) {
    applyControllerMotionParamsFromParams();
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

  if (strcmp(key, "shold") == 0) {
    servoHoldActive = false;
    LOG_NFO("servo hold at target %s. Use 'save' to persist it.\r\n",
            servoHoldEnabled() ? "enabled" : "disabled");
    return;
  }

  if (strcmp(key, "zoff") == 0) {
    applyZeroOffsetFromParams(true);
    LOG_NFO("Use 'save' to persist the logical zero offset.\r\n");
    return;
  }

  if (strcmp(key, "pkdir") == 0) {
    const float directionValue = readParamFloatOrDefault("pkdir", 0.0f);
    stopMotion();
    if (directionValue == 0.0f) {
      jointReferenced = true;
      const float currentPosition = jointGetPositionDeg();
      jointCtrl.reset(currentPosition);
      jointCtrl.setTarget(currentPosition);
      servoTargetZeroedDeg = currentPosition;
      wsSetState(LedState::READY);
      LOG_NFO("pkdir=0: absolute-encoder mode enabled; park is not required. Use 'save' to persist it.\r\n");
    } else if (directionValue == 1.0f || directionValue == -1.0f) {
      jointReferenced = false;
      wsSetState(LedState::BOOT);
      LOG_NFO("pkdir=%+.0f: multi-turn park mode enabled; execute park before motion. Use 'save' to persist it.\r\n",
              directionValue);
    } else {
      LOG_ERR("Invalid pkdir: use -1, 0, or +1\r\n");
    }
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

bool jointIsBusy()
{
  return motionMode == MotionMode::POSITION;
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
  const float microstepsPerSecond = MOTOR_DIRECTION_SIGN * degreesPerSecond * spd;

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
  parkPhase = ParkPhase::IDLE;
  parkCommandVelocityDegS = 0.0f;
  testEnabled = false;
  testStepEnabled = false;
  stepNum = 0;
  servoLastCmdDegS = 0.0f;
  servoHoldActive = false;

  tmc.stopInternalMotion();
  tmc.disableDriver(false); // keep configured chopper state; hardware bridge disabled

  if (motionMode != MotionMode::FAULT) {
    motionMode = MotionMode::IDLE;
  }

  // The position controller always works in zeroed joint coordinates.
  jointCtrl.reset(jointGetPositionDeg());
  wsSetState(jointReferenced ? LedState::READY : LedState::BOOT);
}

void jointControllerInit(float currentJointDeg)
{
  applyControllerParamsFromParams();

  applyJointLimitsFromParams(true);
  jointCtrl.reset(currentJointDeg);

  LOG_NFO("Joint PID + S-curve controller initialized\r\n");
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
      motionMode = MotionMode::FAULT;
      wsSetState(LedState::FAULT);
      return false;
    }

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
      motionMode = MotionMode::FAULT;
      wsSetState(LedState::FAULT);
      return false;
    }

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
    motionMode = MotionMode::FAULT;
    tmc.disableDriver(false);
    wsSetState(LedState::ENCODER_ERROR);
    return false;
  }

  if (!ensureDriverEnabled()) {
    Serial.println("[CAL] Failed to enable TMC2209 power stage");
    motionMode = MotionMode::FAULT;
    tmc.disableDriver(false);
    wsSetState(LedState::FAULT);
    return false;
  }

  delay(50);

  // Apply the same direction convention used by velocity control.
  const bool forward = (targetDegrees * MOTOR_DIRECTION_SIGN) > 0.0f;

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
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    return false;
  }

  if ((targetDegrees > 0.0f && measuredDeltaDeg < 0.0f) ||
      (targetDegrees < 0.0f && measuredDeltaDeg > 0.0f)) {
    Serial.println("[CAL] WARNING: measured direction is opposite to requested direction");
    Serial.println("[CAL] Check MOTOR_DIRECTION_SIGN / DIR wiring / mechanics. Magnitude will still be applied.");
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
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
  }

  return updated;
}

bool setZero()
{
  if (!jointReferenced) {
    Serial.println("[ERR] Zero rejected: execute park first");
    return false;
  }
  testEnabled = false;
  testStepEnabled = false;
  stepNum = 0;
  servoLastCmdDegS = 0.0f;
  servoHoldActive = false;

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

  Serial.printf("mode=%s referenced=%u park_sensor=%u tmc=%u hold=%u shold=%u enc=%.3f joint=%.3f zeroed=%.3f target=%.3f ref=%.3f refv=%.3f measv=%.3f cmd=%.3f stdeg=%.6f jrev=%.6f zoff=%.6f jmin=%.3f jmax=%.3f jtol=%.3f fault=%u\r\n",
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
                jointCtrl.getLastMeasuredVel(),
                servoLastCmdDegS,
                stepsPerDegree(),
                encoder.outputDegreesPerEncoderRevolution(),
                encoderZero,
                jmin,
                jmax,
                jtol,
                static_cast<unsigned>(jointCtrl.faultCode()));

  Serial.printf("pid kp=%.4f ki=%.4f kd=%.4f ffv=%.4f ilim=%.4f | motion vmax=%.4f amax=%.4f sct=%.4f outmax=%.4f | settle ptol=%.4f vtol=%.4f dbent=%.4f dbext=%.4f dbvel=%.4f vtau=%.4f\r\n",
                readParamFloatOrDefault("kp", SERVO_KP),
                readParamFloatOrDefault("ki", SERVO_KI),
                readParamFloatOrDefault("kd", SERVO_KD),
                readParamFloatOrDefault("ffv", SERVO_FF_VEL),
                readParamFloatOrDefault("ilim", SERVO_I_LIMIT),
                readParamFloatOrDefault("vmax", SERVO_VMAX_DEG_S),
                readParamFloatOrDefault("amax", SERVO_AMAX_DEG_S2),
                readParamFloatOrDefault("sct", SERVO_SCURVE_TIME_S),
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

static JointBus::AckCode jointBusMoveAckForRequestedTarget(float requestedDeg)
{
  float clippedDeg = requestedDeg;
  if (!jointClipTargetToLimits(requestedDeg, clippedDeg)) {
    return JointBus::AckCode::Accepted;
  }

  const float eps = 0.005f;
  if (clippedDeg < requestedDeg - eps) {
    jointBusLastLimitClipped = true;
    return JointBus::AckCode::ClippedToMax;
  }
  if (clippedDeg > requestedDeg + eps) {
    jointBusLastLimitClipped = true;
    return JointBus::AckCode::ClippedToMin;
  }

  jointBusLastLimitClipped = false;
  return JointBus::AckCode::Accepted;
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
  const JointBus::AckCode ack = jointBusMoveAckForRequestedTarget(targetDeg);

  if (!jointMoveTo(targetDeg, vmaxDegS, amaxDegS2)) {
    if (jointHasFault()) {
      return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
    }
    if (!jointReferenced) {
      return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
    }
    return JointBus::CommandResult::fail(JointBus::NackCode::RejectedByState);
  }

  return JointBus::CommandResult::ok(ack);
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
  const JointBus::AckCode ack = jointBusMoveAckForRequestedTarget(targetDeg);

  if (!jointMoveToBlended(targetDeg, vmaxDegS, amaxDegS2)) {
    if (jointHasFault()) {
      return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
    }
    if (!jointReferenced) {
      return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
    }
    return JointBus::CommandResult::fail(JointBus::NackCode::RejectedByState);
  }

  return JointBus::CommandResult::ok(ack);
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

  const JointBus::AckCode ack = jointBusMoveAckForRequestedTarget(JOINTBUS_HOME_TARGET_DEG);
  if (!jointMoveTo(JOINTBUS_HOME_TARGET_DEG,
                   JOINTBUS_HOME_VMAX_DEG_S,
                   JOINTBUS_HOME_AMAX_DEG_S2)) {
    if (jointHasFault()) {
      return JointBus::CommandResult::fail(JointBus::NackCode::FaultActive);
    }
    if (!jointReferenced) {
      return JointBus::CommandResult::fail(JointBus::NackCode::NotHomed);
    }
    return JointBus::CommandResult::fail(JointBus::NackCode::RejectedByState);
  }

  LOG_NFO("JointBus HOME accepted: target=%.3f deg vmax=%.3f deg/s amax=%.3f deg/s2\r\n",
          JOINTBUS_HOME_TARGET_DEG,
          JOINTBUS_HOME_VMAX_DEG_S,
          JOINTBUS_HOME_AMAX_DEG_S2);
  return JointBus::CommandResult::ok(ack);
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

  return JointBus::CommandResult::ok(JointBus::AckCode::Accepted);
}

static JointBus::CommandResult jointBusPark(void* context)
{
  (void)context;

  if (startPark()) {
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
  return jointStop()
      ? JointBus::CommandResult::ok(JointBus::AckCode::Accepted)
      : JointBus::CommandResult::fail(JointBus::NackCode::InternalError);
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
    // In servo-hold mode the position controller intentionally remains armed
    // after the target has been reached, so it can correct small disturbances.
    // This must not be reported as SETTLING/BUSY to the external JointBus
    // master: once isSettled() is true, the commanded move is complete.
    if (jointCtrl.isSettled()) {
      return JointBus::JointState::Holding;
    }
    return JointBus::JointState::Moving;
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
  if (servoHoldActive || driverStatus == Tmc2209Driver::Status::Enabled) {
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
  const bool positionCommandBusy = (motionMode == MotionMode::POSITION) && !jointCtrl.isSettled();
  const bool busy = positionCommandBusy ||
                    motionMode == MotionMode::PARK ||
                    motionMode == MotionMode::CALIBRATION ||
                    motionMode == MotionMode::VELOCITY_TEST ||
                    motionMode == MotionMode::STEP_TEST;

  if (busy) {
    flags |= JointBus::QSTAT_BUSY;
  }
  if (!busy && !fault) {
    flags |= JointBus::QSTAT_DONE;
  }
  if (fault) {
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

static void setupJointBusHooks()
{
  JointBus::SlaveHooks hooks;
  hooks.context = nullptr;
  hooks.move = jointBusMove;
  hooks.moveb = jointBusMoveB;
  hooks.home = jointBusHome;
  hooks.zero = jointBusZero;
  hooks.park = jointBusPark;
  hooks.stop = jointBusStop;
  hooks.reboot = jointBusReboot;
  hooks.status = jointBusStatus;
  hooks.quickStatus = jointBusQuickStatus;

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
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    return;
  }

  if (jointCtrl.fault()) {
    setMotorVelocityDegPerSecond(0.0f);
    tmc.disableDriver(false);
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    LOG_ERR("Joint controller fault code=%u\r\n", static_cast<unsigned>(jointCtrl.faultCode()));
    return;
  }

  if (jointCtrl.isSettled()) {
    setMotorVelocityDegPerSecond(0.0f);
    tmc.stopInternalMotion();
    servoLastCmdDegS = 0.0f;
    wsSetState(LedState::READY);

    if (servoHoldEnabled()) {
      if (!servoHoldActive) {
        servoHoldActive = true;
        LOG_NFO("Move complete zeroed=%.3f deg servo-hold active\r\n",
                jointGetPositionDeg());
      }
      return;
    }

    if (!motorHoldEnabled()) {
      tmc.disableDriver(false);
    }

    motionMode = MotionMode::IDLE;
    servoHoldActive = false;
    LOG_NFO("Move complete zeroed=%.3f deg motor=%s\r\n",
            jointGetPositionDeg(),
            motorHoldEnabled() ? "hold" : "disabled");
  } else {
    servoHoldActive = false;
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
  #if MAGNETIC_ENCODER_TYPE == MAGNETIC_ENCODER_AS5048A
  LOG_NFO("PlanetJoint ESP32-S3 - AS5048A PID+S-curve build\r\n");
#else
  LOG_NFO("PlanetJoint ESP32-S3 - AS5600 PID+S-curve build\r\n");
#endif
  LOG_NFO("USB CDC console ready\r\n");

  paramsInit();

  setupJointBusHooks();
  jointBus.setAddress(readJointBusAddressFromParams());
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
  LOG_NFO("MOTOR_DIRECTION_SIGN=1.0 from updated hardware configuration.\r\n");
  LOG_NFO("Runtime params: kp ki kd ffv ilim vmax amax sct outmax ptol vtol dbent dbext dbvel vtau stdeg jrev jmin jmax jtol loglvl mhold shold zoff pkdir pkvel pkenc pkpos addr.\r\n");
  LOG_NFO("trace toggles on/off; trace 0..4 selects output mode.\r\n");
  LOG_NFO("Example: set kp 1.0 / set vmax 3.0 / save\r\n");

  console.setParamSetCallback(onConsoleParamSet);
  console.begin("pj> ");
}

// ===================== LOOP =====================

void loop()
{
  const uint32_t now = millis();

  console.update();
  jointBus.update();

  if (jointBusRebootPending && static_cast<uint32_t>(now - jointBusRebootRequestedMs) >= 150U) {
    Serial.flush();
    ESP.restart();
  }

  wsLedsUpdate();
  servoUpdate();

  if (traceEnabled && now - lastEncoderPrintMs >= ENCODER_PRINT_PERIOD_MS) {
    lastEncoderPrintMs = now;

    const float jointZeroedDeg = jointGetPositionDeg();
    const float targetDeg = jointCtrl.target();
    const float refDeg = jointCtrl.refPos();
    const float errDeg = targetDeg - jointZeroedDeg;
    const float measVel = jointCtrl.getLastMeasuredVel();
    const float refVel = jointCtrl.refVel();
    const uint8_t settled = jointCtrl.isSettled() ? 1 : 0;
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
