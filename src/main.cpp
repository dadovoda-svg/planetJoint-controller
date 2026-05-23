#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "led_status.h"
#include "as5048a.h"
#include "params.h"
#include "SerialConsole.h"
#include "Tmc2209Driver.h"
#include "SCurvePosVelController.h"

// ===================== BAUDRATE =====================

static constexpr uint32_t USB_BAUD    = 115200;
static constexpr uint32_t UART0_BAUD  = 115200;
static constexpr uint32_t TMC_BAUD    = 230400;

// ===================== PIN MAP =====================

static constexpr int PIN_AS5048_SCK  = 12;  // SCK = CLK
static constexpr int PIN_AS5048_MISO = 13;  // MISO = DOUT
static constexpr int PIN_AS5048_MOSI = 11;  // MOSI = DIN
static constexpr int PIN_AS5048_CS   = 10;  // CS

static constexpr int PIN_TMC_TX   = 7;  // ESP32 TX -> PDN_UART through 1k
static constexpr int PIN_TMC_RX   = 8;  // ESP32 RX -> PDN_UART
static constexpr int PIN_TMC_STEP = 4;
static constexpr int PIN_TMC_DIR  = 5;
static constexpr int PIN_TMC_ENN  = 6;

// ===================== MECHANICAL SCALE =====================

// Real hardware note:
// one full 360-degree encoder revolution corresponds to 15.6 degrees of real joint motion.
static constexpr float JOINT_DEGREES_PER_ENCODER_REV = 15.6f;

// Direction correction between positive controller output and real positive joint motion.
// If the first low-speed test moves away from the target, change this to -1.0f.
static constexpr float MOTOR_DIRECTION_SIGN = -1.0f;

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

// Disabled for the first build. Enable later when mechanical range is known.
// static constexpr float SERVO_MIN_DEG = -170.0f;
// static constexpr float SERVO_MAX_DEG = +170.0f;

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

SPIClass EncoderSPI(HSPI);

HardwareSerial SerialFuture(0);
HardwareSerial SerialTMC(2);

AS5048A encoder(EncoderSPI, PIN_AS5048_CS);
Tmc2209Driver tmc(SerialTMC, tmcPins, tmcConfig);
SCurvePosVelController jointCtrl;

PersistentParams params;
SerialConsole console(Serial, params);

// ===================== STATE =====================

static constexpr uint32_t ENCODER_PRINT_PERIOD_MS = 250;
static constexpr uint32_t TEST_PERIOD_MS = 1;

static uint32_t lastServoUs = 0;
static uint32_t lastTestMs = 0;
static uint32_t lastEncoderPrintMs = 0;

static uint16_t encoderRaw = 0;
static float encoderDeg = 0.0f;          // encoder angle modulo 360
static float jointDeg = 0.0f;            // real unrolled joint angle
static float encoderZero = 0.0f;         // real unrolled joint zero
static bool encoderOk = false;
static bool traceEnabled = false;

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

static float servoTargetZeroedDeg = 0.0f;
static float servoLastCmdDegS = 0.0f;
static bool tmcReady = false;

enum class MotionMode : uint8_t {
  IDLE,
  VELOCITY_TEST,
  STEP_TEST,
  POSITION,
  CALIBRATION,
  FAULT
};

static MotionMode motionMode = MotionMode::IDLE;

struct JointMoveCommand {
  // All fields are expressed in real joint degrees, relative to the logical zero.
  float targetDeg = 0.0f;
  float vmaxDegS = 0.0f;
  float amaxDegS2 = 0.0f;
  float sCurveTimeS = 0.0f;   // <= 0 means keep current/default parameter
  float outMaxDegS = 0.0f;    // <= 0 means auto: 1.25 * vmax
};

static constexpr float PLANNER_HARD_VMAX_LIMIT_DEG_S = 12.0f;
static constexpr float PLANNER_HARD_AMAX_LIMIT_DEG_S2 = 100.0f;

// ===================== PARAMS =====================

static bool readParamU8(const char* key, uint8_t& out)
{
  float value = 0.0f;

  if (!params.get(key, value)) {
    Serial.print("[ERR] Parameter not found: ");
    Serial.println(key);
    return false;
  }

  if (value < 0.0f || value > 255.0f) {
    Serial.printf("[ERR] Parameter out of uint8 range: %s=%.6f\r\n", key, value);
    return false;
  }

  out = static_cast<uint8_t>(value);
  return true;
}

static bool readParamU16(const char* key, uint16_t& out)
{
  float value = 0.0f;

  if (!params.get(key, value)) {
    Serial.print("[ERR] Parameter not found: ");
    Serial.println(key);
    return false;
  }

  if (value < 0.0f || value > 65535.0f) {
    Serial.printf("[ERR] Parameter out of uint16 range: %s=%.6f\r\n", key, value);
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

static float readParamMinOrDefault(const char* key, float fallback, float minValue)
{
  const float value = readParamFloatOrDefault(key, fallback);

  if (!isfinite(value) || value < minValue) {
    Serial.printf("[WARN] Invalid parameter %s=%.6f, using default %.6f\r\n", key, value, fallback);
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

void paramsInit()
{
  if (!params.begin()) {
    Serial.println("[ERR] Failed to initialize parameters storage");
    return;
  }

  // Motor / driver parameters
  params.initKey("ustep", 16.0f);
  params.initKey("irun", 10.0f);
  params.initKey("ihold", 4.0f);
  params.initKey("stdeg", 100.0f); // motor microsteps per real joint degree

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

  params.load();

  Serial.println("[PARAMS] Loaded parameters:");
  for (uint8_t i = 0; i < params.count(); i++) {
    const char* key = nullptr;
    float value = 0.0f;

    if (params.getByIndex(i, key, value)) {
      Serial.printf("%s = %.6f\r\n", key, value);
    }
  }
}

static void applyCurrentScaleFromParams()
{
  uint8_t irun = 0;
  uint8_t ihold = 0;

  if (!readParamU8("irun", irun) || !readParamU8("ihold", ihold)) {
    Serial.println("[ERR] Current scale update skipped");
    return;
  }

  tmc.setCurrentScale(irun, ihold, tmcConfig.iholdDelay);
  Serial.printf("[OK] Updated TMC2209 current scale: IRUN=%u, IHOLD=%u\r\n", irun, ihold);
}

static void applyMicrostepResolutionFromParams()
{
  uint16_t ustep = 0;

  if (!readParamU16("ustep", ustep)) {
    Serial.println("[ERR] Microstep resolution update skipped");
    return;
  }

  if (!tmc.setMicrostepResolution(ustep)) {
    Serial.printf("[ERR] Invalid TMC2209 microstep resolution: %u\r\n", ustep);
    return;
  }

  Serial.printf("[OK] Updated TMC2209 microstep resolution: USTEP=%u\r\n", ustep);
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

  Serial.printf("[OK] Servo motion params: vmax=%.4f amax=%.4f sct=%.4f outmax=%.4f\r\n",
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

  Serial.printf("[OK] PID params: kp=%.4f ki=%.4f kd=%.4f ffv=%.4f ilim=%.4f\r\n",
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

  Serial.printf("[OK] Servo settling params: ptol=%.4f vtol=%.4f dbent=%.4f dbext=%.4f dbvel=%.4f vtau=%.4f\r\n",
                ptol, vtol, dbent, dbext, dbvel, vtau);
}

static void applyControllerParamsFromParams()
{
  applyControllerMotionParamsFromParams();
  applyControllerGainsFromParams();
  applyControllerSettlingParamsFromParams();
}

void applyAllParams()
{
  applyCurrentScaleFromParams();
  applyMicrostepResolutionFromParams();
  applyControllerParamsFromParams();
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

  if (strcmp(key, "stdeg") == 0) {
    Serial.printf("[OK] steps/degree now %.6f microsteps/deg\r\n", stepsPerDegree());
    return;
  }

  Serial.print("[PARAMS] No runtime action defined for key: ");
  Serial.println(key);
}

// ===================== ENCODER / TMC INIT =====================

void encoderInit()
{
  EncoderSPI.begin(PIN_AS5048_SCK, PIN_AS5048_MISO, PIN_AS5048_MOSI, PIN_AS5048_CS);
  encoder.begin();
  encoder.setOutputDegreesPerEncoderRevolution(JOINT_DEGREES_PER_ENCODER_REV);

  Serial.println("[OK] HSPI initialized for AS5048A");
  Serial.printf("[OK] Encoder scale: 360.000 encoder deg = %.6f joint deg\r\n",
                encoder.outputDegreesPerEncoderRevolution());
}

bool encoderFirstReadTest()
{
  encoderOk = encoder.readContinuousDegrees(jointDeg);
  encoderRaw = encoder.lastRaw();
  encoderDeg = encoder.lastDegrees();

  if (encoderOk) {
    Serial.print("[OK] AS5048A first read raw=");
    Serial.print(encoderRaw);
    Serial.print(" enc_angle=");
    Serial.print(encoderDeg, 3);
    Serial.print(" deg joint_unrolled=");
    Serial.print(jointDeg, 3);
    Serial.println(" deg");
    return true;
  }

  Serial.println("[ERR] AS5048A first read failed");
  return false;
}

bool tmcInit()
{
  Serial.println("TMC2209 minimal init test");

  Serial.println("Starting communication-only mode...");
  if (!tmc.beginCommunicationOnly()) {
    Serial.println("ERROR: beginCommunicationOnly() failed");
    return false;
  }

  Serial.println("Communication-only mode started");

  Serial.println("Probing TMC2209...");
  if (!tmc.probe()) {
    Serial.println("ERROR: TMC2209 probe failed");
    return false;
  }

  Serial.println("TMC2209 detected");

  uint32_t ioin = 0;
  if (tmc.readIoin(ioin)) {
    Serial.print("IOIN = 0x");
    Serial.println(ioin, HEX);
  } else {
    Serial.println("ERROR: unable to read IOIN");
  }

  uint32_t gstat = 0;
  if (tmc.readGstat(gstat)) {
    Serial.print("GSTAT = 0x");
    Serial.println(gstat, HEX);
  } else {
    Serial.println("ERROR: unable to read GSTAT");
  }

  Serial.println("Configuring driver registers...");
  if (!tmc.configure()) {
    Serial.println("ERROR: configure() failed");
    return false;
  }

  Serial.println("Driver configured, power stage still disabled");

  uint32_t ifcnt = 0;
  if (tmc.readIfcnt(ifcnt)) {
    Serial.print("IFCNT = ");
    Serial.println(ifcnt);
  } else {
    Serial.println("ERROR: unable to read IFCNT");
  }

  Serial.println("Safe init completed");
  Serial.println("ENN is still HIGH: motor bridge disabled");
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
    Serial.println("[ERR] TMC not ready");
    return false;
  }

  const float spd = stepsPerDegree();
  const float microstepsPerSecond = MOTOR_DIRECTION_SIGN * degreesPerSecond * spd;

  if (!tmc.runVelocityMicrostepsPerSecond(microstepsPerSecond)) {
    Serial.println("[ERR] TMC velocity command failed");
    return false;
  }

  servoLastCmdDegS = degreesPerSecond;
  return true;
}

static bool ensureDriverEnabled()
{
  if (!tmcReady) {
    Serial.println("[ERR] TMC not ready");
    return false;
  }

  if (!tmc.armPowerStage()) {
    Serial.println("[ERR] Unable to arm TMC power stage");
    return false;
  }

  if (!tmc.enableDriver()) {
    Serial.println("[ERR] Unable to enable TMC driver");
    return false;
  }

  return true;
}

void stopMotion()
{
  testEnabled = false;
  testStepEnabled = false;
  stepNum = 0;
  servoLastCmdDegS = 0.0f;

  tmc.stopInternalMotion();
  tmc.disableDriver(false); // keep configured chopper state; hardware bridge disabled

  if (motionMode != MotionMode::FAULT) {
    motionMode = MotionMode::IDLE;
  }

  // The position controller always works in zeroed joint coordinates.
  jointCtrl.reset(jointGetPositionDeg());
  wsSetState(LedState::READY);
}

bool jointStop()
{
  stopMotion();
  return true;
}

void jointControllerInit(float currentJointDeg)
{
  applyControllerParamsFromParams();

  // First closed-loop firmware: no hard position limits until the mechanical range is confirmed.
  jointCtrl.disablePositionLimits();
  jointCtrl.reset(currentJointDeg);

  Serial.println("[OK] Joint PID + S-curve controller initialized");
  Serial.println("[SAFE] Servo parameters are runtime-tunable with 'set <key> <value>'. Keep early tests slow.");
}

bool jointMoveTo(const JointMoveCommand& cmd)
{
  if (motionMode == MotionMode::CALIBRATION) {
    Serial.println("[ERR] Calibration in progress");
    return false;
  }

  if (motionMode == MotionMode::FAULT) {
    Serial.println("[ERR] Motion fault latched; use stop/reboot after checking hardware");
    return false;
  }

  if (!encoderOk) {
    Serial.println("[ERR] Encoder not OK");
    return false;
  }

  if (!isfinite(cmd.targetDeg) || !isfinite(cmd.vmaxDegS) || !isfinite(cmd.amaxDegS2)) {
    Serial.println("[ERR] Invalid move command: non-finite value");
    return false;
  }

  if (cmd.vmaxDegS <= 0.0f || cmd.amaxDegS2 <= 0.0f) {
    Serial.println("[ERR] Invalid move command: vmax and amax must be > 0");
    return false;
  }

  const float safeVmax = constrain(fabsf(cmd.vmaxDegS), 0.01f, PLANNER_HARD_VMAX_LIMIT_DEG_S);
  const float safeAmax = constrain(fabsf(cmd.amaxDegS2), 0.01f, PLANNER_HARD_AMAX_LIMIT_DEG_S2);

  stopMotion();

  if (!ensureDriverEnabled()) {
    motionMode = MotionMode::FAULT;
    wsSetState(LedState::FAULT);
    return false;
  }

  jointCtrl.clearFault();
  jointCtrl.setLimits(safeVmax, safeAmax);

  if (isfinite(cmd.sCurveTimeS) && cmd.sCurveTimeS > 0.0f) {
    jointCtrl.setSCurveTime(cmd.sCurveTimeS);
  }

  if (isfinite(cmd.outMaxDegS) && cmd.outMaxDegS > 0.0f) {
    jointCtrl.setOutputMax(cmd.outMaxDegS);
  } else {
    jointCtrl.setOutputMax(safeVmax * 1.25f);
  }

  // IMPORTANT: controller coordinates are zeroed joint degrees.
  const float currentZeroedDeg = jointGetPositionDeg();
  servoTargetZeroedDeg = cmd.targetDeg;
  jointCtrl.reset(currentZeroedDeg);
  jointCtrl.setTarget(cmd.targetDeg);

  lastServoUs = micros();
  motionMode = MotionMode::POSITION;

  Serial.printf("[MOVE] target=%.3f deg current=%.3f deg vmax=%.3f deg/s amax=%.3f deg/s2\r\n",
                cmd.targetDeg, currentZeroedDeg, safeVmax, safeAmax);
  return true;
}

bool jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetDeg;
  cmd.vmaxDegS = vmaxDegS;
  cmd.amaxDegS2 = amaxDegS2;
  return jointMoveTo(cmd);
}

bool jointMoveTo(float targetDeg, float vmaxDegS, float amaxDegS2, float sCurveTimeS)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetDeg;
  cmd.vmaxDegS = vmaxDegS;
  cmd.amaxDegS2 = amaxDegS2;
  cmd.sCurveTimeS = sCurveTimeS;
  return jointMoveTo(cmd);
}

bool moveJointToDeg(float targetZeroedDeg)
{
  JointMoveCommand cmd;
  cmd.targetDeg = targetZeroedDeg;
  cmd.vmaxDegS = readParamMinOrDefault("vmax", SERVO_VMAX_DEG_S, 0.001f);
  cmd.amaxDegS2 = readParamMinOrDefault("amax", SERVO_AMAX_DEG_S2, 0.001f);
  cmd.sCurveTimeS = readParamMinOrDefault("sct", SERVO_SCURVE_TIME_S, 0.001f);
  cmd.outMaxDegS = readParamMinOrDefault("outmax", SERVO_OUTPUT_MAX_DEG_S, 0.0f);
  return jointMoveTo(cmd);
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

void setZero()
{
  testEnabled = false;
  testStepEnabled = false;
  stepNum = 0;
  servoLastCmdDegS = 0.0f;

  tmc.stopInternalMotion();
  tmc.disableDriver(false);

  // Reset continuous tracking so the current physical position becomes a clean software origin.
  encoder.resetContinuousTracking();

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
    return;
  }

  encoderZero = jointDeg;
  servoTargetZeroedDeg = 0.0f;

  // After zeroing, the controller coordinate system is zeroed too.
  jointCtrl.reset(0.0f);
  jointCtrl.setTarget(0.0f);

  motionMode = MotionMode::IDLE;
  wsSetState(LedState::READY);
  Serial.printf("Zero set at joint_abs=%.3f deg; zeroed position is now 0.000 deg\r\n", encoderZero);
}

void printServoStatus()
{
  const float zeroed = jointGetPositionDeg();
  Serial.printf("mode=%s enc=%.3f joint=%.3f zeroed=%.3f target=%.3f ref=%.3f refv=%.3f measv=%.3f cmd=%.3f stdeg=%.6f fault=%u\r\n",
                motionModeName(motionMode),
                encoderDeg,
                jointDeg,
                zeroed,
                servoTargetZeroedDeg,
                jointCtrl.refPos(),
                jointCtrl.refVel(),
                jointCtrl.getLastMeasuredVel(),
                servoLastCmdDegS,
                stepsPerDegree(),
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
    Serial.println("[FAULT] Encoder read failed during control");
    return;
  }

  jointDeg = measured;

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
    Serial.printf("[FAULT] Joint controller fault code=%u\r\n", static_cast<unsigned>(jointCtrl.faultCode()));
    return;
  }

  if (jointCtrl.isSettled()) {
    setMotorVelocityDegPerSecond(0.0f);
    tmc.disableDriver(false);
    motionMode = MotionMode::IDLE;
    servoLastCmdDegS = 0.0f;
    wsSetState(LedState::READY);
    Serial.printf("[OK] Move complete zeroed=%.3f deg\r\n", jointGetPositionDeg());
  }
}

// ===================== SETUP =====================

void setup()
{
  wsLedsInit();
  wsSetState(LedState::BOOT);
  wsLedsUpdate();

  Serial.begin(USB_BAUD);
  delay(500);

  Serial.println();
  Serial.println("[BOOT] PlanetJoint ESP32-S3 - AS5048A PID+S-curve build");
  Serial.println("[BOOT] USB CDC console ready");

  paramsInit();

  SerialFuture.begin(UART0_BAUD);
  Serial.println("[OK] UART0 initialized for future use");

  if (!tmcInit()) {
    Serial.println("[ERR] Failed to initialize TMC2209 driver");
    wsSetState(LedState::FAULT);
    motionMode = MotionMode::FAULT;
  } else {
    tmcReady = true;
    Serial.println("[OK] TMC2209 driver initialized");
    applyAllParams();
  }

  encoderInit();

  if (encoderFirstReadTest()) {
    encoderZero = jointDeg;
    jointControllerInit(jointGetPositionDeg());
    wsSetState(LedState::READY);
  } else {
    wsSetState(LedState::ENCODER_ERROR);
    motionMode = MotionMode::FAULT;
  }

  Serial.println("[BOOT] Init complete");
  Serial.println("[SAFE] First tests: use 'zero', then 'pos 1', 'pos 0', 'pos -1'.");
  Serial.println("[SAFE] MOTOR_DIRECTION_SIGN=-1.0 from real hardware tests.");
  Serial.println("[TUNE] Runtime params: kp ki kd ffv ilim vmax amax sct outmax ptol vtol dbent dbext dbvel vtau.");
  Serial.println("[TRACE] trace toggles on/off; trace 0..4 selects output mode.");
  Serial.println("[TUNE] Example: set kp 1.0 / set vmax 3.0 / save");

  console.setParamSetCallback(onConsoleParamSet);
  console.begin("pj> ");
}

// ===================== LOOP =====================

void loop()
{
  const uint32_t now = millis();

  console.update();
  wsLedsUpdate();
  servoUpdate();

  if (traceEnabled && now - lastEncoderPrintMs >= ENCODER_PRINT_PERIOD_MS) {
    lastEncoderPrintMs = now;

    const float jointZeroedDeg = jointGetPositionDeg();
    const float targetDeg = servoTargetZeroedDeg;
    const float refDeg = jointCtrl.refPos();
    const float errDeg = targetDeg - jointZeroedDeg;
    const float measVel = jointCtrl.getLastMeasuredVel();
    const float refVel = jointCtrl.refVel();
    const uint8_t settled = jointCtrl.isSettled() ? 1 : 0;

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
