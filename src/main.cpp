#include <Arduino.h>
#include <SPI.h>

#include "led_status.h"
#include "as5048a.h"
#include "params.h"
#include "SerialConsole.h"
#include "Tmc2209Driver.h"  

// ===================== BAUDRATE =====================

static constexpr uint32_t USB_BAUD    = 115200;
static constexpr uint32_t UART0_BAUD  = 115200;
static constexpr uint32_t TMC_BAUD    = 115200;

// ===================== PIN MAP =====================
// AS5048A - HSPI
static constexpr int PIN_AS5048_SCK  = 12;  // SCK = CLK   - green
static constexpr int PIN_AS5048_MISO = 13;  // MISO = DOUT - purple 
static constexpr int PIN_AS5048_MOSI = 11;  // MOSI = DIN  - blue
static constexpr int PIN_AS5048_CS   = 10;  // CS = CS     - white

// TMC2209 - per ora solo pin assegnati, driver non gestito
static constexpr int PIN_TMC_TX   = 7;  // ---\/\/\--- 1k blu
static constexpr int PIN_TMC_RX   = 8;  // blu
static constexpr int PIN_TMC_STEP = 4;  // verde
static constexpr int PIN_TMC_DIR  = 5;  // giallo
static constexpr int PIN_TMC_ENN  = 6;  // arancione

Tmc2209Driver::Pins tmcPins {
    .step   = PIN_TMC_STEP,
    .dir    = PIN_TMC_DIR,
    .enn    = PIN_TMC_ENN,
    .uartRx = PIN_TMC_RX,
    .uartTx = PIN_TMC_TX
};

Tmc2209Driver::Config tmcConfig {
    .uartAddress = 0,
    .baud = 115200,
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


// ===================== OGGETTI =====================

SPIClass EncoderSPI(HSPI);

HardwareSerial SerialFuture(0);
HardwareSerial SerialTMC(2);

AS5048A encoder(EncoderSPI, PIN_AS5048_CS);
Tmc2209Driver tmc(SerialTMC, tmcPins, tmcConfig);

PersistentParams params;
SerialConsole console(Serial, params);

// ===================== TIMING =====================

static constexpr uint32_t ENCODER_READ_PERIOD_MS = 20;
static constexpr uint32_t ENCODER_PRINT_PERIOD_MS = 250;
static constexpr uint32_t TEST_PERIOD_MS = 1;

static uint32_t lastTestMs = 0;
static uint32_t lastEncoderPrintMs = 0;
static uint32_t lastEncoderReadMs = 0;

static uint16_t encoderRaw = 0;
static float encoderDeg = 0.0f;
static bool encoderOk = false;
static bool traceEnabled = false;
static bool testEnabled = false;
static bool testForward = false;
static int32_t stepNum = 3200 ; // 16 microstep * 200 steps/rev * 1 rev/s * 60 s/min = 192000 microsteps/min = 3200 microsteps/s -> 5000 microsteps = 1.56s di test

// ===================== SETUP =====================
void paramsInit() {

  if (!params.begin()) {
    Serial.println("[ERR] Failed to initialize parameters storage");
    return;
  }

  params.initKey("kp", 1.0f);
  params.initKey("ki", 0.0f);
  params.initKey("kd", 0.0f);
  params.initKey("ustep", 16.0f);
  params.initKey("irun", 10.0f);
  params.initKey("ihold", 4.0f);

  //merge con i parametri di default, senza sovrascrivere quelli già presenti
  params.load();

  //elenco dei parametri caricati
  Serial.println("[PARAMS] Loaded parameters:");
  for (uint8_t i = 0; i < params.count(); i++) {
    const char* key = nullptr;
    float value = 0.0f;

    if (params.getByIndex(i, key, value)) {
        Serial.printf("%s = %.5f\n", key, value);
    }
  }
}

void encoderInit() {
  EncoderSPI.begin(
    PIN_AS5048_SCK,
    PIN_AS5048_MISO,
    PIN_AS5048_MOSI,
    PIN_AS5048_CS
  );

  encoder.begin();

  Serial.println("[OK] HSPI initialized for AS5048A");
}

bool encoderFirstReadTest() {
  encoderOk = encoder.readRaw(encoderRaw);
  encoderDeg = encoder.lastDegrees();

  if (encoderOk) {
    Serial.print("[OK] AS5048A first read raw=");
    Serial.print(encoderRaw);
    Serial.print(" angle=");
    Serial.print(encoderDeg, 3);
    Serial.println(" deg");
    return true;
  }

  Serial.println("[ERR] AS5048A first read failed");
  return false;
}

void applyAllParams() {
    float irun = 0.0f;
    float ihold = 0.0f;
    float ustep = 0.0f;

    if (params.get("irun", irun) && params.get("ihold", ihold)) {
        tmc.setCurrentScale(static_cast<uint8_t>(irun), static_cast<uint8_t>(ihold), tmcConfig.iholdDelay);
        Serial.printf("[OK] Updated TMC2209 current scale: IRUN=%d, IHOLD=%d\n", static_cast<int>(irun), static_cast<int>(ihold));
    } else {
        Serial.println("[ERR] Failed to get IRUN/IHOLD from parameters");
    }

    if (params.get("ustep", ustep)) {
        tmc.setMicrostepResolution(static_cast<uint16_t>(ustep));
        Serial.printf("[OK] Updated TMC2209 microstep resolution: USTEP=%d\n", static_cast<int>(ustep));
    } else {
        Serial.println("[ERR] Failed to get USTEP from parameters");
    }
}

bool tmcInit() {
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

    Serial.println("Driver configured, but power stage is still disabled");

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

void setup() {
  wsLedsInit();
  wsSetState(LedState::BOOT);  
  wsLedsUpdate();
  
  Serial.begin(USB_BAUD);
  delay(500);

  paramsInit();

  Serial.println();
  Serial.println("[BOOT] PlanetJoint ESP32-S3");
  Serial.println("[BOOT] USB CDC console ready");
  
  SerialFuture.begin(UART0_BAUD);
  Serial.println("[OK] UART0 initialized for future use");

  if (!tmcInit()) {
    Serial.println("[ERR] Failed to initialize TMC2209 driver");
    wsSetState(LedState::FAULT);
  } else {
    Serial.println("[OK] TMC2209 driver initialized");
  }

  encoderInit();

  if (encoderFirstReadTest()) {
    wsSetState(LedState::READY);
  } else {
    wsSetState(LedState::ENCODER_ERROR);
  }

  Serial.println("[BOOT] Init complete");

  console.begin("pj> ");
}

bool toggleTrace() {
  traceEnabled = !traceEnabled;
  return traceEnabled;
}

bool toggleTest(float microstepsPerSecond = 100.0f) {

  testEnabled = !testEnabled;

  if ( testEnabled ) {
    Serial.println("Test motore ENABLED");
    tmc.armPowerStage();
    tmc.enableDriver();
    tmc.runVelocityMicrostepsPerSecond(microstepsPerSecond );
    // testForward  = !testForward;
    // stepNum = 64000;
  } else {
    Serial.println("Test motore DISABLED");
    tmc.stopInternalMotion();
    tmc.disableDriver();
  } 
  return testEnabled;
}

// ===================== LOOP =====================

void loop() {
  const uint32_t now = millis();

  console.update();
  wsLedsUpdate();

  // Lettura encoder periodica
  if (now - lastEncoderReadMs >= ENCODER_READ_PERIOD_MS) {
    lastEncoderReadMs = now;

    float angle = 0.0f;
    encoderOk = encoder.readDegrees(angle);

    if (encoderOk) {
      encoderRaw = encoder.lastRaw();
      encoderDeg = angle;
      wsSetState(LedState::ENCODER_OK);
    } else {
      wsSetState(LedState::ENCODER_ERROR);
    }
  }

  // Stampa diagnostica rallentata
  if (traceEnabled && now - lastEncoderPrintMs >= ENCODER_PRINT_PERIOD_MS) {
    lastEncoderPrintMs = now;

    // Serial.print("@raw:");
    // Serial.print(encoderRaw);

    Serial.print("@rdeg:");
    Serial.print(encoderDeg, 3);

    Serial.print(",cdeg:");
    Serial.println(encoder.computeContinuousAngleDeg(encoderRaw), 3);

    // Serial.print(",ok:");
    // Serial.print(encoderOk ? 1 : 0);

    // Serial.print(",p:");
    // Serial.print(encoder.lastParityOk() ? 1 : 0);

    // Serial.print(",err:");
    // Serial.println(encoder.lastErrorFlag() ? 1 : 0);
  }

  // //test motore per step/dir
  // if (testEnabled && now - lastTestMs >= TEST_PERIOD_MS) {
  //   lastTestMs = now;
  //   if (stepNum > 0) {
  //     tmc.step(testForward, 200); // step in avanti con high time di 1000 microsecondi
  //     stepNum--;
  //   } 
  //   else toggleTest(0); // disabilita test al termine dei step programmati
  // }
}