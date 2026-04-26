#include <Arduino.h>
#include <SPI.h>

#include "led_status.h"
#include "as5048a.h"

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
static constexpr int PIN_TMC_TX = 7;
static constexpr int PIN_TMC_RX = 8;
static constexpr int PIN_STEP   = 4;
static constexpr int PIN_DIR    = 5;
static constexpr int PIN_EN     = 6;

// ===================== OGGETTI =====================

SPIClass EncoderSPI(HSPI);

HardwareSerial SerialFuture(0);
HardwareSerial SerialTMC(2);

AS5048A encoder(EncoderSPI, PIN_AS5048_CS);

// ===================== TIMING =====================

static constexpr uint32_t ENCODER_READ_PERIOD_MS = 20;
static constexpr uint32_t ENCODER_PRINT_PERIOD_MS = 100;

static uint32_t lastEncoderReadMs = 0;
static uint32_t lastEncoderPrintMs = 0;

static uint16_t encoderRaw = 0;
static float encoderDeg = 0.0f;
static bool encoderOk = false;

// ===================== SETUP =====================

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

void setup() {
  Serial.begin(USB_BAUD);
  delay(500);

  Serial.println();
  Serial.println("[BOOT] PlanetJoint ESP32-S3");
  Serial.println("[BOOT] USB CDC console ready");

  wsLedsInit();
  wsSetState(LedState::BOOT);

  SerialFuture.begin(UART0_BAUD);
  Serial.println("[OK] UART0 initialized for future use");

  SerialTMC.begin(TMC_BAUD, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
  Serial.println("[OK] UART2 initialized for future TMC2209 use");

  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_EN, HIGH);

  Serial.println("[OK] Motor GPIO initialized in safe state");

  encoderInit();

  if (encoderFirstReadTest()) {
    wsSetState(LedState::READY);
  } else {
    wsSetState(LedState::ENCODER_ERROR);
  }

  Serial.println("[BOOT] Init complete");
}

// ===================== LOOP =====================

void loop() {
  const uint32_t now = millis();

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
  if (now - lastEncoderPrintMs >= ENCODER_PRINT_PERIOD_MS) {
    lastEncoderPrintMs = now;

    Serial.print("@raw:");
    Serial.print(encoderRaw);

    Serial.print(",rdeg:");
    Serial.print(encoderDeg, 3);

    Serial.print(",cdeg:");
    Serial.print(encoder.computeContinuousAngleDeg(encoderRaw), 3);

    Serial.print(",ok:");
    Serial.print(encoderOk ? 1 : 0);

    Serial.print(",p:");
    Serial.print(encoder.lastParityOk() ? 1 : 0);

    Serial.print(",err:");
    Serial.println(encoder.lastErrorFlag() ? 1 : 0);
  }
}