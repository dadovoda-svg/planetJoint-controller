#include "led_status.h"
#include <Adafruit_NeoPixel.h>

static constexpr int PIN_WS2812 = 21;
static constexpr int NUM_LEDS   = 1;

static Adafruit_NeoPixel wsLed(NUM_LEDS, PIN_WS2812, NEO_GRB + NEO_KHZ800);

static LedState currentState = LedState::BOOT;
static uint32_t lastUpdateMs = 0;
static bool blinkPhase = false;

static void setColor(uint8_t r, uint8_t g, uint8_t b) {
  wsLed.setPixelColor(0, wsLed.Color(r, g, b));
  wsLed.show();
}

void wsLedsInit() {
  wsLed.begin();
  wsLed.clear();
  wsLed.setBrightness(40);
  wsLed.show();

  currentState = LedState::BOOT;
  lastUpdateMs = 0;
  blinkPhase = false;
}

void wsSetState(LedState state) {
  if (state == currentState) {
    return;
  }

  currentState = state;
  lastUpdateMs = 0;
  blinkPhase = false;
}

void wsLedsUpdate() {
  const uint32_t now = millis();

  switch (currentState) {
    case LedState::BOOT:
      // Blu fisso
      setColor(0, 0, 40);
      break;

    case LedState::READY:
      // Verde fisso
      setColor(0, 40, 0);
      break;

    case LedState::ENCODER_OK:
      // Verde tenue
      setColor(0, 20, 0);
      break;

    case LedState::ENCODER_ERROR:
      // Giallo lampeggiante lento
      if (now - lastUpdateMs >= 500) {
        lastUpdateMs = now;
        blinkPhase = !blinkPhase;
        if (blinkPhase) {
          setColor(40, 25, 0);
        } else {
          setColor(0, 0, 0);
        }
      }
      break;

    case LedState::FAULT:
      // Rosso lampeggiante veloce
      if (now - lastUpdateMs >= 150) {
        lastUpdateMs = now;
        blinkPhase = !blinkPhase;
        if (blinkPhase) {
          setColor(50, 0, 0);
        } else {
          setColor(0, 0, 0);
        }
      }
      break;
  }
}