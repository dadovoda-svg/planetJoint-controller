#pragma once

#include <Arduino.h>

enum class LedState : uint8_t {
  BOOT,
  READY,
  ENCODER_OK,
  ENCODER_ERROR,
  FAULT
};

void wsLedsInit();
void wsSetState(LedState state);
void wsLedsUpdate();