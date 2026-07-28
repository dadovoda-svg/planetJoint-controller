#pragma once

#include <stdint.h>

#define MSBFIRST 1
#define SPI_MODE1 1

class SPISettings {
public:
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};

class SPIClass {
public:
  void beginTransaction(const SPISettings&) {}
  uint16_t transfer16(uint16_t value) { return value; }
  void endTransaction() {}
};
