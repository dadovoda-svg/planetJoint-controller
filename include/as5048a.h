#pragma once

#include <Arduino.h>
#include <SPI.h>

class AS5048A {
public:
  explicit AS5048A(SPIClass& spi, int csPin);

  void begin();

  bool readRaw(uint16_t& rawAngle);
  bool readDegrees(float& angleDeg);

  uint16_t lastRaw() const;
  float lastDegrees() const;
  bool lastReadOk() const;
  bool lastParityOk() const;
  bool lastErrorFlag() const;

private:
  SPIClass& _spi;
  int _csPin;

  uint16_t _lastRaw = 0;
  bool _lastReadOk = false;
  bool _lastParityOk = false;
  bool _lastErrorFlag = false;

  uint16_t transfer16(uint16_t value);
  uint16_t buildCommand(uint16_t commandWithoutParity) const;
  bool checkEvenParity(uint16_t value) const;
  bool calcEvenParity(uint16_t value) const;
};