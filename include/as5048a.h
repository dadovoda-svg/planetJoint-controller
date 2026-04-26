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
  float computeContinuousAngleDeg(uint16_t rawAngle);
  bool readContinuousDegrees(float& angleDeg);

private:
  SPIClass& _spi;
  int _csPin;

  static constexpr int32_t COUNTS_PER_REV = 16384;
  static constexpr int32_t HALF_REV = COUNTS_PER_REV / 2;

  uint16_t _lastRaw = 0;
  bool _lastReadOk = false;
  bool _lastParityOk = false;
  bool _lastErrorFlag = false;

  uint16_t transfer16(uint16_t value);
  uint16_t buildCommand(uint16_t commandWithoutParity) const;
  bool checkEvenParity(uint16_t value) const;
  bool calcEvenParity(uint16_t value) const;

  bool _continuousInitialized = false;
  int64_t _turnCount = 0;
  uint16_t _prevRawAngle = 0;
  int64_t _continuousCounts = 0;
};