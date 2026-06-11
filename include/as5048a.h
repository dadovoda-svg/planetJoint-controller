#pragma once

#include <Arduino.h>
#include <SPI.h>

class AS5048A {
public:
  explicit AS5048A(SPIClass& spi, int csPin);

  void begin();

  bool readRaw(uint16_t& rawAngle);

  // Raw encoder angle, modulo one encoder revolution, in encoder degrees.
  bool readDegrees(float& angleDeg);

  // Continuous output/joint angle, in real mechanical output degrees.
  bool readContinuousDegrees(float& angleDeg);

  // Updates the internal unwrap state and returns real output/joint degrees.
  float computeContinuousAngleDeg(uint16_t rawAngle);

  void resetContinuousTracking();

  // Rebase the continuous output position at the current raw encoder angle.
  void setContinuousOutputDegrees(float outputDegrees);

  void setOutputDegreesPerEncoderRevolution(float degreesPerEncoderRev);
  float outputDegreesPerEncoderRevolution() const;

  uint16_t lastRaw() const;
  float lastDegrees() const;
  float lastContinuousDegrees() const;
  float lastContinuousEncoderDegrees() const;

  bool lastReadOk() const;
  bool lastParityOk() const;
  bool lastErrorFlag() const;

private:
  SPIClass& _spi;
  int _csPin;

  static constexpr int32_t COUNTS_PER_REV = 16384;
  static constexpr int32_t HALF_REV = COUNTS_PER_REV / 2;

  uint16_t _lastRaw = 0;
  bool _lastReadOk = false;
  bool _lastParityOk = false;
  bool _lastErrorFlag = false;

  float _outputDegreesPerEncoderRev = 360.0f;
  float _lastContinuousDeg = 0.0f;

  bool _continuousInitialized = false;
  int64_t _turnCount = 0;
  uint16_t _prevRawAngle = 0;
  int64_t _continuousCounts = 0;

  uint16_t transfer16(uint16_t value);
  uint16_t buildCommand(uint16_t commandWithoutParity) const;
  bool checkEvenParity(uint16_t value) const;
  bool calcEvenParity(uint16_t value) const;

  int64_t updateContinuousCounts(uint16_t rawAngle);
  float continuousCountsToOutputDegrees(int64_t counts) const;
  float continuousCountsToEncoderDegrees(int64_t counts) const;
};
