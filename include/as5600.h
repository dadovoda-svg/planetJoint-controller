#pragma once

#include <Arduino.h>
#include <Wire.h>

// AS5600 I2C magnetic encoder driver exposing the same high-level API used by
// the AS5048A driver. The native 12-bit angle is shifted left by two bits so
// all upper layers continue to operate in a virtual 14-bit, 16384-count domain.
class AS5600 {
public:
  explicit AS5600(TwoWire& wire, uint8_t address = 0x36);

  void begin();

  // Configure AS5600 volatile CONF register for deterministic runtime behavior.
  // The default project configuration is PM=NOM, WD=OFF, FTH=OFF, SF=4x.
  bool configureDefaultFilter();


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
  TwoWire& _wire;
  uint8_t _address;

  static constexpr int32_t COUNTS_PER_REV = 16384;
  static constexpr int32_t HALF_REV = COUNTS_PER_REV / 2;
  static constexpr uint16_t DATA_MASK = 0x3FFF;

  uint16_t _lastRaw = 0;
  bool _lastReadOk = false;
  bool _lastParityOk = true;
  bool _lastErrorFlag = false;

  float _outputDegreesPerEncoderRev = 360.0f;
  float _lastContinuousDeg = 0.0f;

  bool _continuousInitialized = false;
  int64_t _turnCount = 0;
  uint16_t _prevRawAngle = 0;
  int64_t _continuousCounts = 0;

  bool readNative12(uint16_t& raw12);
  bool readRegister8(uint8_t reg, uint8_t& value);
  bool writeRegister8(uint8_t reg, uint8_t value);
  int64_t updateContinuousCounts(uint16_t rawAngle);
  float continuousCountsToOutputDegrees(int64_t counts) const;
  float continuousCountsToEncoderDegrees(int64_t counts) const;
};
