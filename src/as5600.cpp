#include "as5600.h"
#include <math.h>

static constexpr uint8_t AS5600_REG_RAW_ANGLE_H = 0x0C;
static constexpr uint16_t AS5600_NATIVE_MASK = 0x0FFF;

AS5600::AS5600(TwoWire& wire, uint8_t address)
  : _wire(wire), _address(address) {}

void AS5600::begin() {
  _lastRaw = 0;
  _lastReadOk = false;
  _lastParityOk = true;
  _lastErrorFlag = false;
  _lastContinuousDeg = 0.0f;
  resetContinuousTracking();
}

void AS5600::resetContinuousTracking() {
  _continuousInitialized = false;
  _turnCount = 0;
  _prevRawAngle = 0;
  _continuousCounts = 0;
  _lastContinuousDeg = 0.0f;
}

void AS5600::setOutputDegreesPerEncoderRevolution(float degreesPerEncoderRev) {
  if (isfinite(degreesPerEncoderRev) && degreesPerEncoderRev > 0.0f) {
    _outputDegreesPerEncoderRev = degreesPerEncoderRev;
  }
}

float AS5600::outputDegreesPerEncoderRevolution() const {
  return _outputDegreesPerEncoderRev;
}

bool AS5600::readNative12(uint16_t& raw12) {
  _wire.beginTransmission(_address);
  _wire.write(AS5600_REG_RAW_ANGLE_H);

  if (_wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t requested = 2;
  const uint8_t received = _wire.requestFrom(_address, requested);
  if (received != requested || _wire.available() < requested) {
    while (_wire.available()) {
      _wire.read();
    }
    return false;
  }

  const uint8_t high = static_cast<uint8_t>(_wire.read());
  const uint8_t low = static_cast<uint8_t>(_wire.read());
  raw12 = ((static_cast<uint16_t>(high) << 8) | low) & AS5600_NATIVE_MASK;
  return true;
}

bool AS5600::readRaw(uint16_t& rawAngle) {
  uint16_t raw12 = 0;

  if (!readNative12(raw12)) {
    _lastReadOk = false;
    _lastParityOk = true;  // AS5600 I2C angle data has no parity bit.
    _lastErrorFlag = true; // Reuses the common API as a transport-error flag.
    return false;
  }

  // Add two zero-valued least-significant bits to map 12-bit data into the
  // same 14-bit count domain used by the AS5048A implementation.
  _lastRaw = static_cast<uint16_t>((raw12 << 2) & DATA_MASK);
  rawAngle = _lastRaw;
  _lastReadOk = true;
  _lastParityOk = true;
  _lastErrorFlag = false;
  return true;
}

bool AS5600::readDegrees(float& angleDeg) {
  uint16_t raw = 0;
  if (!readRaw(raw)) {
    return false;
  }

  angleDeg = lastDegrees();
  return true;
}

uint16_t AS5600::lastRaw() const {
  return _lastRaw;
}

float AS5600::lastDegrees() const {
  return (static_cast<float>(_lastRaw) * 360.0f) /
         static_cast<float>(COUNTS_PER_REV);
}

float AS5600::lastContinuousDegrees() const {
  return _lastContinuousDeg;
}

float AS5600::lastContinuousEncoderDegrees() const {
  return continuousCountsToEncoderDegrees(_continuousCounts);
}

bool AS5600::lastReadOk() const {
  return _lastReadOk;
}

bool AS5600::lastParityOk() const {
  return _lastParityOk;
}

bool AS5600::lastErrorFlag() const {
  return _lastErrorFlag;
}

int64_t AS5600::updateContinuousCounts(uint16_t rawAngle) {
  rawAngle &= DATA_MASK;

  if (!_continuousInitialized) {
    _prevRawAngle = rawAngle;
    _turnCount = 0;
    _continuousCounts = rawAngle;
    _continuousInitialized = true;
  } else {
    const int32_t delta = static_cast<int32_t>(rawAngle) -
                          static_cast<int32_t>(_prevRawAngle);

    if (delta > HALF_REV) {
      _turnCount--;
    } else if (delta < -HALF_REV) {
      _turnCount++;
    }

    _prevRawAngle = rawAngle;
    _continuousCounts = (_turnCount * static_cast<int64_t>(COUNTS_PER_REV)) + rawAngle;
  }

  return _continuousCounts;
}

float AS5600::continuousCountsToEncoderDegrees(int64_t counts) const {
  return (static_cast<float>(counts) * 360.0f) /
         static_cast<float>(COUNTS_PER_REV);
}

float AS5600::continuousCountsToOutputDegrees(int64_t counts) const {
  return (static_cast<float>(counts) * _outputDegreesPerEncoderRev) /
         static_cast<float>(COUNTS_PER_REV);
}

float AS5600::computeContinuousAngleDeg(uint16_t rawAngle) {
  _lastContinuousDeg = continuousCountsToOutputDegrees(updateContinuousCounts(rawAngle));
  return _lastContinuousDeg;
}

bool AS5600::readContinuousDegrees(float& angleDeg) {
  uint16_t raw = 0;
  if (!readRaw(raw)) {
    return false;
  }

  angleDeg = computeContinuousAngleDeg(raw);
  return true;
}
