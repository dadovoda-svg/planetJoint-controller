#include "as5048a.h"
#include <math.h>

// ===================== AS5048A =====================
// AS5048A SPI:
// CPOL = 0, CPHA = 1 -> SPI_MODE1
// Conservative clock: 1 MHz
static SPISettings AS5048A_SPI_SETTINGS(1000000, MSBFIRST, SPI_MODE1);

static constexpr uint16_t AS5048A_REG_ANGLE      = 0x3FFF;
static constexpr uint16_t AS5048A_CMD_READ_FLAG  = 0x4000;
static constexpr uint16_t AS5048A_CLEAR_ERROR    = 0x0001;

static constexpr uint16_t AS5048A_PARITY_BIT     = 0x8000;
static constexpr uint16_t AS5048A_ERROR_FLAG     = 0x4000;
static constexpr uint16_t AS5048A_DATA_MASK      = 0x3FFF;

AS5048A::AS5048A(SPIClass& spi, int csPin)
  : _spi(spi), _csPin(csPin) {}

void AS5048A::begin() {
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  _lastRaw = 0;
  _lastReadOk = false;
  _lastParityOk = false;
  _lastErrorFlag = false;
  _lastContinuousDeg = 0.0f;
  resetContinuousTracking();
}

void AS5048A::resetContinuousTracking() {
  _continuousInitialized = false;
  _turnCount = 0;
  _prevRawAngle = 0;
  _continuousCounts = 0;
  _lastContinuousDeg = 0.0f;
}

void AS5048A::setOutputDegreesPerEncoderRevolution(float degreesPerEncoderRev) {
  if (isfinite(degreesPerEncoderRev) && degreesPerEncoderRev > 0.0f) {
    _outputDegreesPerEncoderRev = degreesPerEncoderRev;
  }
}

float AS5048A::outputDegreesPerEncoderRevolution() const {
  return _outputDegreesPerEncoderRev;
}

uint16_t AS5048A::transfer16(uint16_t value) {
  _spi.beginTransaction(AS5048A_SPI_SETTINGS);

  digitalWrite(_csPin, LOW);
  delayMicroseconds(1);

  uint16_t response = _spi.transfer16(value);

  delayMicroseconds(1);
  digitalWrite(_csPin, HIGH);

  _spi.endTransaction();

  return response;
}

bool AS5048A::calcEvenParity(uint16_t value) const {
  bool parity = false;

  while (value) {
    parity = !parity;
    value &= (value - 1);
  }

  return parity;
}

bool AS5048A::checkEvenParity(uint16_t value) const {
  return calcEvenParity(value) == false;
}

uint16_t AS5048A::buildCommand(uint16_t commandWithoutParity) const {
  uint16_t command = commandWithoutParity & 0x7FFF;

  if (calcEvenParity(command)) {
    command |= AS5048A_PARITY_BIT;
  }

  return command;
}

bool AS5048A::readRaw(uint16_t& rawAngle) {
  const uint16_t readAngleCmd = buildCommand(AS5048A_CMD_READ_FLAG | AS5048A_REG_ANGLE);

  transfer16(readAngleCmd);
  uint16_t response = transfer16(readAngleCmd);

  _lastParityOk = checkEvenParity(response);
  _lastErrorFlag = (response & AS5048A_ERROR_FLAG) != 0;

  if (!_lastParityOk || _lastErrorFlag) {
    _lastReadOk = false;

    const uint16_t clearErrorCmd = buildCommand(AS5048A_CMD_READ_FLAG | AS5048A_CLEAR_ERROR);
    transfer16(clearErrorCmd);

    return false;
  }

  _lastRaw = response & AS5048A_DATA_MASK;
  rawAngle = _lastRaw;
  _lastReadOk = true;

  return true;
}

bool AS5048A::readDegrees(float& angleDeg) {
  uint16_t raw = 0;

  if (!readRaw(raw)) {
    return false;
  }

  angleDeg = lastDegrees();
  return true;
}

uint16_t AS5048A::lastRaw() const {
  return _lastRaw;
}

float AS5048A::lastDegrees() const {
  return (static_cast<float>(_lastRaw) * 360.0f) /
         static_cast<float>(COUNTS_PER_REV);
}

float AS5048A::lastContinuousDegrees() const {
  return _lastContinuousDeg;
}

float AS5048A::lastContinuousEncoderDegrees() const {
  return continuousCountsToEncoderDegrees(_continuousCounts);
}

bool AS5048A::lastReadOk() const {
  return _lastReadOk;
}

bool AS5048A::lastParityOk() const {
  return _lastParityOk;
}

bool AS5048A::lastErrorFlag() const {
  return _lastErrorFlag;
}

int64_t AS5048A::updateContinuousCounts(uint16_t rawAngle) {
  rawAngle &= AS5048A_DATA_MASK;

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

float AS5048A::continuousCountsToEncoderDegrees(int64_t counts) const {
  return (static_cast<float>(counts) * 360.0f) /
         static_cast<float>(COUNTS_PER_REV);
}

float AS5048A::continuousCountsToOutputDegrees(int64_t counts) const {
  return (static_cast<float>(counts) * _outputDegreesPerEncoderRev) /
         static_cast<float>(COUNTS_PER_REV);
}

float AS5048A::computeContinuousAngleDeg(uint16_t rawAngle) {
  _lastContinuousDeg = continuousCountsToOutputDegrees(updateContinuousCounts(rawAngle));
  return _lastContinuousDeg;
}

bool AS5048A::readContinuousDegrees(float& angleDeg) {
  uint16_t raw = 0;

  if (!readRaw(raw)) {
    return false;
  }

  angleDeg = computeContinuousAngleDeg(raw);
  return true;
}
