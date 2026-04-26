#include "as5048a.h"

// ===================== AS5048A =====================
// AS5048A SPI module implementation
// Datasheet: https://ams.com/documents/20143/36005/AS5048A_DS000469_4-00.pdf
// MISO = DOUT - purple 
// MOSI = DIN  - blue
// SCK = CLK   - green
// CS = CS     - white
// 
// AS5048A SPI:
// CPOL = 0, CPHA = 1 -> SPI_MODE1
// Clock prudente: 1 MHz
static SPISettings AS5048A_SPI_SETTINGS(1000000, MSBFIRST, SPI_MODE1);

// Registri/comandi principali
static constexpr uint16_t AS5048A_REG_ANGLE      = 0x3FFF;
static constexpr uint16_t AS5048A_CMD_READ_FLAG  = 0x4000;
static constexpr uint16_t AS5048A_CLEAR_ERROR    = 0x0001;

// Maschere risposta
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
  _continuousInitialized = false;
  _turnCount = 0;
  _prevRawAngle = 0;
  _continuousCounts = 0;
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

  // Il bit 15 viene impostato in modo da ottenere parità pari
  if (calcEvenParity(command)) {
    command |= AS5048A_PARITY_BIT;
  }

  return command;
}

bool AS5048A::readRaw(uint16_t& rawAngle) {
  const uint16_t readAngleCmd = buildCommand(AS5048A_CMD_READ_FLAG | AS5048A_REG_ANGLE);

  // Primo frame: invia richiesta lettura angolo.
  // Secondo frame: riceve la risposta alla richiesta precedente.
  transfer16(readAngleCmd);
  uint16_t response = transfer16(readAngleCmd);

  _lastParityOk = checkEvenParity(response);
  _lastErrorFlag = (response & AS5048A_ERROR_FLAG) != 0;

  if (!_lastParityOk || _lastErrorFlag) {
    _lastReadOk = false;

    // Tentativo pulizia errore per letture successive
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

  angleDeg = (static_cast<float>(raw) * 360.0f) / 16384.0f;
  return true;
}

uint16_t AS5048A::lastRaw() const {
  return _lastRaw;
}

float AS5048A::lastDegrees() const {
  return (static_cast<float>(_lastRaw) * 360.0f) / 16384.0f;
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

// Track the AS5048A angle across multiple turns by unwrapping the 0..COUNTS_PER_REV value.
float AS5048A::computeContinuousAngleDeg(uint16_t rawAngle) {
  if (!_continuousInitialized) {
    _prevRawAngle = rawAngle;
    _continuousCounts = rawAngle;
    _continuousInitialized = true;
  } else {
    int32_t delta = static_cast<int32_t>(rawAngle) -
                    static_cast<int32_t>(_prevRawAngle);

    if (delta > HALF_REV) {
      _turnCount--;
    } else if (delta < -HALF_REV) {
      _turnCount++;
    }

    _prevRawAngle = rawAngle;
    _continuousCounts = (_turnCount * COUNTS_PER_REV) + rawAngle;
  }

  return (static_cast<float>(_continuousCounts) * 360.0f) /
         static_cast<float>(COUNTS_PER_REV);
}

bool AS5048A::readContinuousDegrees(float& angleDeg) {
  uint16_t raw = 0;

  if (!readRaw(raw)) {
    return false;
  }

  angleDeg = computeContinuousAngleDeg(raw);
  return true;
}
