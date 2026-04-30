#include "Tmc2209Driver.h"

Tmc2209Driver::Tmc2209Driver(HardwareSerial& serial, const Pins& pins, const Config& config)
    : _serial(serial), _pins(pins), _cfg(config)
{
}

bool Tmc2209Driver::beginCommunicationOnly()
{
    pinMode(_pins.enn, OUTPUT);
    digitalWrite(_pins.enn, HIGH); // hard-safe: power bridge disabled

    pinMode(_pins.step, OUTPUT);
    digitalWrite(_pins.step, LOW);

    pinMode(_pins.dir, OUTPUT);
    digitalWrite(_pins.dir, LOW);

    _serial.begin(_cfg.baud, SERIAL_8N1, _pins.uartRx, _pins.uartTx);
    clearRxBuffer();

    _status = Status::CommunicationOnly;
    _driverEnableAllowed = false;

    uint32_t ioin = 0;
    if (!readReg(REG_IOIN, ioin)) {
        _status = Status::Fault;
        return false;
    }

    return true;
}

bool Tmc2209Driver::configure()
{
    if (_status != Status::CommunicationOnly && _status != Status::Configured) {
        return false;
    }

    // Stop any internal motion before touching driver configuration.
    if (!writeReg(REG_VACTUAL, 0)) {
        return false;
    }

    // GCONF:
    // bit 0 I_scale_analog = 0 -> use internal reference derived from 5VOUT, not external VREF
    // bit 1 internal_Rsense according to config
    // bit 2 en_SpreadCycle = 0 for StealthChop, 1 for SpreadCycle
    // bit 6 pdn_disable = 1 when using UART interface
    // bit 7 mstep_reg_select = 1 -> microstep resolution from CHOPCONF.MRES
    // bit 8 multistep_filt = 1
    _gconfShadow = 0;
    setBit(_gconfShadow, 1, _cfg.useInternalRsense);
    setBit(_gconfShadow, 2, !_cfg.useStealthChop);
    setBit(_gconfShadow, 6, true);
    setBit(_gconfShadow, 7, true);
    setBit(_gconfShadow, 8, true);
    if (!writeReg(REG_GCONF, _gconfShadow)) {
        return false;
    }

    // Clear global status flags.
    writeReg(REG_GSTAT, 0x00000007UL);

    // Current control.
    if (!setCurrentScale(_cfg.irun, _cfg.ihold, _cfg.iholdDelay)) {
        return false;
    }

    if (!writeReg(REG_TPOWERDOWN, _cfg.tpowerDown)) {
        return false;
    }

    // Chopper configuration remains disabled until armPowerStage()/enableDriver().
    _chopconfShadow = buildChopconf(false);
    if (!writeReg(REG_CHOPCONF, _chopconfShadow)) {
        return false;
    }

    // Conservative StealthChop default. Valid also if SpreadCycle is selected; harmless.
    _pwmconfShadow = 0xC10D0024UL;
    if (!writeReg(REG_PWMCONF, _pwmconfShadow)) {
        return false;
    }

    _status = Status::Configured;
    _driverEnableAllowed = false;
    return true;
}

bool Tmc2209Driver::armPowerStage()
{
    if (_status != Status::Configured) {
        return false;
    }

    // Configure the chopper, but keep ENN high.
    _chopconfShadow = buildChopconf(true);
    if (!writeReg(REG_CHOPCONF, _chopconfShadow)) {
        return false;
    }

    _driverEnableAllowed = true;
    return true;
}

bool Tmc2209Driver::enableDriver()
{
    if (!_driverEnableAllowed || _status != Status::Configured) {
        return false;
    }

    digitalWrite(_pins.enn, LOW); // ENN low = power bridge enabled
    _status = Status::Enabled;
    return true;
}

void Tmc2209Driver::disableDriver(bool alsoDisableChopper)
{
    digitalWrite(_pins.enn, HIGH); // immediate hardware disable
    writeReg(REG_VACTUAL, 0);

    if (alsoDisableChopper) {
        _chopconfShadow = buildChopconf(false);
        writeReg(REG_CHOPCONF, _chopconfShadow);
    }

    if (_status == Status::Enabled) {
        _status = Status::Configured;
    }

    _driverEnableAllowed = false;
}

void Tmc2209Driver::dir(bool forward)
{
    digitalWrite(_pins.dir, forward ? LOW : HIGH);
}

void Tmc2209Driver::step(uint16_t highTimeUs)
{
    digitalWrite(_pins.step, HIGH);
    delayMicroseconds(highTimeUs);
    digitalWrite(_pins.step, LOW);
}

void Tmc2209Driver::step(bool forward, uint16_t highTimeUs)
{
    dir(forward);
    delayMicroseconds(1);
    step(highTimeUs);
}

bool Tmc2209Driver::runVelocityMicrostepsPerSecond(float microstepsPerSecond)
{
    if (_status != Status::Enabled) {
        return false;
    }

    int32_t vactual = velocityToVactual(microstepsPerSecond);
    return writeReg(REG_VACTUAL, encodeSigned24(vactual));
}

bool Tmc2209Driver::stopInternalMotion()
{
    return writeReg(REG_VACTUAL, 0);
}

bool Tmc2209Driver::setMicrostepResolution(uint16_t microsteps)
{
    if (!isValidMicrostepResolution(microsteps)) {
        return false;
    }

    _cfg.microstepResolution = microsteps;

    // If the chip has not been configured yet, only update the cached
    // configuration. configure() will later write CHOPCONF.
    if (_status == Status::NotStarted || _status == Status::CommunicationOnly) {
        return true;
    }

    if (_status != Status::Configured && _status != Status::Enabled) {
        return false;
    }

    // Preserve the current chopper enable state while rebuilding CHOPCONF.
    const bool chopperEnabled = ((_chopconfShadow & 0x0F) != 0);
    const uint32_t oldChopconf = _chopconfShadow;

    _chopconfShadow = buildChopconf(chopperEnabled);
    if (!writeReg(REG_CHOPCONF, _chopconfShadow)) {
        _chopconfShadow = oldChopconf;
        return false;
    }

    return true;
}

uint16_t Tmc2209Driver::microstepResolution() const
{
    return _cfg.microstepResolution;
}

bool Tmc2209Driver::setCurrentScale(uint8_t irun, uint8_t ihold, uint8_t iholdDelay)
{
    irun = constrain(irun, 0, 31);
    ihold = constrain(ihold, 0, 31);
    iholdDelay = constrain(iholdDelay, 0, 15);

    _iholdIrunShadow =
        ((uint32_t)(ihold & 0x1F) << 0) |
        ((uint32_t)(irun & 0x1F) << 8) |
        ((uint32_t)(iholdDelay & 0x0F) << 16);

    return writeReg(REG_IHOLD_IRUN, _iholdIrunShadow);
}

bool Tmc2209Driver::setRunCurrentRms(float targetRmsA, float rsenseOhm, float holdRatio)
{
    if (targetRmsA <= 0.0f || rsenseOhm <= 0.0f) {
        return false;
    }

    const float vfs = _cfg.vsenseLowRange ? 0.180f : 0.325f;

    float csf = targetRmsA * 32.0f * (rsenseOhm + 0.020f) * 1.41421356f / vfs;
    int irun = (int)roundf(csf - 1.0f);
    irun = constrain(irun, 0, 31);

    int ihold = (int)roundf((irun + 1) * holdRatio - 1.0f);
    ihold = constrain(ihold, 0, 31);

    return setCurrentScale((uint8_t)irun, (uint8_t)ihold, _cfg.iholdDelay);
}

bool Tmc2209Driver::writeReg(uint8_t reg, uint32_t value)
{
    uint8_t datagram[8];
    datagram[0] = SYNC;
    datagram[1] = _cfg.uartAddress & 0x03;
    datagram[2] = reg | 0x80;
    datagram[3] = (uint8_t)(value >> 24);
    datagram[4] = (uint8_t)(value >> 16);
    datagram[5] = (uint8_t)(value >> 8);
    datagram[6] = (uint8_t)(value);
    datagram[7] = 0;
    datagram[7] = calcCrc(datagram, sizeof(datagram));

    clearRxBuffer();
    _serial.write(datagram, sizeof(datagram));
    _serial.flush();

    return true;
}

bool Tmc2209Driver::readReg(uint8_t reg, uint32_t& value, uint32_t timeoutMs)
{
    uint8_t request[4];
    request[0] = SYNC;
    request[1] = _cfg.uartAddress & 0x03;
    request[2] = reg & 0x7F;
    request[3] = 0;
    request[3] = calcCrc(request, sizeof(request));

    clearRxBuffer();
    _serial.write(request, sizeof(request));
    _serial.flush();

    // On a single-wire UART, the RX pin can see the transmitted request echo.
    // Therefore we scan the incoming stream for the TMC reply frame:
    // SYNC, 0xFF, register address, data[3..0], crc.
    uint8_t frame[8];
    uint8_t pos = 0;
    uint32_t start = millis();

    while ((millis() - start) < timeoutMs) {
        while (_serial.available()) {
            uint8_t b = (uint8_t)_serial.read();

            if (pos == 0) {
                if (b != SYNC) {
                    continue;
                }
                frame[pos++] = b;
                continue;
            }

            frame[pos++] = b;

            if (pos == 2 && frame[1] != 0xFF) {
                pos = 0;
                continue;
            }

            if (pos == 3 && frame[2] != (reg & 0x7F)) {
                pos = 0;
                continue;
            }

            if (pos == 8) {
                uint8_t crc = calcCrc(frame, sizeof(frame));
                if (crc != frame[7]) {
                    pos = 0;
                    continue;
                }

                value =
                    ((uint32_t)frame[3] << 24) |
                    ((uint32_t)frame[4] << 16) |
                    ((uint32_t)frame[5] << 8) |
                    ((uint32_t)frame[6]);

                return true;
            }
        }

        delay(1);
    }

    return false;
}

bool Tmc2209Driver::probe()
{
    uint32_t ioin = 0;
    if (!readReg(REG_IOIN, ioin)) {
        return false;
    }

    uint8_t version = (uint8_t)((ioin >> 24) & 0xFF);
    return version == 0x21;
}

bool Tmc2209Driver::readGstat(uint32_t& gstat)
{
    return readReg(REG_GSTAT, gstat);
}

bool Tmc2209Driver::readIoin(uint32_t& ioin)
{
    return readReg(REG_IOIN, ioin);
}

bool Tmc2209Driver::readDrvStatus(uint32_t& status)
{
    return readReg(REG_DRV_STATUS, status);
}

bool Tmc2209Driver::readIfcnt(uint32_t& ifcnt)
{
    return readReg(REG_IFCNT, ifcnt);
}

Tmc2209Driver::Status Tmc2209Driver::status() const
{
    return _status;
}

void Tmc2209Driver::setBit(uint32_t& value, uint8_t bit, bool enabled)
{
    if (enabled) {
        value |= (1UL << bit);
    } else {
        value &= ~(1UL << bit);
    }
}

uint8_t Tmc2209Driver::calcCrc(uint8_t* datagram, uint8_t length)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < length - 1; i++) {
        uint8_t currentByte = datagram[i];

        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (currentByte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }

            currentByte >>= 1;
        }
    }

    return crc;
}

void Tmc2209Driver::clearRxBuffer()
{
    while (_serial.available()) {
        _serial.read();
    }
}

uint32_t Tmc2209Driver::buildChopconf(bool chopperEnabled) const
{
    uint32_t value = 0;

    // TOFF: 0 = driver disabled, 3 = conservative enabled default.
    uint8_t toff = chopperEnabled ? 3 : 0;
    value |= (uint32_t)(toff & 0x0F);

    // HSTRT=5, HEND=0: close to reset/default practical starting point.
    value |= (uint32_t)(5 & 0x07) << 4;
    value |= (uint32_t)(0 & 0x0F) << 7;

    // TBL = 2 -> 32 clocks blank time.
    value |= (uint32_t)(2 & 0x03) << 15;

    // vsense bit 17.
    if (_cfg.vsenseLowRange) {
        value |= (1UL << 17);
    }

    // MRES bits 24..27.
    value |= (uint32_t)(mresCode(_cfg.microstepResolution) & 0x0F) << 24;

    // intpol bit 28.
    if (_cfg.interpolate256) {
        value |= (1UL << 28);
    }

    return value;
}

bool Tmc2209Driver::isValidMicrostepResolution(uint16_t microsteps)
{
    switch (microsteps) {
        case 1:
        case 2:
        case 4:
        case 8:
        case 16:
        case 32:
        case 64:
        case 128:
        case 256:
            return true;
        default:
            return false;
    }
}

uint8_t Tmc2209Driver::mresCode(uint16_t microsteps)
{
    switch (microsteps) {
        case 256: return 0;
        case 128: return 1;
        case 64:  return 2;
        case 32:  return 3;
        case 16:  return 4;
        case 8:   return 5;
        case 4:   return 6;
        case 2:   return 7;
        case 1:   return 8;
        default:  return 4; // safe default: 1/16
    }
}

int32_t Tmc2209Driver::velocityToVactual(float microstepsPerSecond) const
{
    float raw = microstepsPerSecond / (_cfg.fclkHz / 16777216.0f); // 2^24
    long value = lroundf(raw);

    if (value >  8388607L) {
        value =  8388607L;
    }

    if (value < -8388608L) {
        value = -8388608L;
    }

    return (int32_t)value;
}

uint32_t Tmc2209Driver::encodeSigned24(int32_t value)
{
    if (value >  8388607L) {
        value =  8388607L;
    }

    if (value < -8388608L) {
        value = -8388608L;
    }

    return ((uint32_t)value) & 0x00FFFFFFUL;
}
