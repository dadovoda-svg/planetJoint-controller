#pragma once

#include <Arduino.h>

class Tmc2209Driver {
public:
    enum class Status : uint8_t {
        NotStarted,
        CommunicationOnly,
        Configured,
        Enabled,
        Fault
    };

    struct Pins {
        int8_t step;
        int8_t dir;
        int8_t enn;       // ENN: HIGH = driver disabled, LOW = driver enabled
        int8_t uartRx;    // ESP32 RX connected to PDN_UART
        int8_t uartTx;    // ESP32 TX connected to PDN_UART through 1k series resistor
    };

    struct Config {
        uint8_t uartAddress = 0;        // selected by MS1/MS2: 0..3
        uint32_t baud = 115200;
        float fclkHz = 12000000.0f;     // internal oscillator nominal value

        // External sense resistors mode. Analog VREF/IREF scaling disabled.
        // Current is set by IRUN/IHOLD register scaling.
        bool useInternalRsense = false;
        bool useStealthChop = true;
        bool interpolate256 = true;
        uint16_t microstepResolution = 16; // 1,2,4,8,16,32,64,128,256

        // Current scale values, not absolute current values.
        // Full scale still depends on Rsense and vsense.
        uint8_t irun = 16;        // 0..31
        uint8_t ihold = 8;        // 0..31
        uint8_t iholdDelay = 4;   // 0..15
        uint8_t tpowerDown = 20;  // 0..255
        bool vsenseLowRange = false; // false = ~325mV full scale, true = ~180mV
    };

    Tmc2209Driver(HardwareSerial& serial, const Pins& pins, const Config& config);

    bool beginCommunicationOnly();
    bool configure();
    bool armPowerStage();
    bool enableDriver();
    void disableDriver(bool alsoDisableChopper = true);

    // Direct STEP/DIR helpers
    void dir(bool forward);
    void step(uint16_t highTimeUs = 2);
    void step(bool forward, uint16_t highTimeUs = 2);

    // Internal pulse generator
    bool runVelocityMicrostepsPerSecond(float microstepsPerSecond);
    bool stopInternalMotion();

    // Current setting
    bool setCurrentScale(uint8_t irun, uint8_t ihold, uint8_t iholdDelay);
    bool setRunCurrentRms(float targetRmsA, float rsenseOhm, float holdRatio = 0.5f);

    // Low-level register access
    bool writeReg(uint8_t reg, uint32_t value);
    bool readReg(uint8_t reg, uint32_t& value, uint32_t timeoutMs = 20);

    // Diagnostics / probe
    bool probe();
    bool readGstat(uint32_t& gstat);
    bool readIoin(uint32_t& ioin);
    bool readDrvStatus(uint32_t& status);
    bool readIfcnt(uint32_t& ifcnt);

    Status status() const;

private:
    static constexpr uint8_t SYNC = 0x05;

    static constexpr uint8_t REG_GCONF      = 0x00;
    static constexpr uint8_t REG_GSTAT      = 0x01;
    static constexpr uint8_t REG_IFCNT      = 0x02;
    static constexpr uint8_t REG_IOIN       = 0x06;
    static constexpr uint8_t REG_IHOLD_IRUN = 0x10;
    static constexpr uint8_t REG_TPOWERDOWN = 0x11;
    static constexpr uint8_t REG_VACTUAL    = 0x22;
    static constexpr uint8_t REG_CHOPCONF   = 0x6C;
    static constexpr uint8_t REG_DRV_STATUS = 0x6F;
    static constexpr uint8_t REG_PWMCONF    = 0x70;

    HardwareSerial& _serial;
    Pins _pins;
    Config _cfg;
    Status _status = Status::NotStarted;
    bool _driverEnableAllowed = false;

    uint32_t _gconfShadow = 0;
    uint32_t _chopconfShadow = 0;
    uint32_t _pwmconfShadow = 0;
    uint32_t _iholdIrunShadow = 0;

    static void setBit(uint32_t& value, uint8_t bit, bool enabled);
    static uint8_t calcCrc(uint8_t* datagram, uint8_t length);
    static uint8_t mresCode(uint16_t microsteps);
    static uint32_t encodeSigned24(int32_t value);

    void clearRxBuffer();
    uint32_t buildChopconf(bool chopperEnabled) const;
    int32_t velocityToVactual(float microstepsPerSecond) const;
};
