#pragma once

#include <Arduino.h>
#include <atomic>
#include "JointBusProtocol.h"

namespace JointBus {

struct CommandResult {
    bool accepted = false;
    AckCode ack = AckCode::Accepted;
    NackCode nack = NackCode::InternalError;
    uint8_t detail = 0;

    static CommandResult ok(AckCode code = AckCode::Accepted, uint8_t detailValue = 0) {
        CommandResult r;
        r.accepted = true;
        r.ack = code;
        r.detail = detailValue;
        return r;
    }

    static CommandResult fail(NackCode code, uint8_t detailValue = 0) {
        CommandResult r;
        r.accepted = false;
        r.nack = code;
        r.detail = detailValue;
        return r;
    }
};

struct SlaveHooks {
    void* context = nullptr;

    CommandResult (*move)(void* context, int16_t targetCdeg, uint16_t vmaxCdegS, uint16_t amaxCdegS2) = nullptr;
    CommandResult (*moveb)(void* context, int16_t targetCdeg, uint16_t vmaxCdegS, uint16_t amaxCdegS2) = nullptr;
    CommandResult (*prepareMoveB)(void* context, uint8_t segmentId, int16_t targetCdeg, uint16_t vmaxCdegS, uint16_t amaxCdegS2) = nullptr;
    CommandResult (*prepareHold)(void* context, uint8_t segmentId) = nullptr;
    CommandResult (*startSegment)(void* context, uint8_t segmentId) = nullptr;
    CommandResult (*scheduleSegment)(void* context, uint8_t segmentId, uint32_t durationMs, uint32_t delayUs) = nullptr;
    CommandResult (*abortSegment)(void* context, uint8_t segmentId) = nullptr;
    CommandResult (*emergencyStop)(void* context) = nullptr;
    CommandResult (*clearFault)(void* context) = nullptr;
    bool (*queueStatus)(void* context, QueueStatus& outStatus) = nullptr;
    bool (*motionState)(void* context, MotionState& outState) = nullptr;
    bool (*segmentTiming)(void* context, uint8_t segmentId, SegmentTiming& outTiming) = nullptr;
    CommandResult (*home)(void* context) = nullptr;
    CommandResult (*zero)(void* context) = nullptr;
    CommandResult (*park)(void* context) = nullptr;
    CommandResult (*stop)(void* context) = nullptr;
    CommandResult (*reboot)(void* context, uint16_t magic) = nullptr;
    bool (*status)(void* context, Status& outStatus) = nullptr;
    bool (*quickStatus)(void* context, uint8_t& outQuickStatus) = nullptr;
    bool (*motionConfig)(void* context, MotionConfig& outConfig) = nullptr;
};

class Slave {
public:
    // rs485RtsPin is optional. Pass -1 to use the UART as a regular serial port.
    // On ESP32-S3, a valid pin is assigned to UART RTS and used by
    // UART_MODE_RS485_HALF_DUPLEX to control the SP3485 DE input automatically.
    Slave(HardwareSerial& serial, uint8_t address, const SlaveHooks& hooks, int8_t rs485RtsPin = -1);

    // Call as the first instruction in setup() when DE has no hardware pull-down.
    // This is only an early-boot mitigation; begin() later transfers the pin to UART RTS.
    static void forceRs485Inactive(int8_t rs485RtsPin);

    // Initializes the UART and, when rs485RtsPin >= 0, enables the ESP32-S3
    // hardware-assisted RS485 half-duplex mode. Returns false on configuration failure.
    bool begin(uint32_t baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1);
    bool hardwareRs485Enabled() const { return _hardwareRs485Enabled; }

    void setAddress(uint8_t address);
    uint8_t address() const { return _address; }

    void setHooks(const SlaveHooks& hooks) { _hooks = hooks; }

    // Non-blocking. Call frequently from loop().
    void update();
    void flushRx();

    uint32_t rxFrames() const { return _rxFrames; }
    uint32_t txFrames() const { return _txFrames; }
    uint32_t crcErrors() const { return _crcErrors; }
    uint32_t lengthErrors() const { return _lengthErrors; }
    uint32_t versionErrors() const { return _versionErrors; }
    uint32_t ignoredFrames() const { return _ignoredFrames; }
    uint32_t broadcastFrames() const { return _broadcastFrames; }
    uint32_t broadcastStartFrames() const { return _broadcastStartFrames; }
    uint32_t broadcastStartAccepted() const { return _broadcastStartAccepted; }
    uint32_t broadcastStartRejected() const { return _broadcastStartRejected; }
    uint8_t lastBroadcastStartSeq() const { return _lastBroadcastStartSeq; }
    NackCode lastBroadcastStartNack() const { return _lastBroadcastStartNack; }
    uint32_t uartErrors() const { return _uartErrors.load(); }
    uint32_t uartBufferFullErrors() const { return _uartBufferFullErrors.load(); }
    uint32_t uartFifoOverflowErrors() const { return _uartFifoOverflowErrors.load(); }
    uint32_t uartFrameErrors() const { return _uartFrameErrors.load(); }
    uint32_t uartParityErrors() const { return _uartParityErrors.load(); }
    uint32_t uartBreakErrors() const { return _uartBreakErrors.load(); }
    void resetDiagnostics();

private:
    HardwareSerial& _serial;
    int8_t _rs485RtsPin = -1;
    bool _hardwareRs485Enabled = false;
    uint8_t _address = 0;
    SlaveHooks _hooks;
    Parser _parser;

    uint32_t _rxFrames = 0;
    uint32_t _txFrames = 0;
    uint32_t _crcErrors = 0;
    uint32_t _lengthErrors = 0;
    uint32_t _versionErrors = 0;
    uint32_t _ignoredFrames = 0;
    uint32_t _broadcastFrames = 0;
    uint32_t _broadcastStartFrames = 0;
    uint32_t _broadcastStartAccepted = 0;
    uint32_t _broadcastStartRejected = 0;
    uint8_t _lastBroadcastStartSeq = 0;
    NackCode _lastBroadcastStartNack = NackCode::InternalError;
    std::atomic<uint32_t> _uartErrors{0};
    std::atomic<uint32_t> _uartBufferFullErrors{0};
    std::atomic<uint32_t> _uartFifoOverflowErrors{0};
    std::atomic<uint32_t> _uartFrameErrors{0};
    std::atomic<uint32_t> _uartParityErrors{0};
    std::atomic<uint32_t> _uartBreakErrors{0};

    void handleFrame(const Frame& request);
    void sendAck(uint8_t seq, AckCode code, uint8_t detail = 0);
    void sendNack(uint8_t seq, NackCode code, uint8_t detail = 0);
    void sendStatus(uint8_t seq, const Status& status);
    void sendQuickStatus(uint8_t seq, uint8_t qstatus);
    void sendQueueStatus(uint8_t seq, const QueueStatus& status);
    void sendMotionState(uint8_t seq, const MotionState& state);
    void sendSegmentTiming(uint8_t seq, const SegmentTiming& timing);
    void sendMotionConfig(uint8_t seq, const MotionConfig& config);
    void sendFrame(const Frame& frame);

    CommandResult dispatchMoveLike(const Frame& request, bool blended);
    CommandResult dispatchPrepareMoveB(const Frame& request);
    CommandResult dispatchPrepareHold(const Frame& request);
};

} // namespace JointBus
