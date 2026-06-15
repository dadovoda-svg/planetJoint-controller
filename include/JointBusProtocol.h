#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

namespace JointBus {

static constexpr uint8_t PROTOCOL_VERSION = 0;
static constexpr uint8_t SOF = 0xA5;
static constexpr size_t MAX_PAYLOAD = 16;
static constexpr size_t MAX_FRAME_SIZE = 1 + 1 + 1 + 1 + 1 + MAX_PAYLOAD + 2;

// Header layout:
// bit 7..4: destination address, 0..15
// bit 3..2: frame type
// bit 1..0: protocol version
static constexpr uint8_t HEADER_ADDR_SHIFT = 4;
static constexpr uint8_t HEADER_ADDR_MASK  = 0xF0;
static constexpr uint8_t HEADER_TYPE_SHIFT = 2;
static constexpr uint8_t HEADER_TYPE_MASK  = 0x0C;
static constexpr uint8_t HEADER_VER_MASK   = 0x03;

enum class FrameType : uint8_t {
    Request  = 0,
    Response = 1,
    Reserved2 = 2,
    Reserved3 = 3
};

enum class Command : uint8_t {
    Nop        = 0x00,
    Move       = 0x01,
    MoveB      = 0x02,
    Home       = 0x03,
    Stop       = 0x04,
    Reboot     = 0x05,
    Status     = 0x06,
    QuickStatus= 0x07,
    Zero       = 0x08,
    Park       = 0x09,
    Ping       = 0x7F,

    Ack        = 0x80,
    Nack       = 0x81,
    StatusRsp  = 0x86,
    QuickStatusRsp = 0x87,
    ErrorRsp   = 0xFF
};

enum class AckCode : uint8_t {
    Accepted           = 0x00,
    ClippedToMin       = 0x01,
    ClippedToMax       = 0x02,
    SafeReplan         = 0x03,
    AlreadyDone        = 0x04,
    BlendAccepted      = 0x05,
    NopAccepted        = 0x06
};

enum class NackCode : uint8_t {
    BadCrc             = 0x01,
    BadLength          = 0x02,
    BadCommand         = 0x03,
    BadPayload         = 0x04,
    Busy               = 0x05,
    FaultActive        = 0x06,
    NotHomed           = 0x07,
    RejectedByState    = 0x08,
    UnsupportedVersion = 0x09,
    InternalError      = 0x0A,
    Timeout            = 0x0B
};

enum class JointState : uint8_t {
    Init     = 0x00,
    Ready    = 0x01,
    Moving   = 0x02,
    Settling = 0x03,
    Holding  = 0x04,
    Stopped  = 0x05,
    Disabled = 0x06,
    Homing   = 0x07,
    Fault    = 0x08,
    Parking  = 0x09
};

enum class JointFault : uint8_t {
    None          = 0x00,
    PositionLimit = 0x01,
    EncoderError  = 0x02,
    DriverError   = 0x03,
    PlannerError  = 0x04,
    BadCommand    = 0x06,
    BadPayload    = 0x07,
    NotHomed      = 0x09,
    InternalError = 0x0A
};

enum QuickStatusFlags : uint8_t {
    QSTAT_BUSY          = 0x01,
    QSTAT_DONE          = 0x02,
    QSTAT_FAULT         = 0x04,
    QSTAT_ENABLED       = 0x08,
    QSTAT_HOMED         = 0x10,
    QSTAT_WARNING       = 0x20,
    QSTAT_LIMIT_CLIPPED = 0x40
};

static constexpr uint16_t REBOOT_MAGIC = 0xB007;

struct Status {
    int16_t posCdeg = 0;
    int16_t targetCdeg = 0;
    int16_t velCdegS = 0;
    JointState state = JointState::Init;
    JointFault fault = JointFault::None;
};

struct Frame {
    uint8_t address = 0;
    FrameType type = FrameType::Request;
    uint8_t version = PROTOCOL_VERSION;
    uint8_t seq = 0;
    Command command = Command::Ping;
    uint8_t payloadLen = 0;
    uint8_t payload[MAX_PAYLOAD] = {0};
};

static inline uint8_t makeHeader(uint8_t address, FrameType type, uint8_t version = PROTOCOL_VERSION) {
    return static_cast<uint8_t>(((address & 0x0F) << HEADER_ADDR_SHIFT) |
                                ((static_cast<uint8_t>(type) & 0x03) << HEADER_TYPE_SHIFT) |
                                (version & HEADER_VER_MASK));
}

static inline uint8_t headerAddress(uint8_t header) {
    return static_cast<uint8_t>((header & HEADER_ADDR_MASK) >> HEADER_ADDR_SHIFT);
}

static inline FrameType headerType(uint8_t header) {
    return static_cast<FrameType>((header & HEADER_TYPE_MASK) >> HEADER_TYPE_SHIFT);
}

static inline uint8_t headerVersion(uint8_t header) {
    return static_cast<uint8_t>(header & HEADER_VER_MASK);
}

static inline void putU16LE(uint8_t* dst, uint16_t value) {
    dst[0] = static_cast<uint8_t>(value & 0xFF);
    dst[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

static inline void putI16LE(uint8_t* dst, int16_t value) {
    putU16LE(dst, static_cast<uint16_t>(value));
}

static inline uint16_t getU16LE(const uint8_t* src) {
    return static_cast<uint16_t>(src[0]) | (static_cast<uint16_t>(src[1]) << 8);
}

static inline int16_t getI16LE(const uint8_t* src) {
    return static_cast<int16_t>(getU16LE(src));
}

// CRC-16/MODBUS: polynomial 0xA001, init 0xFFFF, little-endian on wire.
static inline uint16_t crc16ModbusUpdate(uint16_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; ++i) {
        if (crc & 0x0001) {
            crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
        } else {
            crc = static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

static inline uint16_t crc16Modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = crc16ModbusUpdate(crc, data[i]);
    }
    return crc;
}

// Encodes a frame into outBuffer. Returns number of bytes, or 0 on error.
static inline size_t encodeFrame(const Frame& frame, uint8_t* outBuffer, size_t outCapacity) {
    if (frame.payloadLen > MAX_PAYLOAD) {
        return 0;
    }

    const size_t frameLen = 1 + 1 + 1 + 1 + 1 + frame.payloadLen + 2;
    if (outCapacity < frameLen) {
        return 0;
    }

    size_t idx = 0;
    outBuffer[idx++] = SOF;
    outBuffer[idx++] = makeHeader(frame.address, frame.type, frame.version);
    outBuffer[idx++] = frame.seq;
    outBuffer[idx++] = frame.payloadLen;
    outBuffer[idx++] = static_cast<uint8_t>(frame.command);

    for (uint8_t i = 0; i < frame.payloadLen; ++i) {
        outBuffer[idx++] = frame.payload[i];
    }

    const uint16_t crc = crc16Modbus(&outBuffer[1], 4 + frame.payloadLen);
    outBuffer[idx++] = static_cast<uint8_t>(crc & 0xFF);
    outBuffer[idx++] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return idx;
}

class Parser {
public:
    enum class Result : uint8_t {
        None,
        FrameReady,
        BadCrc,
        BadLength,
        UnsupportedVersion
    };

    Parser() { reset(); }

    void reset() {
        _state = State::WaitSof;
        _idx = 0;
        _expectedPayloadLen = 0;
        _payloadIdx = 0;
        _crcLo = 0;
        _frame = Frame{};
    }

    Result push(uint8_t byte, Frame& outFrame) {
        switch (_state) {
        case State::WaitSof:
            if (byte == SOF) {
                _idx = 0;
                _payloadIdx = 0;
                _expectedPayloadLen = 0;
                _crcLo = 0;
                _frame = Frame{};
                _state = State::Header;
            }
            return Result::None;

        case State::Header:
            _buffer[_idx++] = byte;
            _frame.address = headerAddress(byte);
            _frame.type = headerType(byte);
            _frame.version = headerVersion(byte);
            _state = State::Seq;
            return Result::None;

        case State::Seq:
            _buffer[_idx++] = byte;
            _frame.seq = byte;
            _state = State::Len;
            return Result::None;

        case State::Len:
            _buffer[_idx++] = byte;
            _expectedPayloadLen = byte;
            if (_expectedPayloadLen > MAX_PAYLOAD) {
                reset();
                return Result::BadLength;
            }
            _frame.payloadLen = _expectedPayloadLen;
            _state = State::Cmd;
            return Result::None;

        case State::Cmd:
            _buffer[_idx++] = byte;
            _frame.command = static_cast<Command>(byte);
            if (_expectedPayloadLen == 0) {
                _state = State::CrcLo;
            } else {
                _payloadIdx = 0;
                _state = State::Payload;
            }
            return Result::None;

        case State::Payload:
            _buffer[_idx++] = byte;
            _frame.payload[_payloadIdx++] = byte;
            if (_payloadIdx >= _expectedPayloadLen) {
                _state = State::CrcLo;
            }
            return Result::None;

        case State::CrcLo:
            _crcLo = byte;
            _state = State::CrcHi;
            return Result::None;

        case State::CrcHi: {
            const uint16_t received = static_cast<uint16_t>(_crcLo) | (static_cast<uint16_t>(byte) << 8);
            const uint16_t computed = crc16Modbus(_buffer, _idx);
            const uint8_t version = _frame.version;
            if (received != computed) {
                reset();
                return Result::BadCrc;
            }
            if (version != PROTOCOL_VERSION) {
                reset();
                return Result::UnsupportedVersion;
            }
            outFrame = _frame;
            reset();
            return Result::FrameReady;
        }
        }

        reset();
        return Result::None;
    }

private:
    enum class State : uint8_t {
        WaitSof,
        Header,
        Seq,
        Len,
        Cmd,
        Payload,
        CrcLo,
        CrcHi
    };

    State _state = State::WaitSof;
    uint8_t _buffer[1 + 1 + 1 + 1 + MAX_PAYLOAD] = {0}; // HEADER..PAYLOAD, without SOF/CRC.
    uint8_t _idx = 0;
    uint8_t _expectedPayloadLen = 0;
    uint8_t _payloadIdx = 0;
    uint8_t _crcLo = 0;
    Frame _frame;
};

} // namespace JointBus
