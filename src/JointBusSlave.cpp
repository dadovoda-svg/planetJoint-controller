#include "JointBusSlave.h"
#include "hal/uart_types.h"

namespace JointBus {

Slave::Slave(HardwareSerial& serial, uint8_t address, const SlaveHooks& hooks, int8_t rs485RtsPin)
    : _serial(serial), _rs485RtsPin(rs485RtsPin), _address(address & 0x0F), _hooks(hooks) {
}

void Slave::forceRs485Inactive(int8_t rs485RtsPin) {
    if (rs485RtsPin < 0) {
        return;
    }

    // SP3485 DE is active-high. Keep it low until UART RS485 mode takes ownership.
    pinMode(static_cast<uint8_t>(rs485RtsPin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(rs485RtsPin), LOW);
}

bool Slave::begin(uint32_t baud, uint32_t config, int8_t rxPin, int8_t txPin) {
    _hardwareRs485Enabled = false;

    if (_rs485RtsPin >= 0) {
        forceRs485Inactive(_rs485RtsPin);
    }

    _serial.begin(baud, config, rxPin, txPin);
    _serial.onReceiveError([this](hardwareSerial_error_t error) {
        ++_uartErrors;
        switch (error) {
        case UART_BUFFER_FULL_ERROR: ++_uartBufferFullErrors; break;
        case UART_FIFO_OVF_ERROR: ++_uartFifoOverflowErrors; break;
        case UART_FRAME_ERROR: ++_uartFrameErrors; break;
        case UART_PARITY_ERROR: ++_uartParityErrors; break;
        case UART_BREAK_ERROR: ++_uartBreakErrors; break;
        default: break;
        }
    });

    if (_rs485RtsPin < 0) {
        return true;
    }

    if (!_serial.setPins(-1, -1, -1, _rs485RtsPin)) {
        return false;
    }

    if (!_serial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE)) {
        return false;
    }

    if (!_serial.setMode(UART_MODE_RS485_HALF_DUPLEX)) {
        return false;
    }

    _hardwareRs485Enabled = true;
    return true;
}

void Slave::setAddress(uint8_t address) {
    _address = address & 0x0F;
}

void Slave::resetDiagnostics() {
    _rxFrames = 0;
    _txFrames = 0;
    _crcErrors = 0;
    _lengthErrors = 0;
    _versionErrors = 0;
    _ignoredFrames = 0;
    _broadcastFrames = 0;
    _broadcastStartFrames = 0;
    _broadcastStartAccepted = 0;
    _broadcastStartRejected = 0;
    _lastBroadcastStartSeq = 0;
    _lastBroadcastStartNack = NackCode::InternalError;
    _uartErrors = 0;
    _uartBufferFullErrors = 0;
    _uartFifoOverflowErrors = 0;
    _uartFrameErrors = 0;
    _uartParityErrors = 0;
    _uartBreakErrors = 0;
}

void Slave::flushRx() {
    _parser.reset();
    while (_serial.available() > 0) {
        (void)_serial.read();
    }
}

void Slave::update() {
    while (_serial.available() > 0) {
        const uint8_t b = static_cast<uint8_t>(_serial.read());
        Frame frame;
        const Parser::Result result = _parser.push(b, frame);

        switch (result) {
        case Parser::Result::None:
            break;

        case Parser::Result::FrameReady:
            ++_rxFrames;
            handleFrame(frame);
            break;

        case Parser::Result::BadCrc:
            ++_crcErrors;
            break;

        case Parser::Result::BadLength:
            ++_lengthErrors;
            break;

        case Parser::Result::UnsupportedVersion:
            ++_versionErrors;
            break;
        }
    }
}

void Slave::handleFrame(const Frame& request) {
    const bool isBroadcast = (request.address == BROADCAST_ADDRESS);
    if (isBroadcast) {
        ++_broadcastFrames;
    }

    if (request.address != _address && !isBroadcast) {
        ++_ignoredFrames;
        return;
    }

    if (request.type != FrameType::Request) {
        if (!isBroadcast) {
            sendNack(request.seq, NackCode::RejectedByState);
        }
        return;
    }

    // Broadcast frames are intentionally no-reply to avoid RS485 collisions.
    // Only segment-start/abort and emergency-stop are accepted as broadcast commands in this
    // protocol revision. Addressed use remains available for debugging.
    if (isBroadcast && request.command != Command::StartSegment &&
        request.command != Command::AbortSegment &&
        request.command != Command::EmergencyStop) {
        ++_ignoredFrames;
        return;
    }

    switch (request.command) {
    case Command::Nop:
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
        } else {
            sendAck(request.seq, AckCode::NopAccepted);
        }
        break;

    case Command::Move: {
        const CommandResult r = dispatchMoveLike(request, false);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::MoveB: {
        const CommandResult r = dispatchMoveLike(request, true);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::PrepareMoveB: {
        const CommandResult r = dispatchPrepareMoveB(request);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::PrepareHold: {
        const CommandResult r = dispatchPrepareHold(request);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::StartSegment: {
        if (isBroadcast) {
            ++_broadcastStartFrames;
            _lastBroadcastStartSeq = request.seq;
        }
        if (request.payloadLen != 1 &&
            request.payloadLen != TIMED_START_PAYLOAD_SIZE) {
            if (!isBroadcast) {
                sendNack(request.seq, NackCode::BadLength);
            }
            break;
        }
        const uint8_t segmentId = request.payload[0];
        CommandResult r;
        if (request.payloadLen == TIMED_START_PAYLOAD_SIZE) {
            const uint32_t durationMs = getU32LE(&request.payload[1]);
            const uint32_t delayUs = getU32LE(&request.payload[5]);
            r = _hooks.scheduleSegment
                ? _hooks.scheduleSegment(
                    _hooks.context, segmentId, durationMs, delayUs)
                : CommandResult::fail(NackCode::BadCommand);
        } else {
            r = _hooks.startSegment
                ? _hooks.startSegment(_hooks.context, segmentId)
                : CommandResult::fail(NackCode::BadCommand);
        }
        if (!isBroadcast) {
            r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        } else if (r.accepted) {
            ++_broadcastStartAccepted;
        } else {
            ++_broadcastStartRejected;
            _lastBroadcastStartNack = r.nack;
        }
        break;
    }

    case Command::AbortSegment: {
        if (request.payloadLen != 1) {
            if (!isBroadcast) {
                sendNack(request.seq, NackCode::BadLength);
            }
            break;
        }
        const uint8_t segmentId = request.payload[0];
        const CommandResult r = _hooks.abortSegment ? _hooks.abortSegment(_hooks.context, segmentId)
                                                    : CommandResult::fail(NackCode::BadCommand);
        if (!isBroadcast) {
            r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        }
        break;
    }

    case Command::QueueStatus: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        QueueStatus status;
        if (_hooks.queueStatus && _hooks.queueStatus(_hooks.context, status)) {
            sendQueueStatus(request.seq, status);
        } else {
            sendNack(request.seq, NackCode::InternalError);
        }
        break;
    }

    case Command::MotionState: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        MotionState state;
        if (_hooks.motionState &&
            _hooks.motionState(_hooks.context, state)) {
            sendMotionState(request.seq, state);
        } else {
            sendNack(request.seq, NackCode::InternalError);
        }
        break;
    }

    case Command::SegmentTiming: {
        if (request.payloadLen != 1) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        SegmentTiming timing;
        if (_hooks.segmentTiming &&
            _hooks.segmentTiming(
                _hooks.context, request.payload[0], timing)) {
            sendSegmentTiming(request.seq, timing);
        } else {
            sendNack(request.seq, NackCode::RejectedByState);
        }
        break;
    }

    case Command::Home: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        const CommandResult r = _hooks.home ? _hooks.home(_hooks.context)
                                            : CommandResult::fail(NackCode::BadCommand);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::Zero: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        const CommandResult r = _hooks.zero ? _hooks.zero(_hooks.context)
                                            : CommandResult::fail(NackCode::BadCommand);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::Park: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        const CommandResult r = _hooks.park ? _hooks.park(_hooks.context)
                                            : CommandResult::fail(NackCode::BadCommand);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::Stop: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        const CommandResult r = _hooks.stop ? _hooks.stop(_hooks.context)
                                            : CommandResult::fail(NackCode::BadCommand);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::HoldPosition: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        const CommandResult r = _hooks.holdPosition
            ? _hooks.holdPosition(_hooks.context)
            : CommandResult::fail(NackCode::BadCommand);
        r.accepted ? sendAck(request.seq, r.ack, r.detail)
                   : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::EmergencyStop: {
        if (request.payloadLen != 0) {
            if (!isBroadcast) {
                sendNack(request.seq, NackCode::BadLength);
            }
            break;
        }
        const CommandResult r = _hooks.emergencyStop ? _hooks.emergencyStop(_hooks.context)
                                                     : CommandResult::fail(NackCode::BadCommand);
        if (!isBroadcast) {
            r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        }
        break;
    }

    case Command::ClearFault: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        const CommandResult r = _hooks.clearFault
            ? _hooks.clearFault(_hooks.context)
            : CommandResult::fail(NackCode::BadCommand);
        r.accepted ? sendAck(request.seq, r.ack, r.detail)
                   : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::Reboot: {
        if (request.payloadLen != 2) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        const uint16_t magic = getU16LE(&request.payload[0]);
        const CommandResult r = _hooks.reboot ? _hooks.reboot(_hooks.context, magic)
                                              : CommandResult::fail(NackCode::BadCommand);
        r.accepted ? sendAck(request.seq, r.ack, r.detail) : sendNack(request.seq, r.nack, r.detail);
        break;
    }

    case Command::Status: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        Status status;
        if (_hooks.status && _hooks.status(_hooks.context, status)) {
            sendStatus(request.seq, status);
        } else {
            sendNack(request.seq, NackCode::InternalError);
        }
        break;
    }

    case Command::QuickStatus: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        uint8_t qstatus = 0;
        if (_hooks.quickStatus && _hooks.quickStatus(_hooks.context, qstatus)) {
            sendQuickStatus(request.seq, qstatus);
        } else {
            sendNack(request.seq, NackCode::InternalError);
        }
        break;
    }

    case Command::MotionConfig: {
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
            break;
        }
        MotionConfig config;
        if (_hooks.motionConfig && _hooks.motionConfig(_hooks.context, config)) {
            sendMotionConfig(request.seq, config);
        } else {
            sendNack(request.seq, NackCode::InternalError);
        }
        break;
    }

    case Command::Ping:
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
        } else {
            sendAck(request.seq, AckCode::Accepted);
        }
        break;

    default:
        if (!isBroadcast) {
            sendNack(request.seq, NackCode::BadCommand);
        }
        break;
    }
}

CommandResult Slave::dispatchMoveLike(const Frame& request, bool blended) {
    if (request.payloadLen != 6) {
        return CommandResult::fail(NackCode::BadLength);
    }

    const int16_t targetCdeg = getI16LE(&request.payload[0]);
    const uint16_t vmaxCdegS = getU16LE(&request.payload[2]);
    const uint16_t amaxCdegS2 = getU16LE(&request.payload[4]);

    if (vmaxCdegS == 0 || amaxCdegS2 == 0) {
        return CommandResult::fail(NackCode::BadPayload);
    }

    if (blended) {
        return _hooks.moveb ? _hooks.moveb(_hooks.context, targetCdeg, vmaxCdegS, amaxCdegS2)
                            : CommandResult::fail(NackCode::BadCommand);
    }

    return _hooks.move ? _hooks.move(_hooks.context, targetCdeg, vmaxCdegS, amaxCdegS2)
                       : CommandResult::fail(NackCode::BadCommand);
}


CommandResult Slave::dispatchPrepareMoveB(const Frame& request) {
    if (request.payloadLen != 7) {
        return CommandResult::fail(NackCode::BadLength);
    }

    const uint8_t segmentId = request.payload[0];
    if (segmentId == NO_SEGMENT_ID) {
        return CommandResult::fail(NackCode::BadPayload);
    }

    const int16_t targetCdeg = getI16LE(&request.payload[1]);
    const uint16_t vmaxCdegS = getU16LE(&request.payload[3]);
    const uint16_t amaxCdegS2 = getU16LE(&request.payload[5]);

    if (vmaxCdegS == 0 || amaxCdegS2 == 0) {
        return CommandResult::fail(NackCode::BadPayload);
    }

    return _hooks.prepareMoveB
        ? _hooks.prepareMoveB(_hooks.context, segmentId, targetCdeg, vmaxCdegS, amaxCdegS2)
        : CommandResult::fail(NackCode::BadCommand);
}

CommandResult Slave::dispatchPrepareHold(const Frame& request) {
    if (request.payloadLen != 1) {
        return CommandResult::fail(NackCode::BadLength);
    }
    const uint8_t segmentId = request.payload[0];
    if (segmentId == NO_SEGMENT_ID) {
        return CommandResult::fail(NackCode::BadPayload);
    }
    return _hooks.prepareHold
        ? _hooks.prepareHold(_hooks.context, segmentId)
        : CommandResult::fail(NackCode::BadCommand);
}

void Slave::sendAck(uint8_t seq, AckCode code, uint8_t detail) {
    Frame f;
    f.address = _address;
    f.type = FrameType::Response;
    f.seq = seq;
    f.command = Command::Ack;
    f.payloadLen = 2;
    f.payload[0] = static_cast<uint8_t>(code);
    f.payload[1] = detail;
    sendFrame(f);
}

void Slave::sendNack(uint8_t seq, NackCode code, uint8_t detail) {
    Frame f;
    f.address = _address;
    f.type = FrameType::Response;
    f.seq = seq;
    f.command = Command::Nack;
    f.payloadLen = 2;
    f.payload[0] = static_cast<uint8_t>(code);
    f.payload[1] = detail;
    sendFrame(f);
}

void Slave::sendStatus(uint8_t seq, const Status& status) {
    Frame f;
    f.address = _address;
    f.type = FrameType::Response;
    f.seq = seq;
    f.command = Command::StatusRsp;
    f.payloadLen = 8;

    putI16LE(&f.payload[0], status.posCdeg);
    putI16LE(&f.payload[2], status.targetCdeg);
    putI16LE(&f.payload[4], status.velCdegS);
    f.payload[6] = static_cast<uint8_t>(status.state);
    f.payload[7] = static_cast<uint8_t>(status.fault);

    sendFrame(f);
}

void Slave::sendQuickStatus(uint8_t seq, uint8_t qstatus) {
    Frame f;
    f.address = _address;
    f.type = FrameType::Response;
    f.seq = seq;
    f.command = Command::QuickStatusRsp;
    f.payloadLen = 1;
    f.payload[0] = qstatus;
    sendFrame(f);
}

void Slave::sendQueueStatus(uint8_t seq, const QueueStatus& status) {
    Frame f;
    f.address = _address;
    f.type = FrameType::Response;
    f.seq = seq;
    f.command = Command::QueueStatusRsp;
    f.payloadLen = 5;
    f.payload[0] = status.capacity;
    f.payload[1] = status.freePreparedSlots;
    f.payload[2] = status.activeSegmentId;
    f.payload[3] = status.preparedSegmentId;
    f.payload[4] = status.flags;
    sendFrame(f);
}

void Slave::sendMotionState(uint8_t seq, const MotionState& state) {
    Frame f;
    f.address = _address;
    f.type = FrameType::Response;
    f.seq = seq;
    f.command = Command::MotionStateRsp;
    f.payloadLen = MOTION_STATE_PAYLOAD_SIZE;
    encodeMotionStatePayload(state, f.payload);
    sendFrame(f);
}

void Slave::sendSegmentTiming(uint8_t seq, const SegmentTiming& timing) {
    Frame f;
    f.address = _address;
    f.type = FrameType::Response;
    f.seq = seq;
    f.command = Command::SegmentTimingRsp;
    f.payloadLen = SEGMENT_TIMING_PAYLOAD_SIZE;
    encodeSegmentTimingPayload(timing, f.payload);
    sendFrame(f);
}

void Slave::sendMotionConfig(uint8_t seq, const MotionConfig& config) {
    Frame f;
    f.address = _address;
    f.type = FrameType::Response;
    f.seq = seq;
    f.command = Command::MotionConfigRsp;
    f.payloadLen = MOTION_CONFIG_PAYLOAD_SIZE;
    encodeMotionConfigPayload(config, f.payload);
    sendFrame(f);
}

void Slave::sendFrame(const Frame& frame) {
    uint8_t buffer[MAX_FRAME_SIZE];
    const size_t len = encodeFrame(frame, buffer, sizeof(buffer));
    if (len > 0) {
        const size_t written = _serial.write(buffer, len);
        // UART_MODE_RS485_HALF_DUPLEX controls RTS/DE automatically and
        // releases the bus after the final transmitted bit.
        _serial.flush();

        if (written == len) {
            ++_txFrames;
        }
    }
}


} // namespace JointBus
