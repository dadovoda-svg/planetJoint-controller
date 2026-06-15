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
    if (request.address != _address) {
        ++_ignoredFrames;
        return;
    }

    if (request.type != FrameType::Request) {
        sendNack(request.seq, NackCode::RejectedByState);
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

    case Command::Ping:
        if (request.payloadLen != 0) {
            sendNack(request.seq, NackCode::BadLength);
        } else {
            sendAck(request.seq, AckCode::Accepted);
        }
        break;

    default:
        sendNack(request.seq, NackCode::BadCommand);
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
