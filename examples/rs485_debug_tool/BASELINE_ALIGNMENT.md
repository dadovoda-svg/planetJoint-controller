# Protocol alignment requirement

`planetjoint_rs485_debug.py` is a reference diagnostic implementation of the JointBus protocol.

It must remain aligned with the C++ master/slave library for every change affecting:

- frame structure and header bit layout;
- command and response identifiers;
- payload sizes, types, byte order, and scaling;
- CRC algorithm and byte order;
- ACK/NACK result codes;
- compact and extended status fields;
- startup purge, NOP/SYNC, addressing, timeout, and retry behavior.

A protocol change is not complete until both the C++ implementation and this Python tool have been updated and tested together.

- Protocol commands include ZERO (0x08) and PARK (0x09); keep both the C++ API and Python console synchronized.

## ESP32-S3 hardware RS485 update

The C++ master and slave now use `UART_MODE_RS485_HALF_DUPLEX` with the SP3485 `DE` input connected to UART RTS (GPIO9 in the current hardware). The Python USB-RS485 debug tool is unaffected because USB adapters normally manage their own transmit direction automatically. Protocol framing and command IDs are unchanged by this physical-layer update.
