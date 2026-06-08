# Compile-time magnetic encoder selection

The firmware supports two magnetic encoders while preserving the same
high-level API and all existing joint-angle, unwrap, `jrev`, planner, and
controller logic.

## Build environments

```bash
pio run -e esp32-s3-as5048a
pio run -e esp32-s3-as5600
```

The default environment remains `esp32-s3-as5048a`.

The selection is controlled by `MAGNETIC_ENCODER_TYPE`:

- `1`: AS5048A, 14-bit SPI
- `2`: AS5600, 12-bit I2C

## Shared pins

The AS5600 reuses the pins assigned to the AS5048A data lines:

| Function | GPIO |
|---|---:|
| AS5048A MISO / AS5600 SDA | 13 |
| AS5048A MOSI / AS5600 SCL | 11 |
| AS5048A SCK | 12 |
| AS5048A CS | 10 |

For the AS5600 build, GPIO12 and GPIO10 are not used by the encoder.
The I2C bus runs at 400 kHz and the default AS5600 address is `0x36`.

## Virtual 14-bit representation

The AS5600 native value is 12 bit. The driver maps it into the same 14-bit
count domain used by the AS5048A:

```text
raw14 = raw12 << 2
```

The two added least-significant bits are always zero and therefore add no
precision. This preserves the existing 16384-count-per-revolution math and
allows the same continuous-angle and mechanical-scale code to be reused.

## Common status API

The AS5600 has no parity bit. Therefore:

- `lastParityOk()` returns `true`;
- `lastErrorFlag()` reports an I2C transport/read error;
- `lastReadOk()` reports the result of the latest complete angle read.
