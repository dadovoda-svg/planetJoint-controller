# AS5600 Runtime CONF Register Setup

This firmware explicitly configures the AS5600 volatile `CONF` register at startup.

The AS5600 driver writes only the runtime I2C configuration registers; it does **not** burn OTP/fuse settings.
The configuration is therefore applied again at every firmware boot.

## Selected configuration

| Field | Value | Meaning |
|---|---:|---|
| `PM` | `00` | Nominal power mode |
| `WD` | `0` | Watchdog disabled |
| `FTH` | `000` | Fast filter disabled |
| `SF` | `10` | Slow filter 4x |

## Rationale

The joint controller needs deterministic encoder behavior during position control.
For this reason low-power modes and the AS5600 watchdog are disabled, while a moderate slow filter is enabled to reduce angle jitter.
The fast filter is kept disabled for this baseline to avoid changing the sensor response dynamically depending on the angle step size.

## Register handling

The driver reads both `CONF` bytes, changes only the fields listed above, and preserves the other settings:

- hysteresis (`HYST`)
- output stage (`OUTS`)
- PWM frequency (`PWMF`)

This avoids unexpected changes to the analog/PWM output configuration while still making the I2C angle filtering deterministic.
