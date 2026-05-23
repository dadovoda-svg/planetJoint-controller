# Planetary Joint Controller - PID/S-curve Tunable Build

This build keeps the conservative PID + S-curve controller and exposes the main controller parameters through the serial console parameter dictionary.

## New runtime parameters

All keys are at most 6 characters because the persistent parameter dictionary limits key length to 6.

### PID / feed-forward

| Key | Meaning | Default |
|---|---:|---:|
| `kp` | proportional gain | `0.8` |
| `ki` | integral gain | `0.0` |
| `kd` | derivative gain on measured velocity | `0.02` |
| `ffv` | velocity feed-forward gain | `1.0` |
| `ilim` | absolute integrator limit | `0.0` |

### Motion profile / output limits

| Key | Meaning | Default |
|---|---:|---:|
| `vmax` | S-curve reference max velocity, deg/s | `2.0` |
| `amax` | S-curve reference max acceleration, deg/s² | `6.0` |
| `sct` | S-curve acceleration ramp time, seconds | `0.150` |
| `outmax` | final command velocity clamp, deg/s | `2.5` |

### Settling, deadband and velocity estimate

| Key | Meaning | Default |
|---|---:|---:|
| `ptol` | settled position tolerance, degrees | `0.08` |
| `vtol` | settled velocity tolerance, deg/s | `0.15` |
| `dbent` | deadband enter threshold, degrees | `0.05` |
| `dbext` | deadband exit threshold, degrees | `0.12` |
| `dbvel` | deadband velocity threshold, deg/s | `0.20` |
| `vtau` | measured velocity low-pass filter tau, seconds | `0.050` |

Existing motor/driver parameters are still present:

| Key | Meaning |
|---|---|
| `stdeg` | motor microsteps per real joint degree |
| `ustep` | TMC2209 microstep setting |
| `irun` | TMC2209 run current scale |
| `ihold` | TMC2209 hold current scale |

## Examples

```text
get
servo
set kp 1.0
set kd 0.03
set vmax 3.0
set amax 8.0
pos 1
save
```

Parameter changes are applied immediately at runtime. Use `save` only after the values have been tested safely.

## Safety notes

The default values remain intentionally conservative. The real joint was observed to start losing steps above about 15 deg/s, so early tuning should stay well below that limit.

Recommended first tuning sequence:

```text
zero
trace
pos 1
pos 0
pos -1
pos 0
```

Increase only one parameter at a time.
