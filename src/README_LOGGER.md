# Header-only Logger

The logger is implemented in `include/logger.h`. It is intended for
spontaneous firmware diagnostics and uses the persistent `loglvl` parameter.
The default level is `2`.

## Level matrix

| `loglvl` | `[ERR]` | `[NFO]` | `[DBG]` |
|---:|:---:|:---:|:---:|
| `0` | no | no | no |
| `1` | yes | no | no |
| `2` | yes | yes | no |
| `3` | yes | yes | yes |

Values above `3` are clamped to `3` by the logger.

The runtime setting can be changed and persisted with:

```text
set loglvl 0
set loglvl 1
set loglvl 2
set loglvl 3
save
```

The persistent level is loaded and applied before the normal boot INFO
messages. If parameter-storage initialization fails before the saved level can
be read, the storage error is emitted using the compile-time default logger
level.

## Motion lifecycle diagnostics

At INFO level, one transition message is emitted for each event:

```text
[NFO] Move complete ...
[NFO] Servo correction started ...
[NFO] Servo correction complete ...
```

The move-complete message corresponds to the first valid settled state of an
accepted command. It is emitted exactly once for that command.

A servo-hold correction is a later physical response to a disturbance. It does
not reopen the completed command or make JointBus report BUSY again. Correction
diagnostics are armed only after the completed joint has reached its deadband.

`SCurvePosVelController::isSettled()` remains a live physical/controller
condition and may change repeatedly during hold. It is not the persistent
command-completion state.

At DEBUG level, an additional line may be emitted while a hold correction is
active:

```text
[DBG] Servo correction error=... meas_vel=... cmd=...
```

This detail is rate limited to at most four messages per second.

## Output outside the logger

The following outputs intentionally bypass `loglvl`:

- trace lines beginning with `@`;
- direct console command replies;
- parameter import and export;
- servo status and calibration output written directly to `Serial`;
- the console prompt.

Therefore, text written with `Serial.print*()` is not logger-controlled, even
if that text contains a prefix such as `[ERR]`. This separation keeps command
responses and plotting protocols stable at every diagnostic level.

Logger diagnostics are sent over USB CDC only while `Serial` is available. If
`!Serial`, the diagnostic message is dropped rather than buffered.
