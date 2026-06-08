# Runtime Encoder-to-Joint Scale (`jrev`)

This build replaces the previous hardcoded encoder-to-joint scale with a persistent runtime parameter:

```text
jrev
```

`jrev` means real joint degrees for one complete 360° encoder revolution.

Default value:

```text
jrev = 15.6
```

Examples:

```text
get jrev
set jrev 15.6
save
```

When `jrev` is changed from the console, the firmware stops active motion, applies the new encoder scale, and preserves the current zeroed coordinate as much as possible. For a clean mechanical setup, run `zero` after changing `jrev`.

`jrev` and `stdeg` are different parameters:

- `jrev`: encoder-to-real-joint angle scale
- `stdeg`: motor microsteps per real joint degree

If the mechanics change, both may need to be reviewed.
