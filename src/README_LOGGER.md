# Header-only Logger

This build adds a lightweight header-only logger module:

```text
src/logger.h
```

## Log levels

Persistent parameter:

```text
loglvl
```

Values:

```text
0 = OFF
1 = ERR
2 = NFO
3 = DBG
```

Default:

```text
loglvl = 2
```

## Prefixes

Diagnostic log messages use:

```text
[ERR]
[NFO]
[DBG]
```

## Runtime usage

```text
set loglvl 0
set loglvl 1
set loglvl 2
set loglvl 3
save
```

## Scope

The logger is intended only for spontaneous firmware diagnostics.

The following are intentionally not filtered:

- trace lines beginning with `@`
- direct console command responses
- parameter import/export
- servo status output
- console prompt

This keeps the console protocol and plotting output independent from diagnostic verbosity.
