# Debug Tool Baseline Alignment

This Python tool is part of the JointBus protocol baseline and must be updated whenever the embedded protocol changes.

Keep it aligned with:

- frame layout and CRC
- command IDs
- response IDs
- ACK/NACK codes
- payload layouts and scaling
- status and quick-status fields
- coordinated segment commands
- broadcast/no-response behavior

Current coordinated segment commands supported by this tool:

```text
PREPARE_MOVEB   0x0A
START_SEGMENT   0x0B
ABORT_SEGMENT   0x0C
QUEUE_STATUS    0x0D
QUEUE_STATUS_RSP 0x8D
MOTION_CONFIG    0x0F
MOTION_CONFIG_RSP 0x8F
```

`start <segment_id>` sends broadcast address `15` and expects no response unless an explicit address is provided.
