# Server Team Note: Board1 CAN Feedback Compact Format

Date: 2026-07-05

This document describes only the changed Board1 feedback frames. Command frames are unchanged.

## Summary

Board1 periodic feedback changed from:

```text
old: 0x201 status x 1 + 0x301 per-axis position x 4 = 5 frames / cycle
new: 0x201 compact status x 1 + 0x301 compact position x 1 = 2 frames / cycle
```

There is no new `0x401` frame. Axis state/flags moved into `0x201`.

This is a breaking wire-format change for Board1:

- `0x201 byte2` is no longer plain homing bits for Board1.
- `0x201 byte3` is no longer moving motor id for Board1.
- `0x301` is no longer one axis per frame.
- `0x301` no longer contains `motor_id`, `flags`, `error`, or `sequence`.

Board2/Board3 remain on their existing per-axis position format unless separately changed.

## Changed Frame: `0x201` Board1 Compact Status

Board1 status CAN ID remains `0x201`, DLC remains `8`.

```text
byte0: board state
byte1: error code
byte2: axis0 flags in low nibble, axis1 flags in high nibble
byte3: axis2 flags in low nibble, axis3 flags in high nibble
byte4: limit status bits
byte5: queue free count
byte6: enabled
byte7: status sequence counter
```

Axis flags are 4-bit values:

```text
bit0: position valid / homed
bit1: ready
bit2: moving
bit3: target reached
```

Flag decoding:

```text
axis0_flags = data[2] & 0x0F
axis1_flags = data[2] >> 4
axis2_flags = data[3] & 0x0F
axis3_flags = data[3] >> 4
```

Meaning examples:

```text
0x00 = not homed, not ready, not moving, not reached
0x01 = homed / position valid
0x03 = homed + ready
0x07 = homed + ready + moving
0x0B = homed + ready + target reached
```

Limit bits remain separate in `byte4`:

```text
byte4 bit0: axis0 limit active
byte4 bit1: axis1 limit active
byte4 bit2: axis2 limit active
byte4 bit3: axis3 limit active
```

### `0x201` Example

Example payload:

```text
CAN ID: 0x201
DLC:    8
DATA:   03 00 7B 7B 00 12 01 34
```

Decode:

```text
byte0 = 0x03 -> STATE_MOVING
byte1 = 0x00 -> ERR_NONE

byte2 = 0x7B
  axis0 flags = 0xB -> homed + ready + target reached
  axis1 flags = 0x7 -> homed + ready + moving

byte3 = 0x7B
  axis2 flags = 0xB -> homed + ready + target reached
  axis3 flags = 0x7 -> homed + ready + moving

byte4 = 0x00 -> no limit active
byte5 = 0x12 -> queue free count = 18
byte6 = 0x01 -> enabled
byte7 = 0x34 -> status sequence
```

## Changed Frame: `0x301` Board1 Compact Position

Board1 position CAN ID remains `0x301`, DLC remains `8`.

The payload now contains all four Board1 axis positions in one frame.

```text
byte0~1: axis0 current position, int16 little endian, unit 0.01 degree
byte2~3: axis1 current position, int16 little endian, unit 0.01 degree
byte4~5: axis2 current position, int16 little endian, unit 0.01 degree
byte6~7: axis3 current position, int16 little endian, unit 0.01 degree
```

Position unit is unchanged:

```text
30.00 deg  -> 3000
-15.50 deg -> -1550
```

The firmware clamps the transmitted value to signed 16-bit range:

```text
minimum: -32768 -> -327.68 deg
maximum:  32767 ->  327.67 deg
```

Board1 configured joint limits currently fit inside this range, including axis3 `-170.00..170.00 deg`.

### `0x301` Example

Example positions:

```text
axis0 =  30.00 deg ->   3000 -> 0x0BB8 -> B8 0B
axis1 = -15.50 deg ->  -1550 -> 0xF9F2 -> F2 F9
axis2 =   0.00 deg ->      0 -> 0x0000 -> 00 00
axis3 = 170.00 deg ->  17000 -> 0x4268 -> 68 42
```

Resulting payload:

```text
CAN ID: 0x301
DLC:    8
DATA:   B8 0B F2 F9 00 00 68 42
```

Decode:

```text
axis0 = int16_le(B8 0B) = 3000  ->  30.00 deg
axis1 = int16_le(F2 F9) = -1550 -> -15.50 deg
axis2 = int16_le(00 00) = 0     ->   0.00 deg
axis3 = int16_le(68 42) = 17000 -> 170.00 deg
```

Negative limit example:

```text
axis3 = -170.00 deg -> -17000 -> 0xBD98 -> 98 BD
```

## Server Parser Changes

### Remove old `0x301` assumptions

Do not parse Board1 `0x301` like this anymore:

```text
byte0: motor_id
byte1: flags
byte2~5: int32 position
byte6: error
byte7: sequence
```

The server should no longer wait for four `0x301` frames to update all Board1 joints. One `0x301` frame updates all four Board1 axes.

### Python-style parsing example

```python
def read_i16_le(data, offset):
    value = data[offset] | (data[offset + 1] << 8)
    if value & 0x8000:
        value -= 0x10000
    return value

def parse_board1_status_0x201(data):
    state = data[0]
    error = data[1]
    flags = [
        data[2] & 0x0F,
        (data[2] >> 4) & 0x0F,
        data[3] & 0x0F,
        (data[3] >> 4) & 0x0F,
    ]
    limit_bits = data[4]
    queue_free = data[5]
    enabled = data[6]
    sequence = data[7]

    axis = []
    for i, f in enumerate(flags):
        axis.append({
            "homed": bool(f & 0x01),
            "ready": bool(f & 0x02),
            "moving": bool(f & 0x04),
            "target_reached": bool(f & 0x08),
            "limit_active": bool(limit_bits & (1 << i)),
        })

    return {
        "state": state,
        "error": error,
        "axis": axis,
        "queue_free": queue_free,
        "enabled": bool(enabled),
        "sequence": sequence,
    }

def parse_board1_position_0x301(data):
    raw_positions = [
        read_i16_le(data, 0),
        read_i16_le(data, 2),
        read_i16_le(data, 4),
        read_i16_le(data, 6),
    ]
    positions_deg = [p / 100.0 for p in raw_positions]
    return raw_positions, positions_deg
```

### C/C++-style parsing example

```c
static int16_t read_i16_le(const uint8_t *p)
{
    uint16_t v = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    return (int16_t)v;
}

void parse_board1_status_0x201(const uint8_t data[8])
{
    uint8_t state = data[0];
    uint8_t error = data[1];
    uint8_t axis_flags[4];
    uint8_t limit_bits = data[4];
    uint8_t queue_free = data[5];
    uint8_t enabled = data[6];
    uint8_t sequence = data[7];

    axis_flags[0] = data[2] & 0x0F;
    axis_flags[1] = (data[2] >> 4) & 0x0F;
    axis_flags[2] = data[3] & 0x0F;
    axis_flags[3] = (data[3] >> 4) & 0x0F;

    for (int axis = 0; axis < 4; axis++) {
        uint8_t flags = axis_flags[axis];
        bool homed = (flags & 0x01) != 0;
        bool ready = (flags & 0x02) != 0;
        bool moving = (flags & 0x04) != 0;
        bool target_reached = (flags & 0x08) != 0;
        bool limit_active = (limit_bits & (1u << axis)) != 0;
        (void)homed;
        (void)ready;
        (void)moving;
        (void)target_reached;
        (void)limit_active;
    }

    (void)state;
    (void)error;
    (void)queue_free;
    (void)enabled;
    (void)sequence;
}

void parse_board1_position_0x301(const uint8_t data[8], int16_t pos_001deg[4])
{
    pos_001deg[0] = read_i16_le(&data[0]);
    pos_001deg[1] = read_i16_le(&data[2]);
    pos_001deg[2] = read_i16_le(&data[4]);
    pos_001deg[3] = read_i16_le(&data[6]);
}
```

## Integration Notes

- Do not rely on receiving four Board1 `0x301` frames per cycle anymore.
- Do not rely on `0x301` frame order for axis mapping. Axis mapping is fixed by byte position.
- Do not rely on `0x301` sequence counter anymore. Board1 status sequence is now `0x201 byte7`.
- Do not depend on status/position frame order. Use latest received `0x201` for status and latest received `0x301` for positions.
- If the server supports Board2/Board3, keep their existing parsers unless those boards are explicitly migrated later.
- Existing Board1 command frames are unchanged.

## Quick Acceptance Checks

When firmware is updated, the server-side CAN log should show approximately:

```text
0x201 one frame per 100ms cycle
0x301 one frame per 100ms cycle
```

It should not show four Board1 `0x301` frames per cycle anymore.

Use these payloads as parser smoke tests:

```text
0x201 DATA 03 00 7B 7B 00 12 01 34
0x301 DATA B8 0B F2 F9 00 00 68 42
```

Expected decoded values:

```text
0x201:
  state = 3
  error = 0
  axis0 flags = 0xB
  axis1 flags = 0x7
  axis2 flags = 0xB
  axis3 flags = 0x7
  limit_bits = 0
  queue_free = 18
  enabled = 1
  sequence = 52

0x301:
  axis0 = 3000  ->  30.00 deg
  axis1 = -1550 -> -15.50 deg
  axis2 = 0     ->   0.00 deg
  axis3 = 17000 -> 170.00 deg
```
