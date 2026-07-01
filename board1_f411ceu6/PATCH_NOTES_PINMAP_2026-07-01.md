# Board1 Pin Map Patch — 2026-07-01

Applied the motor-control team pin map from the uploaded table.

## Applied pins

| Motor | STEP | DIR | TMC CS |
|---:|---|---|---|
| 1 | PA1 | PA0 | PA5 |
| 2 | PC15 | PC14 | PA4 |
| 3 | PB9 | PB8 | PB10 |
| 4 | PB7 | PB6 | PB2 |

Additional pins:

| Function | Pin |
|---|---|
| TMC MOSI | PB1 |
| TMC MISO | PB0 |
| TMC CLK | PA6 |
| MOTOR ENABLE | PB3, active-low default |

The TMC CS pins are initialized high, and MOTOR_ENABLE is disabled at boot / disable / ESTOP and enabled on `0x010 Enable`.

## Limit switch note

The uploaded table did not include limit switch pins. To avoid driving a motor forever during homing without a valid limit input, `BOARD1_LIMIT_SWITCHES_ASSIGNED` is set to `0`. In this mode, `0x020` homing is rejected safely with `ERR_HOMING_FAIL`.

When final limit pins are available, set `BOARD1_LIMIT_SWITCHES_ASSIGNED` to `1` and fill `g_stepper_hw[].lim_port/lim_pin`.
