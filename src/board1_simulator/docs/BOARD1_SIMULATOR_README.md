# Board1 Simulator

This package simulates the STM32 Board1 CAN protocol on `vcan0` or `can0`.

It receives:

- `0x001` ESTOP
- `0x010` Enable / Disable
- `0x020` Homing
- `0x030` Clear Error
- `0x101` Position Command

It publishes:

- `0x201` Board Status every 100 ms and on major events

The simulator is intended for testing `arm_can_bridge` without real STM32 hardware.
