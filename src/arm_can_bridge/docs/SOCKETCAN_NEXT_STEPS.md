# Arm SocketCAN bridge notes

The bridge now follows the integrated Board1/2/3 protocol:

- RPi to boards: `0x001`, `0x010`, `0x020`, `0x030`
- RPi to Board1/2/3 motion: `0x101`, `0x102`, `0x103`
- Board1/2/3 status to RPi: `0x201`, `0x202`, `0x203`
- Board1/2/3 actual position feedback to RPi: `0x301`, `0x302`,
  `0x303`

Runtime responsibilities are split as follows:

- `can_protocol.py` packs and unpacks the final Classic CAN payloads.
- `socketcan_transport.py` owns Linux SocketCAN send/receive and filters
  incoming traffic to supported status/feedback frames.
- `board_state.py` tracks fresh board status and conservative queue credit.
- `trajectory_streamer.py` sends motion frames and waits for board status
  completion.
- `commanded_state.py` publishes `/joint_states` from scheduled command time,
  then accepts actual feedback updates when available.
- Board1/2 `0x301/0x302` frames update individual arm joints when their
  position-valid bit is set.
- `board3_feedback.py` assembles Board3 `0x303` groups into nine-servo
  snapshots.
- `board1_simulator` provides a no-hardware Board1/2/3 test target on `vcan0`.
  Its actual-position feedback timer runs every 20ms: Board1 emits 2 frames,
  Board2 emits 1 frame, and Board3 emits 3 frames per tick.

Trajectory completion is inferred from each required board status:

- state is `IDLE`
- error is `NONE`
- Board1/2 moving motor is `255`
- Board3 staging count is `0`
- queue or staging buffer is empty
- board is enabled
- required homing bits are set
- status is not stale

This means each board reported that its queue or staging buffer is empty and it
is idle. For Board1/2 this is not an encoder-based target-reached confirmation.
Actual arm/gripper angles may also arrive through `0x301`, `0x302`, and
`0x303`; those values are used for `/joint_states` when present.

Before testing:

1. Start `vcan0`.
2. Launch `board1_simulator`.
3. Launch `arm_can_bridge`.
4. Call `/arm_board/enable`.
5. Call `/arm_board/home_all`.
6. Send a `FollowJointTrajectory` goal.

For real hardware, replace `vcan0` with `can0` after the STM32 boards are
publishing fresh `0x201`, `0x202`, and `0x203` status frames. The boards should
also publish `0x301`, `0x302`, and `0x303` if actual positions are needed in
`/joint_states`.
