# Board1 SocketCAN bridge notes

The bridge now follows the finalized Board1 protocol:

- RPi to Board1: `0x001`, `0x010`, `0x020`, `0x030`, `0x101`
- Board1 to RPi: `0x201`
- No `0x202` commanded-position feedback frame is used.

Runtime responsibilities are split as follows:

- `can_protocol.py` packs and unpacks the final Classic CAN payloads.
- `socketcan_transport.py` owns Linux SocketCAN send/receive and filters
  incoming traffic to Board1 status frames.
- `board_state.py` tracks fresh `0x201` status and conservative queue credit.
- `trajectory_streamer.py` sends `0x101` frames and waits for Board1 status
  completion.
- `commanded_state.py` publishes open-loop `/joint_states` from the scheduled
  trajectory time, then holds the final commanded target.
- `board1_simulator` provides a no-hardware test target on `vcan0`.

Trajectory completion is inferred only from `0x201` status:

- state is `IDLE`
- error is `NONE`
- moving motor is `255`
- queue free is `32`
- board is enabled
- required homing bits are set
- status is not stale

This means Board1 reported that its queue is empty and it is idle. It is not an
encoder-based target-reached confirmation.

Before testing:

1. Start `vcan0`.
2. Launch `board1_simulator`.
3. Launch `arm_can_bridge`.
4. Call `/arm_board/enable`.
5. Call `/arm_board/home_all`.
6. Send a `FollowJointTrajectory` goal.

For real hardware, replace `vcan0` with `can0` after the STM32 is publishing
fresh `0x201` status frames.
