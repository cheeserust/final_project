# Board1 Simulator Next Steps

1. Add this package to `vicpinky_server_ws/src`.
2. Bring up `vcan0`.
3. Run `ros2 launch board1_simulator board1_simulator.launch.py`.
4. Use `cansend vcan0 010#01` and `cansend vcan0 020#FF00`.
5. Confirm `candump vcan0` shows `0x201` status frames.
6. Connect `arm_can_bridge` services and FollowJointTrajectory server next.
