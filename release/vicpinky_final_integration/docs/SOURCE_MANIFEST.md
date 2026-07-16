# Source and verification manifest

Integrated on 2026-07-11 (Asia/Seoul).

## User-provided inputs (SHA-256)

```text
1ecbb34a6571d6946baadc28e90eb318da3753de773423be916bf10ffc4e7427  final_project-driving_pinky (1).zip
7ae9aaf7a6dff67b9e33e6f805ff975fcf6f96156f7e9ec582d073e7e5772dd7  final_project-driving_server_pj (1).zip
ece84b751a11608ec341bab3870e60f1acb24e43b3465e286f2fd55983ca7af7  robot_arm_final_ws.zip
1f25c5ec7ae1fc1ba804565d9215c20fe82cd4bfa9a68dbd3a5185c3b2f19072  파이널 중앙서버.pdf
cb5e123fd8eee8f80d56fc9ad4a56556f282070dca1ab0108508da9c81d072dd  7월11.zip
db29560a7e557200a77703b61c0709e6e1ef4a83ecd4c4fec2c442070501cf27  gripper_firmware_0710_moving_fix2.zip
```

## Protected Pinky runtime

- VicPinky URDF/xacro는 제공된 Pinky ZIP과 byte-identical이다.
- 기존 `lidar.xacro`, mesh, `laser_filter.yaml`, `bringup.launch.xml`,
  `nav2_params.yaml`, `elevator_door_server.py`는 제공본과 byte-identical이다.
- `sllidar_ros2`는 이 ZIP에 포함하지 않는다. Raspberry Pi에 이미 설정된 팀 driver,
  udev, port와 frame 값을 그대로 사용한다.
- 실행 스크립트는 ROS Domain ID, RMW 또는 네트워크 환경변수를 설정하지 않는다.

## Cross-machine interface identity

The following SHA-256 values are identical in `pj` and `pinky`.

```text
17792fe19933fd0cc85c7b5f06839ecbc5480794690ee0adc8eefa4fedf2b817  ExecuteMission.action
c69864a2c87369727d04469a7a3a8f9e96969e2dc5a0204ec6a5bc768b6f041a  RunTask.action
a3910d7c010468fa42990634c8f5ca74c4fcf90b91104f3be10b8d076736aaaf  DetectedMarker.msg
f3340a21ce7eaf63658d14ae4f1927cd039b3f7d4b4515cf070f6803f9dda466  MissionStatus.msg
```

## Verification result

- clean isolated build: `pj` 10/10 packages
- clean isolated build: `pinky` 6/6 packages
- central release tests: 174 total, 0 failures, 6 intentional copyright skips
- post-home/retiming/raw-limit packages: 123 passed, 2 copyright skips
- Pinky motion-safety functional tests: 4 passed
- Xacro expanded and `check_urdf` passed, including arm/camera/button/grasp TF
- both final launch files parsed with `ros2 launch ... --show-args`
- full 29-step mock mission result: `success=true`, `final_state=DONE`

The provided Pinky source has pre-existing style-lint findings, so it was not
mechanically reformatted. Runtime files covered by the LiDAR/URDF preservation rule
remain byte-identical to the user ZIP.

Physical hardware, elevator and camera scenes were unavailable on the integration PC;
the first real run therefore remains subject to `PREFLIGHT_CHECKLIST.md`.

The supplied arm firmware limits and 100 ms staging timeout were used as the
authority for the host-side raw command limits. Arm frames remain 7 ms apart;
normal points are at least 40 ms and the post-home Board1 escape is 300 ms.
