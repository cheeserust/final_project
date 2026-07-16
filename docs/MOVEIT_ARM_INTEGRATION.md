# MoveIt2 - Central Server Arm Integration

> Board1 V3 + Board2 legacy 운영 경고: MoveIt `JointTrajectory` waypoint 실행은 제거되었다.
> MoveIt은 plan-only 검토에만 쓰며 계획의 마지막 point도 실행 goal로 재사용하지
> 않는다. 실제 팔 실행은 검증된 최종각을
> `/arm_controller/execute_joint_goal` (`ExecuteArmGoal`)로 별도 전송한다.
> 이 직행 동작은 MoveIt 충돌 회피 경로를 보존하지 않는다. Board3 gripper의
> `FollowJointTrajectory`만 유지된다.

이 문서는 ROS 2 Jazzy 기준으로 MoveIt2 팀 워크스페이스를 중앙서버와
연결하는 운영 절차다. ZIP의 `build/install/log`는 사용하지 않았고,
`roscue_arm_pick` 소스와 필요한 description/MoveIt 설정만 중앙 workspace에
병합되어 있다.

## 1. 최종 제어 흐름

```text
mission client / GUI
  -> /mission/execute                         ExecuteMission
  -> mission_manager
  -> /arm/execute                             RunTask
  -> roscue_arm_pick task_executor_node
  -> /move_action                             MoveGroup (plan only)
  -> /planned_arm_trajectory                  debug copy
  -> /arm_controller/follow_joint_trajectory FollowJointTrajectory
  -> arm_can_bridge
  -> Board1 / Board2 CAN frames
  -> STM32
```

그리퍼는 `/gripper_controller/follow_joint_trajectory`를 통해 Board3로 간다.
`/planned_arm_trajectory`와 `/planned_gripper_trajectory`는 확인용이며 실제
실행 입력이 아니다.

## 2. 장비별 배치

| 장비 | 실행 항목 |
| --- | --- |
| RPi | front/rear USB camera, Pinky/Nav2, 주행·엘리베이터 task servers |
| 중앙 PC | wrist `realsense2_camera`, ArUco detector, `robot_state_publisher`, `move_group`, `task_executor_node`, `arm_can_bridge`, mission/GUI |
| STM32 | Board1/2 팔 구동, Board3 그리퍼 구동 |

USB-CAN과 wrist RealSense는 모두 중앙 PC에 연결한다. RPi front/rear 영상은
엘리베이터 task가 RPi에서 직접 소비한다.

## 3. 기존 통신 환경 사용

네트워크, 접속 주소, ROS Domain ID와 RMW는 팀에서 이미 구성한 환경을 그대로
사용한다. 이 통합 코드와 배포 스크립트는 관련 값을 설정하거나 변경하지 않는다.

## 4. 빌드

PC에서는 전체 workspace를 다시 빌드한다.

```bash
cd ~/vicpinky_server_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

RPi에는 final Pinky source와 중앙과 동일한 interface를 빌드한다.

```bash
cd ~/vicpinky_final/pinky
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`ExecuteMission.action`과 `DetectedMarker.msg`가 변경되었으므로 RPi, PC,
GUI 및 모든 action client를 같은 소스 revision으로 다시 빌드해야 한다.

## 5. 첫 실행은 plan-only

중앙 PC에서 wrist camera와 검출기를 실행한다.

```bash
source ~/vicpinky_final/pj/install/setup.bash
ros2 launch roscue_arm_pick aruco_perception.launch.py
```

중앙 PC에서는 기본값인 `plan_only`로 arm stack을 실행한다.

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 launch central_bringup arm_hardware_bringup.launch.py \
  can_interface:=can0 \
  execution_mode:=plan_only \
  use_rviz:=false
```

이 모드에서 `/arm/*` mission action goal은 거부된다. 수동 디버그 명령만
marker를 받아 MoveIt 경로를 계산하고 계획 trajectory를 발행하며 CAN으로
실행하지 않는다. `arm_can_bridge`도 FJT goal과 enable/home/clear-error
서비스를 거부하므로 GUI의 직접 관절 명령으로 우회할 수 없다. disable과
ESTOP은 안전 명령이라 계속 사용할 수 있다.

```bash
ros2 topic pub --once /arm_task std_msgs/msg/String \
  "{data: pick_object_2}"
```

확인 항목:

```bash
ros2 topic echo /detected_marker
ros2 topic echo /arm_task_status
ros2 topic echo /planned_arm_trajectory
ros2 topic echo /planned_gripper_trajectory
```

## 6. TF 확인

정상 체인은 다음과 같다.

```text
map -> odom -> base_link -> arm_base_link -> ... -> gripper_base_link
                                             |-> camera_link
                                             |-> button_contact_link
camera_link -> camera_color_optical_frame
```

`gripper_base_link -> camera_link`는 URDF가 발행한다. optical frame 체인은
RealSense 드라이버가 발행하므로 별도 static TF를 추가하면 안 된다.

```bash
ros2 run tf2_ros tf2_echo base_link gripper_base_link
ros2 run tf2_ros tf2_echo gripper_base_link camera_link
ros2 run tf2_ros tf2_echo camera_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

URDF에는 PDF 실측값인 arm mount `[-0.295,0.075,0.665] m`, camera mount
`[0,-0.070,0.057] m / [90,-90,0] deg`, button contact TCP가 반영돼 있다.
MoveIt에는 `world -> base_link` static virtual joint가 없으며 Nav2가 base TF를 소유한다.

## 7. 구성의 단일 기준과 보정

실제 sign, offset, home, limit의 실행 기준은
`src/arm_can_bridge/config/arm_can_bridge.yaml` 하나다.
`joint_conversion.yaml`은 사용하지 않는다.

각 축을 enable/homing한 뒤 독립적으로 `+5 deg`, `-5 deg`만 시험한다.
측정 결과는 다음 순서로 반영한다.

1. `arm_can_bridge.yaml`: board/motor, sign, offset, home, limit
2. URDF joint limits와 camera mount
3. MoveIt `joint_limits.yaml`
4. `fixed_poses.yaml`: home, ready, observation poses
5. `gripper_profiles.yaml`: object/button 각도, effort, duration
6. GUI joint 범위

구성 검사는 bridge, URDF, MoveIt, named pose, gripper profile의 joint 이름과
범위를 교차 확인한다. PDF 값을 반영한 현재 설정은 `calibration.complete=true`이고
교차 검사를 통과한다. 단, PDF에 없던 cabin-5 관찰 pose는 cabin-4 pose를 재사용하며
현장 카메라 시야 확인이 필요하다. 검사가 다시 깨지면 hardware launch는 시작 전에 멈춘다.

## 8. Hardware 실행

CAN 상태를 먼저 확인하고 작은 단독 이동을 완료한 뒤 실행 모드를 바꾼다.

```bash
ros2 launch central_bringup arm_hardware_bringup.launch.py \
  can_interface:=can0 \
  execution_mode:=hardware \
  use_rviz:=false
```

별도 터미널에서 준비한다.

```bash
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/enable std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/home_all std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
```

정상 완료에서만 task executor가 home으로 복귀한다. MoveGroup/FJT/CAN 실패나
취소가 발생하면 진행 중인 하위 action을 취소하고 추가 home trajectory를
보내지 않는다. `/arm_task` 토픽은 hardware 모드에서도 계획과 debug topic
발행만 하며, 실제 실행은 `/arm/*` `RunTask` action으로만 시작된다.

## 9. Mission 연결

일반 CLI에서는 concrete arm task가 필수다.

```bash
ros2 run mission_manager send_mission \
  --pickup-location 402 \
  --delivery-location object_place \
  --object object_1 \
  --arm-task-name deliver_object_1_from_tray
```

`send_demo_mission`도 같은 최종 task를 사용한다. 초기 ID54 pickup-to-tray와
5층 tray-to-delivery는 서로 다른 concrete task로 실행되며, 적재판에서는 보이지
않는 object marker를 다시 요구하지 않는다.

버튼 action 매핑:

| 요청 | concrete task |
| --- | --- |
| `elevator_call`, 4F | `press_elevator_up` |
| `elevator_call`, 5F | `press_elevator_down` |
| `floor_select`, 4 | `press_floor_4` |
| `floor_select`, 5 | `press_floor_5` |

기존 `arm_task_server`는 호환 테스트용 deprecated 패키지다. 운영 launch에서
실행하지 않으며 `roscue_arm_pick`과 동시에 실행하면 `/arm/*` 서버 이름이
충돌한다.

## 10. 단계별 현장 검증

1. `colcon test`로 bridge, simulator, mission, arm task 테스트를 통과시킨다.
2. `vcan0 + board1_simulator`에서 Board1/2/3 frame 분배를 확인한다.
3. `ros2 action list -t`에서 각 `/arm/*`와 FJT 서버가 하나씩인지 확인한다.
4. 중앙 PC wrist camera를 연결하고 TF 및 marker pose 흔들림을 확인한다.
5. 실제 CAN에서 enable/homing 후 모든 관절을 독립적으로 `+/-5 deg` 시험한다.
6. camera mount, `observe_object_start`, waypoint, gripper profile을 실측한다.
7. plan-only에서 named pose, 버튼, object trajectory 순으로 확인한다.
8. hardware에서 named pose, 그리퍼, 버튼 1개, object 1개 순으로 실행한다.
9. 마지막에 mission_manager 전체 미션을 실행한다.

vcan 준비 명령:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

서로 다른 터미널에서 simulator와 bridge를 실행한다.

```bash
ros2 launch board1_simulator board1_simulator.launch.py
ros2 launch arm_can_bridge arm_can_bridge.launch.py \
  can_interface:=vcan0 execution_mode:=hardware
```

그 다음 enable, homing, `send_test_trajectory` 순으로 보내고 `candump vcan0`에서
`0x101`, `0x102`, `0x103` 분배를 확인한다.

ArUco 설정은 `DICT_4X4_100`, ID `50~55`, marker size `0.05 m`다. 카메라 TF가
정상이라면 `marker_pose_correction_m`은 `[0, 0, 0]`으로 유지한다.
