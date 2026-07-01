# VicPinky 중앙서버

이 workspace는 VicPinky 로봇의 중앙서버 ROS 2 패키지를 모아둔 곳이다. 중앙서버는 전체 미션 순서를 조율하고, 하위 기능 서버에 일을 맡기고, MoveIt2와 STM32 보드 사이를 CAN으로 연결한다.

중요한 원칙은 두 가지다.

1. 중앙서버는 로봇팔 경로를 직접 계산하지 않는다. 팔 trajectory는 MoveIt2가 만든다.
2. 중앙서버는 베이스 주행 경로를 직접 계산하지 않는다. 주행 경로와 제어는 VicPinky Nav2가 담당하고, 중앙서버는 `/nav/go_to`를 Nav2 `NavigateToPose`로 연결한다.

## 문서 구성

| 문서 | 내용 |
| --- | --- |
| [docs/SRC_STUDY_GUIDE.md](docs/SRC_STUDY_GUIDE.md) | `src/` 패키지/파일/코드 흐름을 공부하기 위한 상세 가이드 |
| [PROJECT_ONBOARDING.md](PROJECT_ONBOARDING.md) | ROS 2 기본 개념, 패키지/노드/토픽/서비스/액션 설명 |
| [ARM_CAN_PROTOCOL.md](ARM_CAN_PROTOCOL.md) | Board1/Board2/Board3 CAN 프로토콜 상세 |

## 패키지 구성

```text
src/
├── mission_manager/          # 전체 미션 순서 제어
├── vicpinky_interfaces/      # 공통 Action/Msg 인터페이스
├── mock_task_servers/        # 하위 기능 서버 mock
├── central_bringup/          # mock 통합 실행 launch
├── vicpinky_nav_adapter/     # /nav/go_to -> Nav2 NavigateToPose adapter
├── arm_can_bridge/           # MoveIt2 trajectory -> STM32 CAN bridge
├── vicpinky_gui/             # 브라우저 기반 미션/Arm 관제 GUI
├── board1_simulator/         # vcan0 기반 Board1/Board2/Board3 simulator
├── roscue_arm_description/   # URDF/mesh/RViz 표시 패키지
└── roscue_arm_moveit_config/ # MoveIt2 설정 패키지
```

## 현재 구현 상태

### Mission Manager

`mission_manager`는 `/mission/execute` Action Server다. 미션 goal을 받으면 `mission_flow.yaml`에 정의된 순서대로 하위 `RunTask` Action Server를 호출한다.

현재 설정된 주요 task:

| task | action server | 현재 상태 |
| --- | --- | --- |
| `go_to` | `/nav/go_to` | mock 또는 `vicpinky_nav_adapter` |
| `dock_to_marker` | `/dock/align` | mock 제공 |
| `pick` | `/arm/pick` | mock 제공 |
| `place` | `/arm/place` | mock 제공 |
| `press_button` | `/arm/press_button` | mock 제공 |
| `elevator_ride` | `/elevator/ride` | 주행팀 엘리베이터 FSM 또는 mock |

### VicPinky Navigation Adapter

`vicpinky_nav_adapter`는 `mission_manager`가 보내는 `/nav/go_to` `RunTask` goal을 받아 VicPinky Nav2의 `/navigate_to_pose` Action으로 넘긴다.

```text
mission_manager
  -> /nav/go_to                       RunTask
  -> vicpinky_nav_adapter
  -> /navigate_to_pose                nav2_msgs/action/NavigateToPose
  -> VicPinky Nav2 / controller
```

목표 좌표는 `mission_manager/config/locations.yaml`의 각 location에 `pose`로 넣는다.

```yaml
locations:
  room_402:
    floor: 4
    marker_id: -1
    type: navigation_goal
    pose:
      frame_id: map
      x: 0.0
      y: 0.0
      yaw: 0.0
```

`yaw`는 radian이다. 위 `x`, `y`, `yaw`는 예시값이므로 주행팀 SLAM map 기준 실제 좌표로 채워야 한다.

### Arm CAN Bridge

`arm_can_bridge`는 MoveIt2의 팔/그리퍼 `FollowJointTrajectory` goal을 받아 STM32 보드용 CAN frame으로 변환한다.

현재 Action Server는 MoveIt팀 설정과 맞춰 두 개로 분리되어 있다.

| Controller | Action | 대상 |
| --- | --- | --- |
| `arm_controller` | `/arm_controller/follow_joint_trajectory` | 팔 5축, Board1/Board2 |
| `gripper_controller` | `/gripper_controller/follow_joint_trajectory` | 그리퍼 9축, Board3 |

| Joint | 범위 | Home | Board | Motor ID |
| --- | --- | --- | --- | --- |
| `base_joint` | -90 deg ~ 180 deg | -90 deg | Board2 | 0 |
| `arm_joint_1` | -90 deg ~ 90 deg | -90 deg | Board1 | 0 |
| `arm_joint_2` | -80 deg ~ 80 deg | -80 deg | Board1 | 1 |
| `arm_joint_3` | -90 deg ~ 90 deg | -90 deg | Board1 | 2 |
| `arm_joint_4` | -170 deg ~ 170 deg | -170 deg | Board1 | 3 |
| `finger_1_base_joint` | -70.3 deg ~ 70.3 deg | 0 deg | Board3 | 0 |
| `finger_1_middle_joint` | -137.7 deg ~ 52.7 deg | 0 deg | Board3 | 1 |
| `finger_1_tip_joint` | -111.3 deg ~ 111.3 deg | 0 deg | Board3 | 2 |
| `finger_2_base_joint` | -70.3 deg ~ 70.3 deg | 0 deg | Board3 | 3 |
| `finger_2_middle_joint` | -137.7 deg ~ 52.7 deg | 0 deg | Board3 | 4 |
| `finger_2_tip_joint` | -111.3 deg ~ 111.3 deg | 0 deg | Board3 | 5 |
| `finger_3_base_joint` | -70.3 deg ~ 70.3 deg | 0 deg | Board3 | 6 |
| `finger_3_middle_joint` | -137.7 deg ~ 52.7 deg | 0 deg | Board3 | 7 |
| `finger_3_tip_joint` | -111.3 deg ~ 111.3 deg | 0 deg | Board3 | 8 |

MoveIt2 팀이 arm trajectory를 보내면 중앙서버는 Board1/Board2 CAN frame으로 나눠 전송한다. gripper trajectory는 별도 Action으로 받아 Board3 CAN frame으로 전송한다. `/joint_states`는 실제 위치 피드백이 들어오면 `0x301/0x302/0x303` actual position을 우선 사용하고, 피드백이 아직 없을 때는 commanded estimate를 사용한다.

### CAN 보드

| Board | 대상 | 명령 CAN ID | 상태 CAN ID | 위치 피드백 CAN ID |
| --- | --- | --- | --- | --- |
| Board1 | `arm_joint_1`~`arm_joint_4` step motor | `0x101` | `0x201` | `0x301` |
| Board2 | `base_joint` step motor | `0x102` | `0x202` | `0x302` |
| Board3 | three-finger gripper servo 9개 | `0x103` | `0x203` | `0x303` |

공통 control command는 CAN ID를 공유하지만 payload 구조가 명령마다 다르다.
전체 broadcast는 `0xFF`를 기본으로 쓰고, legacy 호환용 `0x00`도 전체로 해석한다.

| CAN ID | 의미 | 8-byte payload 시작 |
| --- | --- | --- |
| `0x001` | ESTOP | `[1, 0, 0, 0, 0, 0, 0, 0]` |
| `0x010` | Enable / Disable | `[enable, target_board, 0, 0, 0, 0, 0, 0]` |
| `0x020` | Homing | `[target_board, target_local_motor, mode, 0, 0, 0, 0, 0]` |
| `0x030` | Clear Error | `[target_board, target_local_motor, 0, 0, 0, 0, 0, 0]` |

상세 payload는 [ARM_CAN_PROTOCOL.md](ARM_CAN_PROTOCOL.md)를 본다.

Board1/2 실제 각도는 `0x301/0x302`로 받는다. Payload는 `[local_motor_id, flags, current_pos int32 little-endian, error_code, sequence]`이고, 각도 단위는 command target과 같은 0.01도다. Board1은 20ms마다 2프레임, Board2는 20ms마다 1프레임을 보낸다. Board3 `0x203` status는 Board1/2와 일부 byte 의미가 다르다. Byte3은 moving motor가 아니라 `staging_count`, Byte5는 32-slot queue가 아니라 9개 gripper staging buffer의 free count, Byte7은 `fault_motor_id`다. Board3 실제 각도는 별도 `0x303` 3프레임 압축 피드백으로 받으며, 각도 단위는 `int16` 0.01도이고 중앙서버가 radian으로 변환한다. Board3는 20ms마다 3프레임을 보낸다. `0x303` Byte7의 모터별 2-bit 상태는 `00 OK`, `01 MOVING`, `10 CONTACT_HOLD`, `11 ERROR`로 사용한다.

## 전체 흐름

```mermaid
flowchart LR
    Client[Mission Client] -->|ExecuteMission| MM[mission_manager]
    MM -->|RunTask| Nav[/nav/go_to]
    Nav -->|NavigateToPose| Nav2[VicPinky Nav2]
    MM -->|RunTask| Task[/arm, dock, elevator tasks]
    MM -->|MissionStatus| Status[/mission/status]

    MoveIt[MoveIt2] -->|arm FollowJointTrajectory| Bridge[arm_can_bridge]
    MoveIt -->|gripper FollowJointTrajectory| Bridge
    Bridge -->|0x101| Board1[STM32 Board1]
    Bridge -->|0x102| Board2[STM32 Board2]
    Bridge -->|0x103| Board3[STM32 Board3]
    Board1 -->|0x201| Bridge
    Board2 -->|0x202| Bridge
    Board3 -->|0x203| Bridge
    Board1 -->|0x301 actual positions| Bridge
    Board2 -->|0x302 actual positions| Bridge
    Board3 -->|0x303 actual positions| Bridge
    Bridge -->|joint positions| JointStates[/joint_states]
```

## 빌드

```bash
cd ~/vicpinky_server_ws
colcon build --symlink-install
source install/setup.bash
```

## Mock 미션 서버 실행

터미널 1:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 launch central_bringup bringup_mock.launch.py
```

터미널 2:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 run mission_manager send_demo_mission
```

사용 가능한 위치 이름 확인:

```bash
ros2 run mission_manager send_mission --list-locations
```

기본 데모는 `room_402`에서 출발해 `room_501`로 가는 4층 -> 5층 미션이다.
목적지가 4층이면 `--target-floor`를 생략해도 delivery location 기준으로 4층을 자동 추론하고, 엘리베이터 관련 step은 자동 skip된다.

```bash
# 402호에서 501호로 이동, target floor는 room_501 기준 5층으로 자동 추론
ros2 run mission_manager send_mission \
  --pickup-location room_402 \
  --delivery-location room_501 \
  --object box

# 402호에서 같은 4층의 401호로 이동, 엘리베이터 step 자동 skip
ros2 run mission_manager send_mission \
  --pickup-location room_402 \
  --delivery-location room_401 \
  --object box
```

상태 확인:

```bash
ros2 topic echo /mission/status
```

## GUI 관제 실행

미션 실행, 취소, `/mission/status`, `/arm_board/*` 서비스, `/arm_board/status_log`, `/joint_states`를 브라우저에서 확인한다.
GUI backend는 Flask를 사용한다. 실행 환경에 Flask가 없으면 먼저 설치한다.

```bash
sudo apt install python3-flask
```

```bash
cd ~/vicpinky_server_ws
source install/setup.bash
ros2 launch vicpinky_gui vicpinky_gui.launch.py
```

브라우저에서 연다.

```text
http://localhost:8080
```

포트를 바꿀 때:

```bash
ros2 launch vicpinky_gui vicpinky_gui.launch.py port:=8081
```

기본값은 `8080`이고, 이미 사용 중이면 `8081`, `8082` 순서로 빈 포트를 찾아 실행한다. 터미널에 출력되는 `VicPinky GUI ready: http://...` 주소를 열면 된다.

주행팀 엘리베이터 FSM 상태는 `RunTask.Feedback.phase` 또는 feedback detail/status message에 아래 문자열로 들어오면 GUI의 `Elevator FSM` 영역에 자동 표시된다.

```text
WAIT_BOARD -> BOARDING -> RIDING -> EXITING -> DONE
```

## 실제 VicPinky 주행 연결

mock 서버를 쓰지 않고 실제 주행을 붙일 때는 VicPinky의 Nav2 launch와 `vicpinky_nav_adapter`를 같이 띄운다.

터미널 1, VicPinky 기본 bringup:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch vicpinky_bringup bringup.launch.xml
```

터미널 2, 저장된 맵으로 Nav2 실행:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch vicpinky_navigation bringup_launch.xml map:=/path/to/map.yaml
```

터미널 3, 중앙서버 mission manager:

```bash
cd ~/vicpinky_server_ws
source install/setup.bash
ros2 launch mission_manager mission_manager.launch.py
```

터미널 4, `/nav/go_to`를 Nav2로 넘기는 adapter:

```bash
cd ~/vicpinky_server_ws
source install/setup.bash
ros2 launch vicpinky_nav_adapter nav_adapter.launch.py
```

주의: 이때 `central_bringup bringup_mock.launch.py`는 쓰지 않는다. mock도 `/nav/go_to`를 열기 때문에 실제 adapter와 이름이 충돌한다.

## Arm Bridge 시뮬레이터 테스트

`vcan0` 준비:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

이미 `vcan0`가 있으면 아래만 실행한다.

```bash
sudo ip link set up vcan0
```

터미널 1:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 launch board1_simulator board1_simulator.launch.py
```

현재 패키지 이름은 `board1_simulator`지만, 개발 편의를 위해 Board1 `0x201`, Board2 `0x202`, Board3 `0x203` status를 함께 흉내낸다.

터미널 2:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 launch arm_can_bridge arm_can_bridge.launch.py
```

터미널 3:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/enable std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/home_all std_srvs/srv/Trigger '{}'
ros2 run arm_can_bridge send_test_trajectory
```

`/joint_states` 확인:

```bash
ros2 topic echo /joint_states
```

## MoveIt2 controller 설정

MoveIt2 controller 설정은 `arm_can_bridge`의 Action 이름과 joint 순서가 같아야 한다.

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - base_joint
      - arm_joint_1
      - arm_joint_2
      - arm_joint_3
      - arm_joint_4

  gripper_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - finger_1_base_joint
      - finger_1_middle_joint
      - finger_1_tip_joint
      - finger_2_base_joint
      - finger_2_middle_joint
      - finger_2_tip_joint
      - finger_3_base_joint
      - finger_3_middle_joint
      - finger_3_tip_joint
```

최종 Action 이름:

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

MoveIt2 팀의 URDF/SRDF/controller 설정도 위 joint 이름과 순서를 맞춰야 한다. 중앙서버는 arm controller에서 Board1/Board2를, gripper controller에서 Board3를 담당한다.

## 실제 STM32 연결

실제 CAN interface는 `src/arm_can_bridge/config/arm_can_bridge.yaml`에서 바꾼다.

```yaml
can_interface: "can0"
```

can0 설정:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip link show can0
```

물리 연결:

```text
CAN_H <-> CAN_H
CAN_L <-> CAN_L
GND   <-> GND
```

CAN은 UART처럼 TX/RX를 교차 연결하지 않는다. 모든 노드가 같은 `CAN_H`, `CAN_L`, `GND` 버스에 붙는다.

종단저항은 전원 OFF 상태에서 `CAN_H`와 `CAN_L` 사이를 측정한다.

| 측정값 | 의미 |
| --- | --- |
| 약 60 ohm | 정상 |
| 약 120 ohm | 종단저항 1개만 있음 |
| OL | 종단저항 없음 또는 버스가 열려 있음 |

## 테스트

```bash
cd ~/vicpinky_server_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select arm_can_bridge board1_simulator --event-handlers console_direct+
colcon test --packages-select vicpinky_nav_adapter --event-handlers console_direct+
```

현재 확인된 결과:

```text
arm_can_bridge: 45 passed, 1 skipped
board1_simulator: 9 passed
vicpinky_nav_adapter: 2 passed, 1 skipped
```

## 자주 보는 문제

### `No such device: vcan0`

`vcan0`가 아직 만들어지지 않은 상태다.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

### `Reject trajectory: required boards are not ready`

아래 중 하나가 준비되지 않은 상태다.

- Board1, Board2, Board3 status가 들어오지 않음
- status가 stale임
- enable 안 됨
- homing 안 됨
- error 또는 ESTOP 상태
- `/joint_states` 기준 현재 위치가 아직 유효하지 않음

확인 순서:

```bash
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/enable std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/home_all std_srvs/srv/Trigger '{}'
```

### `/nav/go_to`가 안 뜸

mock 미션 테스트에서는 `central_bringup bringup_mock.launch.py`가 `/nav/go_to`를 제공한다. 실제 VicPinky 주행에서는 mock을 끄고 아래 adapter를 띄운다.

```bash
ros2 launch vicpinky_nav_adapter nav_adapter.launch.py
```

### `Navigation goal has no pose`

`locations.yaml`의 해당 location에 `pose`가 없는 상태다. 실제 맵 좌표를 아래 형식으로 추가한다.

```yaml
pose:
  frame_id: map
  x: 1.20
  y: 0.50
  yaw: 0.0
```

### `Nav2 NavigateToPose action server is not available`

VicPinky Nav2가 아직 실행되지 않았거나 Action 이름이 다르다. 먼저 확인한다.

```bash
ros2 action list | grep navigate
```
