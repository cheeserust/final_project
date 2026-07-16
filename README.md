# VicPinky 중앙서버

이 workspace는 VicPinky 로봇의 중앙서버 ROS 2 패키지를 모아둔 곳이다. 중앙서버는 전체 미션 순서를 조율하고 하위 기능 서버와 STM32 보드를 연결한다.

중요한 원칙은 두 가지다.

1. Board1/2 팔 실행은 검증된 최종 관절각 5개와 duration만 받는 V3 direct goal이다. MoveIt waypoint를 CAN으로 streaming하지 않는다.
2. 중앙서버는 베이스 주행 경로를 직접 계산하지 않는다. 주행 경로와 제어는 VicPinky Nav2가 담당하고, 중앙서버는 `/nav/go_to`를 Nav2 `NavigateToPose`로 연결한다.

## 문서 구성

| 문서 | 내용 |
| --- | --- |
| [docs/FINAL_MISSION_SCENARIO.md](docs/FINAL_MISSION_SCENARIO.md) | 4층/5층 엘리베이터 최종 미션 상태 머신과 Action 연동표 |
| [docs/SRC_STUDY_GUIDE.md](docs/SRC_STUDY_GUIDE.md) | `src/` 패키지/파일/코드 흐름을 공부하기 위한 상세 가이드 |
| [docs/CODE_FILE_GROUPS.md](docs/CODE_FILE_GROUPS.md) | 실제 운용용 코드와 테스트/개발용 파일 구분표 |
| [docs/HARDWARE_BRINGUP.md](docs/HARDWARE_BRINGUP.md) | 실제 STM32/CAN 연결 후 하드웨어 테스트 절차 |
| [docs/MOVEIT_ARM_INTEGRATION.md](docs/MOVEIT_ARM_INTEGRATION.md) | MoveIt2, ArUco, 중앙서버의 RPi/PC 통합 및 현장 보정 절차 |
| [docs/PROJECT_ONBOARDING.md](docs/PROJECT_ONBOARDING.md) | ROS 2 기본 개념, 패키지/노드/토픽/서비스/액션 설명 |
| [docs/ARM_CAN_PROTOCOL.md](docs/ARM_CAN_PROTOCOL.md) | Board1/Board2/Board3 CAN 프로토콜 상세 |

## 패키지 구성

```text
src/
├── mission_manager/          # 전체 미션 순서 제어
├── vicpinky_interfaces/      # 공통 Action/Msg 인터페이스
├── mock_task_servers/        # 하위 기능 서버 mock
├── central_bringup/          # mock 통합 실행 launch
├── vicpinky_nav_adapter/     # /nav/go_to -> Nav2 NavigateToPose adapter
├── arm_can_bridge/           # direct arm goal + Board3 trajectory -> STM32 CAN
├── vicpinky_gui/             # 브라우저 기반 미션/Arm 관제 GUI
├── board1_simulator/         # vcan0 기반 Board1/Board2/Board3 simulator
├── roscue_arm_description/   # URDF/mesh/RViz 표시 패키지
├── roscue_arm_moveit_config/ # MoveIt2 설정 패키지
└── roscue_arm_pick/          # ArUco 검출, MoveIt 계획, mission arm task 실행
```

## 현재 구현 상태

### Mission Manager

`mission_manager`는 `/mission/execute` Action Server다. 미션 goal을 받으면 `mission_flow.yaml`에 정의된 순서대로 하위 `RunTask` Action Server를 호출한다.

현재 설정된 주요 task:

| task | action server | 현재 상태 |
| --- | --- | --- |
| `go_to` | `/nav/go_to` | mock 또는 `vicpinky_nav_adapter` |
| `dock_to_marker` | `/dock/align` | mock 제공 |
| `board_elevator` | `/elevator/board` | 주행팀 엘리베이터 탑승 서버 |
| `exit_elevator` | `/elevator/exit` | 주행팀 엘리베이터 하차 서버 |
| `arm_execute` | `/arm/execute` | `roscue_arm_pick`, 운영 concrete task 실행 |
| `pick` | `/arm/pick` | mock 또는 `roscue_arm_pick` |
| `place` | `/arm/place` | mock 또는 `roscue_arm_pick` |
| `press_button` | `/arm/press_button` | mock 또는 `roscue_arm_pick` |
| `wait_door_open` | `/elevator/wait_door_open` | 주행팀 문 열림 감지 서버 |
| `check_floor` | `/floor/check` | 주행팀 floor tag 확인 서버 |
| `map_switch` | `/map/switch` | 주행팀 map 전환 서버 |
| `ready_and_approach` | `/mission/ready_and_approach` | ready와 2초 지연 27 cm 주행 join |
| `base_rotate` | `/base/rotate` | Pinky odom 기반 회전 |

최종 미션 flow는 아래 상태 머신을 따른다.

```text
ARM_HOMING
ARM_READY_AT_PICKUP
PICK_OBJECT_TO_TRAY
GO_TO_ELEVATOR_FRONT
ALIGN_ELEVATOR_TAG
PRESS_ELEVATOR_CALL_BUTTON
READY_AND_APPROACH_ELEVATOR_4F
FACE_ELEVATOR_4F
WAIT_ELEVATOR_OPEN
ENTER_ELEVATOR
PRESS_5F_BUTTON
WAIT_5F
EXIT_ELEVATOR
SWITCH_5F_MAP
GO_TO_TARGET_PLACE
ROTATE_AT_DELIVERY
DELIVER_OBJECT_FROM_TRAY
RETURN_TO_ELEVATOR
ALIGN_ELEVATOR_TAG_RETURN
PRESS_ELEVATOR_CALL_BUTTON_RETURN
READY_AND_APPROACH_ELEVATOR_5F
FACE_ELEVATOR_5F
WAIT_ELEVATOR_OPEN_RETURN
ENTER_ELEVATOR_RETURN
PRESS_4F_BUTTON
WAIT_4F
EXIT_ELEVATOR_RETURN
SWITCH_4F_MAP
RETURN_HOME
DONE
```

### VicPinky Navigation Adapter

`vicpinky_nav_adapter`는 `mission_manager`가 보내는 `/nav/go_to` `RunTask` goal을 받아 VicPinky Nav2의 `/navigate_to_pose` Action으로 넘긴다.

```text
mission_manager
  -> /nav/go_to                       RunTask
  -> vicpinky_nav_adapter
  -> /navigate_to_pose                nav2_msgs/action/NavigateToPose
  -> VicPinky Nav2 / controller
```

목표 좌표는 `mission_manager/config/locations.yaml`의 `points`에 넣는다.
`MissionFlowLoader`가 `home`, `room_402`, `elevator_front_4f` 같은
미션 location 이름을 좌표로 확장한다. `/nav/go_to`의 `target_name`은
주행팀 `nav_points.yaml` 키에 맞춰 `home`, `402`, `elevator_front`처럼
전달하고, 같은 좌표는 `extra_json.pose`에도 함께 넣는다.

```yaml
points:
  "4":
    "402":
      frame_id: map
      x: 2.998
      y: -12.433
      yaw: 0.0
```

`yaw`는 radian이다. marker/button/map 작업처럼 좌표가 없는 대상은 같은 파일의
`locations`에 `marker_id`와 `extra`만 둔다.

### Arm CAN Bridge

`arm_can_bridge`는 팔의 `ExecuteArmGoal`을 Board1 Goal V3와 Board2 Arduino
legacy goal로, 그리퍼의 `FollowJointTrajectory`를 기존 Board3 frame으로
변환한다.

현재 Action Server는 MoveIt팀 설정과 맞춰 두 개로 분리되어 있다.

| Controller | Action | 대상 |
| --- | --- | --- |
| `arm_controller` | `/arm_controller/execute_joint_goal` | 팔 최종각 5축 + duration, Board1/Board2 |
| `gripper_controller` | `/gripper_controller/follow_joint_trajectory` | 그리퍼 9축, Board3 |

| Joint | Board ID | Payload Motor ID | Min deg | Max deg | Home deg | Min raw | Max raw | Home raw |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `base_joint` | 1 | 3 | -90 | 180 | -90 | -9000 | 18000 | -9000 |
| `arm_joint_1` | 1 | 0 | -86.5 | 90 | -86.5 | -8650 | 9000 | -8650 |
| `arm_joint_2` | 1 | 1 | -78.1 | 80 | -78.1 | -7810 | 8000 | -7810 |
| `arm_joint_3` | 1 | 2 | -91.5 | 90 | -91.5 | -9150 | 9000 | -9150 |
| `arm_joint_4` | 2 | 0 | -90 | 90 | -90 | -9000 | 9000 | -9000 |

| Joint | 범위 | Home | Board | Motor ID |
| --- | --- | --- | --- | --- |
| `finger_1_base_joint` | -70.3 deg ~ 70.3 deg | 0 deg | Board3 | 0 |
| `finger_1_middle_joint` | -137.7 deg ~ 52.7 deg | 0 deg | Board3 | 1 |
| `finger_1_tip_joint` | -111.3 deg ~ 111.3 deg | 0 deg | Board3 | 2 |
| `finger_2_base_joint` | -70.3 deg ~ 70.3 deg | 0 deg | Board3 | 3 |
| `finger_2_middle_joint` | -137.7 deg ~ 52.7 deg | 0 deg | Board3 | 4 |
| `finger_2_tip_joint` | -111.3 deg ~ 111.3 deg | 0 deg | Board3 | 5 |
| `finger_3_base_joint` | -70.3 deg ~ 70.3 deg | 0 deg | Board3 | 6 |
| `finger_3_middle_joint` | -137.7 deg ~ 52.7 deg | 0 deg | Board3 | 7 |
| `finger_3_tip_joint` | -111.3 deg ~ 111.3 deg | 0 deg | Board3 | 8 |

팔 direct goal은 joint name으로 매핑해 Board1 4 frame과 Board2 1 frame을
만든다. Board1 READY 뒤 Board2 legacy frame을 보내고 Board1만 START한다.
최종각 직행은 MoveIt 충돌 회피 경로를 보존하지 않으므로 검증된 joint pose에만
사용한다. gripper trajectory는 기존 Board3 Action을 유지한다. `/joint_states`는
`0x301/0x302/0x303` actual position feedback을 우선 사용한다.

### CAN 보드

| Board | 대상 | 명령 CAN ID | 상태 CAN ID | 위치 피드백 CAN ID |
| --- | --- | --- | --- | --- |
| Board1 | `arm_joint_1~3 + base_joint` step motor | `0x101` | `0x201` | `0x301` |
| Board2 | `arm_joint_4` step motor | `0x102` | `0x202` | `0x302` |
| Board3 | three-finger gripper servo 9개 | `0x103` | `0x203` | `0x303` |

공통 control command는 CAN ID를 공유하지만 payload 구조가 명령마다 다르다.
공통 제어 frame payload에는 Board ID를 넣지 않는다. 보드는 CAN ID와 각
firmware의 명령 지원 범위로 구분한다.

| CAN ID | 의미 | 8-byte payload 시작 |
| --- | --- | --- |
| `0x001` | ESTOP | `[1, 0, 0, 0, 0, 0, 0, 0]` |
| `0x010` | Enable / Disable | `[enable, 0, 0, 0, 0, 0, 0, 0]` |
| `0x020` | Board1/2 Homing | `[target_local_motor, mode, 0, 0, 0, 0, 0, 0]` |
| `0x023` | Board3 Gripper Home | `[target_local_motor, mode, duration, 0, 0, 0, 0, 0]` |
| `0x030` | Clear Error | `[target_local_motor, 0, 0, 0, 0, 0, 0, 0]` |

상세 payload는 [ARM_CAN_PROTOCOL.md](ARM_CAN_PROTOCOL.md)를 본다.

Board1/2 실제 각도는 `0x301/0x302`의 packed `int16` 4개로 받는다.
Board2는 첫 번째 값만 사용하며 단위는 command target과 같은 0.01도다.
Board3 실제 각도는 `0x303` 3프레임 압축 피드백으로 받고 중앙서버가
radian으로 변환한다.

## 전체 흐름

```mermaid
flowchart LR
    Client[Mission Client] -->|ExecuteMission| MM[mission_manager]
    MM -->|RunTask| Nav[/nav/go_to]
    Nav -->|NavigateToPose| Nav2[VicPinky Nav2]
    MM -->|RunTask /arm/execute| ArmTask[roscue_arm_pick]
    MM -->|RunTask| Task[dock, elevator tasks]
    MM -->|MissionStatus| Status[/mission/status]

    Detector[RPi ArUco detector] -->|DetectedMarker| ArmTask
    ArmTask -->|MoveGroup plan| MoveIt[MoveIt2 move_group]
    MoveIt -->|planned trajectory| ArmTask
    ArmTask -->|arm ExecuteArmGoal| Bridge[arm_can_bridge]
    ArmTask -->|gripper FollowJointTrajectory| Bridge
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

기본 데모는 4층 `402`에서 인형을 적재하고 5층 `object_place`에 배치한 뒤
4층 `402`로 복귀하는 최종 엘리베이터 미션이다.

```bash
# 402 -> object_place -> 402
ros2 run mission_manager send_mission \
  --pickup-location 402 \
  --delivery-location object_place \
  --object object_1 \
  --arm-task-name deliver_object_1_from_tray
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

중앙서버 최종 미션 상태는 `/mission/status` 또는 `ExecuteMission.Feedback`으로 올라오며 GUI의 `Mission FSM` 영역에 자동 표시된다.

```text
ARM_HOMING -> PICK_OBJECT_TO_TRAY -> ... -> RETURN_HOME -> DONE
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
ros2 launch arm_can_bridge arm_can_bridge.launch.py execution_mode:=hardware
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

## Motion Action

최종 Action 이름은 다음과 같다.

```text
/arm_controller/execute_joint_goal
/gripper_controller/follow_joint_trajectory
```

MoveIt은 plan-only 안전 검토에 사용할 수 있지만 그 trajectory 또는 마지막
point를 Board1/2 실행 입력으로 재사용하지 않는다. 실행할 arm target은 direct
joint-goal API로 별도 제공해야 한다.

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
colcon test --packages-select mission_manager roscue_arm_pick central_bringup vicpinky_gui \
  --event-handlers console_direct+
```

현재 확인된 결과:

```text
arm_can_bridge: 82 passed, 1 skipped
board1_simulator: 17 passed
mission_manager: 7 passed, 1 skipped
roscue_arm_pick: 26 passed, 1 skipped
central_bringup: 2 passed, 1 skipped
vicpinky_gui: 1 passed
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

`locations.yaml`의 `points`에 해당 좌표가 없거나 location alias가 잘못된 상태다.
실제 맵 좌표를 층별 `points`에 추가한다.

```yaml
points:
  "4":
    "402":
      frame_id: map
      x: 2.998
      y: -12.433
      yaw: 0.0
```

### `Nav2 NavigateToPose action server is not available`

VicPinky Nav2가 아직 실행되지 않았거나 Action 이름이 다르다. 먼저 확인한다.

```bash
ros2 action list | grep navigate
```
