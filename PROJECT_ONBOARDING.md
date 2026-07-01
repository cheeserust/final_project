# VicPinky 중앙서버 온보딩

이 문서는 ROS 2를 처음 보는 팀원도 `vicpinky_server_ws`가 무슨 일을 하는지 이해할 수 있도록 정리한 가이드다.

## 1. 먼저 알아야 할 것

현재 repo의 역할은 세 가지다.

1. 미션 순서를 관리한다.
2. 미션의 주행 step을 VicPinky Nav2 `NavigateToPose`로 연결한다.
3. MoveIt2가 만든 팔 trajectory를 STM32 CAN 명령으로 바꾼다.
4. 실제 STM32 없이 `vcan0`에서 Board1/Board2/Board3 동작을 테스트할 수 있게 한다.

현재 repo가 하지 않는 것도 중요하다.

- 실제 base motor 제어와 경로 추종은 VicPinky Nav2/base driver가 한다.
- `/nav/go_to`는 mock 서버 또는 `vicpinky_nav_adapter` 중 하나만 띄운다.
- 현재 URDF/MoveIt 설정은 5축 arm + 9축 gripper 모델 기준이다.
- Arm과 gripper는 MoveIt controller가 분리되어 있고, 중앙서버도 Action Server를 두 개 연다.

## 2. ROS 2 기본 개념

| 용어 | 쉬운 설명 | 이 프로젝트 예시 |
| --- | --- | --- |
| Workspace | 여러 ROS 패키지를 모아 빌드하는 폴더 | `~/vicpinky_server_ws` |
| Package | 기능별 코드 묶음 | `mission_manager`, `arm_can_bridge` |
| Node | 실행되는 프로그램 하나 | `mission_manager`, `arm_can_bridge` |
| Topic | 계속 흘러가는 방송형 데이터 | `/joint_states`, `/mission/status` |
| Service | 요청 1번, 응답 1번 | `/arm_board/enable` |
| Action | 오래 걸리는 작업 요청, feedback/result 포함 | `/mission/execute` |
| Launch | 여러 노드와 설정을 한 번에 실행 | `arm_can_bridge.launch.py` |
| Parameter | 노드 설정값 | `can_interface: vcan0` |
| URDF/Xacro | 로봇 링크와 조인트 구조 설명 | `roscue_arm_description/urdf` |
| RViz | 로봇 모델과 움직임을 보는 GUI | `rviz2` |
| SocketCAN | Linux CAN 통신 방식 | `vcan0`, `can0` |
| vcan0 | 하드웨어 없는 가상 CAN | simulator 테스트 |
| can0 | 실제 CAN 어댑터 | STM32 연결 |

## 3. 큰 그림

### 미션 흐름

```text
사용자 / UI / 테스트 클라이언트
  -> /mission/execute
  -> mission_manager
  -> /nav/go_to, /arm/pick, /arm/place 등 하위 task 호출
  -> /mission/status
```

`mission_manager`는 “무엇을 어떤 순서로 할지”만 결정한다. 예를 들어 먼저 이동하고, 도킹하고, 버튼을 누르고, 문 열림을 기다리는 식이다.

### 주행 흐름

```text
mission_manager
  -> /nav/go_to RunTask
  -> vicpinky_nav_adapter
  -> /navigate_to_pose Nav2 Action
  -> VicPinky Nav2
  -> base driver /cmd_vel
```

중앙서버는 목적지 좌표를 Nav2에 넘긴다. 실제 global/local planning, obstacle avoidance, motor 제어는 VicPinky navigation 쪽 책임이다.

### 팔 제어 흐름

```text
MoveIt2 또는 send_test_trajectory
  -> /arm_controller/follow_joint_trajectory
  -> /gripper_controller/follow_joint_trajectory
  -> arm_can_bridge
  -> CAN 0x101 Board1 command
  -> CAN 0x102 Board2 command
  -> CAN 0x103 Board3 command
  -> STM32 또는 simulator
  -> CAN 0x201 / 0x202 / 0x203 status
  -> arm_can_bridge
  -> /joint_states
```

MoveIt2가 경로를 만들고, `arm_can_bridge`는 그 경로를 CAN frame으로 바꾼다. 현재 `/joint_states`는 encoder feedback이 아니라 중앙서버가 시간 기준으로 추정한 commanded position이다.

## 4. 패키지별 역할

| 패키지 | 역할 |
| --- | --- |
| `vicpinky_interfaces` | 중앙서버가 쓰는 custom Action/Message 정의 |
| `mission_manager` | 전체 미션을 YAML 순서대로 실행하는 노드 |
| `mock_task_servers` | navigation/arm/elevator 서버가 없을 때 쓰는 mock Action 서버 |
| `central_bringup` | mock 서버와 mission manager를 같이 실행하는 launch 패키지 |
| `vicpinky_nav_adapter` | `/nav/go_to`를 Nav2 `/navigate_to_pose`로 넘기는 실제 주행 adapter |
| `arm_can_bridge` | MoveIt2 trajectory를 Board1/Board2/Board3 CAN 프로토콜로 변환하는 bridge |
| `board1_simulator` | `vcan0`에서 Board1/Board2/Board3 status와 queue 동작을 흉내내는 simulator |
| `roscue_arm_description` | URDF, mesh, RViz 설정 패키지 |
| `roscue_arm_moveit_config` | MoveIt2 planning/controller 설정 패키지 |
| `dummy_servers` | 현재 실사용 로직은 없는 placeholder 패키지 |

## 5. 노드와 인터페이스

### 실행 노드

| 노드 | 패키지 | 실행 파일 | 역할 |
| --- | --- | --- | --- |
| `mission_manager` | `mission_manager` | `mission_manager_node` | 전체 미션 Action Server |
| `mock_task_servers` | `mock_task_servers` | `mock_task_servers_node` | 여러 fake `RunTask` Action Server |
| `vicpinky_nav_adapter` | `vicpinky_nav_adapter` | `nav_adapter_node` | `/nav/go_to`를 Nav2 goal로 변환 |
| `arm_can_bridge` | `arm_can_bridge` | `arm_can_bridge_node` | arm CAN bridge, board service, `/joint_states` |
| `board1_simulator` | `board1_simulator` | `board1_simulator_node` | Board1/Board2/Board3 CAN simulator |
| `send_test_trajectory` | `arm_can_bridge` | `send_test_trajectory` | 팔 5축 + gripper 9축 테스트 trajectory 전송 |
| `send_mission` | `mission_manager` | `send_mission` | 미션 goal 전송 CLI |
| `send_demo_mission` | `mission_manager` | `send_demo_mission` | demo mission 전송 CLI |

### Topic

| Topic | 타입 | 발행자 | 의미 |
| --- | --- | --- | --- |
| `/mission/status` | `vicpinky_interfaces/msg/MissionStatus` | `mission_manager` | 현재 미션 상태 |
| `/arm_board/status_log` | `std_msgs/msg/String` | `arm_can_bridge` | Board1/Board2/Board3 상태 로그 |
| `/joint_states` | `sensor_msgs/msg/JointState` | `arm_can_bridge` | open-loop commanded joint state |
| `/tf` | `tf2_msgs/msg/TFMessage` | `robot_state_publisher` | RViz에서 보는 링크 좌표 |

### Service

| Service | 타입 | 서버 | 의미 |
| --- | --- | --- | --- |
| `/arm_board/enable` | `std_srvs/srv/Trigger` | `arm_can_bridge` | Board1/2/3 enable broadcast |
| `/arm_board/disable` | `std_srvs/srv/Trigger` | `arm_can_bridge` | Board1/2/3 disable broadcast |
| `/arm_board/home_all` | `std_srvs/srv/Trigger` | `arm_can_bridge` | Board1/Board2 homing |
| `/arm_board/clear_error` | `std_srvs/srv/Trigger` | `arm_can_bridge` | error clear broadcast |
| `/arm_board/estop` | `std_srvs/srv/Trigger` | `arm_can_bridge` | emergency stop broadcast |
| `/arm_board/status` | `std_srvs/srv/Trigger` | `arm_can_bridge` | 최신 board status 확인 |

### Action

| Action | 타입 | 서버 | 클라이언트 |
| --- | --- | --- | --- |
| `/mission/execute` | `ExecuteMission` | `mission_manager` | UI, `send_mission`, `send_demo_mission` |
| `/nav/go_to` | `RunTask` | mock 또는 `vicpinky_nav_adapter` | `mission_manager` |
| `/navigate_to_pose` | `NavigateToPose` | VicPinky Nav2 | `vicpinky_nav_adapter` |
| `/dock/align` | `RunTask` | mock 또는 실제 docking 서버 | `mission_manager` |
| `/arm/pick` | `RunTask` | mock 또는 실제 arm task 서버 | `mission_manager` |
| `/arm/place` | `RunTask` | mock 또는 실제 arm task 서버 | `mission_manager` |
| `/arm/press_button` | `RunTask` | mock 또는 실제 arm task 서버 | `mission_manager` |
| `/elevator/ride` | `RunTask` | mock 또는 주행팀 elevator FSM 서버 | `mission_manager` |
| `/arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` | `arm_can_bridge` | MoveIt2, `send_test_trajectory` |
| `/gripper_controller/follow_joint_trajectory` | `FollowJointTrajectory` | `arm_can_bridge` | MoveIt2, `send_test_trajectory` |

## 6. Arm + gripper joint와 보드 mapping

현재 `arm_can_bridge/config/arm_can_bridge.yaml` 기준 controller는 두 개다.

| Controller | Action | Joint |
| --- | --- | --- |
| `arm_controller` | `/arm_controller/follow_joint_trajectory` | `base_joint`, `arm_joint_1`~`arm_joint_4` |
| `gripper_controller` | `/gripper_controller/follow_joint_trajectory` | `finger_*` 9개 |

| Joint | Min | Max | Home | Board | Motor ID |
| --- | --- | --- | --- | --- | --- |
| `base_joint` | -90 deg | 180 deg | -90 deg | Board2 | 0 |
| `arm_joint_1` | -90 deg | 90 deg | -90 deg | Board1 | 0 |
| `arm_joint_2` | -80 deg | 80 deg | -80 deg | Board1 | 1 |
| `arm_joint_3` | -90 deg | 90 deg | -90 deg | Board1 | 2 |
| `arm_joint_4` | -170 deg | 170 deg | -170 deg | Board1 | 3 |
| `finger_1_base_joint` | -70.3 deg | 70.3 deg | 0 deg | Board3 | 0 |
| `finger_1_middle_joint` | -137.7 deg | 52.7 deg | 0 deg | Board3 | 1 |
| `finger_1_tip_joint` | -111.3 deg | 111.3 deg | 0 deg | Board3 | 2 |
| `finger_2_base_joint` | -70.3 deg | 70.3 deg | 0 deg | Board3 | 3 |
| `finger_2_middle_joint` | -137.7 deg | 52.7 deg | 0 deg | Board3 | 4 |
| `finger_2_tip_joint` | -111.3 deg | 111.3 deg | 0 deg | Board3 | 5 |
| `finger_3_base_joint` | -70.3 deg | 70.3 deg | 0 deg | Board3 | 6 |
| `finger_3_middle_joint` | -137.7 deg | 52.7 deg | 0 deg | Board3 | 7 |
| `finger_3_tip_joint` | -111.3 deg | 111.3 deg | 0 deg | Board3 | 8 |

## 7. CAN 통신 요약

| Board | 대상 | Command | Status |
| --- | --- | --- | --- |
| Board1 | `arm_joint_1`~`arm_joint_4` | `0x101` | `0x201` |
| Board2 | `base_joint` | `0x102` | `0x202` |
| Board3 | 그리퍼 9서보 | `0x103` | `0x203` |

Position command payload:

| Byte | 의미 |
| --- | --- |
| 0 | flags + local motor id |
| 1~4 | target position, int32 little-endian, 0.01 deg |
| 5~6 | speed, uint16 little-endian |
| 7 | duration, 5ms tick |

Control command는 공통 CAN ID를 쓰지만 payload 구조가 명령마다 다르다.
전체 broadcast는 `0xFF`를 기본으로 쓰고, 기존 호환용 `0x00`도 전체로 허용한다.

```text
0x001 ESTOP       [1, 0, 0, 0, 0, 0, 0, 0]
0x010 Enable      [enable, target_board, 0, 0, 0, 0, 0, 0]
0x020 Homing      [target_board, target_local_motor, mode, 0, 0, 0, 0, 0]
0x030 Clear Error [target_board, target_local_motor, 0, 0, 0, 0, 0, 0]
```

자세한 내용은 [ARM_CAN_PROTOCOL.md](ARM_CAN_PROTOCOL.md)를 본다.

Board3 `0x203`은 Board1/2와 다르게 Byte3을 `staging_count`, Byte5를 `buffer_free`, Byte7을 `fault_motor_id`로 쓴다. 중앙서버도 이 차이를 반영해서 완료 조건을 판단한다.

## 8. 실행 방법

### 빌드

```bash
cd ~/vicpinky_server_ws
colcon build --symlink-install
source install/setup.bash
```

### Mock 미션 테스트

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

### Arm bridge simulator 테스트

`vcan0` 준비:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

터미널 1:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 launch board1_simulator board1_simulator.launch.py
```

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

## 9. 실제 주행을 붙이는 방법

실제 빅핑키 주행을 붙일 때는 mock 서버 대신 `vicpinky_nav_adapter`를 띄운다.

구조:

```text
mission_manager
  -> /nav/go_to RunTask goal
  -> vicpinky_nav_adapter
  -> /navigate_to_pose Nav2 Action
  -> VicPinky Nav2
```

목표 좌표는 `mission_manager/config/locations.yaml`에 넣는다.

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

실행 순서:

```bash
ros2 launch vicpinky_bringup bringup.launch.xml
ros2 launch vicpinky_navigation bringup_launch.xml map:=/path/to/map.yaml
ros2 launch mission_manager mission_manager.launch.py
ros2 launch vicpinky_nav_adapter nav_adapter.launch.py
```

mock 서버와 adapter를 동시에 띄우면 둘 다 `/nav/go_to`를 열려고 해서 충돌한다.

## 10. 파일별 역할

### mission_manager

| 파일 | 역할 |
| --- | --- |
| `mission_manager_node.py` | `/mission/execute` Action Server 메인 노드 |
| `mission_flow_loader.py` | YAML 설정을 읽어 미션 step으로 변환 |
| `task_executor.py` | 하위 `RunTask` Action 호출과 timeout/retry 처리 |
| `mission_state.py` | 미션 상태/progress 관리 |
| `send_mission.py` | CLI에서 미션 goal 전송 |
| `send_demo_mission.py` | demo mission goal 전송 |
| `config/mission_flow.yaml` | 미션 step 순서 |
| `config/action_servers.yaml` | task 이름과 action server 이름 mapping |
| `config/locations.yaml` | pickup/delivery/elevator 위치 설정 |
| `launch/mission_manager.launch.py` | mission manager 실행 launch |

### vicpinky_interfaces

| 파일 | 역할 |
| --- | --- |
| `action/ExecuteMission.action` | 전체 미션 Action 정의 |
| `action/RunTask.action` | 하위 task Action 정의 |
| `msg/MissionStatus.msg` | 미션 상태 topic 메시지 |
| `CMakeLists.txt` | ROS interface build 설정 |
| `package.xml` | interface package 의존성 |

### mock_task_servers

| 파일 | 역할 |
| --- | --- |
| `mock_task_servers_node.py` | `/nav/go_to` 등 fake `RunTask` Action Server |
| `config/mock_tasks.yaml` | mock task별 duration/success 설정 |
| `launch/mock_servers.launch.py` | mock 서버 실행 launch |

### central_bringup

| 파일 | 역할 |
| --- | --- |
| `launch/bringup_mock.launch.py` | mission manager와 mock task server를 함께 실행 |

### vicpinky_nav_adapter

| 파일 | 역할 |
| --- | --- |
| `nav_adapter_node.py` | `/nav/go_to` `RunTask` goal을 `/navigate_to_pose` goal로 변환 |
| `config/nav_adapter.yaml` | action 이름, 기본 frame, timeout 설정 |
| `launch/nav_adapter.launch.py` | 실제 주행 adapter 실행 launch |

### arm_can_bridge

| 파일 | 역할 |
| --- | --- |
| `arm_can_bridge_node.py` | services, status log, `/joint_states`, arm/gripper `FollowJointTrajectory` Action Server |
| `can_protocol.py` | Board1/2/3 CAN ID와 payload pack/unpack |
| `socketcan_transport.py` | Linux SocketCAN 송수신 |
| `board_state.py` | Board1/2/3 status, stale, queue credit 추적 |
| `trajectory_converter.py` | MoveIt trajectory를 CAN frame batch로 변환 |
| `trajectory_streamer.py` | queue 상태를 보며 CAN frame 송신 |
| `commanded_state.py` | open-loop `/joint_states` 추정 |
| `send_test_trajectory.py` | arm controller와 gripper controller에 테스트 trajectory 전송 |
| `config/arm_can_bridge.yaml` | CAN interface, joint limit, board/motor mapping |
| `launch/arm_can_bridge.launch.py` | arm bridge 실행 launch |

### board1_simulator

| 파일 | 역할 |
| --- | --- |
| `board1_simulator_node.py` | `vcan0`에서 Board1/Board2/Board3 CAN frame 처리 |
| `model.py` | Board1 4축, Board2 1축, Board3 9축 queue/status 모델 |
| `config/board1_simulator.yaml` | simulator CAN interface와 주기 설정 |
| `launch/board1_simulator.launch.py` | simulator 실행 launch |

### roscue_arm_description

| 파일 | 역할 |
| --- | --- |
| `urdf/` | 로봇팔 URDF/Xacro |
| `meshes/` | RViz 표시용 mesh |
| `rviz/display.rviz` | RViz 표시 설정 |
| `launch/display.launch.py` | robot_state_publisher와 RViz 실행 |

현재 패키지는 MoveIt팀에게 받은 5축 arm + 9축 gripper URDF/mesh 기준이다.

### roscue_arm_moveit_config

| 파일 | 역할 |
| --- | --- |
| `config/moveit_controllers.yaml` | MoveIt controller manager 설정 |
| `config/ros2_controllers.yaml` | ros2_control controller 설정 |
| `config/joint_limits.yaml` | MoveIt joint limit 설정 |
| `config/kinematics.yaml` | kinematics solver 설정 |
| `config/roscue_arm.srdf` | MoveIt planning group 설정 |
| `launch/move_group.launch.py` | MoveIt move_group 실행 |
| `launch/moveit_rviz.launch.py` | MoveIt RViz 실행 |
| `launch/rsp.launch.py` | robot_state_publisher 실행 |

URDF가 다시 바뀌면 MoveIt Setup Assistant로 이 패키지도 다시 맞추는 것이 좋다.

## 11. 개발할 때 기억할 것

- `/joint_states`는 실제 encoder가 아니라 commanded estimate다.
- Board1, Board2, Board3 status가 모두 fresh여야 trajectory를 받는다.
- Homing 후 current position은 0이 아니라 설정된 home pose로 잡는다.
- Board1은 4개 motor frame을 staging해서 하나의 4축 point로 처리한다.
- Board2는 1개 frame이 팔 5번째 축 point다.
- Board3는 9개 servo frame을 staging해서 하나의 gripper point로 처리한다.
- Board1/Board2 완전한 하드웨어 tick 동기화는 요구하지 않는다.
- Board3 status는 개별 servo bit가 아니라 전체 ready/fault만 쓴다.
