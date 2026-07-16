# Code File Groups

이 문서는 `src` 아래 파일들을 실제 운용용 코드와 테스트/개발용 코드로
구분하기 위한 정리표다.

주의: 아래 파일들을 실제로 다른 폴더로 옮기면 ROS 2 package 경로,
`setup.py` entry point, launch file, import path가 깨질 수 있다. 코드는 현재
위치에 두고, 이 문서를 기준으로 "실전용/테스트용"을 구분해서 보면 된다.

## 1. 실제 로봇 운용 핵심 파일

실제 Raspberry Pi, STM32, MoveIt2, Nav2, GUI를 붙여 운용할 때 필요한
코드다.

### Arm CAN Bridge

MoveIt2 trajectory를 STM32 CAN frame으로 바꾸고, STM32 status/actual
feedback을 받아 `/joint_states`로 다시 내보내는 핵심 패키지다.

| 파일 | 역할 |
|---|---|
| `src/arm_can_bridge/arm_can_bridge/arm_can_bridge_node.py` | 실제 실행 node. services, Action server, CAN 송수신, `/joint_states` 발행 |
| `src/arm_can_bridge/arm_can_bridge/can_protocol.py` | Board1/2/3 CAN ID, payload packing/unpacking, 각도 변환 |
| `src/arm_can_bridge/arm_can_bridge/socketcan_transport.py` | Linux SocketCAN `can0`/`vcan0` 저수준 송수신 |
| `src/arm_can_bridge/arm_can_bridge/trajectory_converter.py` | MoveIt2 `JointTrajectory`를 Board별 CAN frame batch로 변환 |
| `src/arm_can_bridge/arm_can_bridge/trajectory_streamer.py` | STM queue 상태를 보며 CAN frame batch를 순서대로 전송 |
| `src/arm_can_bridge/arm_can_bridge/board_state.py` | Board1/2/3 status, ready, queue, stale 상태 추적 |
| `src/arm_can_bridge/arm_can_bridge/commanded_state.py` | `/joint_states`에 내보낼 현재 joint position 캐시 |
| `src/arm_can_bridge/arm_can_bridge/board3_feedback.py` | Board3 `0x303` 3-frame gripper actual feedback 조립 |
| `src/arm_can_bridge/config/arm_can_bridge.yaml` | CAN interface, joint name, board/motor mapping, joint limit 설정 |
| `src/arm_can_bridge/launch/arm_can_bridge.launch.py` | `arm_can_bridge_node` 실행 launch |
| `src/arm_can_bridge/package.xml` | ROS dependency 선언 |
| `src/arm_can_bridge/setup.py` | Python package, console script 등록 |

실제 STM32 연결 시 `arm_can_bridge.yaml`의 `can_interface`를 `can0`로
설정한다.

```yaml
can_interface: "can0"
```

### Mission Manager

배송 미션의 전체 순서를 관리하는 실제 운용 패키지다.

| 파일 | 역할 |
|---|---|
| `src/mission_manager/mission_manager/mission_manager_node.py` | `/mission/execute` Action server. 미션 전체 상태 관리 |
| `src/mission_manager/mission_manager/task_executor.py` | 주행, 팔, 그리퍼 등 하위 task Action 호출 |
| `src/mission_manager/mission_manager/mission_state.py` | 미션 상태 enum/data 구조 |
| `src/mission_manager/mission_manager/mission_flow_loader.py` | YAML 기반 미션 흐름 로딩 |
| `src/mission_manager/config/mission_flow.yaml` | 실제 미션 task 순서 |
| `src/mission_manager/config/locations.yaml` | 4층/5층 dock, elevator, room 좌표 |
| `src/mission_manager/config/action_servers.yaml` | 하위 Action server 이름 설정 |
| `src/mission_manager/launch/mission_manager.launch.py` | mission manager 실행 launch |

### Navigation Adapter

Mission Manager의 `/nav/go_to` 요청을 실제 Nav2
`/navigate_to_pose`로 연결하는 adapter다.

| 파일 | 역할 |
|---|---|
| `src/vicpinky_nav_adapter/vicpinky_nav_adapter/nav_adapter_node.py` | `/nav/go_to` Action server, Nav2 Action client |
| `src/vicpinky_nav_adapter/config/nav_adapter.yaml` | Action name, frame, timeout 설정 |
| `src/vicpinky_nav_adapter/launch/nav_adapter.launch.py` | nav adapter 실행 launch |

### GUI

브라우저에서 미션, board service, `/joint_states`를 확인하고 조작하는 실제
관제 패키지다.

| 파일 | 역할 |
|---|---|
| `src/vicpinky_gui/vicpinky_gui/gui_node.py` | ROS node + Flask HTTP server |
| `src/vicpinky_gui/static/index.html` | GUI 화면 구조 |
| `src/vicpinky_gui/static/app.js` | GUI 동작, API 호출 |
| `src/vicpinky_gui/static/app.css` | GUI 스타일 |
| `src/vicpinky_gui/launch/vicpinky_gui.launch.py` | GUI 실행 launch |

실행 전 `python3-flask`가 설치되어 있어야 한다.

```bash
sudo apt install python3-flask
```

### Arm Description / MoveIt Config

RViz 표시, TF, MoveIt2 planning에 필요한 실제 설정 패키지다.

| 파일/폴더 | 역할 |
|---|---|
| `src/roscue_arm_description/urdf/` | 로봇 URDF/Xacro |
| `src/roscue_arm_description/meshes/` | RViz/MoveIt에서 보이는 mesh |
| `src/roscue_arm_description/launch/bridge_display.launch.py` | `arm_can_bridge`의 `/joint_states`를 RViz에 표시 |
| `src/roscue_arm_description/rviz/display.rviz` | RViz 표시 설정 |
| `src/roscue_arm_moveit_config/config/` | MoveIt2 joint limit, controller, kinematics 설정 |
| `src/roscue_arm_moveit_config/launch/move_group.launch.py` | MoveIt2 `move_group` 실행 |
| `src/roscue_arm_moveit_config/launch/moveit_rviz.launch.py` | MoveIt RViz 실행 |

## 2. 수동 테스트 / 개발 편의 파일

실제 미션 운용 중 자동으로 쓰는 파일은 아니지만, 개발 중 팔/그리퍼/CAN을
직접 검증할 때 쓰는 파일이다. 지우면 테스트와 bring-up이 불편해진다.

### Arm CAN Bridge Manual Tools

| 파일 | 역할 |
|---|---|
| `src/arm_can_bridge/arm_can_bridge/send_test_trajectory.py` | 팔과 그리퍼를 조금 움직였다가 되돌리는 통합 테스트 CLI |
| `src/arm_can_bridge/arm_can_bridge/send_arm_pose.py` | 팔 5축 목표 각도를 CLI로 직접 전송 |
| `src/arm_can_bridge/arm_can_bridge/send_gripper_pose.py` | 그리퍼 9축 목표 각도 또는 open/close CLI 전송 |
| `src/arm_can_bridge/arm_can_bridge/board3_can_smoke_test.py` | 실제 Board3 단독 CAN bring-up smoke test |

이 파일들은 실제 미션 흐름에서 자동 호출되지는 않는다. 사람이 터미널에서
직접 테스트할 때 사용한다.

### Simulator

실제 STM32 없이 `vcan0`에서 Board1/2/3 동작을 흉내내는 개발용 패키지다.

| 파일 | 역할 |
|---|---|
| `src/board1_simulator/board1_simulator/board1_simulator_node.py` | simulator ROS node |
| `src/board1_simulator/board1_simulator/model.py` | Board1/2/3 CAN protocol 동작 model |
| `src/board1_simulator/config/board1_simulator.yaml` | simulator 주기, queue 설정 |
| `src/board1_simulator/launch/board1_simulator.launch.py` | simulator 실행 launch |

실제 STM32 연결 시에는 이 패키지를 띄우지 않는다.

### Mock Mission / Mock Task Server

실제 주행, 팔, 그리퍼 서버 없이 mission manager 흐름만 확인하는 개발용
패키지다.

| 파일 | 역할 |
|---|---|
| `src/mock_task_servers/mock_task_servers/mock_task_servers_node.py` | mock 주행/팔/그리퍼 task Action server |
| `src/mock_task_servers/config/mock_tasks.yaml` | mock task 동작 시간/성공 여부 설정 |
| `src/mock_task_servers/launch/mock_servers.launch.py` | mock server 실행 launch |
| `src/central_bringup/launch/bringup_mock.launch.py` | mission manager + mock servers 통합 실행 |

실제 Nav2/MoveIt/STM32를 붙이는 운용에서는 mock 패키지를 끈다.

### Mission CLI

미션을 터미널에서 수동으로 넣을 때 쓰는 개발/운용 보조 도구다.

| 파일 | 역할 |
|---|---|
| `src/mission_manager/mission_manager/send_mission.py` | `/mission/execute` goal을 CLI로 전송 |
| `src/mission_manager/mission_manager/send_demo_mission.py` | demo mission goal 전송 |

GUI를 쓰면 이 파일들을 직접 실행하지 않아도 된다.

## 3. 자동 테스트 파일

`colcon test`에서 실행되는 테스트 파일이다. 실제 로봇 운용 중에는 실행하지
않지만, 코드가 깨졌는지 확인하는 데 필요하다.

### arm_can_bridge tests

| 파일 | 확인 내용 |
|---|---|
| `src/arm_can_bridge/test/test_can_protocol.py` | CAN payload packing/unpacking, feedback parser |
| `src/arm_can_bridge/test/test_trajectory_converter.py` | MoveIt trajectory -> CAN frame 변환 |
| `src/arm_can_bridge/test/test_board_state.py` | Board status, queue, ready/stale 판단 |
| `src/arm_can_bridge/test/test_board3_feedback.py` | Board3 `0x303` group 조립 |
| `src/arm_can_bridge/test/test_socketcan_transport.py` | SocketCAN encode/decode/filter |
| `src/arm_can_bridge/test/test_flake8.py` | Python style check |
| `src/arm_can_bridge/test/test_pep257.py` | docstring style check |
| `src/arm_can_bridge/test/test_copyright.py` | copyright check |

### board1_simulator tests

| 파일 | 확인 내용 |
|---|---|
| `src/board1_simulator/test/test_board1_simulator_model.py` | Board1/2/3 simulator protocol, feedback, interpolation |

### package lint tests

아래 패키지들의 `test/` 폴더는 주로 flake8, pep257, copyright 검사다.

```text
src/mission_manager/test/
src/mock_task_servers/test/
src/central_bringup/test/
src/vicpinky_nav_adapter/test/
```

## 4. 실제 운용 시 실행 조합

### 팔/그리퍼만 실제 STM32로 테스트

```bash
ros2 launch arm_can_bridge arm_can_bridge.launch.py execution_mode:=hardware
ros2 launch roscue_arm_description bridge_display.launch.py
```

이때 `board1_simulator`는 끈다.

### 팔/그리퍼를 simulator로 테스트

```bash
ros2 launch board1_simulator board1_simulator.launch.py
ros2 launch arm_can_bridge arm_can_bridge.launch.py execution_mode:=hardware
ros2 launch roscue_arm_description bridge_display.launch.py
```

### 전체 미션 실제 운용

```bash
ros2 launch mission_manager mission_manager.launch.py
ros2 launch vicpinky_nav_adapter nav_adapter.launch.py
ros2 launch central_bringup arm_hardware_bringup.launch.py \
  execution_mode:=hardware use_rviz:=false
ros2 launch vicpinky_gui vicpinky_gui.launch.py
```

여기에 실제 VicPinky Nav2 launch와 MoveIt2 launch가 같이 떠 있어야 한다.

## 5. 외울 때 기준

```text
arm_can_bridge_node.py      = 실제 CAN bridge 본체
can_protocol.py             = CAN 말투 사전
trajectory_converter.py     = MoveIt trajectory를 CAN frame으로 번역
trajectory_streamer.py      = CAN frame을 STM queue 상태 보며 흘려보냄
socketcan_transport.py      = can0/vcan0 실제 송수신
board_state.py              = STM status 판단
commanded_state.py          = MoveIt/RViz에 보여줄 joint position 저장
board3_feedback.py          = gripper 0x303 feedback 조립

send_*.py                   = 사람이 터미널에서 직접 때려보는 테스트/보조 CLI
board1_simulator/           = STM32 없이 vcan0에서 돌리는 simulator
test/                       = colcon test용 자동 테스트
mock_task_servers/          = 실제 하위 서버 없이 mission만 테스트하는 mock
```
