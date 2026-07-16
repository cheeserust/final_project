# src 코드 공부 가이드

> Board1/2 팔 경로는 V3 direct joint-goal로 변경되었다. 아래 legacy
> `FollowJointTrajectory`, Queue Free, waypoint converter/streamer 설명 중 arm에
> 관한 내용은 더 이상 실행 경로가 아니며 Board3 gripper에만 해당한다. 현재 팔
> 구현은 `arm_goal_v3.py`, `can_writer.py`, `ExecuteArmGoal.action`을 기준으로 본다.

이 문서는 `src/` 아래 패키지와 파일을 팀원에게 설명할 수 있을 정도로 공부하기 위한 노트다.

읽는 순서는 아래처럼 잡으면 좋다.

1. `1. 전체 그림`에서 이 repo가 어디까지 담당하는지 잡는다.
2. `2. ROS 2 기본 개념`으로 용어를 먼저 맞춘다.
3. `3. 패키지 지도`에서 폴더별 역할을 외운다.
4. `4. 핵심 실행 흐름`에서 실제 데이터가 어떻게 지나가는지 따라간다.
5. `5. 파일별 코드 해설`에서 팀원이 파일명을 물어봐도 대답할 수 있게 만든다.
6. `6. 실제 사용법`과 `7. 팀원 질문 대비`를 반복해서 본다.

## 1. 전체 그림

이 workspace는 VicPinky 중앙서버다. 중앙서버라는 말은 "모든 것을 직접 제어한다"는 뜻이 아니라, 여러 팀이 만든 기능을 ROS 2 인터페이스로 묶어 미션 순서대로 호출한다는 뜻에 가깝다.

현재 `src/`의 큰 역할은 네 가지다.

| 역할 | 담당 패키지 |
| --- | --- |
| 미션 순서 관리 | `mission_manager` |
| 미션/하위 task 공통 인터페이스 | `vicpinky_interfaces` |
| 실제 주행 연결 또는 mock task | `vicpinky_nav_adapter`, `mock_task_servers`, `central_bringup` |
| 로봇팔 MoveIt trajectory를 CAN으로 변환 | `arm_can_bridge`, `board1_simulator`, `roscue_arm_description`, `roscue_arm_moveit_config` |
| 브라우저 관제 | `vicpinky_gui` |

중요한 책임 분리는 아래와 같다.

| 질문 | 답 |
| --- | --- |
| 중앙서버가 주행 경로를 계산하나? | 아니다. Nav2가 계산한다. 중앙서버는 `/nav/go_to` task를 호출한다. |
| 중앙서버가 팔 경로를 계산하나? | 아니다. MoveIt2가 `FollowJointTrajectory`를 만든다. 중앙서버의 `arm_can_bridge`는 CAN frame으로 변환한다. |
| 엘리베이터 탑승/하차 FSM은 어디서 도나? | 지금 최종 flow에서는 중앙서버가 엘리베이터 절차를 `ENTER_ELEVATOR`, `WAIT_5F`, `SWITCH_5F_MAP`처럼 명시적인 mission state로 쪼개서 관리하고, 각 세부 동작은 주행/팔 task 서버가 담당한다. |
| GUI는 ROS 노드인가? | 맞다. `vicpinky_gui_node`가 ROS action/service/topic을 붙잡고, 동시에 HTTP 서버로 웹 화면을 제공한다. |

전체 연결은 이렇게 보면 된다.

```text
GUI 또는 send_mission
  -> /mission/execute                    ExecuteMission action
  -> mission_manager
  -> /nav/go_to, /dock/align, /arm/pick,
     /arm/place, /arm/press_button,
     /elevator/wait_door_open,
     /floor/check, /map/switch           RunTask action
  -> /mission/status                     MissionStatus topic
```

팔 쪽은 별도 흐름이다.

```text
MoveIt2 또는 send_test_trajectory
  -> /arm_controller/follow_joint_trajectory
  -> /gripper_controller/follow_joint_trajectory
  -> arm_can_bridge
  -> SocketCAN can0/vcan0
  -> STM32 Board1/Board2/Board3 또는 board1_simulator
  -> /arm_board/status_log, /joint_states
```

## 2. ROS 2 기본 개념

| 개념 | 뜻 | 이 프로젝트 예시 |
| --- | --- | --- |
| Workspace | 여러 ROS 패키지를 모아서 빌드하는 최상위 폴더 | `~/vicpinky_server_ws` |
| Package | 기능 단위 코드 묶음 | `mission_manager`, `arm_can_bridge` |
| Node | 실행되는 프로그램 하나 | `mission_manager`, `arm_can_bridge`, `vicpinky_gui` |
| Topic | 계속 흘러가는 방송형 데이터 | `/mission/status`, `/joint_states` |
| Service | 요청 1번, 응답 1번 | `/arm_board/enable` |
| Action | 오래 걸리는 작업. goal, feedback, result가 있음 | `/mission/execute`, `/nav/go_to` |
| Launch | 여러 노드와 설정을 한 번에 실행 | `mission_manager.launch.py` |
| Parameter | 노드 실행 설정값 | `can_interface`, `mission_flow_file` |
| Msg | Topic에 쓰는 데이터 타입 | `MissionStatus.msg` |
| Action definition | Action goal/feedback/result 타입 정의 | `ExecuteMission.action`, `RunTask.action` |
| URDF/Xacro | 로봇 링크, 조인트, mesh, control 구조 설명 | `roscue_arm_description/urdf` |
| SRDF | MoveIt이 쓰는 planning group, end-effector 등 semantic 정보 | `roscue_arm_moveit_config/config/roscue_arm.srdf` |
| SocketCAN | Linux에서 CAN을 socket처럼 쓰는 방식 | `vcan0`, `can0` |
| vcan0 | 하드웨어 없이 테스트하는 가상 CAN interface | simulator 테스트 |
| can0 | 실제 USB-CAN, MCP2515 같은 CAN interface | STM32 연결 |

Action을 특히 잘 알아야 한다. 미션은 한 번 요청하면 몇 초 이상 걸리므로 service보다 action이 맞다.

```text
Goal: 시작 요청
Feedback: 진행 중 상태
Result: 최종 성공/실패
Cancel: 중간 취소
```

## 3. 패키지 지도

`src/` 바로 아래 패키지는 이렇게 나뉜다.

| 폴더 | 한 줄 역할 |
| --- | --- |
| `vicpinky_interfaces/` | 미션과 task가 공통으로 쓰는 custom msg/action 정의 |
| `mission_manager/` | YAML에 적힌 미션 순서를 실행하는 중앙 미션 노드 |
| `mock_task_servers/` | 실제 주행/팔/엘리베이터 서버가 없을 때 fake `RunTask` 서버 제공 |
| `central_bringup/` | mock 서버와 mission manager를 같이 띄우는 launch |
| `vicpinky_nav_adapter/` | `/nav/go_to`를 Nav2 `/navigate_to_pose`로 변환 |
| `vicpinky_gui/` | 브라우저 GUI와 ROS bridge |
| `arm_can_bridge/` | MoveIt `FollowJointTrajectory`를 Board CAN frame으로 변환 |
| `board1_simulator/` | `vcan0`에서 Board1/2/3의 CAN 응답을 흉내냄 |
| `roscue_arm_description/` | 로봇팔 URDF, mesh, RViz 표시 자료 |
| `roscue_arm_moveit_config/` | MoveIt2 planning/controller 설정 |
| `dummy_servers/` | 현재 실사용 로직 없는 placeholder |

무시해도 되는 생성물도 있다.

| 폴더 | 의미 |
| --- | --- |
| `build/`, `install/`, `log/` | `colcon build/test/launch`가 만든 결과물 |
| `__pycache__/` | Python bytecode cache |
| `.pytest_cache/` | pytest cache |

`src/arm_can_bridge/build`, `src/arm_can_bridge/install`, `src/arm_can_bridge/log`처럼 소스 패키지 안에 섞인 빌드 산출물도 있다. 공부할 대상은 아니고, 보통 Git에 올릴 대상도 아니다.

## 4. 핵심 실행 흐름

### 4.1 미션 실행 흐름

1. 사용자가 GUI 또는 CLI에서 `/mission/execute` goal을 보낸다.
2. `mission_manager_node.py`의 `MissionManager`가 goal을 받는다.
3. `MissionFlowLoader`가 `mission_flow.yaml`, `locations.yaml`, `action_servers.yaml`을 읽어 `MissionStep` 목록을 만든다.
4. `MissionManager.execute_mission_callback()`이 step을 순서대로 돈다.
5. 각 step은 `TaskExecutor.execute()`를 통해 해당 `RunTask` action server로 전달된다.
6. 하위 task feedback은 `/mission/status`와 `ExecuteMission.Feedback`으로 다시 올라온다.
7. 하나라도 실패하고 retry도 끝나면 미션은 `FAILED`.
8. 모든 step이 성공하면 미션은 `DONE`.

현재 최종 엘리베이터 flow는 중앙서버 state로 명시되어 있다.

```yaml
- state: GO_TO_ELEVATOR_FRONT
  task: go_to
  target: elevator_front
  location: elevator_front_4f

- state: WAIT_5F
  task: check_floor
  target: floor_5_marker
  location: floor_5_marker

- state: SWITCH_5F_MAP
  task: map_switch
  target: map_5f
  location: map_5f
```

이 뜻은 mission manager가 "지금 어떤 절차를 실행 중인지"는 명시적으로 알고,
실제 정렬/문 감지/층 확인/지도 전환 구현은 각 task server에 맡긴다는 뜻이다.

### 4.2 주행 연결 흐름

mock 테스트에서는 `mock_task_servers`가 `/nav/go_to`를 fake로 제공한다.

실제 주행에서는 흐름이 이렇게 바뀐다.

```text
mission_manager
  -> /nav/go_to RunTask
  -> vicpinky_nav_adapter
  -> /navigate_to_pose NavigateToPose
  -> VicPinky Nav2
```

`vicpinky_nav_adapter`는 `MissionFlowLoader`가 `locations.yaml`의 `points`에서
풀어 넣은 `RunTask.Goal.extra_json.pose`를 `PoseStamped`로 바꾼다.
`yaw`는 quaternion으로 변환된다.

### 4.3 팔 CAN 흐름

1. MoveIt2가 `/arm_controller/follow_joint_trajectory` 또는 `/gripper_controller/follow_joint_trajectory` goal을 보낸다.
2. `ArmCanBridgeNode`가 goal을 받는다.
3. `ArmTrajectoryConverter`가 joint 이름, 범위, 시간, board/motor mapping을 검증한다.
4. trajectory point가 CAN frame 묶음인 `TrajectoryBatch`로 바뀐다.
5. `TrajectoryStreamer`가 board queue 여유를 확인하면서 frame을 보낸다.
6. `SocketCanTransport`가 Linux SocketCAN으로 frame을 쏜다.
7. Board status frame이 들어오면 `unpack_status()`로 파싱된다.
8. `BoardStateTracker`가 enable, homing, queue, error, stale 상태를 추적한다.
9. `/joint_states`는 `CommandedStateEstimator`가 open-loop로 추정해서 발행한다.

주의할 점: 현재 `/joint_states`는 엔코더 실측이 아니라 "우리가 명령한 trajectory가 시간대로 움직였을 것"이라고 추정한 commanded state다.

### 4.4 GUI 흐름

```text
Browser
  -> HTTP GET /                 static/index.html
  -> HTTP GET /api/snapshot     현재 ROS 상태 JSON
  -> HTTP POST /api/mission/start
  -> HTTP POST /api/mission/cancel
  -> HTTP POST /api/arm/<cmd>
  -> vicpinky_gui_node
  -> ROS action/service/topic
```

`vicpinky_gui_node`는 ROS 노드이면서 Flask app을 Werkzeug WSGI server thread로 같이 띄운다. 그래서 터미널에 출력되는 `http://localhost:8080` 주소로 들어가면 브라우저 화면에서 미션과 arm board 상태를 볼 수 있다.

## 5. 파일별 코드 해설

### 5.1 `vicpinky_interfaces`

이 패키지는 Python 로직보다 인터페이스 정의가 핵심이다. 다른 패키지들이 모두 여기에 의존한다.

| 파일 | 역할 |
| --- | --- |
| `package.xml` | ROS 패키지 메타데이터. `rosidl_interface_packages` 그룹에 들어간다. |
| `CMakeLists.txt` | `rosidl_generate_interfaces()`로 msg/action 코드를 생성한다. |
| `action/ExecuteMission.action` | 전체 미션 실행 action 타입. GUI/CLI가 mission manager에 보낸다. |
| `action/RunTask.action` | mission manager가 하위 task 서버에 보내는 공통 task action 타입. |
| `msg/MissionStatus.msg` | `/mission/status` topic 타입. GUI와 디버깅에서 본다. |
| `include/`, `src/` | 현재 직접 로직은 거의 없고 인터페이스 패키지 관례상 있는 폴더다. |

`ExecuteMission.action`은 전체 미션용이다.

```text
Goal:
  mission_id
  pickup_location
  delivery_location
  target_floor
  object_label

Feedback:
  current_state
  current_task
  progress
  detail

Result:
  success
  final_state
  message
```

`RunTask.action`은 하위 기능 서버 공통 타입이다.

```text
Goal:
  task_id
  target_name
  target_floor
  marker_id
  extra_json

Feedback:
  phase
  progress
  detail

Result:
  success
  message
```

팀원 질문에 대한 핵심 답:

> 왜 하위 task마다 action 타입을 새로 안 만들고 `RunTask` 하나를 쓰나?

초기 통합 단계에서는 주행, 도킹, 팔, 엘리베이터를 같은 방식으로 호출해야 하므로 공통 envelope이 편하다. 구체 데이터는 `target_name`, `marker_id`, `extra_json`으로 넘긴다.

### 5.2 `mission_manager`

#### 폴더와 설정 파일

| 파일/폴더 | 역할 |
| --- | --- |
| `package.xml` | Python ROS 패키지 의존성 정의. `vicpinky_interfaces`, `rclpy`, `PyYAML` 등이 중요하다. |
| `setup.py` | console script 등록. `mission_manager_node`, `send_mission`, `send_demo_mission` 실행 파일을 만든다. |
| `setup.cfg` | 설치 경로와 테스트 설정. |
| `resource/mission_manager` | ament index에서 패키지를 찾기 위한 marker 파일. |
| `launch/mission_manager.launch.py` | mission manager 노드 실행 launch. config 파일 경로를 parameter로 넘긴다. |
| `config/mission_flow.yaml` | 미션 step 순서 정의. 이 파일을 바꾸면 미션 절차가 바뀐다. |
| `config/locations.yaml` | 주행팀 map 좌표 `points`와 elevator marker/map 같은 location metadata를 담는다. |
| `config/action_servers.yaml` | task 이름을 실제 action server 이름, timeout, retry로 매핑한다. |
| `test/` | flake8, pep257, copyright 테스트. 현재 일부 스타일 이슈가 남아 있을 수 있다. |

#### `mission_state.py`

작은 dataclass와 enum을 모아둔 파일이다.

| 코드 | 역할 |
| --- | --- |
| `MissionRuntimeState` | `IDLE`, `RUNNING`, `DONE`, `FAILED`, `CANCELED` 같은 전체 미션 상태 enum. |
| `MissionContext` | goal에서 받은 mission_id, pickup, delivery, target_floor, object_label을 한 묶음으로 보관. |
| `MissionStep` | YAML에서 resolved된 실행 step. state, task, server, target, marker_id, extra_json, timeout, retry를 포함. |
| `TaskExecutionResult` | 하위 task 실행 결과. success, canceled, message를 담는다. |

이 파일은 "데이터 모양"을 정리하는 곳이다. 실제 실행 로직은 없다.

#### `mission_flow_loader.py`

YAML을 읽어 실제 실행 계획으로 바꾸는 파일이다.

중요 함수:

| 함수 | 하는 일 |
| --- | --- |
| `__init__()` | mission flow, locations, action server YAML을 로드한다. |
| `_load_yaml()` | YAML 파일을 dict로 읽는다. |
| `_validate_static_config()` | 필수 key가 있는지, 설정이 말이 되는지 시작 시 검증한다. |
| `_context_values()` | mission goal을 `$pickup_location`, `$target_floor` 같은 치환용 dict로 만든다. |
| `_resolve_value()` | YAML 값 안의 `$변수`를 실제 goal 값으로 치환한다. |
| `_resolve_marker_id()` | location이나 floor 기준으로 marker_id를 찾는다. |
| `build_plan()` | 최종적으로 `list[MissionStep]`을 만든다. |

예를 들어 `mission_flow.yaml`에 이런 값이 있으면:

```yaml
location: "$pickup_location"
```

사용자가 goal에서 `pickup_location=room_402`를 보냈을 때 실제 step에는 `room_402`가 들어간다.

#### `task_executor.py`

하위 `RunTask` action server를 호출하는 공통 실행기다.

| 함수 | 하는 일 |
| --- | --- |
| `_get_client(server_name)` | action server 이름별로 `ActionClient`를 만들고 재사용한다. |
| `execute(step, mission_goal_handle, feedback_callback)` | `MissionStep`을 `RunTask.Goal`로 바꿔 보내고, feedback/result/cancel/timeout을 처리한다. |
| `_cancel_child_goal()` | 전체 미션 취소가 들어오면 현재 하위 task goal도 취소한다. |

중요한 점은 mission manager가 각 task의 내부 구현을 모른다는 것이다. `TaskExecutor`는 그냥 `RunTask` goal을 보내고 결과만 기다린다.

#### `mission_manager_node.py`

미션의 중심 노드다.

핵심 구조:

| 코드 | 역할 |
| --- | --- |
| `MissionManager(Node)` | ROS 2 node 클래스. |
| `self.execute_action_server` | `/mission/execute` ActionServer. |
| `self.status_publisher` | `/mission/status` publisher. |
| `goal_callback()` | 이미 미션 실행 중이면 새 goal reject. 아니면 accept. |
| `cancel_callback()` | 전체 미션 cancel 허용. |
| `publish_status()` | GUI/CLI가 볼 수 있게 `MissionStatus` topic 발행. |
| `publish_action_feedback()` | action client에게 feedback 발행. |
| `finish_canceled()` | cancel 처리 공통 함수. |
| `execute_mission_callback()` | 미션 step loop의 본체. |

`execute_mission_callback()`을 읽을 때는 아래 순서로 보면 된다.

1. goal request를 `MissionContext`로 만든다.
2. `flow_loader.build_plan(context)`로 실행 계획을 만든다.
3. `/mission/status`에 `Mission started`를 낸다.
4. `for step_index, step in enumerate(plan)`으로 step을 순회한다.
5. step마다 retry 횟수만큼 `TaskExecutor.execute()`를 호출한다.
6. child feedback을 전체 progress로 환산한다.
7. 실패하면 `goal_handle.abort()`.
8. 성공하면 `goal_handle.succeed()`.
9. 마지막 `finally`에서 `mission_active=False`로 풀어준다.

팀원 질문에 대한 핵심 답:

> Mission manager가 엘리베이터 FSM 상태를 직접 들고 있어야 하지 않나?

지금 최종 구조에서는 어느 정도 들고 있다. 중앙서버는
`GO_TO_ELEVATOR_FRONT`, `ENTER_ELEVATOR`, `WAIT_5F`, `SWITCH_5F_MAP` 같은
큰 절차 상태를 알고 순서를 관리한다. 대신 `/cmd_vel` 제어, ArUco 정렬,
문 열림 판정, Nav2 map switching 같은 실제 구현은 `/dock/align`,
`/elevator/wait_door_open`, `/floor/check`, `/map/switch` task server가 담당한다.

#### `send_mission.py`, `send_demo_mission.py`

| 파일 | 역할 |
| --- | --- |
| `send_mission.py` | CLI argument를 받아 `/mission/execute` goal을 보내는 테스트 클라이언트. |
| `send_demo_mission.py` | 기본 demo 값으로 `send_mission`을 호출하는 편의 실행 파일. |

실제 사용 예:

```bash
ros2 run mission_manager send_demo_mission
```

또는 직접:

```bash
ros2 run mission_manager send_mission \
  --mission-id demo_001 \
  --pickup-location room_402 \
  --delivery-location room_501 \
  --target-floor 5 \
  --object cup \
  --arm-task-name pick_object_2
```

### 5.3 `mock_task_servers`

실제 주행, 도킹, 팔, 엘리베이터 서버가 아직 없을 때 전체 미션 흐름을 테스트하기 위한 fake action server 패키지다.

| 파일 | 역할 |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | Python ROS 패키지 기본 구성. |
| `launch/mock_servers.launch.py` | mock task server 노드를 띄운다. |
| `config/mock_tasks.yaml` | 어떤 action server를 만들고, 어떤 phase/progress를 낼지 정의한다. |
| `mock_task_servers/mock_task_servers_node.py` | 여러 개의 `RunTask` ActionServer를 동적으로 생성한다. |
| `test/` | 스타일 테스트. |

`mock_task_servers_node.py`의 핵심:

| 함수 | 역할 |
| --- | --- |
| `load_config()` | YAML에서 server 목록을 읽는다. |
| `create_action_servers()` | `/nav/go_to`, `/dock/align`, `/arm/pick` 같은 action server를 만든다. |
| `goal_callback()` | goal 수락 로그를 찍고 accept. |
| `cancel_callback()` | cancel 허용. |
| `execute_callback()` | config에 적힌 phase를 순서대로 feedback으로 내보내고 result를 성공 처리. |

최종 mission mock은 이런 식으로 중앙서버 state가 진행된다.

```text
GO_TO_ELEVATOR_FRONT -> ... -> SWITCH_5F_MAP -> ... -> RETURN_HOME -> DONE
```

그래서 GUI가 실제 주행팀 서버 없이도 Mission FSM 표시를 테스트할 수 있다.

### 5.4 `central_bringup`

mock 통합 실행용 launch 패키지다.

| 파일 | 역할 |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | 기본 Python 패키지 구성. |
| `central_bringup/__init__.py` | 패키지 marker. 실행 로직 없음. |
| `launch/bringup_mock.launch.py` | `mock_task_servers` launch를 먼저 include하고, 약간 뒤에 `mission_manager` launch를 include한다. |
| `test/` | 스타일 테스트. |

사용:

```bash
ros2 launch central_bringup bringup_mock.launch.py
```

실제 Nav2 adapter를 붙일 때는 이 launch를 쓰면 안 된다. mock도 `/nav/go_to`를 열기 때문에 실제 `vicpinky_nav_adapter`와 action 이름이 충돌한다.

### 5.5 `vicpinky_nav_adapter`

mission manager의 추상 task `/nav/go_to`를 실제 Nav2 action `/navigate_to_pose`로 바꿔주는 adapter다.

| 파일 | 역할 |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | Python ROS 패키지 구성. |
| `config/nav_adapter.yaml` | Nav2 action 이름, 기본 frame, timeout 등 parameter. |
| `launch/nav_adapter.launch.py` | adapter 노드 실행. |
| `vicpinky_nav_adapter/nav_adapter_node.py` | 실제 변환 로직. |
| `test/` | 스타일 테스트. |

`nav_adapter_node.py` 핵심 클래스:

| 코드 | 역할 |
| --- | --- |
| `NavigationTarget` | Nav2에 넘길 frame_id, x, y, yaw, label을 담는 dataclass. |
| `VicPinkyNavAdapter(Node)` | `/nav/go_to` RunTask server이자 `/navigate_to_pose` Nav2 client. |

중요 함수:

| 함수 | 역할 |
| --- | --- |
| `_goal_callback()` | `RunTask` goal 수락 여부 판단. |
| `_cancel_callback()` | cancel 허용. |
| `_execute_callback()` | RunTask goal을 Nav2 goal로 변환하고 결과를 기다린다. |
| `_target_from_goal()` | `extra_json.pose`를 읽어 `NavigationTarget` 생성. |
| `_parse_extra_json()` | JSON 문자열 파싱. |
| `_extract_yaw()` | pose dict에서 yaw를 읽는다. |
| `_build_pose()` | yaw를 quaternion으로 바꿔 `PoseStamped` 생성. |
| `_wait_for_nav_server()` | Nav2 action server가 뜰 때까지 대기. |
| `_nav_feedback()` | Nav2 feedback을 RunTask feedback으로 변환. |
| `_cancel_nav_goal()` | 상위 미션 cancel 시 Nav2 goal cancel. |

팀원 질문에 대한 핵심 답:

> 왜 mission manager가 직접 `/navigate_to_pose`를 호출하지 않나?

mission manager는 "go_to라는 task"만 알아야 한다. Nav2를 쓰든 다른 주행 시스템을 쓰든 adapter만 바꾸면 되도록 분리한 구조다.

### 5.6 `vicpinky_gui`

브라우저로 미션과 arm board 상태를 관제하는 패키지다.

| 파일 | 역할 |
| --- | --- |
| `package.xml` | GUI 노드 의존성. `rclpy`, `std_srvs`, `sensor_msgs`, `vicpinky_interfaces`, `python3-flask` 등이 중요하다. |
| `setup.py` | `vicpinky_gui_node` console script와 static 파일 설치 설정. |
| `setup.cfg`, `resource/` | Python ROS 패키지 기본 파일. |
| `launch/vicpinky_gui.launch.py` | host, port, auto_port, mission 기본값 등을 parameter로 받아 GUI 노드 실행. |
| `vicpinky_gui/gui_node.py` | ROS와 Flask HTTP API를 이어주는 backend. |
| `static/index.html` | 브라우저 화면 구조. |
| `static/app.css` | 화면 스타일. |
| `static/app.js` | `/api/snapshot` polling, 버튼 동작, 화면 렌더링. |

`gui_node.py`의 구조:

| 코드 | 역할 |
| --- | --- |
| `ARM_COMMANDS` | GUI 버튼 이름을 `/arm_board/enable` 같은 service 이름으로 매핑. |
| `STATUS_NAME_BY_CODE` | Board 상태 숫자를 사람이 읽는 문자열로 변환. |
| `Flask` app | GET/POST route를 정의하고 static 파일과 JSON API 응답을 담당한다. |
| Werkzeug WSGI server | Flask app을 별도 thread에서 serve한다. |
| `VicPinkyGuiNode(Node)` | ROS subscription/action/service client와 Flask server를 모두 가진 노드. |

주요 endpoint:

| HTTP | 의미 |
| --- | --- |
| `GET /` | `index.html` 반환. |
| `GET /api/snapshot` | mission, arm, joint, config, event log를 JSON으로 반환. |
| `POST /api/mission/start` | `/mission/execute` action goal 전송. |
| `POST /api/mission/cancel` | 진행 중 mission goal cancel. |
| `POST /api/arm/enable` | `/arm_board/enable` service call. |
| `POST /api/arm/disable` | `/arm_board/disable` service call. |
| `POST /api/arm/home_all` | `/arm_board/home_all` service call. |
| `POST /api/arm/clear_error` | `/arm_board/clear_error` service call. |
| `POST /api/arm/estop` | `/arm_board/estop` service call. |
| `POST /api/arm/status` | `/arm_board/status` service call. |

`VicPinkyGuiNode` 주요 함수:

| 함수 | 역할 |
| --- | --- |
| `_declare_parameters()` | host, port, auto_port, mission 기본값 선언. |
| `_start_http_server()` | Flask/Werkzeug server thread 시작. |
| `_create_flask_app()` | dashboard static route와 JSON API route 정의. |
| `_bind_http_server()` | port가 사용 중이면 다음 port를 찾고 Werkzeug server를 만든다. |
| `_load_gui_config()` | mission config를 읽어 GUI 선택지 구성. |
| `_mission_status_callback()` | `/mission/status` 수신. |
| `_arm_status_callback()` | `/arm_board/status_log` 수신 후 board/controller 정보 파싱. |
| `_joint_state_callback()` | `/joint_states` 수신. |
| `snapshot()` | 브라우저가 볼 JSON 상태 생성. |
| `call_arm_service()` | arm board service 호출. |
| `start_mission()` | ExecuteMission goal 전송. |
| `_mission_goal_from_payload()` | browser payload를 `ExecuteMission.Goal`로 변환. |
| `_mission_feedback_callback()` | mission feedback 저장. |
| `_mission_result_callback()` | mission result 저장. |
| `cancel_mission()` | goal cancel 요청. |
| `_parse_arm_status()` | text status log를 board/controller 구조로 파싱. |

`static/app.js`는 frontend 상태 머신이라고 보면 된다.

| 함수/역할 | 설명 |
| --- | --- |
| 주기 polling | `GET /api/snapshot`을 반복 호출해서 화면 갱신. |
| mission form | mission_id, pickup, delivery, floor, object_label을 모아 start API 호출. |
| arm buttons | enable, disable, home, clear_error, estop, status API 호출. |
| mission rendering | 현재 미션 상태, progress, active task 표시. |
| mission FSM rendering | mission state/feedback에서 최종 상태 머신 문자열을 찾아 단계 표시. |
| board rendering | Board1/2/3 상태, queue, error, homing 표시. |
| joint table | `/joint_states`의 joint position/velocity/effort 표시. |
| event log | 최근 mission/arm event 표시. |

실행:

```bash
ros2 launch vicpinky_gui vicpinky_gui.launch.py
```

주소:

```text
http://localhost:8080
```

이미 8080을 쓰고 있으면 8081, 8082처럼 다음 빈 포트로 자동 이동한다.

### 5.7 `arm_can_bridge`

가장 코드가 많은 패키지다. MoveIt trajectory를 CAN frame으로 변환하고, 보드 상태를 추적하고, service/action/topic을 제공한다.

#### 폴더와 설정 파일

| 파일/폴더 | 역할 |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | Python ROS 패키지 구성. |
| `config/arm_can_bridge.yaml` | CAN interface, action 이름, joint 이름, board/motor mapping, joint limit, home position, timeout 설정. |
| `launch/arm_can_bridge.launch.py` | config 파일을 parameter로 넣어 노드 실행. |
| `docs/SOCKETCAN_NEXT_STEPS.md` | SocketCAN 관련 후속 작업 메모. |
| `test/` | protocol, board state, socket transport, trajectory converter 테스트. |

#### `can_protocol.py`

CAN frame의 "언어"를 정의하는 파일이다.

상수:

| 상수 | 의미 |
| --- | --- |
| `CAN_ID_ESTOP = 0x001` | emergency stop command. |
| `CAN_ID_ENABLE = 0x010` | enable/disable command. |
| `CAN_ID_HOMING = 0x020` | homing command. |
| `CAN_ID_CLEAR_ERROR = 0x030` | error clear command. |
| `CAN_ID_BOARD1_POSITION_COMMAND = 0x101` | Board1 position command. |
| `CAN_ID_BOARD2_POSITION_COMMAND = 0x102` | Board2 position command. |
| `CAN_ID_BOARD3_POSITION_COMMAND = 0x103` | Board3 servo command. |
| `CAN_ID_BOARD1_STATUS = 0x201` | Board1 status. |
| `CAN_ID_BOARD2_STATUS = 0x202` | Board2 status. |
| `CAN_ID_BOARD3_STATUS = 0x203` | Board3 status. |
| `DURATION_TICK_NS = 5_000_000` | duration 1 tick = 5ms. |
| `ANGLE_RAW_PER_DEGREE = 100.0` | CAN target position 단위 = 0.01 degree. |

enum/dataclass:

| 코드 | 의미 |
| --- | --- |
| `BoardState` | DISABLED, IDLE, HOMING, MOVING, ESTOP 등 board 상태. |
| `BoardError` | error code enum. |
| `CanFrame` | `can_id`, `data`를 담는 immutable frame. payload는 최대 8 byte. |
| `PositionControl` | Byte0 control flag를 사람이 읽는 구조로 푼 것. |
| `BoardStatus` | status frame을 파싱한 결과. state, error, homed_mask, queue_free 등 포함. |

중요 함수:

| 함수 | 역할 |
| --- | --- |
| `rad_to_angle_raw()` | radian을 0.01 degree 정수로 변환. |
| `angle_raw_to_rad()` | 0.01 degree 정수를 radian으로 변환. |
| `duration_ns_to_ticks()` | ns duration을 5ms tick으로 변환. 255 초과 불가. |
| `validate_board_id()` | board id가 1,2,3 또는 broadcast인지 검증. |
| `move_can_id_for_board()` | board id -> command CAN ID. |
| `status_can_id_for_board()` | board id -> status CAN ID. |
| `board_id_from_status_can_id()` | status CAN ID -> board id. |
| `motor_count_for_board()` | Board1=4, Board2=1, Board3=9. |
| `build_control_byte()` | execute/relative/step/motor id를 Byte0로 pack. |
| `decode_control_byte()` | Byte0를 다시 `PositionControl`로 unpack. |
| `pack_position_command()` | position command CAN frame 생성. |
| `pack_estop()` | ESTOP frame 생성. |
| `pack_enable()` | enable/disable frame 생성. |
| `pack_homing()` | homing frame 생성. |
| `pack_clear_error()` | clear error frame 생성. |
| `unpack_status()` | status CAN frame을 `BoardStatus`로 파싱. |

최종 통합 공통 제어 payload는 모두 8 byte 고정이다.

| CAN ID | 함수 | payload |
| --- | --- | --- |
| `0x001` | `pack_estop()` | `[1, 0, 0, 0, 0, 0, 0, 0]` |
| `0x010` | `pack_enable()` | `[enable, target_board, 0, 0, 0, 0, 0, 0]` |
| `0x020` | `pack_homing()` | `[target_board, target_local_motor, mode, 0, 0, 0, 0, 0]` |
| `0x030` | `pack_clear_error()` | `[target_board, target_local_motor, 0, 0, 0, 0, 0, 0]` |

전체 broadcast는 `0xFF`를 기본으로 쓰고, legacy 호환용 `0x00`도 전체 target으로 허용한다.
Board3 homing은 limit switch 원점 탐색이 아니라 9개 gripper joint를 `0.00 deg`로 보내는 home posture 명령이다.

팀원 질문에 대한 핵심 답:

> CAN payload 위치는 어디서 정하나?

`can_protocol.py`가 유일한 기준이다. 다른 코드는 직접 `bytes`를 조립하지 않고 `pack_*`, `unpack_status()`를 써야 한다.

#### `board_state.py`

보드가 trajectory를 받을 수 있는 상태인지 판단하는 파일이다.

주요 dataclass:

| 코드 | 역할 |
| --- | --- |
| `BoardRuntimeConfig` | board id, queue capacity, required homing mask, stale timeout 설정. |
| `BoardStateSnapshot` | GUI/log용 현재 board 상태 snapshot. |
| `MultiBoardStateSnapshot` | 여러 board snapshot 묶음. |

`BoardStateTracker`는 board 하나를 추적한다.

중요 함수:

| 함수 | 역할 |
| --- | --- |
| `update_status()` | 새 status frame을 반영한다. |
| `is_status_stale()` | 최근 status가 너무 오래됐는지 판단. |
| `has_error()` | error code가 있는지 확인. |
| `is_estop()` | ESTOP 상태인지 확인. |
| `is_enabled()` | board가 enable 상태인지 확인. |
| `all_axes_homed()` | 필요한 축이 homing 완료됐는지 확인. |
| `available_queue_slots()` | 현재 쓸 수 있는 queue slot 수 계산. |
| `can_accept_new_trajectory()` | 새 trajectory 시작 가능 여부 판단. |
| `can_stream_slots()` | 특정 frame 묶음을 지금 보낼 수 있는지 판단. |
| `reserve_queue_slots()` | 보낼 frame 수만큼 queue slot을 예약. |
| `refund_queue_slots()` | 실패/cancel 시 예약 slot 반환. |
| `is_trajectory_complete()` | board queue와 moving 상태가 비었는지 판단. |
| `snapshot()` | 상태를 읽기 좋은 구조로 반환. |

`MultiBoardStateTracker`는 controller 하나가 여러 board를 쓸 때 묶어서 본다. 예를 들어 arm controller는 Board1과 Board2를 같이 써야 하므로 둘 다 ready여야 trajectory를 받을 수 있다.

#### `trajectory_converter.py`

ROS `JointTrajectory`를 CAN frame batch로 바꾸는 파일이다.

주요 코드:

| 코드 | 역할 |
| --- | --- |
| `TrajectoryConversionError` | trajectory가 잘못됐을 때 raise. |
| `TrajectoryBatch` | 변환된 CAN frame 목록과 예상 duration, 마지막 joint position. |
| `ArmTrajectoryConverter` | 실제 변환 클래스. |

검증하는 것:

| 검증 | 이유 |
| --- | --- |
| joint 이름이 controller 설정과 맞는가 | MoveIt 설정과 bridge 설정 불일치 방지. |
| position 개수가 joint 개수와 맞는가 | payload 누락 방지. |
| joint limit 안에 있는가 | 하드웨어 안전. |
| time_from_start가 증가하는가 | 뒤로 가는 trajectory 방지. |
| duration tick이 1~255 범위인가 | CAN payload Byte7 한계. |
| board/motor mapping이 유효한가 | 잘못된 board로 명령 전송 방지. |

중요 함수:

| 함수 | 역할 |
| --- | --- |
| `_validate_configuration()` | bridge 설정 자체가 맞는지 시작 시 검증. |
| `_duration_to_ns()` | ROS Duration을 ns로 변환. |
| `_make_joint_index_map()` | goal joint 순서를 controller 내부 순서로 맞추는 map 생성. |
| `_reorder_positions()` | incoming position을 controller joint 순서로 재정렬. |
| `_split_duration_ticks()` | 긴 duration을 255 tick 이하 여러 구간으로 나눔. |
| `_build_frames()` | board/motor별 position command frame 생성. |
| `convert()` | 전체 변환 entry point. |

#### `trajectory_streamer.py`

변환된 CAN frame을 실제로 흘려보내는 파일이다.

| 함수 | 역할 |
| --- | --- |
| `stream()` | batch를 순서대로 보내고 완료까지 기다린다. |
| `_wait_for_queue_slots()` | board queue 여유가 생길 때까지 대기. |
| `_wait_for_completion()` | 모든 board가 trajectory 완료 상태가 될 때까지 대기. |
| `stop_by_disable()` | cancel/stop 시 disable frame 전송. |
| `_raise_if_cancelled()` | action cancel 요청이 있으면 중단. |

핵심은 보드 queue를 무시하고 한꺼번에 frame을 밀어 넣지 않는다는 점이다. status를 보면서 보낼 수 있을 때만 보낸다.

#### `socketcan_transport.py`

Linux SocketCAN wrapper다.

| 코드 | 역할 |
| --- | --- |
| `encode_socketcan_frame()` | `CanFrame`을 Linux raw CAN binary 구조로 pack. |
| `decode_socketcan_frame()` | raw socket bytes를 `CanFrame`으로 unpack. |
| `build_socketcan_filter_data()` | 특정 CAN ID만 받는 filter 구조 생성. |
| `SocketCanTransport.open()` | socket 생성, bind, filter 설정, receive thread 시작. |
| `send_frame()` | CAN frame 전송. |
| `_receive_loop()` | thread에서 계속 frame 수신. |
| `_dispatch_frame()` | 수신 frame callback 호출. |
| `_report_error()` | socket 에러 callback 호출. |

CAN frame은 최대 8 byte payload라 `CanFrame.__post_init__()`에서도 길이를 검증한다.

#### `commanded_state.py`

실측 encoder가 없는 상황에서 `/joint_states`를 만들기 위한 open-loop estimator다.

| 코드 | 역할 |
| --- | --- |
| `ScheduledSegment` | 시작/끝 시간과 시작/끝 position을 가진 trajectory segment. |
| `CommandedStateEstimator` | 시간에 따라 position을 선형 보간한다. |

중요 함수:

| 함수 | 역할 |
| --- | --- |
| `reset_invalid()` | 현재 position 추정값을 invalid로 만든다. |
| `mark_positions_valid()` | home 또는 known position으로 valid 처리. |
| `mark_homed_zero()` | homing 후 zero/home position으로 처리. |
| `start_trajectory()` | batch의 마지막 position과 duration으로 segment 등록. |
| `to_joint_state_msg()` | `sensor_msgs/JointState` 메시지 생성. |
| `_update_locked()` | 현재 시간 기준 position 보간. |

#### `arm_can_bridge_node.py`

패키지의 중심 노드다.

주요 구성:

| 코드 | 역할 |
| --- | --- |
| `TrajectoryControllerContext` | arm controller 또는 gripper controller 하나에 필요한 객체 묶음. |
| `ArmCanBridgeNode(Node)` | service, action, topic, CAN transport를 모두 연결하는 node. |

`__init__()`에서 하는 일:

1. parameter 선언/읽기.
2. `/arm_board/status_log` publisher 생성.
3. `SocketCanTransport` open.
4. arm controller context 생성.
5. gripper controller context 생성.
6. `/arm_board/enable`, `/disable`, `/home_all`, `/clear_error`, `/estop`, `/status` service 생성.
7. `/joint_states` publisher와 timer 생성.
8. 두 `FollowJointTrajectory` ActionServer 생성.

중요 함수:

| 함수 | 역할 |
| --- | --- |
| `_declare_parameters()` | 모든 설정 parameter 기본값 선언. |
| `_create_controller_context()` | joint mapping, board tracker, converter, streamer 생성. |
| `_create_board_state()` | board id 목록에서 `MultiBoardStateTracker` 생성. |
| `_validate_combined_joint_names()` | arm과 gripper joint 이름 중복 확인. |
| `_handle_can_frame()` | status CAN frame 수신 후 board tracker update. |
| `_send_frame()` | transport로 frame 전송. |
| `_wait_until()` | service 처리 중 특정 상태를 timeout까지 기다림. |
| `_handle_enable()` | enable broadcast 후 enabled 상태까지 대기. |
| `_handle_disable()` | disable broadcast. |
| `_handle_home_all()` | homing command 후 homed 상태까지 대기. |
| `_handle_clear_error()` | clear error broadcast 후 error clear 확인. |
| `_handle_estop()` | ESTOP broadcast 후 상태 invalid 처리. |
| `_handle_status_request()` | 현재 status 문자열 반환. |
| `_goal_callback_follow_joint_trajectory()` | 새 trajectory goal 수락 여부 판단. |
| `_cancel_callback_follow_joint_trajectory()` | trajectory cancel 허용. |
| `_execute_follow_joint_trajectory()` | trajectory 변환, 전송, result 반환. |
| `_publish_status_log()` | 사람이 읽는 status log publish. |
| `_publish_joint_states()` | commanded state 기반 `/joint_states` publish. |
| `_format_status()` | board/controller 상태 문자열 생성. |

실제 사용할 때의 순서는 보통 아래다.

```bash
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/enable std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/home_all std_srvs/srv/Trigger '{}'
ros2 run arm_can_bridge send_test_trajectory
```

#### `send_test_trajectory.py`

arm과 gripper action server에 테스트 trajectory를 보내는 CLI다.

| 코드 | 역할 |
| --- | --- |
| `ARM_ACTION_NAME` | `/arm_controller/follow_joint_trajectory`. |
| `GRIPPER_ACTION_NAME` | `/gripper_controller/follow_joint_trajectory`. |
| `ARM_JOINT_NAMES` | arm 5축 joint 이름. |
| `GRIPPER_JOINT_NAMES` | gripper 9축 joint 이름. |
| `TestTrajectoryClient` | 두 action client를 만들고 goal을 보낸다. |
| `_point()` | `JointTrajectoryPoint` 생성 helper. |

#### `board3_can_smoke_test.py`

ROS action을 통하지 않고 Board3 CAN만 직접 확인하는 하드웨어 smoke test다.

주요 용도:

| 기능 | 설명 |
| --- | --- |
| Board3 status monitor | `0x203` status frame을 기다리고 출력. |
| enable/home/clear/error test | Board3가 CAN common control을 받는지 확인. |
| gripper set command | 9개 servo target을 직접 frame으로 보냄. |

이 파일은 실제 Board3 문제를 ROS/MoveIt 계층 없이 좁혀 볼 때 유용하다.

### 5.8 `board1_simulator`

이름은 `board1_simulator`지만 현재는 Board1, Board2, Board3까지 함께 흉내낼 수 있다. `vcan0`로 `arm_can_bridge`를 테스트하기 위한 패키지다.

| 파일 | 역할 |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | Python ROS 패키지 구성. |
| `config/board1_simulator.yaml` | CAN interface, board id, status rate, tick rate 등 설정. |
| `launch/board1_simulator.launch.py` | simulator node 실행. |
| `docs/BOARD1_SIMULATOR_README.md` | simulator 사용 설명. |
| `docs/BOARD1_SIMULATOR_NEXT_STEPS.md` | 후속 작업 메모. |
| `test/test_board1_simulator_model.py` | simulator model 단위 테스트. |

#### `model.py`

ROS에 의존하지 않는 순수 simulator logic이다.

주요 코드:

| 코드 | 역할 |
| --- | --- |
| `QueuedCommand` | 아직 실행 전 queue에 들어간 command. |
| `ActiveCommand` | 현재 실행 중 command. |
| `Board1SimulatorModel` | board 하나의 enable, homing, queue, motion, status logic. |
| `make_board2_simulator_model()` | Board2 설정으로 model 생성. |
| `make_board3_simulator_model()` | Board3 설정으로 model 생성. |

중요 함수:

| 함수 | 역할 |
| --- | --- |
| `handle_frame()` | 수신 CAN frame이 이 board에 해당하면 처리. |
| `_handle_estop()` | ESTOP 상태 전환. |
| `_handle_enable()` | enable/disable 처리. |
| `_handle_homing()` | homing 시작/완료 처리. |
| `_handle_clear_error()` | error clear 처리. |
| `_handle_position_command()` | position command staging/queue 처리. |
| `_start_ready_commands()` | queue에서 실행 가능한 command 시작. |
| `tick(delta_s)` | 시간 진행. motion 완료 처리. |
| `build_status_frame()` | 현재 상태를 CAN status frame으로 pack. |

Board1은 4축 command를 한 세트로 staging하고, Board2는 1축, Board3는 9서보 staging을 다룬다.

#### `board1_simulator_node.py`

`model.py`를 ROS node와 SocketCAN에 붙이는 wrapper다.

| 함수 | 역할 |
| --- | --- |
| `__init__()` | parameter 읽기, model 생성, SocketCAN open, timer 생성. |
| `_handle_frame()` | transport에서 들어온 frame을 각 model에 전달. |
| `_tick()` | 일정 주기로 model 시간을 진행. |
| `_send_status()` | 각 model의 status frame을 CAN으로 송신. |
| `destroy_node()` | transport close. |

실행:

```bash
ros2 launch board1_simulator board1_simulator.launch.py
```

### 5.9 `roscue_arm_description`

로봇팔 모델 표현 패키지다. 실제 제어 로직은 없고, RViz/MoveIt/robot_state_publisher가 읽을 robot description을 제공한다.

| 파일/폴더 | 역할 |
| --- | --- |
| `package.xml`, `CMakeLists.txt` | ament CMake 패키지 구성. launch, urdf, mesh, config 등을 install한다. |
| `README.md` | 모델 패키지 설명. |
| `robot_data.yaml` | 링크, 조인트, mesh 등 robot metadata. |
| `config/joint_state.yaml` | `joint_state_publisher_gui` 기본 joint 설정. |
| `config/ros2_controllers.yaml` | ros2_control controller 설정 예시. |
| `docs/transforms.md` | 각 joint transform과 kinematic chain 문서. |
| `images/robot.png` | 모델 이미지. |
| `rviz/display.rviz` | RViz 표시 설정. |
| `launch/display.launch.py` | robot_state_publisher, joint_state_publisher_gui, RViz 표시 launch. |
| `launch/bridge_display.launch.py` | bridge와 함께 표시하기 위한 launch. |
| `urdf/roscue_arm.urdf.xacro` | xacro 기반 최상위 robot model. |
| `urdf/roscue_arm.urdf` | xacro가 펼쳐진 URDF. |
| `urdf/assemblies/*.urdf.xacro` | base, arm link, finger link 조립 단위 xacro. |
| `meshes/**/*.dae` | RViz/MoveIt에서 보이는 3D mesh. |

중요한 joint 이름:

```text
base_joint
arm_joint_1
arm_joint_2
arm_joint_3
arm_joint_4
finger_1_base_joint
finger_1_middle_joint
finger_1_tip_joint
finger_2_base_joint
finger_2_middle_joint
finger_2_tip_joint
finger_3_base_joint
finger_3_middle_joint
finger_3_tip_joint
```

이 이름은 세 군데가 반드시 맞아야 한다.

1. `roscue_arm_description` URDF joint 이름.
2. `roscue_arm_moveit_config` controller/joint limit/SRDF 설정.
3. `arm_can_bridge/config/arm_can_bridge.yaml` joint mapping.

### 5.10 `roscue_arm_moveit_config`

MoveIt2 설정 패키지다. MoveIt Setup Assistant가 만든 구조와 비슷하다.

| 파일/폴더 | 역할 |
| --- | --- |
| `package.xml`, `CMakeLists.txt` | MoveIt config 패키지 구성. |
| `config/roscue_arm.srdf` | planning group, virtual joint 등 semantic robot description. |
| `config/joint_limits.yaml` | MoveIt planning에 쓰는 joint limit override. |
| `config/kinematics.yaml` | kinematics solver 설정. |
| `config/moveit_controllers.yaml` | MoveIt controller manager가 사용할 controller/action 이름. |
| `config/ros2_controllers.yaml` | ros2_control controller 설정. |
| `config/pilz_cartesian_limits.yaml` | Pilz planner cartesian limit. |
| `config/moveit.rviz` | MoveIt RViz 설정. |
| `launch/demo.launch.py` | MoveIt demo 구성 실행. |
| `launch/move_group.launch.py` | `move_group` 실행. |
| `launch/moveit_rviz.launch.py` | MoveIt RViz 실행. |
| `launch/rsp.launch.py` | robot_state_publisher 실행. |
| `launch/spawn_controllers.launch.py` | controller spawner 실행. |
| `launch/static_virtual_joint_tfs.launch.py` | world-base fixed transform 발행. |
| `launch/setup_assistant.launch.py` | MoveIt Setup Assistant 실행. |
| `launch/warehouse_db.launch.py` | MoveIt warehouse DB 실행. |

`moveit_controllers.yaml`에서 중요한 점:

```yaml
arm_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  joints:
    - base_joint
    - arm_joint_1
    - arm_joint_2
    - arm_joint_3
    - arm_joint_4

gripper_controller:
  type: FollowJointTrajectory
  action_ns: follow_joint_trajectory
  joints:
    - finger_1_base_joint
    ...
```

MoveIt은 controller 이름과 `action_ns`를 합쳐 아래 action으로 goal을 보낸다.

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

이 이름이 `arm_can_bridge` action server 이름과 같아야 한다.

### 5.11 `dummy_servers`

현재 실사용 로직이 없는 placeholder 패키지다.

| 파일 | 역할 |
| --- | --- |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | 기본 Python ROS 패키지 구성. |
| `dummy_servers/__init__.py` | 빈 package marker. |
| `test/` | 스타일 테스트. |

팀원이 물어보면 이렇게 답하면 된다.

> 지금은 실제 기능 서버가 아니라 패키지 자리만 잡혀 있는 상태다. 실제 mock은 `mock_task_servers`가 담당한다.

## 6. 실제 사용법

### 6.1 빌드

```bash
cd ~/vicpinky_server_ws
colcon build --symlink-install
source install/setup.bash
```

특정 패키지만:

```bash
colcon build --symlink-install --packages-select mission_manager mock_task_servers vicpinky_gui
source install/setup.bash
```

### 6.2 Mock 미션 실행

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

상태 확인:

```bash
ros2 topic echo /mission/status
```

### 6.3 GUI 실행

```bash
cd ~/vicpinky_server_ws
source install/setup.bash
ros2 launch vicpinky_gui vicpinky_gui.launch.py
```

브라우저:

```text
http://localhost:8080
```

포트가 충돌하면 자동으로 다음 포트를 찾는다. 터미널에 찍힌 `VicPinky GUI ready: http://...` 주소를 열면 된다.

### 6.4 실제 주행 adapter 실행

mock을 쓰지 않을 때:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 launch mission_manager mission_manager.launch.py
```

다른 터미널:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 launch vicpinky_nav_adapter nav_adapter.launch.py
```

그리고 VicPinky Nav2가 `/navigate_to_pose` action server를 제공해야 한다.

확인:

```bash
ros2 action list | grep navigate
```

### 6.5 Arm bridge simulator 테스트

`vcan0` 준비:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

이미 있으면:

```bash
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

### 6.6 실제 CAN 사용

`src/arm_can_bridge/config/arm_can_bridge.yaml`:

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

CAN은 UART처럼 TX/RX를 교차하지 않는다. 같은 버스에 `CAN_H`, `CAN_L`, `GND`로 붙는다.

## 7. 팀원 질문 대비

### Q. `mission_manager`는 정확히 뭐 하는 노드야?

전체 미션 action server다. `/mission/execute` goal을 받으면 YAML에 적힌 step을 순서대로 `RunTask` action server에 보내고, 진행 상황을 `/mission/status`로 publish한다.

### Q. 미션 순서를 바꾸려면 코드 고쳐야 해?

대부분은 `mission_manager/config/mission_flow.yaml`을 고치면 된다. 새 task 종류가 생기면 `action_servers.yaml`에도 task 이름과 action server 이름을 추가한다.

### Q. 장소나 marker id는 어디서 바꿔?

`mission_manager/config/locations.yaml`에서 바꾼다. 주행 좌표는 `points`에
층별로 두고, marker 기반 작업은 `locations`의 `marker_id`에 둔다.

### Q. 엘리베이터 FSM은 어디에 반영됐어?

미션 flow에 `GO_TO_ELEVATOR_FRONT`, `ENTER_ELEVATOR`, `WAIT_5F`,
`SWITCH_5F_MAP`, `RETURN_HOME` 같은 중앙서버 state로 반영되어 있다.
GUI는 `/mission/status`와 `ExecuteMission.Feedback`의 state를 감지해
Mission FSM으로 단계 표시한다.

### Q. `/nav/go_to`는 누가 제공해?

테스트 때는 `mock_task_servers`. 실제 주행 때는 `vicpinky_nav_adapter`.

### Q. `vicpinky_nav_adapter`가 왜 필요해?

mission manager가 Nav2 세부 타입에 직접 묶이지 않도록 중간 adapter를 둔 것이다. mission manager는 `RunTask`만 알고, adapter가 Nav2 `NavigateToPose`로 변환한다.

### Q. MoveIt이랑 CAN bridge는 어떻게 연결돼?

MoveIt이 `/arm_controller/follow_joint_trajectory`와 `/gripper_controller/follow_joint_trajectory` action goal을 보낸다. `arm_can_bridge`가 이 action server를 열고 있다가 goal을 CAN frame으로 변환한다.

### Q. `/joint_states`는 실제 encoder야?

아니다. 현재는 commanded estimate다. `CommandedStateEstimator`가 보낸 trajectory와 시간을 기준으로 추정한다.

### Q. 보드가 trajectory를 안 받는 조건은?

status가 없거나 stale, enable 안 됨, homing 안 됨, error/ESTOP 상태, queue 부족, commanded position invalid 등이면 reject된다.

### Q. Board1, Board2, Board3 CAN ID는?

| Board | Command | Status |
| --- | --- | --- |
| Board1 | `0x101` | `0x201` |
| Board2 | `0x102` | `0x202` |
| Board3 | `0x103` | `0x203` |

### Q. Board3 status가 Board1/2랑 다른 점은?

Board3는 9개 servo staging/buffer 개념이 있어서 status byte 일부 의미가 다르다. `can_protocol.BoardStatus`의 `board3_staging_count`, `board3_buffer_free`, `board3_fault_motor_id` property를 보면 된다.

### Q. GUI port 충돌이면?

기본은 8080이고, `auto_port`가 켜져 있어서 8081, 8082처럼 다음 빈 포트를 찾는다. 터미널에 출력된 주소를 열면 된다.

### Q. `dummy_servers`는 써?

지금은 실사용하지 않는 placeholder다. 현재 mock은 `mock_task_servers`가 담당한다.

### Q. `roscue_arm_description`이랑 `roscue_arm_moveit_config` 차이는?

`description`은 로봇의 물리적 모델, 즉 링크/조인트/mesh/URDF다. `moveit_config`는 그 모델을 MoveIt에서 어떻게 planning하고 어떤 controller로 보낼지에 대한 설정이다.

### Q. joint 이름이 왜 중요해?

URDF, MoveIt config, arm_can_bridge YAML의 joint 이름이 모두 같아야 trajectory가 올바른 motor id로 변환된다. 이름이 하나라도 다르면 converter가 reject하거나 엉뚱한 축을 움직일 수 있다.

## 8. 변경할 때 보는 위치

| 하고 싶은 일 | 보는 파일 |
| --- | --- |
| 미션 순서 변경 | `src/mission_manager/config/mission_flow.yaml` |
| 새 task action server 추가 | `src/mission_manager/config/action_servers.yaml` |
| 장소/좌표/marker 변경 | `src/mission_manager/config/locations.yaml` |
| mock task phase 변경 | `src/mock_task_servers/config/mock_tasks.yaml` |
| GUI mission 기본값/port 변경 | `src/vicpinky_gui/launch/vicpinky_gui.launch.py` |
| GUI 화면 수정 | `src/vicpinky_gui/static/index.html`, `app.css`, `app.js` |
| GUI backend API 수정 | `src/vicpinky_gui/vicpinky_gui/gui_node.py` |
| CAN ID/payload 변경 | `src/arm_can_bridge/arm_can_bridge/can_protocol.py` |
| arm/gripper joint mapping 변경 | `src/arm_can_bridge/config/arm_can_bridge.yaml` |
| trajectory 검증/변환 변경 | `src/arm_can_bridge/arm_can_bridge/trajectory_converter.py` |
| board readiness 조건 변경 | `src/arm_can_bridge/arm_can_bridge/board_state.py` |
| SocketCAN 송수신 변경 | `src/arm_can_bridge/arm_can_bridge/socketcan_transport.py` |
| simulator 동작 변경 | `src/board1_simulator/board1_simulator/model.py` |
| URDF link/joint 변경 | `src/roscue_arm_description/urdf` |
| MoveIt controller/action 변경 | `src/roscue_arm_moveit_config/config/moveit_controllers.yaml` |

## 9. 공부 체크리스트

아래 질문에 답할 수 있으면 팀원 설명은 꽤 안정적으로 할 수 있다.

- `ExecuteMission`과 `RunTask`의 차이를 설명할 수 있다.
- `/mission/status`가 어디서 publish되는지 말할 수 있다.
- `mission_flow.yaml`, `locations.yaml`, `action_servers.yaml`의 역할 차이를 설명할 수 있다.
- mock 미션과 실제 Nav2 연결의 차이를 설명할 수 있다.
- `/dock/align`, `/floor/check`, `/map/switch`가 엘리베이터 flow에서 어떤 역할인지 말할 수 있다.
- MoveIt controller action 이름 두 개를 말할 수 있다.
- Board1/2/3 command/status CAN ID를 말할 수 있다.
- `can_protocol.py`와 `trajectory_converter.py`의 책임 차이를 설명할 수 있다.
- 왜 `/joint_states`가 encoder feedback이 아닌지 설명할 수 있다.
- simulator를 왜 `vcan0`로 띄우는지 설명할 수 있다.
- URDF, SRDF, MoveIt controller config의 차이를 설명할 수 있다.
- GUI가 ROS topic/action/service를 HTTP API로 바꾸는 구조를 설명할 수 있다.

## 10. 빠른 명령 모음

빌드:

```bash
cd ~/vicpinky_server_ws
colcon build --symlink-install
source install/setup.bash
```

mock 전체:

```bash
ros2 launch central_bringup bringup_mock.launch.py
```

demo mission:

```bash
ros2 run mission_manager send_demo_mission
```

GUI:

```bash
ros2 launch vicpinky_gui vicpinky_gui.launch.py
```

mission status:

```bash
ros2 topic echo /mission/status
```

arm board status:

```bash
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
```

vcan simulator:

```bash
ros2 launch board1_simulator board1_simulator.launch.py
```

arm bridge:

```bash
ros2 launch arm_can_bridge arm_can_bridge.launch.py execution_mode:=hardware
```

test trajectory:

```bash
ros2 run arm_can_bridge send_test_trajectory
```
