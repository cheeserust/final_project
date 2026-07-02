# Final 4F/5F Mission Scenario

이 문서는 실제 통합 시 중앙서버가 실행할 최종 엘리베이터 미션 흐름을
정리한 것이다. 구현 기준 파일은 `src/mission_manager/config/mission_flow.yaml`이다.

## 시나리오

```text
4F HOME
  -> 4F Elevator Front
  -> 엘리베이터 호출 버튼
  -> 엘리베이터 탑승
  -> 5F 이동
  -> 물건 Pick 또는 Place
  -> 5F Elevator Front
  -> 4F 복귀
  -> HOME
```

## 중앙서버 상태 머신

```text
GO_TO_ELEVATOR_FRONT
ALIGN_ELEVATOR_TAG
PRESS_ELEVATOR_CALL_BUTTON
WAIT_ELEVATOR_OPEN
ENTER_ELEVATOR
ALIGN_INSIDE_ELEVATOR_TAG
PRESS_5F_BUTTON
WAIT_5F
SWITCH_5F_MAP
EXIT_ELEVATOR
GO_TO_TARGET_PLACE
ARM_TASK_AT_TARGET
RETURN_TO_ELEVATOR
PRESS_4F_BUTTON
WAIT_4F
SWITCH_4F_MAP
RETURN_HOME
DONE
```

`DONE`은 마지막 step인 `RETURN_HOME`이 성공하면 `mission_manager`가 결과로
발행하는 최종 상태다. YAML에는 별도 step으로 넣지 않는다.

## Action 연동

| Mission state | Task profile | Action server | 담당 |
|---|---|---|---|
| `GO_TO_ELEVATOR_FRONT` | `go_to` | `/nav/go_to` | 주행 |
| `ALIGN_ELEVATOR_TAG` | `dock_to_marker` | `/dock/align` | 주행/ArUco |
| `PRESS_ELEVATOR_CALL_BUTTON` | `press_button` | `/arm/press_button` | 팔 |
| `WAIT_ELEVATOR_OPEN` | `wait_door_open` | `/elevator/wait_door_open` | 주행/LiDAR |
| `ENTER_ELEVATOR` | `dock_to_marker` | `/dock/align` | 주행/ArUco+Odom |
| `ALIGN_INSIDE_ELEVATOR_TAG` | `dock_to_marker` | `/dock/align` | 주행/ArUco+Odom |
| `PRESS_5F_BUTTON` | `press_button` | `/arm/press_button` | 팔 |
| `WAIT_5F` | `check_floor` | `/floor/check` | 주행/Floor Tag |
| `SWITCH_5F_MAP` | `map_switch` | `/map/switch` | 주행/Nav2 |
| `EXIT_ELEVATOR` | `go_to` | `/nav/go_to` | 주행 |
| `GO_TO_TARGET_PLACE` | `go_to` | `/nav/go_to` | 주행 |
| `ARM_TASK_AT_TARGET` | `place` | `/arm/place` | 팔 |
| `RETURN_TO_ELEVATOR` | `go_to` | `/nav/go_to` | 주행 |
| `PRESS_4F_BUTTON` | `press_button` | `/arm/press_button` | 팔 |
| `WAIT_4F` | `check_floor` | `/floor/check` | 주행/Floor Tag |
| `SWITCH_4F_MAP` | `map_switch` | `/map/switch` | 주행/Nav2 |
| `RETURN_HOME` | `go_to` | `/nav/go_to` | 주행 |

모든 Task Action 타입은 `vicpinky_interfaces/action/RunTask`를 사용한다.

```text
Goal:
  task_id
  target_name
  target_floor
  marker_id
  extra_json

Result:
  success
  message

Feedback:
  phase
  progress
  detail
```

## 주행팀 구현 필요 항목

현재 중앙서버는 아래 Action 이름으로 연동한다.

```text
/nav/go_to
/dock/align
/elevator/wait_door_open
/floor/check
/map/switch
```

주행팀이 전달한 필수 확인 항목:

```bash
ros2 action info /navigate_to_pose
ros2 topic list | grep map
ros2 topic list | grep odom
ros2 topic list | grep amcl
```

실제 주행 구현에서 필요한 topic:

```text
/tag_pose       ArUco 정렬
/cmd_vel        정렬/탑승/하차 속도 명령
/scan_filtered  엘리베이터 문 열림 감지
/tag/floor_id   층 인식
```

층 전환은 `/map/switch`에서 Nav2 `load_map`과 `/initialpose` 재설정을
수행하는 것을 기준으로 둔다. 현재 mock 서버에도 `/map/switch`가 있으므로
실제 구현 전까지 End-to-End 테스트가 가능하다.

## 팔팀 구현 필요 항목

현재 중앙서버는 아래 Action 이름으로 연동한다.

```text
/arm/pick
/arm/place
/arm/press_button
/arm/homing
```

최종 flow의 `ARM_TASK_AT_TARGET`은 현재 기본값이 `/arm/place`다. 5층에서
물건을 집는 미션으로 바꾸려면 `mission_flow.yaml`에서 해당 step의
`task: place`를 `task: pick`으로 바꾸면 된다. pick/place를 실행 시점에
동적으로 고르려면 이후 `ExecuteMission.action`에 `arm_task` 같은 필드를
추가하는 방식으로 확장한다.

## Mock 테스트

```bash
cd ~/vicpinky_server_ws
colcon build --symlink-install --packages-select mission_manager mock_task_servers
source install/setup.bash
ros2 launch central_bringup bringup_mock.launch.py
```

다른 터미널:

```bash
source ~/vicpinky_server_ws/install/setup.bash
ros2 run mission_manager send_demo_mission
```

기본 데모는 `home`에서 시작해 5층 `object_place`로 이동하고, 팔 작업 후
4층 `home`으로 복귀하는 흐름이다.
