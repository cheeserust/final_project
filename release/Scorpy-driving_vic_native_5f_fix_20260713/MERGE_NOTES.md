# team_src 병합 노트 (branch: merge/team-src)

## 구조

```
final_project/
├── vicpinky_interfaces/      # RunTask / ExecuteMission / MissionStatus (팀 원본)
├── vicpinky_task_servers/    # 층주행 + 층간주행 통합 (로봇측 전체)
├── vic_pinky/                # description + navigation (팀 원본)
└── legacy/vicpinky_mission/  # 구 토픽 기반 mock 미션 — COLCON_IGNORE, 빌드 제외
```

`pinky_delivery` 패키지는 제거됨 (노드/config/maps 전부 `vicpinky_task_servers`로 이관).

## RunTask Action 서버 (로봇측)

| 서버 | task_id | 파일 | 비고 |
| --- | --- | --- | --- |
| `/nav/go_to` | `go_to` | nav_go_to_server.py | nav_points.yaml 기반 |
| `/dock/align` | `dock_to_marker`, `align`, `tag_align` | dock_align_server.py | 팀 실구현. 기본 서버명 `/tag/align`→`/dock/align` 복원 (README 스펙) |
| `/elevator/wait_door_open` | `wait_door_open` | elevator_door_server.py | LiDAR |
| `/floor/check` | `check_floor`, `floor_check` | floor_check_server.py | `/tag/floor_id` 구독 |
| `/map/switch` | `map_switch`, `switch_map` | map_switcher.py | **신규 서버화**. `target_floor`(4/5) 또는 `marker_id`(10=엘베 내부)로 맵 선택 → load_map + `/initialpose` |
| `/elevator/board` | `board_elevator`, `enter_elevator` | elevator_board_off.py | **신규 서버화**. 문열림 대기 → 전진 탑승(50cm 정지) |
| `/elevator/exit` | `exit_elevator`, `exit` | elevator_board_off.py | **신규 서버화**. `target_floor` 마커 확인 → 후진 하차(160cm) → 좌 90도 |
| `/base/rotate` | (팀 원본) | base_rotate_server.py | odom 기반 회전 |

## 토픽 계약

- `/tag/floor_id` (Int32): elevator_board_off가 탑승 완료 후 후방 카메라로 랜딩 마커(4/5)
  안정 검출 시 발행 → floor_check_server 소비. 구 `/floor/arrived` 폐기.
- `/tag/target_offset_x`, `/tag/target_distance`, `/tag/marker_id`:
  aruco_pose_publisher → dock_align_server (엘리베이터 앞 정렬, 마커 20).

## 미션 상태 ↔ 서버 매핑 (엘리베이터 구간)

```
ENTER_ELEVATOR  → /elevator/board   (기존 문서의 /dock/align 대신 — 캐빈 내 무측위 구간)
WAIT_5F         → /floor/check      (target_floor=5; 출발층 마커는 불일치라 자연 대기
                                     → RIDING_MIN_SEC 불필요)
SWITCH_5F_MAP   → /map/switch       (target_floor=5)
EXIT_ELEVATOR   → /elevator/exit    (target_floor=5; 기존 문서의 /nav/go_to 대신)
```

중앙서버 `mission_flow.yaml`의 ENTER/EXIT_ELEVATOR task를 위 서버로 바꿔야 함.

## 실행

```bash
cd ~/fp_ws/final_project && colcon build --symlink-install
source install/setup.bash
ros2 launch vicpinky_task_servers task_servers.launch.py   # 전 서버 실동작 모드
```

단독 테스트 예:

```bash
ros2 action send_goal /elevator/board vicpinky_interfaces/action/RunTask \
  "{task_id: board_elevator}"
ros2 action send_goal /elevator/exit vicpinky_interfaces/action/RunTask \
  "{task_id: exit_elevator, target_floor: 5}"
ros2 action send_goal /map/switch vicpinky_interfaces/action/RunTask \
  "{task_id: map_switch, target_floor: 5}"
```

map_switcher 단독 자동전환(구 동작): `enable_topic_trigger:=true` → `/tag/floor_id` 수신 시 전환.

## 기타 변경

- config: `nav_points.yaml` 팀 버전 채택(402 yaw=0, `402_return_test` 포함),
  `floor_markers.yaml` 내 버전 유지(마커 10 포함), `key_coords.yaml` 폐기.
- maps는 `share/vicpinky_task_servers/maps/`로 일원화, 경로는
  `get_package_share_directory` 기반(절대경로 → map_server CWD 이슈 회피).
- **버그 수정**: dock_align / floor_check / elevator_door 서버에
  `ReentrantCallbackGroup` 추가. 기존엔 실동작 모드에서 execute 루프가 기본
  MutuallyExclusive 그룹을 점유해 구독 콜백이 막힘 → `latest_*`가 갱신 안 되고
  타임아웃까지 멈추는 구조였음 (mock에서만 동작하던 이유).
- `pinky_delivery/src/` 잔재 제거 (find_packages가 `src`를 top-level 패키지로
  설치하던 문제 포함).

## 알려진 한계

1. `/map/switch`의 `/initialpose`는 floor_markers.yaml의 고정 랜딩 좌표를 씀.
   미션 순서상 SWITCH가 EXIT보다 먼저라 하차 완료 전 pose가 세팅됨 — 하차가
   cmd_vel 서보라 기능상 문제는 없지만, 정확한 재측위는 추후 solvePnP 기반
   pose 복원으로 교체 예정. 임시 대안: 미션 flow에서 EXIT → SWITCH 순서로 변경.
2. 마커 10(엘베 내부)의 initial_pose는 placeholder(0,0,0).
3. elevator_front_sequence_test 등 시퀀스 스크립트는 `/tag/align` 이름을 하드코딩
   → 테스트 시 `dock_align_server`에 `server_name:=/tag/align` 넘기거나 스크립트 수정 필요.
