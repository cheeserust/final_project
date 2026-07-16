# VicPinky 최종 통합 시나리오

구현 기준은 `src/mission_manager/config/mission_flow.yaml`이다. 한 번의
`/mission/execute` goal이 아래 전 과정을 수행하며, 중간 Action 하나라도 실패하거나
timeout/cancel되면 다음 단계로 진행하지 않는다.

## 장비 배치

```text
Raspberry Pi
  front/rear USB camera + Pinky motor + LiDAR + Nav2
  dock/elevator/floor/map/base-motion Action servers
Central PC
  wrist RealSense + MoveIt2 + USB2CAN-FD + STM32 boards 1/2/3
  nav adapter + mission manager + browser GUI (:8080)
External PC browser
```

장비 간 연결, 접속 주소, ROS Domain ID와 RMW는 팀에서 이미 구성한 값을 그대로
사용한다. 이 통합 배포본은 해당 네트워크 설정을 만들거나 변경하지 않는다.

## 최종 29단계 상태 머신

1. `ARM_HOMING`: Board1~3 enable 확인 후 팔/그리퍼 Homing. 기계 home 완료 뒤
   `/arm_board/home_all` 내부에서 Board1 J1을 `-86.5° → -85.0°`로 이동하는
   300 ms 안전영역 진입 batch까지 끝나야 성공한다.
2. `ARM_READY_AT_PICKUP`: 팔 `ready`, 그리퍼 open.
3. `PICK_OBJECT_TO_TRAY`: 402에서 관찰 자세 → ID 54 안정 검출 → MoveIt2
   top-down pick → middle 세 축 -100° → tip 세 축 -70° → `tray_drop` → open → ready.
4. `GO_TO_ELEVATOR_FRONT`: Pinky 4F `elevator_front` 좌표로 Nav2 이동.
5. `ALIGN_ELEVATOR_TAG`: 전방 ID 20 기준 1.37 m, lateral 0 m 정렬.
6. `PRESS_ELEVATOR_CALL_BUTTON`: `observe_call_4f` → ID 50 버튼 누름.
7. `READY_AND_APPROACH_ELEVATOR_4F`: 팔 ready를 시작하고, Action goal 수락
   2초 뒤 베이스를 0.27 m, 0.15 m/s로 전진. 두 동작을 모두 완료할 때까지 barrier.
8. `FACE_ELEVATOR_4F`: 좌 80° 회전.
9. `WAIT_ELEVATOR_OPEN`: 기존 `/scan_filtered` LiDAR 판정으로 문 열림 대기.
10. `ENTER_ELEVATOR`: 문이 열린 뒤 ID 10 기준 50 cm까지 탑승.
11. `PRESS_5F_BUTTON`: `observe_cabin_5_button` → ID 52 버튼 누름 → ready.
12. `WAIT_5F`: 후방 카메라 ID 5 안정 검출 대기.
13. `EXIT_ELEVATOR`: 후방 ID 5 기반으로 마커 60 cm 지점까지 후진 하차한 뒤
    좌 90° 회전.
14. `SWITCH_5F_MAP`: 물리적 하차 완료 후 5F map load와 AMCL initial pose.
15. `GO_TO_TARGET_PLACE`: Pinky `object_place` 좌표로 Nav2 이동.
16. `ROTATE_AT_DELIVERY`: 배치 방향으로 좌 180° 회전.
17. `DELIVER_OBJECT_FROM_TRAY`: `tray_pick` → staged object close →
    `delivery_5f` → open → ready. 적재판에서는 ID 54를 다시 찾지 않는다.
18. `RETURN_TO_ELEVATOR`: Pinky 5F `elevator_front`로 이동.
19. `ALIGN_ELEVATOR_TAG_RETURN`: ID 20 기준 1.37 m 정렬.
20. `PRESS_ELEVATOR_CALL_BUTTON_RETURN`: `observe_call_5f` → ID 53 누름.
21. `READY_AND_APPROACH_ELEVATOR_5F`: ready 시작 2초 뒤 0.27 m 전진, 두 동작 join.
22. `FACE_ELEVATOR_5F`: 좌 80° 회전.
23. `WAIT_ELEVATOR_OPEN_RETURN`: 기존 `/scan_filtered` LiDAR 판정으로 문 열림 대기.
24. `ENTER_ELEVATOR_RETURN`: 문이 열린 뒤 ID 10 기준 50 cm까지 탑승.
25. `PRESS_4F_BUTTON`: `observe_cabin_4_button` → ID 51 누름 → ready.
26. `WAIT_4F`: 후방 카메라 ID 4 안정 검출 대기.
27. `EXIT_ELEVATOR_RETURN`: 후방 ID 4 기반으로 마커 60 cm 지점까지 후진
    하차한 뒤 좌 90° 회전.
28. `SWITCH_4F_MAP`: 물리적 하차 완료 후 4F map load와 AMCL initial pose.
29. `RETURN_HOME`: mission pickup location인 402로 복귀하고 `DONE`.

기존 주행팀의 LiDAR 문 열림 서버와 `/scan_filtered` 입력을 그대로 사용한다.
`/elevator/board`의 cabin marker 대기는 문 열림 성공 뒤 탑승 거리 제어에만 사용한다.

## Pinky 권위 좌표

| Floor | Name | x (m) | y (m) | yaw (rad) |
|---:|---|---:|---:|---:|
| 4 | 402 | 2.998 | -12.433 | 0.000 |
| 4 | elevator_front | 0.470 | -0.455 | 1.983 |
| 4 | dock/home (legacy) | 2.980 | -14.100 | -3.040 |
| 5 | elevator_front | 14.930 | 1.900 | 0.000 |
| 5 | object_place | 2.840 | 1.170 | 0.109 |

최종 `home` 의미는 legacy `home` point가 아니라 pickup location `402`다.
중앙 PC의 `locations.yaml`과 RPi의 `nav_points.yaml`은 이 값을 동일하게 유지한다.

## 팔·그리퍼 설정

팔 순서는 `[BASE, J1, J2, J3, J4]`, 단위는 degree다.

| Pose | Degrees |
|---|---|
| travel_stow | `[-90, -85, -62.2, -91.5, -90]` |
| ready / observe_pickup_402 | `[0, -15, -53, -91.5, -1]` |
| tray_drop | `[-60, -30, 70, 70, -1]` |
| tray_pick | `[-60, -40, 80, 60, -1.2]` |
| observe_call_4f / 5f | `[-4, -56, -53, -91.5, -1]` |
| observe_cabin_4_button | `[-40, -55.5, -53, 90, -1]` |
| delivery_5f | `[-5, 40, -40, -40, -90]` |

그리퍼 순서는 `[F1_BASE,F1_MIDDLE,F1_TIP, F2_..., F3_...]`다.

- open: `[0,0,0] × 3`
- object close stage 1: `[0,-100,0] × 3`, effort 700
- object close stage 2: `[0,-100,-70] × 3`, effort 700
- button: `[0,0,-90, 0,0,-90, 0,-90,0]`, effort 900

버튼은 marker normal 방향으로 10 cm 앞에 접근하고 중심보다 5 mm 안쪽까지
이동해 2초 유지한 뒤 12 cm로 후퇴한다.

Arm CAN 송신은 frame 사이 `7 ms`를 유지한다. 일반 궤적은 각 point가 최소
`8 tick = 40 ms`가 되도록 서버에서 재시간화하며, firmware raw 각도 한계를 넘는
목표는 SocketCAN 송신 전에 실패 처리한다. 이 방어 로직은 ROS/MoveIt joint limit나
URDF를 변경하지 않는다.

| ID | 역할 | Marker center → button center (m) |
|---:|---|---|
| 50 | 4F 외부 호출 | `[0, -0.07, 0]` |
| 51 | 객실 4F | `[+0.08, 0, 0]` |
| 52 | 객실 5F | `[-0.08, 0, 0]` |
| 53 | 5F 외부 호출 | `[0, -0.13, 0]` |
| 54 | object_1 doll | top-down pick |

## TF

- `base_link → arm_base_link`: xyz `[-0.295, 0.075, 0.665] m`, rpy `[0,0,0]`
- `gripper_base_link → camera_link`: xyz `[0,-0.070,0.057] m`,
  rpy `[90°,-90°,0°]`
- `gripper_base_link → button_contact_link`: xyz
  `[-0.073434,-0.044148,0.179650] m`, rpy `[0,0,30°]`
- `gripper_base_link → grasp_tcp_link`: xyz `[0,0,0.134295] m`, rpy `[0,0,0]`.
  object-close 관절각에서 세 fingertip joint 원점의 중심으로 계산한 파지 TCP다.

MoveIt SRDF에는 `world → base_link` 고정 virtual joint를 두지 않는다. Nav2가
`map → odom → base_link`를 소유하고 PC robot_state_publisher는 그 아래 팔 TF만 추가한다.

## 자료 내 모순을 처리한 기준

- 본문 27 cm와 부록의 기존 성공값 60 cm가 충돌하므로 본문 27 cm를 기본값으로 채택했다.
  값은 `mission_flow.yaml`에서 현장 보정할 수 있다.
- PDF의 객실 4F pose J3 `+91.5°`는 제공된 최대 `+90°`를 넘으므로 `+90°`로 제한했다.
- `observe_cabin_5_button` 각도는 제공되지 않아 객실 전체 패널을 보는 4F 관찰 pose를
  재사용했다. ID 52의 -8 cm offset으로 5F 버튼 중심을 구분한다.
- 배치 전 회전 각도는 제공된 Pinky sequence의 방향 전환값을 바탕으로 180°를
  YAML 값으로 두었다. 현장에서 선반과 팔의 왼쪽 방향을 확인해 이 한 값만 보정한다.
- `grasp_tcp_link`는 제공된 URDF와 object-close 각도로 계산한 값이며 실측 제공값은
  아니다. 최초 적재 시험에서 높이를 확인한 뒤 필요하면 이 고정 TF만 보정한다.
- 4F/5F map switch initial pose는 Pinky `nav_points.yaml`의 각 층
  `elevator_front` 좌표와 동기화했다. 실제 하차 지점과 차이가 있으면 두 파일을
  같은 값으로 함께 보정한다.

## 실행

RPi:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_final/pinky/install/setup.bash
ros2 launch vicpinky_final_bringup final_robot.launch.py
```

Central PC:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_final/pj/install/setup.bash
ros2 launch central_bringup final_system.launch.py can_interface:=can0
```

팀에서 사용하던 기존 주소로 GUI를 열고 지도에서 402 initial pose를 지정한 뒤
Mission Execute를 누른다. Mission Manager는 필요한 child Action server가 하나라도
offline이면 goal을 시작 전에 거절한다.
