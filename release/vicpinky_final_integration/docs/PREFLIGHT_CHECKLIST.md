# 최초 실기 실행 체크리스트

## 공통

- [ ] 팀의 기존 네트워크·Domain ID·RMW 환경을 활성화했다.
- [ ] `pj/run.sh`와 `pinky/run.sh`가 해당 값을 덮어쓰지 않는 것을 확인했다.
- [ ] 비상정지와 수동 전원 차단 담당자가 로봇 옆에 있다.

## Pinky

- [ ] 모터와 LiDAR가 팀의 기존 장치·udev·port·frame 설정으로 동작한다.
- [ ] 전방 `/dev/video0`, 후방 `/dev/video2`가 실제 장치와 맞다.
- [ ] `/scan_filtered`, `/odom`, `/front_camera/image_raw`,
  `/rear_camera/image_raw`가 연속 갱신된다.
- [ ] `/elevator/wait_door_open`이 `/scan_filtered`로 실제 문 열림을 판정한다.
- [ ] 4F/5F map과 AMCL initial pose를 실제 엘리베이터 하차 위치에서 확인했다.
- [ ] ID 20 정렬 1.37 m, ID 10 탑승 0.35 m에서 수동 정지 시험을 했다.
- [ ] odom 입력을 끊었을 때 직진·회전 action이 0.5초 안에 정지·실패한다.

## 팔·카메라·CAN

- [ ] `can0`가 500 kbit/s, `UP` 상태이고 STM 1/2/3 응답이 정상이다.
- [ ] `candump -tz can0`에서 0x201/0x202/0x203 상태가 끊기지 않는다.
- [ ] 중앙 PC의 `arm_inter_frame_delay_ms`가 `7.0`,
  `arm_trajectory_min_duration_ticks`가 `8`인지 확인했다.
- [ ] `/arm_board/home_all` 성공 메시지가
  `Homing and post-home escape confirmed`이고, J1이 기계 home `-86.5°`에서
  firmware 명령 가능 경계 `-85.0°`로 먼저 이동한다.
- [ ] Homing 직후 첫 0x101 escape batch가 Board1 motor 0~3만 포함하고,
  목표 raw가 `[-8500, -7810, -9150, -9000]`, duration byte가 `60`인지
  저속·무부하 상태에서 확인했다.
- [ ] escape 완료 후 `arm_ready`를 단독 실행했을 때 Board1에
  `INVALID_CMD`, `ERROR`, `ESTOP`이 발생하지 않는다.
- [ ] 팔 주변과 적재판·버튼 사이에 사람이 없고 충돌물이 없다.
- [ ] `base_link → arm_base_link`, wrist camera, button TCP 실측 장착값을 확인했다.
- [ ] ID 50~54 검출 pose가 흔들리지 않고 TF 변환된다.
- [ ] Homing을 취소하면 `/arm_board/estop`이 실행되고 실제 모터가 멈춘다.
- [ ] `arm_ready`, 적재판 drop/pick을 각각 수동 저속 시험했다.
- [ ] 닫힌 손가락 중심 `grasp_tcp_link` 높이 0.134295 m를 실물로 확인했다.
- [ ] 버튼 접근 방향을 RViz와 저속 plan으로 확인했다.

CAN 확인 중 `candump`가 정상인데 GUI가 빨강/초록으로 깜빡이면 통신선 자체보다
bridge status timeout, 여러 bridge 중복 실행, 또는 보드가 명령을 거부해 ERROR로
전환되는지 먼저 확인한다. `can0`를 임의로 재설정하거나 frame 간격을 7 ms보다
줄이지 않는다.

## 제공 자료에서 현장 확정이 필요한 값

- `observe_cabin_5_button` 각도가 없어 cabin 4 관찰 자세를 재사용한다.
- cabin 자세 J3 `+91.5°`는 기구 한계 때문에 `+90°`로 제한했다.
- 배송 장소 도착 후 회전은 `180°`로 설정했다.
- 5축 팔의 IK는 position-only다. 버튼 법선 방향은 named observe pose와 실제
  plan을 최초 현장에서 확인한 뒤 운영해야 한다.
- map switch seed는 Pinky `nav_points.yaml`의 `elevator_front`와 동기화했지만,
  실제 하차 위치가 다르면 두 파일을 같은 좌표로 함께 보정한다.

위 항목이 하나라도 확인되지 않으면 GUI Mission Execute를 누르지 않는다.
