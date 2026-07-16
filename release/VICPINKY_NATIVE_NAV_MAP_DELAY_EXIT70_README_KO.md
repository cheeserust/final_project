# VicPinky 자체 주행 서버용 배포 안내

이 묶음은 중앙 PC의 `vicpinky_nav_adapter`를 사용하지 않고, Pinky의
`vicpinky_task_servers/nav_go_to_server`가 `/nav/go_to`를 제공하는 구성용이다.

## 변경 내용

- 5층 ID 5 하차 정지거리: 60 cm -> 70 cm
- 4층 ID 4 하차 정지거리: 60 cm -> 70 cm
- 5층 `SWITCH_5F_MAP` 성공 후 첫 Nav2 goal 전송 전 3초 대기
- 4층 `SWITCH_4F_MAP` 성공 후 첫 Nav2 goal 전송 전 3초 대기
- ID 10 탑승 거리, ID 20 정렬 및 나머지 ArUco 동작은 변경하지 않음

## ZIP 구성

- `central/mission_manager`: 거리와 지연 값을 RunTask `extra_json`으로 전달
- `pinky/vicpinky_task_servers`: 하차 거리 적용 및 Pinky 자체 Nav2 goal 지연

ROS interface 변경은 없으므로 `vicpinky_interfaces`를 교체할 필요는 없다.

## 중앙 PC 배포

기존 workspace의 `src/mission_manager`를 ZIP의 패키지로 교체한 뒤 빌드한다.

```bash
colcon build --symlink-install --packages-select mission_manager
source install/setup.bash
```

## Pinky PC 배포

기존 workspace의 `src/vicpinky_task_servers`를 ZIP의 패키지로 교체한 뒤
빌드한다.

```bash
colcon build --symlink-install --packages-select vicpinky_task_servers
source install/setup.bash
```

Pinky 운용 launch에서 `nav_go_to_server`가 실행되어야 한다. 제공된 패키지의
실행 파일 이름은 다음과 같다.

```bash
ros2 run vicpinky_task_servers nav_go_to_server --ros-args -p mock_mode:=false
```

기존 Pinky 통합 launch가 이 노드를 이미 실행한다면 별도로 중복 실행하지
않는다.

## 중복 서버 확인

`/nav/go_to` Action Server는 반드시 하나만 실행해야 한다.

```bash
ros2 action info /nav/go_to -v
ros2 node list | grep -E 'vicpinky_nav_adapter|nav_go_to_server'
```

Pinky 자체 서버를 사용하는 이 구성에서는 `nav_go_to_server`만 보여야 하며
`vicpinky_nav_adapter`는 실행하지 않는다.

## 적용 확인 로그

5층과 4층의 맵 전환 뒤 다음 로그가 나와야 한다.

```text
Waiting 3.0s before Nav2 goal: object_place
Waiting 3.0s before Nav2 goal: 402
```

하차 완료 거리는 ID 5와 ID 4에서 약 70 cm로 표시되어야 한다.

```text
하차 완료 (70.xcm) -> 좌 90도 회전
```

첫 현장 시험에서는 RViz의 `/scan_filtered`, AMCL pose, footprint 및 global
costmap을 함께 확인한다. 마커와의 거리는 10 cm 늘었지만 기존 고정
`/initialpose`와 실제 정지 위치 사이에도 약 10 cm 차이가 추가될 수 있다.
