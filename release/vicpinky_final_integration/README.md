# VicPinky 최종 통합 배포본

이 폴더 하나에 중앙 PC용 `pj`와 Raspberry Pi용 `pinky`를 분리했다.
두 장비의 `vicpinky_interfaces`는 동일하며, 중앙 PC의 Mission Execute 한 번으로
402 → 5층 배송 → 402 복귀의 29단계를 실행한다.

## 폴더

- `pj/`: MoveIt2, 팔 CAN bridge, wrist RealSense, Mission Manager, GUI, Nav2 adapter
- `pinky/`: 모터, LiDAR, Nav2, 전·후방 카메라, 주행·엘리베이터 task server
- `docs/FINAL_MISSION_SCENARIO.md`: 실제 실행 순서와 좌표·관절각·마커 표
- `docs/PREFLIGHT_CHECKLIST.md`: 최초 실기 실행 전 필수 확인
- `docs/ARM_POST_HOME_SAFETY.md`: Board1 `INVALID_CMD` 원인과 실기 확인 절차

## 최초 설치

두 장비 모두 Ubuntu 24.04와 ROS 2 Jazzy 기준이다. 압축을 푼 뒤 각 장비에서
해당 폴더로 이동한다.

Raspberry Pi:

```bash
cd pinky
./install_dependencies.sh
./build.sh
```

중앙 PC:

```bash
cd pj
./install_dependencies.sh
./build.sh
```

네트워크, ROS Domain ID, RMW 설정은 팀에서 이미 구성한 기존 환경을 그대로
사용한다. 이 배포본의 스크립트는 관련 환경변수를 설정하거나 덮어쓰지 않는다.
VicPinky URDF와 기존 LiDAR driver·udev·port·frame 설정도 수정하거나 포함하지 않는다.

## 실행 순서

1. Raspberry Pi에서 기존 팀 LiDAR workspace를 source한 뒤 `./run.sh`.
2. 중앙 PC에서 SocketCAN `can0`를 500 kbit/s로 올린다.
3. 중앙 PC에서 `./run.sh`.
4. 중앙 PC에서 `./check_runtime.sh`로 필수 action/service를 확인한다.
5. 외부 PC에서 팀이 사용하던 기존 주소로 GUI에 접속.
6. GUI 지도에서 402 initial pose를 지정하고 `Mission Execute`를 한 번 누른다.

SocketCAN 예시:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

`pj`나 `pinky` 폴더의 **내용만** `~/fp_ws/final_project`로 복사했다면 상위
`run.sh`는 함께 복사되지 않는다. 이 경우 빌드 후 다음 launch를 직접 실행한다.

```bash
# Raspberry Pi
source /opt/ros/jazzy/setup.bash
source ~/fp_ws/final_project/install/setup.bash
ros2 launch vicpinky_final_bringup final_robot.launch.py

# 중앙 PC
source /opt/ros/jazzy/setup.bash
source ~/fp_ws/final_project/install/setup.bash
ros2 launch central_bringup final_system.launch.py \
  execution_mode:=hardware can_interface:=can0
```

## 2026-07-11 팔 안전 보정

- Arm CAN frame 간격은 기존 요청대로 `7 ms`를 유지한다.
- 일반 arm 궤적의 각 CAN point는 최소 `8 tick = 40 ms`로 재시간화한다.
- 기계 Homing의 J1 `-86.5°`는 firmware 명령 최소값 `-85°` 밖이므로,
  `/arm_board/home_all` 안에서 Board1을 `[-85.0, -78.1, -91.5, -90.0]°`로
  `300 ms` 동안 먼저 이동한 뒤 Homing 성공을 반환한다.
- ROS/MoveIt 모델 한계는 바꾸지 않고, 실제 CAN raw 목표에 firmware min/max를
  별도로 적용해 범위 밖 프레임은 송신 전에 거부한다.
- 네트워크·ROS Domain ID·VicPinky URDF·LiDAR 설정은 변경하지 않았다.

## 검증 범위

- 중앙 PC 10개 패키지 격리 빌드 성공
- Pinky 6개 패키지 격리 빌드 성공(기존 팀 LiDAR driver는 배포본에서 제외)
- 이번 팔 안전 변경 패키지 테스트 123개 통과, 2개 copyright 검사 skip
- 중앙 통합 174개 테스트에서 failure 0, Pinky motion-safety 4개 테스트 통과
- LiDAR 문 열림 gate를 포함한 29단계 mock Action E2E가 `DONE`으로 완료
- Xacro/URDF, YAML, XML, Python entrypoint, OpenCV 4.6 fallback 검증

실제 카메라·STM·모터·엘리베이터가 이 개발 PC에 없어 실물 동작까지 대신 검증할
수는 없다. 특히 최초 실기는 저속·비상정지 대기 상태에서
`docs/PREFLIGHT_CHECKLIST.md`의 현장 보정 항목을 확인해야 한다.
