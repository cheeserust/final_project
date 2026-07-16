# Arm Homing 이후 INVALID_CMD 방지

## 원인

Board1 firmware의 motor0 기계 home은 `-86.5°` (`-8650`)지만 일반 위치 명령의
최소 허용값은 `-85.0°` (`-8500`)다. 기존 서버는 Homing 직후 3초짜리 ready 이동을
20 ms 단위로 보간해 첫 목표 `-86.02°`를 만들었다. 이 값은 firmware 위치 명령
범위 밖이므로 Board1이 정상적으로 `INVALID_CMD`를 반환한 것이다. `candump` 상태가
계속 보였던 이유도 통신 단절이 아니라 명령 거부였기 때문이다.

## 적용한 권장 동작

1. `/arm_board/home_all`은 Board1~3 Homing 완료를 기다린다.
2. J1이 일반 명령 범위 밖이면 Board1 한 batch만 전송한다.
3. 목표 raw는 motor0~3 순서로 `[-8500, -7810, -9150, -9000]`이다.
4. duration은 `60 tick = 300 ms`, frame 간격은 `7 ms`다.
5. batch 완료를 확인한 뒤에만 Homing service를 성공 처리한다.
6. 이후 일반 arm trajectory는 point당 최소 `8 tick = 40 ms`로 재시간화한다.
7. 모든 일반 CAN 목표는 sign/offset 변환과 0.01° 정수 변환을 끝낸 raw 값으로
   firmware min/max 검사를 통과해야 송신된다.

## 바꾸지 않은 항목

- 네트워크, ROS Domain ID, RMW
- VicPinky URDF와 Pinky 좌표
- LiDAR driver, udev, port, frame, filter
- ROS/MoveIt 관절 모델 범위
- Arm CAN frame 간격 `7 ms`

## 실기 확인

```bash
# 중앙 PC 터미널 1
candump -tz can0

# 중앙 PC 터미널 2
source /opt/ros/jazzy/setup.bash
source ~/fp_ws/final_project/install/setup.bash
ros2 service call /arm_board/home_all std_srvs/srv/Trigger "{}"
```

성공 조건은 service message에 `Homing and post-home escape confirmed`가 보이고,
Board1 상태가 `ERROR/INVALID_CMD`로 바뀌지 않는 것이다. 그 다음 `arm_ready`만
단독 실행해 첫 정상 궤적도 확인한 후 전체 mission을 실행한다.
