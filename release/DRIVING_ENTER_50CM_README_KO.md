# 주행 Enter 대기 + 엘리베이터 50cm 배포 안내

## 포함 패키지

- `pc/mission_manager`: PC의 `src/mission_manager`에 교체
- `pinky/vicpinky_task_servers`: Raspberry Pi의
  `src/vicpinky_task_servers`에 교체

## 빌드

PC:

```bash
cd ~/fp_ws/final_project
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mission_manager
source install/setup.bash
```

Raspberry Pi:

```bash
cd ~/fp_ws/final_project
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vicpinky_task_servers
source install/setup.bash
```

## 실행 순서

Raspberry Pi:

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch vicpinky_final_bringup final_robot.launch.py
```

PC 터미널 1:

```bash
cd ~/fp_ws/final_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch central_bringup driving_only_system.launch.py
```

PC 터미널 2(Enter 전용, 미션 시작 전에 반드시 실행):

```bash
cd ~/fp_ws/final_project
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 run mission_manager operator_confirm_console
```

확인:

```bash
ros2 action list | grep /operator/confirm
```

## 미션 시작

GUI의 Start 또는 다음 HTTP 요청 중 하나를 사용한다. 둘 다 동일한
driving-only mission flow를 실행한다.

```bash
curl -sS -X POST http://127.0.0.1:8080/api/mission/start \
  -H 'Content-Type: application/json' \
  -d '{
    "mission_id":"driving_test_001",
    "pickup_location":"402",
    "delivery_location":"object_place",
    "target_floor":5,
    "object_label":"object_1",
    "arm_task_name":"deliver_object_1_from_tray"
  }'
```

## 반영 동작

다음 다섯 곳에서 시간 제한 없이 Enter를 기다린다. Mission Cancel은
계속 작동하며, 안내 전에 누른 Enter는 다음 단계로 전달되지 않는다.

1. 4층 엘리베이터 정렬 완료 후
2. 5층 도착 및 문 열림 확인 후
3. `object_place` 도착 후
4. 5층 엘리베이터 정렬 완료 후
5. 4층 도착 및 문 열림 확인 후

엘리베이터 탑승은 ID 10 ArUco 마커로부터 50cm에서 정지한다.
주행 전용 흐름, 전체 미션 설정, Raspberry Pi 실동작 기본값에 모두
50cm가 반영되어 있다.
