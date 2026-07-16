# 4층→5층→4층 주행-only 시험

## 반영된 동작

양방향 모두 다음 순서로 실행한다.

1. Nav2로 엘리베이터 앞으로 이동
2. ArUco ID 20 기준 1.27 m 정렬
3. 정렬 허용 범위 안에서 3초 연속 유지 확인
4. 운영자가 호출 및 안전 상태를 확인하고 Enter
5. Enter 확인 후 2초 정지
6. Raspberry Pi의 `/base/drive_straight` Action으로 0.27 m 전진
   (`speed_mps: 0.15`)
7. Raspberry Pi의 `/base/rotate` Action으로 좌 80도 회전
8. LiDAR로 문 열림 대기
9. ArUco ID 10 기준 50 cm까지 탑승
10. 목표 층 도착/문 열림 확인 후 운영자가 Enter
11. 하차 Action이 층 마커를 재확인한 뒤 하차, 맵 전환

5층의 `object_place`에서는 실제 미션과 같이 좌 180도 회전한 뒤
엘리베이터로 돌아오며, 마지막 목적지는 시작 위치인 402이다.

실제 미션도 같은 정렬/직진/회전/탑승 Action과 같은 수치를 사용한다.
차이는 실제 미션의 2초가 팔 Action 수락 시점부터 계산되는 반면,
팔이 없는 주행-only 시험은 정렬 완료 후 운영자가 Enter를 누른
시점부터 2초를 계산한다.

## Raspberry Pi 터미널

```bash
cd ~/Scorpy-driving_pinky
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch vicpinky_final_bringup final_robot.launch.py
```

## PC 터미널

Raspberry Pi와 같은 `ROS_DOMAIN_ID`를 사용한다.

```bash
cd ~/Scorpy-driving_server_pj
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 launch central_bringup driving_only_system.launch.py
```

이 launch는 Nav adapter, driving-only mission manager, GUI만 실행한다.
팔 노드와 `ready_and_approach` coordinator는 실행하지 않는다.

## PC Enter 확인 터미널

`driving_only_system.launch.py`와 별개인 새 PC 터미널에서 실행한다.
launch 자식 프로세스는 키보드 입력이 보장되지 않으므로 이 실행 파일은
반드시 별도 터미널에서 `ros2 run`으로 실행한다.

```bash
cd ~/Scorpy-driving_server_pj
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
ros2 run mission_manager operator_confirm_console
```

미션은 4층 정렬 후, 5층 하차 전, `object_place` 도착 후, 5층 정렬 후,
4층 하차 전의 총 다섯 지점에서 멈춘다. 위 터미널에 안내가 표시된 뒤
Enter를 누르면 다음 단계로 진행한다. Enter 대기는 시간 제한이 없지만
GUI의 Mission Cancel은 계속 동작한다. 대기 전에 누른 Enter는 버린다.

## 시험 시작

PC에서 402 초기 위치를 넣는다.

```bash
curl -sS -X POST http://127.0.0.1:8080/api/driving/initial-pose \
  -H 'Content-Type: application/json' \
  -d '{"x":2.998,"y":-12.433,"yaw":0.0,"frame_id":"map"}'
```

PC 또는 같은 네트워크의 휴대폰에서 `http://PC_IP:8080`을 열고
기본값 `402 → object_place`, `5층`을 확인한 뒤 GUI의 Start를 누른다.
Cancel도 같은 GUI에서 누른다. 터미널의 `send_mission`으로 시작하면
GUI가 해당 goal handle을 소유하지 않아 GUI Cancel을 사용할 수 없다.

사람은 안내에 맞춰 엘리베이터 버튼을 조작하고, 실제 상태를 확인한 뒤
Enter를 누른다. 하차 Action은 Enter 이후에도 층 마커를 확인하므로
잘못된 층에서 곧바로 하차하지 않는다.

상태는 다음 명령으로 볼 수 있다.

```bash
ros2 topic echo /mission/status
```

참고로 0.27 m는 Action 목표값이며 현재 odometry 완료 허용 오차는
0.03 m이다. 실측 27 cm 정밀 검증은 현장에서 odometry와 바닥 기준으로
별도 측정해야 한다.
