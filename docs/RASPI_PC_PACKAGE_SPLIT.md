# Raspberry Pi 5 / PC 패키지 분리 구성

## 기준 구조

Arm/카메라 통합은 아래 배치를 고정 기준으로 사용한다.

```text
RealSense
  -> Raspberry Pi 5: realsense2_camera + aruco_detector_node
  -> LAN: DetectedMarker(ID + pose + image timestamp/frame)
  -> Central PC: task_executor + move_group + arm_can_bridge
  -> USB-CAN can0
  -> STM32 Board1/Board2/Board3
```

원본 color image는 RPi 안에서만 처리한다. PC는 marker 결과만 구독한다.

## 패키지 배치

| 장비 | 패키지/노드 |
| --- | --- |
| RPi | `vicpinky_interfaces`, `roscue_arm_pick/aruco_detector_node`, `realsense2_camera` |
| PC | `vicpinky_interfaces`, `roscue_arm_description`, `roscue_arm_moveit_config`, `roscue_arm_pick/task_executor_node` |
| PC | `arm_can_bridge`, `mission_manager`, `central_bringup`, `vicpinky_gui` 선택 |

`arm_task_server`는 deprecated 호환 패키지다. 운영 중에는 실행하지 않는다.

## RPi 빌드와 실행

```bash
sudo apt install ros-jazzy-realsense2-camera ros-jazzy-cv-bridge

cd ~/vicpinky_server_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select vicpinky_interfaces roscue_arm_pick
source install/setup.bash

ros2 launch roscue_arm_pick aruco_perception.launch.py
```

RPi에서 확인한다.

```bash
ros2 topic hz /detected_marker
ros2 topic echo /detected_marker --once
```

## PC 빌드와 실행

```bash
cd ~/vicpinky_server_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

처음에는 반드시 plan-only로 실행한다.

```bash
ros2 launch central_bringup arm_hardware_bringup.launch.py \
  can_interface:=can0 \
  execution_mode:=plan_only \
  use_rviz:=false
```

실측 보정을 마치고 구성 검사를 통과한 뒤에만 hardware로 바꾼다.

```bash
ros2 launch central_bringup arm_hardware_bringup.launch.py \
  can_interface:=can0 \
  execution_mode:=hardware \
  use_rviz:=false
```

## 기존 통신 환경

네트워크, 접속 주소, ROS Domain ID와 RMW는 팀이 이미 구성한 값을 그대로 쓴다.
이 문서와 실행 코드에서는 해당 값을 새로 지정하거나 변경하지 않는다.

PC에서 다음 항목이 보여야 한다.

```text
/detected_marker
/move_action
/arm/execute
/arm/pick
/arm/place
/arm/press_button
/arm/homing
/arm_controller/execute_joint_goal
/gripper_controller/follow_joint_trajectory
/joint_states
```

## 보드 매핑

실행 기준은 `arm_can_bridge/config/arm_can_bridge.yaml`이다.

| Board | Joint | Local motor |
| --- | --- | --- |
| Board1 | `arm_joint_1`, `arm_joint_2`, `arm_joint_3`, `base_joint` | `0`, `1`, `2`, `3` |
| Board2 | `arm_joint_4` | `0` |
| Board3 | gripper 9축 | `0~8` |

세부 빌드, TF, 보정, plan-only 및 현장 검증 순서는
[MOVEIT_ARM_INTEGRATION.md](MOVEIT_ARM_INTEGRATION.md)를 따른다.
