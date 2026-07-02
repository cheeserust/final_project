# Hardware Bring-Up Guide

이 문서는 `vcan0 + board1_simulator`가 아니라 실제 Raspberry Pi/PC와
STM32 Board1/Board2/Board3를 CAN으로 연결해서 팔과 그리퍼를 테스트하는
절차다.

## 1. 실제 연결 구조

```text
MoveIt2 / CLI / Mission Manager
        |
        | ROS 2 FollowJointTrajectory / Service / Action
        v
arm_can_bridge
        |
        | SocketCAN can0
        v
CAN bus
        |
        +-- Board1: arm_joint_1 ~ arm_joint_4
        +-- Board2: base_joint
        +-- Board3: gripper 9 joints
```

현재 중앙서버 기준 arm mapping은 아래와 같다.

| Joint | Board | Local Motor ID | Command | Status | Feedback |
|---|---:|---:|---:|---:|---:|
| `base_joint` | 2 | 0 | `0x102` | `0x202` | `0x302` |
| `arm_joint_1` | 1 | 0 | `0x101` | `0x201` | `0x301` |
| `arm_joint_2` | 1 | 1 | `0x101` | `0x201` | `0x301` |
| `arm_joint_3` | 1 | 2 | `0x101` | `0x201` | `0x301` |
| `arm_joint_4` | 1 | 3 | `0x101` | `0x201` | `0x301` |
| gripper 9축 | 3 | 0~8 | `0x103` | `0x203` | `0x303` |

## 2. 하드웨어 배선 체크

CAN bus는 모든 보드가 같은 두 선을 공유한다.

```text
Raspberry Pi / USB-CAN CAN_H  -> STM Board1/2/3 CAN_H
Raspberry Pi / USB-CAN CAN_L  -> STM Board1/2/3 CAN_L
GND                           -> STM Board1/2/3 GND
```

확인할 것:

1. CAN_H와 CAN_L이 뒤집히지 않았는지 확인한다.
2. CAN bus 양 끝에 120 ohm termination이 있는지 확인한다.
3. Raspberry Pi/PC와 STM 보드들의 GND가 공통인지 확인한다.
4. 모터 전원과 제어 보드 전원 용량이 충분한지 확인한다.
5. 처음 테스트할 때는 모터가 공중에서 안전하게 움직일 수 있게 둔다.
6. 비상정지 또는 전원 차단 방법을 바로 사용할 수 있게 준비한다.

## 3. 실제 CAN interface 준비

`can-utils`가 없으면 설치한다.

```bash
sudo apt update
sudo apt install can-utils
```

`can0`를 500 kbps로 올린다.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

CAN frame을 확인할 터미널을 하나 열어둔다.

```bash
candump can0
```

정상이라면 STM 보드가 주기적으로 아래 frame을 보내야 한다.

```text
0x201  Board1 status
0x202  Board2 status
0x203  Board3 status
0x301  Board1 actual position feedback
0x302  Board2 actual position feedback
0x303  Board3 actual position feedback
```

## 4. 빌드

```bash
cd ~/vicpinky_server_ws
colcon build --symlink-install --packages-select \
  arm_can_bridge \
  roscue_arm_description \
  roscue_arm_moveit_config \
  mission_manager \
  vicpinky_nav_adapter \
  vicpinky_gui
source install/setup.bash
```

GUI를 쓸 경우 Flask가 필요하다.

```bash
sudo apt install python3-flask
```

## 5. 실제 STM으로 팔/그리퍼만 테스트

중요: 실제 STM을 붙일 때는 `board1_simulator`를 실행하지 않는다.

터미널 1, CAN bridge:

```bash
cd ~/vicpinky_server_ws
source install/setup.bash
ros2 launch arm_can_bridge arm_can_bridge.launch.py can_interface:=can0
```

터미널 2, RViz:

```bash
cd ~/vicpinky_server_ws
source install/setup.bash
ros2 launch roscue_arm_description bridge_display.launch.py
```

터미널 3, board 상태 확인:

```bash
cd ~/vicpinky_server_ws
source install/setup.bash
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
```

처음에는 보통 `enabled=False`, `ready=0x00`일 수 있다.

Enable:

```bash
ros2 service call /arm_board/enable std_srvs/srv/Trigger '{}'
```

Homing:

```bash
ros2 service call /arm_board/home_all std_srvs/srv/Trigger '{}'
```

다시 status 확인:

```bash
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
```

정상 기준:

```text
board2: enabled=True, ready=0x01
board1: enabled=True, ready=0x0F
board3: enabled=True, ready=0x01
accept_traj=True
```

## 6. 아주 작은 이동부터 테스트

실제 하드웨어에서는 큰 각도를 바로 보내지 말고, 1~2도부터 확인한다.

팔 base joint만 +2도:

```bash
ros2 run arm_can_bridge send_arm_pose \
  --relative-degrees 2 0 0 0 0 \
  --duration 3.0
```

다시 -2도:

```bash
ros2 run arm_can_bridge send_arm_pose \
  --relative-degrees -2 0 0 0 0 \
  --duration 3.0
```

그리퍼 조금 닫기:

```bash
ros2 run arm_can_bridge send_gripper_pose \
  --close --step 5 --duration 2.0
```

그리퍼 조금 열기:

```bash
ros2 run arm_can_bridge send_gripper_pose \
  --open --step 5 --duration 2.0
```

## 7. MoveIt2와 연결해서 테스트

`arm_can_bridge`가 아래 Action server를 제공한다.

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

MoveIt2 controller 설정의 action name이 이 이름과 맞아야 한다.

MoveIt2 실행:

```bash
cd ~/vicpinky_server_ws
source install/setup.bash
ros2 launch roscue_arm_moveit_config move_group.launch.py
```

MoveIt RViz를 따로 띄울 경우:

```bash
ros2 launch roscue_arm_moveit_config moveit_rviz.launch.py
```

주의:

1. `arm_can_bridge`가 먼저 떠 있어야 trajectory action을 받을 수 있다.
2. `/joint_states`가 들어와야 MoveIt2가 현재 상태를 안다.
3. trajectory 첫 point가 현재 `/joint_states`와 크게 다르면 reject될 수 있다.
4. 현재 허용 오차는 `arm_can_bridge.yaml`의 `start_position_tolerance_rad`다.

## 8. 전체 미션으로 붙일 때

실제 미션 운용은 아래 노드들이 함께 떠야 한다.

```text
VicPinky Nav2
MoveIt2
arm_can_bridge
mission_manager
vicpinky_nav_adapter
vicpinky_gui
```

예시:

```bash
ros2 launch arm_can_bridge arm_can_bridge.launch.py can_interface:=can0
ros2 launch mission_manager mission_manager.launch.py
ros2 launch vicpinky_nav_adapter nav_adapter.launch.py
ros2 launch vicpinky_gui vicpinky_gui.launch.py port:=8081
```

실제 주행팀 Nav2 launch도 별도 터미널에서 실행되어야 한다.

## 9. 저지연 PC/Raspberry Pi 분산 구성

RViz를 실제 운용에서 쓰지 않고, GUI와 MoveIt2는 PC에서 돌리고,
로봇에 직접 붙는 노드는 Raspberry Pi에서 돌리는 구성을 권장한다.

### 9.1 권장 배치

```text
PC / 노트북
  - MoveIt2 move_group
  - MoveIt2 planning client 또는 자체 조작 UI
  - vicpinky_gui
  - 개발/모니터링 터미널

Raspberry Pi / 로봇 내부
  - arm_can_bridge
  - mission_manager
  - vicpinky_nav_adapter
  - VicPinky Nav2 / 주행팀 node
  - SocketCAN can0
  - STM32 Board1/2/3와 CAN 통신
```

실시간에 가까운 경로는 Raspberry Pi 안에서 끝나야 한다.

```text
arm_can_bridge -> can0 -> STM32
```

PC에서 넘어가는 것은 대부분 "trajectory goal" 또는 "mission goal"이다.
이건 매 1ms마다 제어하는 신호가 아니라 goal 단위 명령이므로, 유선 LAN이면
지연 부담이 작다.

### 9.2 실제 데이터 흐름

팔 제어:

```text
PC MoveIt2
  -> /arm_controller/follow_joint_trajectory
  -> Raspberry Pi arm_can_bridge
  -> can0
  -> STM32 Board1/2
  -> motor
```

그리퍼 제어:

```text
PC MoveIt2 또는 GUI/CLI
  -> /gripper_controller/follow_joint_trajectory
  -> Raspberry Pi arm_can_bridge
  -> can0
  -> STM32 Board3
  -> servo
```

상태 feedback:

```text
STM32
  -> 0x201/0x202/0x203 status
  -> 0x301/0x302/0x303 actual position
  -> Raspberry Pi arm_can_bridge
  -> /joint_states
  -> PC MoveIt2 / GUI
```

### 9.3 네트워크 설정

PC와 Raspberry Pi는 같은 유선 LAN에 두는 것을 권장한다. Wi-Fi도 동작은
가능하지만 지연과 packet loss가 튈 수 있다.

양쪽에서 같은 값을 사용한다.

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

둘 다 같은 DDS middleware를 쓰는 것이 좋다. 예를 들어 CycloneDDS를 쓴다면
양쪽 모두 동일하게 설정한다.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

네트워크 확인:

```bash
# Raspberry Pi에서
hostname -I

# PC에서
ping <raspberry_pi_ip>
ros2 node list
ros2 topic list
```

PC에서 `/joint_states`, `/arm_board/status_log`,
`/arm_controller/follow_joint_trajectory`가 보여야 한다.

### 9.4 지연시간 최소화 체크리스트

1. PC와 Raspberry Pi를 유선 Ethernet으로 연결한다.
2. Raspberry Pi에서 RViz, 브라우저, 무거운 GUI를 돌리지 않는다.
3. Raspberry Pi에서 `arm_can_bridge`, Nav2, mission 관련 필수 node만 돌린다.
4. CAN을 쓰는 `arm_can_bridge`는 반드시 Raspberry Pi에서 돌린다.
5. `board1_simulator`는 실제 운용에서 끈다.
6. PC GUI는 관제/명령 입력용으로만 쓰고, 제어 loop 안에 넣지 않는다.
7. STM32가 실제 모터 제어와 1ms tick 보간을 담당하게 한다.
8. `candump can0 -tz`로 Board1 4-frame 묶음이 20ms 안에 나가는지 확인한다.
9. Raspberry Pi CPU governor를 performance로 두면 지터를 줄일 수 있다.

CPU governor 설정 예시:

```bash
sudo apt install linux-cpupower
sudo cpupower frequency-set -g performance
```

`cpupower`가 없는 Raspberry Pi OS에서는 아래처럼 확인한다.

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

### 9.5 CAN 지연 확인

터미널 하나에서 timestamp와 함께 CAN frame을 본다.

```bash
candump can0 -tz
```

Board1 명령 `0x101`은 한 trajectory point마다 local motor `0 -> 1 -> 2 -> 3`
순서로 4 frame이 나가야 한다. 이 4 frame 사이가 20ms 안이면 정상이다.

예상:

```text
0x101 motor 0
0x101 motor 1
0x101 motor 2
0x101 motor 3
```

보통은 20ms보다 훨씬 짧게 붙어서 보여야 한다.

### 9.6 지연이 심할 때 우선순위

가장 먼저 바꿀 것:

```text
Wi-Fi -> 유선 LAN
Raspberry Pi에서 RViz/브라우저 종료
PC와 Raspberry Pi의 ROS_DOMAIN_ID/RMW 설정 통일
CAN bus error frame 확인
Raspberry Pi CPU governor performance 설정
```

그래도 문제가 있으면:

```text
1. MoveIt2 planning은 PC에 둔다.
2. arm_can_bridge는 Raspberry Pi에 둔다.
3. mission_manager도 Raspberry Pi에 둔다.
4. GUI는 PC에 둔다.
5. CAN frame 송신 간격은 candump로 실제 측정한다.
```

MoveIt2가 trajectory 전체를 action goal로 보내는 구조라면 PC에서 돌려도
괜찮다. 하지만 MoveIt Servo처럼 아주 빠른 주기로 servo command를 계속
streaming하는 구조를 쓰게 되면, 그때는 servo 관련 node를 Raspberry Pi에
가깝게 두는 구성을 다시 검토한다.

## 10. 문제 확인 순서

### Service가 trajectory를 거부한다

```bash
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
```

확인할 항목:

```text
enabled=True
ready=...
error=NONE 또는 ERR_NONE
stale=False
position_valid=True
accept_traj=True
```

### status가 stale이다

STM의 `0x201/0x202/0x203` status frame이 안 들어오거나 너무 늦다.

```bash
candump can0
```

에서 status frame이 보이는지 확인한다.

### RViz에서 순간이동처럼 보인다

STM이 actual feedback에 목표각만 보내고 있을 가능성이 있다.

Board1/2/3는 이동 중에도 현재 중간 각도를 계속 보내야 한다.

```text
Board1: 0x301
Board2: 0x302
Board3: 0x303
```

### CAN 자체가 안 보인다

1. `ip -details link show can0`
2. bitrate 500000 확인
3. CAN_H/CAN_L 배선 확인
4. termination 확인
5. GND 공통 확인
6. STM firmware가 주기 status를 보내는지 확인

## 11. 실제 연결 핵심 요약

```text
1. board1_simulator는 끈다.
2. can0를 500 kbps로 올린다.
3. STM Board1/2/3가 0x201/0x202/0x203 status를 보내는지 candump로 본다.
4. arm_can_bridge를 can_interface:=can0로 실행한다.
5. /arm_board/enable -> /arm_board/home_all -> /arm_board/status 순서로 준비한다.
6. accept_traj=True 확인 후 작은 각도부터 움직인다.
7. RViz는 bridge_display.launch.py로 /joint_states를 본다.
```
