# Arm CAN Protocol

이 문서는 VicPinky 중앙서버와 STM32 Board1/Board2/Board3 사이의 CAN 프로토콜이다.

## 1. Board 구성

| Board ID | 대상 | Move CAN ID | Status CAN ID | Payload Motor ID |
| ---: | --- | ---: | ---: | --- |
| `1` | Board1, 팔 1~4축 step motor | `0x101` | `0x201` | `0~3` |
| `2` | Board2, 팔 5축 step motor | `0x102` | `0x202` | `0` |
| `3` | Board3, gripper servo 9개 | `0x103` | `0x203` | `0~8` |

MoveIt2와 중앙서버 사이의 ROS 2 Action은 두 개다.

| MoveIt controller | Action | 담당 Board |
| --- | --- | --- |
| `arm_controller` | `/arm_controller/follow_joint_trajectory` | Board1, Board2 |
| `gripper_controller` | `/gripper_controller/follow_joint_trajectory` | Board3 |

서버 내부 global joint id와 CAN payload motor id는 분리한다.

| Global ID | Joint | Board ID | CAN ID | Motor ID |
| ---: | --- | ---: | ---: | ---: |
| `0` | `base_joint` | `1` | `0x101` | `0` |
| `1` | `arm_joint_1` | `1` | `0x101` | `1` |
| `2` | `arm_joint_2` | `1` | `0x101` | `2` |
| `3` | `arm_joint_3` | `1` | `0x101` | `3` |
| `4` | `arm_joint_4` | `2` | `0x102` | `0` |
| `5` | `finger_1_base_joint` | `3` | `0x103` | `0` |
| `6` | `finger_1_middle_joint` | `3` | `0x103` | `1` |
| `7` | `finger_1_tip_joint` | `3` | `0x103` | `2` |
| `8` | `finger_2_base_joint` | `3` | `0x103` | `3` |
| `9` | `finger_2_middle_joint` | `3` | `0x103` | `4` |
| `10` | `finger_2_tip_joint` | `3` | `0x103` | `5` |
| `11` | `finger_3_base_joint` | `3` | `0x103` | `6` |
| `12` | `finger_3_middle_joint` | `3` | `0x103` | `7` |
| `13` | `finger_3_tip_joint` | `3` | `0x103` | `8` |

## 2. Joint Limit

중앙서버와 MoveIt2는 같은 joint limit을 사용해야 한다. CAN payload의 target position은 radian이 아니라 0.01 degree 단위로 변환되어 전송된다.

### Arm

| Joint | Min | Max | Home |
| --- | ---: | ---: | ---: |
| `base_joint` | -90 deg | 180 deg | -90 deg |
| `arm_joint_1` | -90 deg | 90 deg | -90 deg |
| `arm_joint_2` | -80 deg | 80 deg | -80 deg |
| `arm_joint_3` | -90 deg | 90 deg | -90 deg |
| `arm_joint_4` | -170 deg | 170 deg | -170 deg |

### Gripper

세 손가락은 같은 limit을 사용한다.

| Joint type | Joint 예시 | Min | Max | 방향 |
| --- | --- | ---: | ---: | --- |
| base | `finger_1_base_joint` | -70.3 deg | 70.3 deg | 손 안쪽 방향(-) ~ 손 바깥 방향(+) |
| middle | `finger_1_middle_joint` | -137.7 deg | 52.7 deg | 손 안쪽 방향(-) ~ 손 바깥 방향(+) |
| tip | `finger_1_tip_joint` | -111.3 deg | 111.3 deg | 손 안쪽 방향(-) ~ 손 바깥 방향(+) |

적용 대상:

```text
finger_1_base_joint, finger_2_base_joint, finger_3_base_joint
finger_1_middle_joint, finger_2_middle_joint, finger_3_middle_joint
finger_1_tip_joint, finger_2_tip_joint, finger_3_tip_joint
```

## 3. CAN ID

### Move / Servo Command

| CAN ID | 방향 | 용도 |
| ---: | --- | --- |
| `0x101` | RPi -> Board1 | Board1 4축 position command |
| `0x102` | RPi -> Board2 | Board2 5축 position command |
| `0x103` | RPi -> Board3 | Board3 servo command |

### Status

| CAN ID | 방향 | 용도 |
| ---: | --- | --- |
| `0x201` | Board1 -> RPi | Board1 status |
| `0x202` | Board2 -> RPi | Board2 status |
| `0x203` | Board3 -> RPi | Board3 status |

### Common Control

| CAN ID | 방향 | 용도 |
| ---: | --- | --- |
| `0x001` | RPi -> STM32 | Emergency Stop |
| `0x010` | RPi -> STM32 | Enable / Disable |
| `0x020` | RPi -> STM32 | Homing Start |
| `0x030` | RPi -> STM32 | Clear Error |

Common control command는 CAN ID가 보드별로 분리되지 않는다. 따라서 payload 첫 byte에 `board_id`를 넣는다.

| Board ID | 의미 |
| ---: | --- |
| `1` | Board1 |
| `2` | Board2 |
| `3` | Board3 |
| `255` | 전체 board broadcast |

각 STM32는 common control frame을 받으면 Byte0의 `board_id`를 확인한다.

- 자기 board id이면 처리한다.
- `255`이면 처리한다.
- 그 외 값이면 무시한다.

## 4. Position / Servo Command Payload

대상 CAN ID:

```text
Board1: 0x101
Board2: 0x102
Board3: 0x103
```

Payload는 8 byte 고정이다.

| Byte | 필드 | 자료형 | 설명 |
| ---: | --- | --- | --- |
| 0 | Control & Motor ID | `uint8_t` | 상위 4bit flags, 하위 4bit local motor id |
| 1~4 | Target Position | `int32_t` | little-endian, 0.01 degree |
| 5~6 | Speed | `uint16_t` | little-endian |
| 7 | Duration | `uint8_t` | 5ms tick |

Byte0:

```text
Bit7 Bit6 Bit5 Bit4 | Bit3 Bit2 Bit1 Bit0
Exec Rel  Step Rsv  | Motor ID
```

| Bit | 이름 | 의미 |
| ---: | --- | --- |
| 7 | Execute | `1`: 실행 / queue push |
| 6 | Relative | 현재 `0` 고정 |
| 5 | Step Mode | 현재 `0` 고정, angle mode |
| 4 | Reserved | `0` 고정 |
| 3~0 | Motor ID | board 내부 local motor id |

예시:

```text
0x80 = motor 0 실행
0x81 = motor 1 실행
0x82 = motor 2 실행
```

Target Position은 0.01 degree 단위다.

```text
3000  = 30.00 deg
9000  = 90.00 deg
-1500 = -15.00 deg
```

Duration은 5ms 단위다.

```text
duration_ms = Byte7 * 5
```

예시:

```text
Byte7 = 10  -> 50ms
Byte7 = 100 -> 500ms
Byte7 = 200 -> 1000ms
```

## 5. Board1 규칙

Board1은 팔의 `base_joint`, `arm_joint_1`, `arm_joint_2`, `arm_joint_3`을 담당한다.

```text
CAN ID: 0x101
Motor ID: 0~3
```

MoveIt2 trajectory point 하나는 아래 순서로 네 frame을 보낸다.

| 순서 | Motor ID | Byte0 |
| ---: | ---: | ---: |
| 1 | `0` | `0x80` |
| 2 | `1` | `0x81` |
| 3 | `2` | `0x82` |
| 4 | `3` | `0x83` |

Board1 STM32는 네 frame을 staging한 뒤 하나의 4축 point로 queue에 넣는다.

Staging 조건:

- Motor ID는 `0 -> 1 -> 2 -> 3` 순서
- 첫 frame 수신 후 20ms 안에 4개 frame 모두 수신
- 4개 frame의 Duration은 모두 같아야 함
- Execute=1
- Relative=0
- StepMode=0
- Reserved=0

조건을 만족하지 않으면 staging을 폐기하고 `ERR_INVALID_CMD`를 보고한다.

## 6. Board2 규칙

Board2는 팔의 `arm_joint_4` 한 축을 담당한다.

```text
CAN ID: 0x102
Motor ID: 0
```

Board2 command 조건:

- Motor ID는 `0`만 허용
- Execute=1
- Relative=0
- StepMode=0
- Reserved=0

Board1과 Board2의 완전한 하드웨어 tick 동기화는 요구하지 않는다. RPi는 같은 trajectory point 기준으로 Board1 frame 묶음과 Board2 frame을 순서대로 전송한다. CAN 전송 순서에 따른 작은 시간 차이는 허용한다.

## 7. Board3 규칙

Board3는 three-finger gripper의 서보 9개를 담당한다.

```text
CAN ID: 0x103
Motor ID: 0~8
```

Classic CAN payload는 8 byte라서 9개 서보 값을 한 frame에 담지 않는다. 서보 하나당 `0x103` frame 하나를 사용한다.

동시 제어가 필요하면 아래 순서로 보낸다.

```text
Motor ID 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8
```

Board3 staging 조건:

- Motor ID는 `0 -> 1 -> ... -> 8` 순서
- 첫 frame 수신 후 20ms 안에 9개 frame 모두 수신
- 9개 frame의 Duration은 모두 같아야 함
- Execute=1
- Relative=0
- StepMode=0
- Reserved=0

조건을 만족하면 Board3는 9개 servo command를 queue에 넣고 같은 tick에서 시작한다.

## 8. Common Control Payload

### ESTOP

```text
CAN ID: 0x001
Payload: [board_id]
```

권장 사용:

```text
전체 정지: board_id = 255
```

대상 STM32는 즉시 motion을 정지하고 queue를 clear하고 `STATE_ESTOP`으로 전환한다.

### Enable / Disable

```text
CAN ID: 0x010
Payload: [board_id, enable]
```

| Byte | 의미 |
| ---: | --- |
| 0 | Board ID |
| 1 | `0`: Disable, `1`: Enable |

### Homing

```text
CAN ID: 0x020
Payload: [board_id, motor_id, mode]
```

| Byte | 의미 |
| ---: | --- |
| 0 | Board ID |
| 1 | Motor ID 또는 `255` |
| 2 | Homing Mode, 현재 `0` |

Board1:

- Motor ID `0~3`
- 전체 homing은 `255`

Board2:

- Motor ID `0`
- 전체 homing은 `255`

Board3:

- homing을 사용하지 않는다.
- `0x020`은 무시하는 것을 권장한다.

### Clear Error

```text
CAN ID: 0x030
Payload: [board_id, motor_id]
```

| Byte | 의미 |
| ---: | --- |
| 0 | Board ID |
| 1 | Motor ID 또는 `255` |

ESTOP 상태는 Clear Error만으로 해제하지 않는다. ESTOP 해제는 `0x010 Enable=1`에서 처리한다.

## 9. Status Payload

대상 CAN ID:

```text
Board1: 0x201
Board2: 0x202
Board3: 0x203
```

STM32는 100ms마다 status를 송신한다. Enable, Homing 완료, Error, Queue Full, ESTOP 같은 이벤트 발생 시 즉시 한 번 더 송신한다.

Payload:

| Byte | 필드 | 설명 |
| ---: | --- | --- |
| 0 | State | 현재 보드 상태 |
| 1 | Error Code | 현재 error code |
| 2 | Ready / Homing | Board1/2는 homing bits, Board3는 전체 ready |
| 3 | Moving / Staging | Board1/2는 moving motor id, Board3는 staging count |
| 4 | Limit / Fault | Board1/2는 limit bits, Board3는 전체 fault |
| 5 | Queue / Buffer Free | Board1/2는 queue free, Board3는 staging buffer free |
| 6 | Enabled | `0`: disabled, `1`: enabled |
| 7 | Reserved / Fault ID | Board1/2는 reserved, Board3는 fault motor id |

Board1:

| Byte | 의미 |
| ---: | --- |
| 2 | bit0~3 = axis0~3 homing done |
| 4 | bit0~3 = axis0~3 limit active |

Board2:

| Byte | 의미 |
| ---: | --- |
| 2 | bit0 = axis0 homing done |
| 4 | bit0 = axis0 limit active |

Board3:

| Byte | 의미 |
| ---: | --- |
| 2 | `0`: not ready, `1`: all servos ready |
| 3 | staging된 `0x103` frame 개수, `0~9` |
| 4 | `0`: no servo fault, `1`: one or more servo fault |
| 5 | staging buffer free, `9 - staging_count` |
| 7 | fault motor id, fault 없으면 `255` |

Board3는 서보 9개 각각의 ready/fault bit를 보내지 않는다. 상세 fault가 필요하면 나중에 diagnostic CAN ID나 ROS diagnostic topic으로 확장한다.

## 10. State Values

| 값 | 이름 |
| ---: | --- |
| `0` | `STATE_INIT` |
| `1` | `STATE_IDLE` |
| `2` | Board1/2: `STATE_HOMING`, Board3: `STATE_STAGING` |
| `3` | `STATE_MOVING` |
| `4` | `STATE_ERROR` |
| `5` | `STATE_ESTOP` |
| `6` | Board3: `STATE_DISABLED` |

## 11. Error Codes

| 값 | 이름 | 의미 |
| ---: | --- | --- |
| `0` | `ERR_NONE` | 정상 |
| `1` | `ERR_INVALID_CMD` | 잘못된 command, motor id, flag 등 |
| `2` | `ERR_LIMIT_DETECTED` | limit 감지 |
| `3` | `ERR_DRIVER_FAULT` | driver 또는 통신 초기화 실패 |
| `4` | `ERR_HOMING_FAIL` | homing 실패 |
| `5` | `ERR_QUEUE_FULL` | queue full |
| `6` | `ERR_RESERVED` | 예약 |

## 12. 완료 조건

Board1 완료 조건:

```text
state == STATE_IDLE
error == ERR_NONE
moving_motor_id == 255
queue_free == 32
enabled == 1
homing_done_bits == 0x0F
status is fresh
```

Board2 완료 조건:

```text
state == STATE_IDLE
error == ERR_NONE
moving_motor_id == 255
queue_free == 32
enabled == 1
homing_done bit0 == 1
status is fresh
```

Board3 완료 조건:

```text
state == STATE_IDLE
error == ERR_NONE
moving_motor_id == 255
queue_free == max_queue_free
enabled == 1
ready == 1
fault == 0
status is fresh
```

이 완료 조건은 각 보드가 queue 처리를 끝내고 idle 상태라고 보고했다는 의미다. encoder 기반 실제 도착 검증이라는 의미는 아니다.
