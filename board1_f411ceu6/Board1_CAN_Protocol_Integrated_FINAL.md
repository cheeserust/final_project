# Board1 CAN Protocol FINAL — Integrated Mapping + 0x301 Position Feedback

## 0. 문서 목적

이 문서는 첨부된 통합 기준인 **Board1 / Board2 / Board3 CAN Protocol and STM32 Implementation**에 맞춘 Board1 전용 최종 프로토콜입니다.

핵심 원칙은 다음과 같습니다.

```text
서버/RPi 쪽에서는 내부 board_id로 대상 보드를 고르지만,
실제 CAN frame payload에는 별도 Board ID를 넣지 않는다.
보드 구분은 CAN ID로 한다.
```

기존 `0x201 / 0x202 / 0x203` status frame은 그대로 유지하고, MoveIt2 `/joint_states` actual position용 current position feedback CAN ID를 별도로 추가합니다.

---

## 1. 통합 보드 매핑

| Board ID | 대상 | Move CAN ID | Status CAN ID | Position Feedback CAN ID | Payload Motor ID |
|---:|---|---:|---:|---:|---|
| `1` | Board1, 팔 2~5축 | `0x101` | `0x201` | `0x301` | `0~3` |
| `2` | Board2, 베이스 1축 | `0x102` | `0x202` | `0x302` | `0` |
| `3` | Board3, 서보 9개 | `0x103` | `0x203` | `0x303` | `0~8` |

중요:

```text
Board1은 베이스 1축을 담당하지 않는다.
Board1 local Motor ID 0~3은 실제 팔 2~5축이다.
Board2가 베이스 1축을 담당한다.
```

서버 내부의 global joint id와 CAN payload의 local Motor ID는 분리합니다.

| Global Joint ID | 실제 축 | Board ID | Move CAN ID | Payload Motor ID |
|---:|---|---:|---:|---:|
| `0` | 베이스 1축 | `2` | `0x102` | `0` |
| `1` | 팔 2축 | `1` | `0x101` | `0` |
| `2` | 팔 3축 | `1` | `0x101` | `1` |
| `3` | 팔 4축 | `1` | `0x101` | `2` |
| `4` | 팔 5축 | `1` | `0x101` | `3` |

---

## 2. Board1 Joint Limit / Home Position

각도 값은 CAN payload의 `Target Pos`와 같은 `0.01도` 단위 raw angle로 변환해서 사용합니다.

| Joint | 실제 축 | Board1 Local Motor ID | Min deg | Max deg | Home deg | Min raw | Max raw | Home raw |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `arm_joint_1` | 팔 2축 | `0` | `-90` | `90` | `-90` | `-9000` | `9000` | `-9000` |
| `arm_joint_2` | 팔 3축 | `1` | `-80` | `80` | `-80` | `-8000` | `8000` | `-8000` |
| `arm_joint_3` | 팔 4축 | `2` | `-90` | `90` | `-90` | `-9000` | `9000` | `-9000` |
| `arm_joint_4` | 팔 5축 | `3` | `-170` | `170` | `-170` | `-17000` | `17000` | `-17000` |

Board1 펌웨어는 팔 2~5축 4개 축의 limit/home을 적용합니다.

---

## 3. Board1 CAN ID

| CAN ID | 방향 | 용도 | payload Board ID 사용 여부 |
|---:|---|---|---|
| `0x001` | 중앙서버/RPi → 전체 보드 | Emergency Stop | 없음 |
| `0x010` | 중앙서버/RPi → 전체 보드 | Enable / Disable broadcast | 없음 |
| `0x020` | 중앙서버/RPi → Board1 + Board2 | Arm Homing broadcast | 없음 |
| `0x030` | 중앙서버/RPi → 전체 보드 | Clear Error broadcast | 없음 |
| `0x101` | 중앙서버/RPi → Board1 | Board1 4축 trajectory point 구성 frame | 없음 |
| `0x201` | Board1 → 중앙서버/RPi | Board1 status | 없음 |
| `0x301` | Board1 → 중앙서버/RPi | Board1 current position feedback | 없음 |

---

## 4. `0x101` Board1 Position Command

Board1 위치 명령은 8바이트 고정 길이입니다.

```text
CAN ID = 0x101
DLC = 8
대상 = Board1 local Motor ID 0~3
실제 축 = 팔 2~5축
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | 상위 4bit flags, 하위 4bit local motor id |
| 1 | Target Pos LSB | `int32_t` 일부 | little endian |
| 2 | Target Pos | `int32_t` 일부 | little endian |
| 3 | Target Pos | `int32_t` 일부 | little endian |
| 4 | Target Pos MSB | `int32_t` 일부 | little endian |
| 5 | Speed LSB | `uint16_t` 일부 | little endian |
| 6 | Speed MSB | `uint16_t` 일부 | little endian |
| 7 | Duration | `uint8_t` | 5ms 단위 |

### 4.1 Byte0 구조

```text
Bit7 Bit6 Bit5 Bit4 | Bit3 Bit2 Bit1 Bit0
Exec Rel  Step Rsv  | Motor ID
```

| Bit | 이름 | 의미 |
|---:|---|---|
| 7 | Execute | `1`: 명령 실행/queue push 후보, `0`: 무시 |
| 6 | Relative | `1`: 현재 위치 기준 상대 이동 |
| 5 | Step Mode | `0`: Target Pos는 0.01도 단위 angle, `1`: Target Pos는 step |
| 4 | Reserved | 현재 미사용, 반드시 `0` |
| 3~0 | Motor ID | Board1에서는 `0~3`만 유효 |

일반 절대 각도 명령은 다음과 같습니다.

```text
Execute = 1
Relative = 0
Step Mode = 0
Reserved = 0
Local Motor ID = 0~3
```

| 실제 축 | Board1 Local Motor ID | Byte0 |
|---|---:|---:|
| 팔 2축 | 0 | `0x80` |
| 팔 3축 | 1 | `0x81` |
| 팔 4축 | 2 | `0x82` |
| 팔 5축 | 3 | `0x83` |

### 4.2 Target Pos

`Byte1~4`는 `int32_t` little endian입니다.

| Step Mode | Target Pos 의미 |
|---:|---|
| `0` | 0.01도 단위 angle |
| `1` | step 수 |

예시:

```text
30.00 deg  -> 3000
-15.50 deg -> -1550
```

### 4.3 각도 → step 변환

Board1은 출력축 기준 각도를 수신하고, STM32 내부에서 축별 감속비와 모터 full-step 수를 반영해 step으로 변환합니다.

```c
step = angle_raw * gear_ratio[motor_id] * motor_steps_per_rev[motor_id] * 16 / 36000;
```

| Board1 Motor ID | 실제 축 | Gear ratio | Motor full steps/rev |
|---:|---|---:|---:|
| `0` | 팔 2축 | `20` | `200` |
| `1` | 팔 3축 | `50` | `200` |
| `2` | 팔 4축 | `30` | `200` |
| `3` | 팔 5축 | `120` | `48` |

### 4.4 Speed

`Speed`는 `uint16_t` little endian입니다.

| 항목 | 단위 |
|---|---|
| Speed | 0.01도/s |

현재 테스트 펌웨어에서는 Speed를 4축 point에 저장하지만, 1ms 선형 보간 계산에는 사용하지 않습니다. 실제 이동 시간은 `Duration`이 결정합니다.

### 4.5 Duration

`Duration`은 5ms 단위입니다.

```text
duration_ms = Byte7 × 5
```

`Byte7 = 0`이면 STM32 내부에서 최소 `1ms` segment로 처리합니다.

---

## 5. Board1 4-frame Trajectory Point Staging 규칙

Board1의 `0x101` 이동 명령은 **한 축씩 바로 실행되는 구조가 아닙니다.**

MoveIt2 trajectory point 하나는 반드시 아래 순서로 4개의 CAN frame을 연속 전송해야 합니다.

| 순서 | Board1 Local Motor ID | 실제 축 | Byte0 예시 |
|---:|---:|---|---:|
| 1 | `0` | 팔 2축 | `0x80` |
| 2 | `1` | 팔 3축 | `0x81` |
| 3 | `2` | 팔 4축 | `0x82` |
| 4 | `3` | 팔 5축 | `0x83` |

조건:

```text
1. Motor ID는 반드시 0 -> 1 -> 2 -> 3 순서여야 한다.
2. 첫 frame 수신 후 20ms 안에 4개 frame이 모두 들어와야 한다.
3. 4개 frame의 Duration 값은 모두 같아야 한다.
4. Execute=1이어야 한다.
5. Reserved bit는 0이어야 한다.
6. 안 움직이는 축도 생략하면 안 된다.
7. 안 움직이는 축은 해당 point의 유지 목표 위치를 그대로 보내야 한다.
```

4번째 frame까지 정상 수신되면 STM32가 하나의 4축 trajectory point로 queue에 넣고, 네 축을 같은 1ms tick에서 동시에 시작합니다.

주의:

```text
Motor ID 0 frame 하나만 보내면 팔 2축만 바로 움직이지 않는다.
STM32는 4축 point 대기 상태가 되고, 나머지 Motor ID 1/2/3 frame을 기다린다.
```

---

## 6. 공통 제어 명령

## 6.1 Emergency Stop, CAN ID `0x001`

Emergency Stop은 전체 보드 broadcast입니다.

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | ESTOP | `1` |
| 1~7 | Reserved | `0` |

Board1 수신 시 동작:

```text
1. 모든 STEP 출력 정지 또는 simulated motion 정지
2. motor disable
3. trajectory queue clear
4. staging buffer clear
5. state = STATE_ESTOP
6. 0x201 status 즉시 송신
```

ESTOP 해제는 Clear Error가 아니라 `0x010 Enable=1`에서 처리합니다.

## 6.2 Enable / Disable, CAN ID `0x010`

Enable / Disable은 전체 보드 broadcast입니다.

| Byte | 값 | 의미 |
|---:|---:|---|
| 0 | `0` | Disable |
| 0 | `1` | Enable |
| 1~7 | `0` | Reserved |

Enable 수신 시:

```text
1. ESTOP flag 해제
2. error_code = ERR_NONE
3. motor enable 상태
4. state = STATE_IDLE
5. 0x201 status 즉시 송신
```

Disable 수신 시:

```text
1. queue clear
2. staging clear
3. motion 정지
4. motor disable 상태
5. state = STATE_DISABLED
6. 0x201 status 즉시 송신
```

## 6.3 Arm Homing Broadcast, CAN ID `0x020`

`0x020`은 Board1 팔 2~5축과 Board2 베이스 1축이 동시에 처리하는 전체 스텝모터 homing broadcast입니다.

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Target Motor | `0xFF`: arm 전체 homing |
| 1 | Homing Mode | 현재 `0`만 사용 |
| 2~7 | Reserved | `0` |

Board1 처리 조건:

```text
CAN ID == 0x020
Target Motor == 0xFF
Homing Mode == 0
enabled == 1
state != STATE_ESTOP
state != STATE_ERROR
```

정상 처리 시:

```text
Board1 local motor 0~3 전체 homing 수행
homing 완료 후 homing_done_bits bit0~3 = 1
현재 위치를 각 축 Home raw 값으로 설정
0x201 status 즉시 송신
```

테스트 펌웨어에서는 실제 limit switch homing 대신 simulated homing을 수행합니다. simulated homing 완료 시 현재 위치는 아래 Home raw 기준으로 설정됩니다.

```text
Motor 0 / 팔 2축: -9000
Motor 1 / 팔 3축: -8000
Motor 2 / 팔 4축: -9000
Motor 3 / 팔 5축: -17000
```

## 6.4 Clear Error Broadcast, CAN ID `0x030`

`0x030`은 전체 보드 Clear Error broadcast입니다.

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Target Motor | `0xFF`: 전체 error clear |
| 1~7 | Reserved | `0` |

Clear Error 수신 시:

```text
1. error_code = ERR_NONE
2. staging buffer clear
3. enabled == 1이면 state = STATE_IDLE
4. enabled == 0이면 state = STATE_DISABLED
5. 0x201 status 즉시 송신
```

ESTOP 상태는 Clear Error만으로 해제하지 않습니다. ESTOP 해제는 `0x010 Enable=1`에서 처리합니다.

---

## 7. Board1 Status, CAN ID `0x201`

Board1은 100ms마다 status를 송신합니다. 주요 이벤트 발생 시에도 즉시 status를 송신합니다.

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State | 현재 보드 상태 |
| 1 | Error Code | 현재 error code |
| 2 | Homing Done Bits | bit0~3 = Board1 local motor 0~3 homing done |
| 3 | Moving Motor ID | 이동 또는 homing 중인 첫 motor id, 없으면 `255` |
| 4 | Limit Status Bits | bit0~3 = Board1 local motor 0~3 limit active |
| 5 | Queue Free | 외부 `0x101` command slot 기준 남은 슬롯 수 |
| 6 | Enabled | `0`: disabled, `1`: enabled |
| 7 | Reserved | 현재 `0` |

### Queue Free 기준

Board1 STM32 trajectory queue는 외부 status 기준으로 32개의 `0x101` command slot입니다.

내부 구현은 4축 point queue 8개이며, 한 4축 point가 `0x101` frame 4개를 사용합니다.

```text
internal free point = 8 - queued_point_count
status Byte5 Queue Free = internal free point × 4
범위 = 0~32
```

예시:

```text
내부 point queue 8개가 모두 비어 있으면 Queue Free = 32
내부 point queue 7개가 비어 있으면 Queue Free = 28
내부 point queue 0개가 비어 있으면 Queue Free = 0
```

### State Values

| 값 | 이름 | 설명 |
|---:|---|---|
| `0` | `STATE_INIT` | 초기화 중 |
| `1` | `STATE_IDLE` | 대기 상태 |
| `2` | `STATE_HOMING` | 원점복귀 중 |
| `3` | `STATE_MOVING` | 이동 중 |
| `4` | `STATE_ERROR` | 에러 발생 |
| `5` | `STATE_ESTOP` | 비상정지 상태 |
| `6` | `STATE_DISABLED` | Disable 상태 |

### Error Codes

| 값 | 이름 | 의미 |
|---:|---|---|
| `0` | `ERR_NONE` | 정상 |
| `1` | `ERR_INVALID_CMD` | 잘못된 명령, motor id, homing 전 move 등 |
| `2` | `ERR_LIMIT_SWITCH_DETECTED` | limit switch 감지 또는 예약 |
| `3` | `ERR_DRIVER_FAULT` | MCP2515 init 실패 등 driver fault |
| `4` | `ERR_HOMING_FAIL` | homing 실패 또는 예약 |
| `5` | `ERR_QUEUE_FULL` | trajectory queue full |
| `6` | `ERR_RESERVED` | 예약 |

---

## 8. Board1 Current Position Feedback, CAN ID `0x301`

기존 `0x201` status frame은 그대로 유지합니다. MoveIt2 `/joint_states` actual position 입력을 위해 별도 current position feedback frame `0x301`을 추가합니다.

Board1은 100ms마다 local motor `0 -> 1 -> 2 -> 3` 순서로 `0x301` frame 4개를 보냅니다.

```text
CAN ID = 0x301
DLC = 8
방향 = Board1 -> 중앙서버/RPi
주기 = 100ms
송신 순서 = local motor 0 -> 1 -> 2 -> 3
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Local Motor ID | `uint8_t` | Board1 local motor id `0~3` |
| 1 | Flags | `uint8_t` | position valid / homed / moving / target reached |
| 2 | Current Pos LSB | `int32_t` 일부 | little endian |
| 3 | Current Pos | `int32_t` 일부 | little endian |
| 4 | Current Pos | `int32_t` 일부 | little endian |
| 5 | Current Pos MSB | `int32_t` 일부 | little endian |
| 6 | Error / Fault Code | `uint8_t` | error code, 없으면 `0` |
| 7 | Sequence Counter | `uint8_t` | 송신 순서 확인용 counter |

### 8.1 Position Feedback Flags, Byte1

| Bit | 이름 | 의미 |
|---:|---|---|
| bit0 | Position Valid | `1`: current position 값을 MoveIt2 actual position으로 사용 가능 |
| bit1 | Homed / Ready | `1`: homing 완료 또는 ready |
| bit2 | Moving | `1`: 이동 또는 homing 중 |
| bit3 | Target Reached | `1`: 목표 위치 도달 |
| bit4~7 | Reserved | 현재 `0` |

### 8.2 Position 단위

`current_pos_001deg`는 모터 step 값이 아니라 중앙서버 / MoveIt2 joint 기준 출력축 각도입니다.

단위는 command `target_pos`와 동일하게 `0.01도`입니다.

```text
30.00 deg  -> 3000
-15.50 deg -> -1550
```

Board1 STM32 내부 변환은 현재 step position을 출력축 각도로 역변환합니다.

```c
current_pos_001deg = current_step * 36000 / (gear_ratio[motor_id] * motor_steps_per_rev[motor_id] * 16);
```

Board1 예시:

```text
CAN ID = 0x301
Byte0 = 0
Byte1 = 0x0B  // valid + homed + target reached
Byte2~5 = 3000 = B8 0B 00 00
Byte6 = 0
Byte7 = sequence counter
```

---

## 9. Queue and Error Policy

Queue full 상태에서 새 위치 명령이 오면:

```text
1. 새 명령은 저장하지 않는다. 즉 Drop Tail이다.
2. 기존 queue 내용은 유지한다.
3. ERR_QUEUE_FULL을 설정한다.
4. STATE_ERROR로 전환한다.
5. 0x201 status를 즉시 송신한다.
6. error가 clear되기 전까지 추가 move 명령은 무시한다.
```

Board1 staging 실패 조건:

```text
1. Motor ID 순서가 0 -> 1 -> 2 -> 3이 아님
2. 첫 frame 수신 후 20ms 초과
3. Duration 불일치
4. Execute=0
5. Reserved bit=1
6. Motor ID가 0~3 범위를 벗어남
7. Enable 전 move 수신
8. Homing 완료 전 move 수신
9. ESTOP/ERROR/HOMING 상태에서 move 수신
```

실패 시:

```text
staging buffer clear
ERR_INVALID_CMD 또는 ERR_QUEUE_FULL 설정
0x201 status 즉시 송신
```

---

## 10. C Constant Example

```c
#define CAN_ID_ESTOP        0x001
#define CAN_ID_ENABLE       0x010
#define CAN_ID_ARM_HOMING   0x020
#define CAN_ID_CLEAR_ERROR  0x030

#define CAN_ID_BOARD1_MOVE      0x101
#define CAN_ID_BOARD1_STATUS    0x201
#define CAN_ID_BOARD1_FEEDBACK  0x301

#define BOARD1_MOTOR_COUNT      4
#define BOARD1_POINT_QUEUE_SIZE 8
#define BOARD1_COMMAND_SLOTS    32
#define MOTOR_ALL               0xFF
```

---

## 11. Board1 Test Command Summary

```bash
# 전체 Enable
cansend can0 010#0100000000000000

# Board1 + Board2 전체 스텝모터 homing
cansend can0 020#FF00000000000000

# 4축 trajectory point 전송: 팔 2축 30.00도, 나머지 home 유지 예시
python3 send_board1_4axis_point.py can0 3000 -8000 -9000 -17000 --speed 1000 --duration 10

# Board1 position feedback 확인
candump can0,301:7FF | python3 decode_board1_feedback.py

# Board1 status 확인
candump can0,201:7FF

# 전체 보드 error clear
cansend can0 030#FF00000000000000

# 전체 ESTOP
cansend can0 001#0100000000000000
```

---

## 12. 현재 테스트 코드 구현 범위

현재 제공한 STM32F411CEU6 테스트 코드는 첨부 통합 프로토콜 중 Board1에 필요한 아래 항목을 구현합니다.

```text
구현됨:
- MCP2515 SPI2 기반 CAN RX/TX
- MCP2515 CS = PA9
- MCP2515 INT = PA10
- 0x001 ESTOP
- 0x010 Enable / Disable
- 0x020 Homing Start
- 0x030 Clear Error
- 0x101 Board1 4-frame trajectory point staging
- 내부 4축 point queue 8개
- status Queue Free를 외부 0x101 command slot 기준 0~32로 보고
- Board1 gear ratio {20, 50, 30, 120}
- Board1 motor full steps/rev {200, 200, 200, 48}
- simulated homing 후 home raw {-9000, -8000, -9000, -17000} 적용
- TIM3 1ms simulated trajectory interpolation
- 0x201 100ms status
- 0x301 100ms current position feedback, Motor ID 0->1->2->3
```

아직 실제 하드웨어 구동용으로 제한되는 항목:

```text
- 실제 STEP/DIR pulse 출력은 테스트 코드에서 구동하지 않음
- 실제 limit switch homing/debounce는 테스트 코드에서 simulated 처리
- driver fault 입력은 실제 핀 기반으로 읽지 않음
- Speed는 저장하지만 가감속/속도 프로파일에는 사용하지 않음
```

즉, 이 코드는 **프로토콜/CAN 통신/queue/feedback 검증용 Board1 테스트 코드**입니다.
