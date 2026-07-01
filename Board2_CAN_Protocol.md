# Board2 CAN Protocol FINAL — Base Joint + 0x302 Position Feedback

## 0. 문서 목적

이 문서는 첨부된 통합 프로토콜 기준에 맞춘 **Board2 전용 CAN 프로토콜**입니다.

핵심 기준은 다음과 같습니다.

```text
1. 서버/RPi 내부에서는 board_id로 대상 보드를 고른다.
2. 실제 CAN frame payload에는 별도 Board ID를 넣지 않는다.
3. 보드 구분은 CAN ID로 한다.
4. Board2는 베이스 1축 단일 축 보드이다.
5. 기존 status frame 0x202는 변경하지 않는다.
6. MoveIt2 /joint_states actual position용 position feedback은 0x302로 별도 송신한다.
```

---

## 1. Board2 역할과 매핑

| 항목 | 값 |
|---|---:|
| 담당 보드 | Board2 |
| 담당 축 | 베이스 1축 / `base_joint` |
| 서버 Global Joint ID | `0` |
| Board2 Local Motor ID | `0` |
| Move Command CAN ID | `0x102` |
| Status CAN ID | `0x202` |
| Position Feedback CAN ID | `0x302` |

중앙서버 / MoveIt2 기준 매핑은 다음과 같습니다.

```text
base_joint / Global Joint ID 0 → Board2 local Motor ID 0 → Move CAN ID 0x102
base_joint actual position      → Board2 local Motor ID 0 → Position Feedback CAN ID 0x302
```

중요:

```text
서버의 global joint id와 CAN payload Motor ID는 분리한다.
Board2의 서버 global joint id는 0이다.
Board2의 CAN payload local Motor ID도 0이다.
두 값이 우연히 같을 뿐, 개념은 다르다.
```

---

## 2. Board2 CAN ID

| CAN ID | 방향 | 용도 | payload Board ID 사용 여부 |
|---:|---|---|---|
| `0x001` | 중앙서버/RPi → 전체 보드 | Emergency Stop broadcast | 없음 |
| `0x010` | 중앙서버/RPi → 전체 보드 | Enable / Disable broadcast | 없음 |
| `0x020` | 중앙서버/RPi → Board1 + Board2 | Stepper Homing broadcast | 없음 |
| `0x030` | 중앙서버/RPi → 전체 보드 | Clear Error broadcast | 없음 |
| `0x102` | 중앙서버/RPi → Board2 | Board2 base motor trajectory point | 없음 |
| `0x202` | Board2 → 중앙서버/RPi | Board2 status response | 없음 |
| `0x302` | Board2 → 중앙서버/RPi | Board2 current position feedback | 없음 |

중요:

```text
0x010, 0x020, 0x030 payload에는 Target Board를 넣지 않는다.
기존 Target Board 방식인 020#02FF..., 030#02FF... 는 사용하지 않는다.
```

---

## 3. Board2 Joint Limit / Home Position

각도 값은 CAN payload의 Target Pos와 같은 `0.01도` 단위 raw angle로 사용합니다.

| Joint | Board ID | Payload Motor ID | Min deg | Max deg | Home deg | Min raw | Max raw | Home raw |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `base_joint` | `2` | `0` | `-90` | `180` | `-90` | `-9000` | `18000` | `-9000` |

Board2 펌웨어는 angle mode 명령에서 위 limit을 적용합니다.

```text
raw_target < -9000 또는 raw_target > 18000이면 ERR_INVALID_CMD 처리
```

Homing 완료 후 Board2의 현재 위치는 다음 값으로 설정합니다.

```text
current_pos_001deg = -9000   // -90.00 deg, base_joint home
```

---

## 4. `0x102` Board2 Position Command

Board2 위치 명령은 8바이트 고정 길이입니다.

```text
CAN ID = 0x102
DLC = 8
대상 = Board2 local Motor ID 0
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
| 7 | Execute | `1`: 명령 실행 또는 queue push, `0`: 무시 |
| 6 | Relative | `1`: 현재 위치 기준 상대 이동 |
| 5 | Step Mode | `0`: Target Pos는 0.01도 단위 angle, `1`: Target Pos는 step |
| 4 | Reserved | 현재 미사용, 반드시 `0` |
| 3~0 | Motor ID | Board2에서는 `0`만 유효 |

Board2 일반 절대 각도 명령은 항상 다음 값을 사용합니다.

```text
Execute = 1
Relative = 0
Step Mode = 0
Reserved = 0
Local Motor ID = 0

Byte0 = 0x80
```

Board2에서 `0x81~0x8F`는 local motor id가 0이 아니므로 잘못된 명령입니다.

### 4.2 Target Pos

`Byte1~4`는 `int32_t` little endian입니다.

| Step Mode | Target Pos 의미 |
|---:|---|
| `0` | 0.01도 단위 출력축 angle |
| `1` | step 수 |

예시:

| 실제 각도 | 전송 값 | little-endian |
|---:|---:|---|
| `30.00 deg` | `3000` | `B8 0B 00 00` |
| `-15.50 deg` | `-1550` | `F2 F9 FF FF` |
| `-90.00 deg` | `-9000` | `D8 DC FF FF` |

### 4.3 각도 → step 변환

Board2는 베이스 출력축 기준 각도를 수신하고, STM32 내부에서 감속비를 반영해 step으로 변환합니다.

```c
step = angle_raw * BOARD2_GEAR_RATIO * MOTOR_STEPS_PER_REV * MICROSTEP / 36000;
```

Board2 베이스 기본값:

```c
#define BOARD2_GLOBAL_JOINT_ID   0
#define BOARD2_LOCAL_MOTOR_ID    0
#define BOARD2_GEAR_RATIO        20
#define MOTOR_STEPS_PER_REV      200
#define MICROSTEP                16
#define BOARD2_MIN_POS_001DEG    -9000
#define BOARD2_MAX_POS_001DEG    18000
#define BOARD2_HOME_POS_001DEG   -9000
```

예시:

```text
30.00 deg → angle_raw = 3000
step = 3000 × 20 × 200 × 16 / 36000
     = 5333 steps
```

### 4.4 Speed

`Speed`는 `uint16_t` little endian입니다.

초기 테스트 구현에서는 Speed를 queue에 저장하되, 실제 1ms 보간 계산에는 사용하지 않을 수 있습니다. 실제 이동 시간은 `Duration`이 결정합니다.

### 4.5 Duration

`Duration`은 5ms 단위입니다.

```text
duration_ms = Byte7 × 5
```

| Byte7 | 실제 시간 |
|---:|---:|
| `1` | `5ms` |
| `10` | `50ms` |
| `20` | `100ms` |

`Byte7 = 0`이면 STM32 내부에서 최소 `1ms` segment로 처리합니다.

### 4.6 Position Command 예시

Board2 베이스 local Motor ID 0을 30.00도로 50ms 동안 이동:

```text
CAN ID = 0x102
Byte0 = 0x80
Target Pos = 3000 = 0x00000BB8 little endian = B8 0B 00 00
Speed = 1000 = 0x03E8 little endian = E8 03
Duration = 10 = 0A
```

```bash
cansend can0 102#80B80B0000E8030A
```

Board2 베이스 home 위치 -90.00도로 50ms 동안 이동:

```bash
cansend can0 102#80D8DCFFFFE8030A
```

---

## 5. 공통 제어 명령

## 5.1 Emergency Stop, CAN ID `0x001`

Emergency Stop은 전체 보드 broadcast입니다.

```text
CAN ID = 0x001
payload는 현재 사용하지 않는다.
```

Board2 수신 시 동작:

```text
1. STEP 출력 정지
2. 모터 드라이버 Disable
3. trajectory queue clear
4. state = STATE_ESTOP
5. 0x202 status 즉시 송신
6. 0x302 position feedback 즉시 송신
```

ESTOP 해제는 Clear Error가 아니라 `0x010 Enable=1`에서 처리합니다.

예시:

```bash
# payload는 사용하지 않지만, cansend 편의상 8바이트로 보내도 처리 가능
cansend can0 001#0000000000000000
```

---

## 5.2 Enable / Disable, CAN ID `0x010`

Enable / Disable은 전체 보드 broadcast입니다.

```text
payload에 Target Board를 넣지 않는다.
```

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | Enable | `0`: Disable, `1`: Enable |
| 1~7 | Reserved | `0` |

예시:

```bash
# 전체 Enable
cansend can0 010#0100000000000000

# 전체 Disable
cansend can0 010#0000000000000000
```

Board2는 `0x010`을 수신하면 항상 처리합니다.

Enable 수신 시:

```text
1. ESTOP flag 해제
2. error_code = ERR_NONE
3. motor enable pin 활성화
4. state = STATE_IDLE
5. 0x202 status 즉시 송신
6. 0x302 position feedback 즉시 송신
```

Disable 수신 시:

```text
1. queue clear
2. STEP 출력 정지
3. motor disable
4. state = STATE_DISABLED
5. 0x202 status 즉시 송신
6. 0x302 position feedback 즉시 송신
```

---

## 5.3 Stepper Homing Broadcast, CAN ID `0x020`

`0x020`은 Board1 팔 축과 Board2 베이스 축이 동시에 처리하는 전체 스텝모터 homing broadcast입니다.

```text
대상 = Board1 + Board2
Board1 = 팔 2~5축 local motor 0~3 homing
Board2 = 베이스 1축 local motor 0 homing
payload에 Target Board를 넣지 않는다.
```

Board3는 기본적으로 `0x020`을 처리하지 않습니다.

### Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Motor | `uint8_t` | `0xFF`: 전체 스텝모터 homing |
| 1 | Homing Mode | `uint8_t` | 현재 `0`만 사용 |
| 2~7 | Reserved | - | `0` |

현재 최종 통합 기준에서는 `Byte0 = 0xFF`만 사용합니다.

Board2 처리 조건:

```text
CAN ID == 0x020
Target Motor == 0xFF
Homing Mode == 0
enabled == 1
state != STATE_ESTOP
```

정상 처리 시:

```text
Board2 base local motor 0 homing 수행
homing 완료 후 current_pos_001deg = -9000
homing_done bit0 = 1
0x202 status 즉시 송신
0x302 position feedback 즉시 송신
```

조건이 맞지 않으면 `ERR_INVALID_CMD`를 설정하고 `0x202` status를 즉시 송신합니다.

예시:

```bash
# Board1 + Board2 전체 스텝모터 homing
cansend can0 020#FF00000000000000
```

주의:

```text
기존 Target Board 방식인 020#02FF... 또는 020#0200... 은 사용하지 않는다.
```

---

## 5.4 Clear Error Broadcast, CAN ID `0x030`

`0x030`은 전체 보드 Clear Error broadcast입니다.

```text
payload에 Target Board를 넣지 않는다.
Board1, Board2, Board3가 동시에 error clear를 수행한다.
```

Payload 구조:

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Motor | `uint8_t` | `0xFF`: 전체 error clear |
| 1~7 | Reserved | - | `0` |

Board2 처리 조건:

```text
CAN ID == 0x030
Target Motor == 0xFF
```

Clear Error 수신 시:

```text
1. error_code = ERR_NONE
2. fault 관련 내부 flag clear
3. queue 관련 error flag clear
4. enabled == 1이면 state = STATE_IDLE
5. enabled == 0이면 state = STATE_DISABLED 또는 STATE_INIT
6. 0x202 status 즉시 송신
7. 0x302 position feedback 즉시 송신
```

단, ESTOP 상태는 Clear Error만으로 해제하지 않습니다. ESTOP 해제는 `0x010 Enable=1`에서 처리합니다.

예시:

```bash
# 전체 보드 error clear
cansend can0 030#FF00000000000000
```

주의:

```text
기존 Target Board 방식인 030#02FF... 또는 030#0200... 은 사용하지 않는다.
```

---

## 6. Board2 Status, CAN ID `0x202`

기존 status frame은 변경하지 않습니다. Board2는 100ms마다 status를 송신하고, 주요 이벤트 발생 시에도 즉시 status를 송신합니다.

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State | 현재 보드 상태 |
| 1 | Error Code | 현재 error code |
| 2 | Homing Done Bits | Board2: bit0 = base axis homing done |
| 3 | Moving Motor ID | 이동 또는 homing 중이면 `0`, 없으면 `255` |
| 4 | Limit Status Bits | Board2: bit0 = base limit active |
| 5 | Queue Free | trajectory queue 남은 슬롯 수, `0x102` command slot 기준 |
| 6 | Enabled | `0`: disabled, `1`: enabled |
| 7 | Reserved | 현재 `0` |

예시:

```text
0x202 01 00 01 FF 00 20 01 00
```

해석:

```text
STATE_IDLE
ERR_NONE
base homing 완료
이동 중인 모터 없음
Limit inactive
Queue free 32
Enabled
Reserved 0
```

---

## 7. Board2 Current Position Feedback, CAN ID `0x302`

MoveIt2 `/joint_states`의 actual position 입력을 위해 별도 current position feedback frame을 사용합니다.

```text
CAN ID = 0x302
DLC = 8
송신 주기 = 100ms
대상 = Board2 local motor 0 / base_joint
```

100ms 주기 status 송신 시점에 `0x202` status를 먼저 송신하고, 그 직후 `0x302` position feedback을 송신합니다. 주요 이벤트 발생 시에도 `0x202`와 `0x302`를 즉시 송신할 수 있습니다.

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Local Motor ID | `uint8_t` | Board2는 항상 `0` |
| 1 | Flags | `uint8_t` | position valid / homed / moving / target reached |
| 2 | Current Pos LSB | `int32_t` 일부 | little endian |
| 3 | Current Pos | `int32_t` 일부 | little endian |
| 4 | Current Pos | `int32_t` 일부 | little endian |
| 5 | Current Pos MSB | `int32_t` 일부 | little endian |
| 6 | Error / Fault Code | `uint8_t` | error code, 없으면 `0` |
| 7 | Sequence Counter | `uint8_t` | 송신 순서 확인용 counter |

### Position Feedback Flags, Byte1

| Bit | 이름 | 의미 |
|---:|---|---|
| 0 | Position Valid | `1`: current position 값을 MoveIt2 actual position으로 사용 가능 |
| 1 | Homed / Ready | `1`: homing 완료 또는 ready |
| 2 | Moving | `1`: 이동 또는 homing 중 |
| 3 | Target Reached | `1`: 목표 위치 도달 |
| 4~7 | Reserved | 현재 `0` |

`current_pos_001deg`는 모터 step 값이 아니라 중앙서버 / MoveIt2 joint 기준 출력축 각도입니다. 단위는 command `target_pos`와 같은 0.01도입니다.

```text
30.00 deg  -> 3000
-15.50 deg -> -1550
-90.00 deg -> -9000
```

STM32 내부 역변환:

```c
current_pos_001deg = current_step * 36000 / (BOARD2_GEAR_RATIO * MOTOR_STEPS_PER_REV * MICROSTEP);
```

예시:

```text
초기 disabled / not homed, home 위치 기준:
CAN ID = 0x302
Byte0 = 0
Byte1 = 0x01  // position valid
Byte2~5 = -9000 = D8 DC FF FF
Byte6 = 0
Byte7 = sequence counter

302#0001D8DCFFFF00XX
```

```text
homing 완료, idle, target reached, -90.00도:
Byte1 = 0x0B  // valid + homed + target reached
Byte2~5 = -9000 = D8 DC FF FF

302#000BD8DCFFFF00XX
```

```text
30.00도 이동 완료:
Byte1 = 0x0B
Byte2~5 = 3000 = B8 0B 00 00

302#000BB80B000000XX
```

---

## 8. State Values

| 값 | 이름 | 설명 |
|---:|---|---|
| `0` | `STATE_INIT` | 초기화 중 |
| `1` | `STATE_IDLE` | 대기 상태 |
| `2` | `STATE_HOMING` | 원점복귀 중 |
| `3` | `STATE_MOVING` | 이동 중 |
| `4` | `STATE_ERROR` | 에러 발생 |
| `5` | `STATE_ESTOP` | 비상정지 상태 |
| `6` | `STATE_DISABLED` | Disable 상태 |

---

## 9. Error Codes

| 값 | 이름 | 의미 |
|---:|---|---|
| `0` | `ERR_NONE` | 정상 |
| `1` | `ERR_INVALID_CMD` | 잘못된 명령, motor id, homing 전 move, limit 범위 초과 등 |
| `2` | `ERR_LIMIT_SWITCH_DETECTED` | 예약됨 또는 리미트 감지 |
| `3` | `ERR_DRIVER_FAULT` | MCP2515, TMC, 드라이버 fault |
| `4` | `ERR_HOMING_FAIL` | 예약됨 또는 원점복귀 실패 |
| `5` | `ERR_QUEUE_FULL` | trajectory queue full |
| `6` | `ERR_RESERVED` | 예약 |

---

## 10. Homing and Move Policy

Board2는 오픈루프 스텝모터 보드이므로 원점복귀 완료 전에는 위치 명령을 실행하지 않습니다.

권장 순서:

```text
1. 중앙서버가 전체 Enable 전송: 010#0100000000000000
2. 중앙서버가 Stepper Homing 전송: 020#FF00000000000000
3. Board1이 팔 2~5축 원점복귀 수행
4. Board2가 base_joint 원점복귀 수행
5. 중앙서버가 0x201 / 0x202 status에서 homing 완료 확인
6. 이후 0x101 / 0x102 trajectory 전송 허용
```

위치 명령 수신 시 내부 조건:

```c
if (enabled == 0 || state == STATE_ESTOP || state == STATE_ERROR) {
    error_code = ERR_INVALID_CMD;
    send_status_0x202();
    return;
}

if (homing_done == 0) {
    error_code = ERR_INVALID_CMD;
    send_status_0x202();
    return;
}

if (target_angle_raw < -9000 || target_angle_raw > 18000) {
    error_code = ERR_INVALID_CMD;
    send_status_0x202();
    return;
}
```

---

## 11. Queue and Error Policy

Board2 STM32 trajectory queue는 단일 축 point queue로 구성합니다. 외부 status의 Queue Free는 `0x102` command slot 기준으로 보고합니다.

Queue full 상태에서 새 `0x102` 명령이 오면:

```text
1. 새 명령은 저장하지 않음, Drop Tail
2. 기존 queue 내용은 유지
3. ERR_QUEUE_FULL 설정
4. STATE_ERROR 전환
5. 0x202 status 즉시 송신
6. 0x302 position feedback 즉시 송신
7. Clear Error 전까지 추가 move 명령 무시
```

Board2 command 규칙:

```text
1. 한 point당 0x102 frame 하나만 사용한다.
2. Motor ID는 0이어야 한다.
3. Execute=1이어야 한다.
4. Reserved bit는 0이어야 한다.
5. Relative와 Step Mode는 Byte0 정의에 따라 해석한다.
6. angle mode 목표 위치는 base_joint limit -9000~18000 raw 안에 있어야 한다.
```

---

## 12. 중앙서버 매핑 주의사항

첨부 통합 프로토콜 기준 매핑은 다음과 같습니다.

```text
Global Joint ID 0 → base_joint → Board2, CAN ID 0x102, local Motor ID 0
Global Joint ID 1 → arm_joint_1 / 팔 2축 → Board1, CAN ID 0x101, local Motor ID 0
Global Joint ID 2 → arm_joint_2 / 팔 3축 → Board1, CAN ID 0x101, local Motor ID 1
Global Joint ID 3 → arm_joint_3 / 팔 4축 → Board1, CAN ID 0x101, local Motor ID 2
Global Joint ID 4 → arm_joint_4 / 팔 5축 → Board1, CAN ID 0x101, local Motor ID 3
```

Position feedback 매핑:

```text
0x302, local motor 0 → base_joint / Global Joint ID 0
0x301, local motor 0 → arm_joint_1 / Global Joint ID 1
0x301, local motor 1 → arm_joint_2 / Global Joint ID 2
0x301, local motor 2 → arm_joint_3 / Global Joint ID 3
0x301, local motor 3 → arm_joint_4 / Global Joint ID 4
```

Board2 핵심 규칙:

```text
Move command 0x102 Byte0 = 0x80
Homing 0x020 Byte0 = 0xFF
Clear Error 0x030 Byte0 = 0xFF
Position feedback 0x302 Byte0 = 0x00
```

---

## 13. C Constant Example

```c
#define CAN_ID_ESTOP        0x001
#define CAN_ID_ENABLE       0x010
#define CAN_ID_HOMING       0x020
#define CAN_ID_CLEAR_ERROR  0x030

#define CAN_ID_BOARD2_MOVE  0x102
#define CAN_ID_BOARD2_STAT  0x202
#define CAN_ID_BOARD2_POS   0x302

#define BOARD2_GLOBAL_JOINT_ID   0
#define BOARD2_LOCAL_MOTOR_ID    0
#define BOARD2_GEAR_RATIO        20
#define MOTOR_STEPS_PER_REV      200
#define MICROSTEP                16
#define MOTOR_ALL                0xFF

#define BOARD2_MIN_POS_001DEG    -9000
#define BOARD2_MAX_POS_001DEG    18000
#define BOARD2_HOME_POS_001DEG   -9000

#define STATE_INIT          0
#define STATE_IDLE          1
#define STATE_HOMING        2
#define STATE_MOVING        3
#define STATE_ERROR         4
#define STATE_ESTOP         5
#define STATE_DISABLED      6

#define ERR_NONE                   0
#define ERR_INVALID_CMD            1
#define ERR_LIMIT_SWITCH_DETECTED  2
#define ERR_DRIVER_FAULT           3
#define ERR_HOMING_FAIL            4
#define ERR_QUEUE_FULL             5
#define ERR_RESERVED               6
```

---

## 14. Board2 Test Command Summary

```bash
# 전체 Enable
cansend can0 010#0100000000000000

# Board1 + Board2 전체 스텝모터 homing
cansend can0 020#FF00000000000000

# Board2 base_joint 30.00도 이동, 50ms
cansend can0 102#80B80B0000E8030A

# Board2 base_joint home -90.00도 이동, 50ms
cansend can0 102#80D8DCFFFFE8030A

# 전체 보드 error clear
cansend can0 030#FF00000000000000

# 전체 ESTOP
cansend can0 001#0000000000000000

# 기존 status 확인
candump can0,202:7FF

# current position feedback 확인
candump can0,302:7FF
```

---

## 15. 최종 요약

```text
1. Board2는 베이스 1축 base_joint를 담당한다.
2. Board2의 서버 global joint id는 0이다.
3. Board2의 CAN payload local Motor ID는 0이다.
4. Board2 위치 명령은 0x102, status는 0x202, position feedback은 0x302이다.
5. 기존 0x202 status frame은 변경하지 않는다.
6. 0x302 position feedback은 Byte0 local motor id, Byte1 flags, Byte2~5 int32 current_pos_001deg 구조이다.
7. current_pos_001deg 단위는 command target_pos와 같은 0.01도이다.
8. Board2 base_joint limit은 -90~180도, raw -9000~18000이다.
9. Board2 home position은 -90도, raw -9000이다.
10. Board2 gear ratio는 20이다.
11. Enable/Disable 0x010은 전체 broadcast이며 payload Board ID가 없다.
12. Homing 0x020은 Board1+Board2 stepper homing broadcast이며 payload Board ID가 없다.
13. Clear Error 0x030은 전체 broadcast이며 payload Board ID가 없다.
14. 서버/RPi 내부 board_id는 CAN ID 선택용이고, 실제 CAN payload에는 Board ID를 넣지 않는다.
```
