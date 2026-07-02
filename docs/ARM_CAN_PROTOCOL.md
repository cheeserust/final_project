# Board1 CAN Protocol FINAL — Integrated Version

## 0. 문서 목적

이 문서는 **전체 CAN 통합 기준에 맞춘 Board1 최종 프로토콜 수정본**입니다.

Board1은 같은 로봇팔 내부의 1~4축 스텝모터를 담당합니다. Board2는 같은 로봇팔의 5축을 담당하므로, Board1과 Board2는 중앙서버 / MoveIt2 관점에서는 하나의 로봇팔 제어계로 묶어서 동작해야 합니다.

이번 최종본에서 가장 중요한 수정 사항은 다음입니다.

```text
1. Board1의 Homing / Clear Error 명령에서 Byte0을 더 이상 Motor ID로 해석하지 않는다.
2. 공통 제어 명령 0x020 / 0x030은 Target Board + Target Local Motor ID 구조로 통일한다.
3. Board2와 충돌하지 않도록 Board1은 Target Board가 1 또는 전체 broadcast일 때만 처리한다.
4. 위치 명령 0x101은 기존처럼 Board1 전용이므로 local Motor ID 0~3을 사용한다.
```

---

## 1. 전체 통합 기준

### 1.1 Board 역할

| Board | 역할 | 전체 Axis ID | Board 내부 Local Motor ID | Move CAN ID | Status CAN ID | Position Feedback CAN ID |
|---|---|---:|---:|---:|---:|---:|
| Board1 | 로봇팔 1~4축 스텝모터 | 0~3 | 0~3 | `0x101` | `0x201` | `0x301` |
| Board2 | 로봇팔 5축 스텝모터 | 4 | 0 | `0x102` | `0x202` | `0x302` |
| Board3 | 그리퍼 서보 | gripper | 0~8 | `0x103` | `0x203` | `0x303` |

Board1과 Board2는 같은 로봇팔 내부의 축을 나누어 제어합니다.

```text
MoveIt2 joint1 → 전체 Axis ID 0 → Board1 local Motor ID 0 → CAN ID 0x101
MoveIt2 joint2 → 전체 Axis ID 1 → Board1 local Motor ID 1 → CAN ID 0x101
MoveIt2 joint3 → 전체 Axis ID 2 → Board1 local Motor ID 2 → CAN ID 0x101
MoveIt2 joint4 → 전체 Axis ID 3 → Board1 local Motor ID 3 → CAN ID 0x101
MoveIt2 joint5 → 전체 Axis ID 4 → Board2 local Motor ID 0 → CAN ID 0x102
```

### 1.2 Board1/Board2 Actual Position Feedback

기존 `0x201 / 0x202 / 0x203` status frame은 상태 판단용으로 그대로 유지한다.
MoveIt2 `/joint_states` actual position에 사용할 실제 위치는 별도 CAN ID로 송신한다.

| Board | Position Feedback CAN ID | 대상 |
|---|---:|---|
| Board1 | `0x301` | 1~4축 |
| Board2 | `0x302` | 5축 |

Payload는 8바이트 고정이다.

| Byte | 내용 |
|---:|---|
| 0 | Local Motor ID |
| 1 | Flags |
| 2~5 | `current_pos_001deg`, `int32_t` little endian |
| 6 | error/fault code, 없으면 `0` |
| 7 | reserved 또는 sequence counter |

Flags, Byte1:

| Bit | 의미 |
|---:|---|
| bit0 | position valid |
| bit1 | homed / ready |
| bit2 | moving |
| bit3 | target reached |
| bit4~7 | reserved, `0` |

`current_pos_001deg`는 모터 step 값이 아니라 중앙서버 / MoveIt2 joint 기준
출력축 각도다. 단위는 command `target_pos`와 동일하게 0.01도다.

| 실제 각도 | 전송 값 |
|---:|---:|
| `30.00 deg` | `3000` |
| `-15.50 deg` | `-1550` |

Board1은 20ms마다 local motor 순서대로 `0x301` frame 2개를 송신한다.
예를 들어 첫 주기에는 `0 -> 1`, 다음 주기에는 `2 -> 3`을 보내고,
이 패턴을 반복한다. Board2는 20ms마다 local motor `0`에 대해 `0x302`
frame 1개를 송신한다.

---

## 2. Board ID와 Broadcast 규칙

공통 제어 명령에서 사용하는 Target Board 값은 아래와 같이 통일합니다.

| 값 | 의미 |
|---:|---|
| `0x00` | 전체 보드 broadcast, legacy 호환용 |
| `0x01` | Board1 |
| `0x02` | Board2 |
| `0x03` | Board3 |
| `0xFF` | 전체 보드 broadcast, 최종 권장값 |

최종 중앙서버에서는 전체 명령을 보낼 때 `0xFF`를 권장합니다.
기존 전체 프로토콜과 호환이 필요한 경우 `0x00`도 전체 broadcast로 허용합니다.

---

## 3. Board1 CAN ID

| CAN ID | 방향 | 용도 | 처리 방식 |
|---:|---|---|---|
| `0x001` | 중앙서버 → 전체 보드 | Emergency Stop | Board1 처리 |
| `0x010` | 중앙서버 → 전체 보드 | Enable / Disable | Board1 처리 |
| `0x020` | 중앙서버 → 전체 보드 | Homing Start | Target Board 확인 후 처리 |
| `0x030` | 중앙서버 → 전체 보드 | Clear Error | Target Board 확인 후 처리 |
| `0x101` | 중앙서버 → Board1 | Board1 trajectory point | Board1 전용 처리 |
| `0x201` | Board1 → 중앙서버 | Board1 status | Board1 전용 송신 |
| `0x301` | Board1 → 중앙서버 | Board1 actual position feedback | Board1 전용 송신 |

---

## 4. `0x101` Board1 Position Command

Board1 위치 명령은 8바이트 고정 길이입니다.

```text
CAN ID = 0x101
DLC = 8
대상 = Board1 local Motor ID 0~3
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Local Motor ID | `uint8_t` | 상위 4bit flags, 하위 4bit local motor id |
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
Exec Rel  Step Rsv  | Local Motor ID
```

| Bit | 이름 | 의미 |
|---:|---|---|
| 7 | Execute | `1`: 명령 실행 또는 queue push, `0`: 무시 |
| 6 | Relative | `1`: 현재 위치 기준 상대 이동 |
| 5 | Step Mode | `0`: Target Pos는 0.01도 단위 angle, `1`: Target Pos는 step |
| 4 | Reserved | 현재 미사용, 반드시 `0` |
| 3~0 | Local Motor ID | Board1에서는 `0~3`만 유효 |

일반 절대 각도 명령은 다음과 같습니다.

```text
Execute = 1
Relative = 0
Step Mode = 0
Reserved = 0
Local Motor ID = 0~3
```

예시:

| Board1 축 | Local Motor ID | Byte0 |
|---:|---:|---:|
| 1축 | 0 | `0x80` |
| 2축 | 1 | `0x81` |
| 3축 | 2 | `0x82` |
| 4축 | 3 | `0x83` |

### 4.2 Target Pos

Target Pos는 `int32_t` little endian입니다.

| Step Mode | Target Pos 의미 |
|---:|---|
| `0` | 0.01도 단위 angle |
| `1` | step 수 |

예시:

| 실제 각도 | 전송 값 |
|---:|---:|
| `30.00 deg` | `3000` |
| `7.50 deg` | `750` |
| `-15.50 deg` | `-1550` |

### 4.3 각도 → step 변환

Board1은 출력축 기준 각도를 수신하고, STM32 내부에서 감속비를 반영해 step으로 변환합니다.

```c
step = angle_raw * gear_ratio[local_motor_id] * 200 * 16 / 36000;
```

| Local Motor ID | 실제 축 | Gear ratio |
|---:|---|---:|
| 0 | 1축 | 20 |
| 1 | 2축 | 20 |
| 2 | 3축 | 75 |
| 3 | 4축 | 30 |

### 4.4 Speed

`Speed`는 `uint16_t` little endian입니다.

| 항목 | 단위 |
|---|---|
| Speed | 0.01도/s |

초기 구현에서는 Speed를 queue에 저장하되, 실제 1ms 보간 계산에는 사용하지 않을 수 있습니다.
실제 이동 시간은 `Duration`이 결정합니다.

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

Board1 1축, 즉 local Motor ID 0을 30.00도로 50ms 동안 이동:

```text
CAN ID = 0x101
Byte0 = 0x80
Target Pos = 3000 = 0x00000BB8 little endian = B8 0B 00 00
Speed = 1000 = 0x03E8 little endian = E8 03
Duration = 10 = 0A
```

```bash
cansend can0 101#80B80B0000E8030A
```

---

## 5. 공통 제어 명령

## 5.1 Emergency Stop, CAN ID `0x001`

Emergency Stop은 안전 명령이므로 전체 보드 broadcast로 처리합니다.

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | ESTOP | `1` |
| 1~7 | Reserved | `0` |

예시:

```bash
cansend can0 001#0100000000000000
```

Board1 수신 시 동작:

```text
1. 모든 STEP 출력 정지
2. 모터 드라이버 Disable
3. trajectory queue clear
4. state = STATE_ESTOP
5. error_code = ERR_NONE 또는 ERR_ESTOP 정책값
6. 0x201 status 즉시 송신
```

ESTOP 해제는 Clear Error가 아니라 `0x010 Enable=1`에서 처리합니다.

---

## 5.2 Enable / Disable, CAN ID `0x010`

Enable / Disable은 기본적으로 전체 보드에 적용합니다.

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | Enable | `0`: Disable, `1`: Enable |
| 1 | Target Board | `0x00` 또는 `0xFF`: 전체, `0x01`: Board1, `0x02`: Board2, `0x03`: Board3 |
| 2~7 | Reserved | `0` |

호환성을 위해 Byte1이 `0x00`이면 전체 broadcast로 처리합니다.

예시:

```bash
# 전체 Enable, 기존 명령 호환
cansend can0 010#0100000000000000

# 전체 Enable, 최종 권장 broadcast
cansend can0 010#01FF000000000000

# Board1만 Enable
cansend can0 010#0101000000000000

# 전체 Disable
cansend can0 010#00FF000000000000
```

Board1 처리 조건:

```text
Target Board == 0x00 또는 0x01 또는 0xFF
```

Enable 수신 시:

```text
1. ESTOP flag 해제
2. error_code = ERR_NONE
3. motor enable pin 활성화
4. state = STATE_IDLE
5. 0x201 status 즉시 송신
```

Disable 수신 시:

```text
1. queue clear
2. STEP 출력 정지
3. motor disable
4. state = STATE_IDLE 또는 STATE_DISABLED 정책값
5. 0x201 status 즉시 송신
```

---

## 5.3 Homing Start, CAN ID `0x020`

`0x020`은 여러 보드가 함께 수신하는 공통 CAN ID입니다.
따라서 Board1은 반드시 Target Board를 확인한 뒤 처리합니다.

### 5.3.1 Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Board | `uint8_t` | `0x00` 또는 `0xFF`: 전체, `0x01`: Board1 |
| 1 | Target Local Motor ID | `uint8_t` | Board1에서는 `0~3` 또는 `255` |
| 2 | Homing Mode | `uint8_t` | 현재 `0`만 사용 |
| 3~7 | Reserved | - | `0` |

### 5.3.2 Board1 처리 조건

Board1은 아래 조건을 모두 만족할 때만 Homing을 수행합니다.

```text
Target Board == 0x00 또는 0x01 또는 0xFF
Target Local Motor ID == 0~3 또는 255
Homing Mode == 0
enabled == 1
state != STATE_ESTOP
```

조건이 맞지 않으면 `ERR_INVALID_CMD`를 설정하고 `0x201` status를 즉시 송신합니다.

### 5.3.3 Homing 명령 예시

```bash
# 전체 보드 전체 축 homing, legacy 호환
cansend can0 020#00FF000000000000

# 전체 보드 전체 축 homing, 최종 권장
cansend can0 020#FFFF000000000000

# Board1 전체 homing
cansend can0 020#01FF000000000000

# Board1 1축, local Motor ID 0 homing
cansend can0 020#0100000000000000

# Board1 3축, local Motor ID 2 homing
cansend can0 020#0102000000000000
```

주의:

```text
기존 Board1 방식처럼 0x020에서 Byte0을 Motor ID로 해석하면 안 됩니다.
Byte0은 항상 Target Board입니다.
```

---

## 5.4 Clear Error, CAN ID `0x030`

`0x030`도 공통 CAN ID입니다.
Board1은 Target Board를 확인한 뒤 처리합니다.

### 5.4.1 Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Board | `uint8_t` | `0x00` 또는 `0xFF`: 전체, `0x01`: Board1 |
| 1 | Target Local Motor ID | `uint8_t` | Board1에서는 `0~3` 또는 `255` |
| 2~7 | Reserved | - | `0` |

### 5.4.2 Board1 처리 조건

```text
Target Board == 0x00 또는 0x01 또는 0xFF
Target Local Motor ID == 0~3 또는 255
```

Clear Error 수신 시:

```text
1. error_code = ERR_NONE
2. fault 관련 내부 flag clear
3. 필요 시 staging/queue 관련 error flag clear
4. enabled == 1이면 state = STATE_IDLE
5. enabled == 0이면 state = STATE_INIT 또는 STATE_DISABLED 정책값
6. 0x201 status 즉시 송신
```

단, ESTOP 상태는 Clear Error만으로 해제하지 않습니다.
ESTOP 해제는 `0x010 Enable=1`에서 처리합니다.

### 5.4.3 Clear Error 명령 예시

```bash
# 전체 보드 전체 error clear
cansend can0 030#FFFF000000000000

# Board1 전체 error clear
cansend can0 030#01FF000000000000

# Board1 3축, local Motor ID 2 error clear
cansend can0 030#0102000000000000
```

---

## 6. Board1 Status, CAN ID `0x201`

Board1은 100ms마다 status를 송신합니다.
또한 ESTOP, Enable, Disable, Homing 완료, Clear Error, Queue Full 같은 주요 이벤트에서도 즉시 한 번 송신합니다.

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State | 현재 보드 상태 |
| 1 | Error Code | 현재 error code |
| 2 | Homing Done Bits | bit0~3 = local motor 0~3 homing done |
| 3 | Moving Local Motor ID | 이동 또는 homing 중인 첫 local motor id, 없으면 `255` |
| 4 | Limit Status Bits | bit0~3 = local motor 0~3 limit active |
| 5 | Queue Free | trajectory queue 남은 슬롯 수 |
| 6 | Enabled | `0`: disabled, `1`: enabled |
| 7 | Reserved | 현재 `0` |

예시:

```text
0x201 01 00 0F FF 00 20 01 00
```

해석:

```text
STATE_IDLE
ERR_NONE
local motor 0~3 homing 완료
이동 중인 모터 없음
limit inactive
queue free 32
enabled
reserved 0
```

---

## 7. State Values

| 값 | 이름 | 설명 |
|---:|---|---|
| `0` | `STATE_INIT` | 초기화 중 |
| `1` | `STATE_IDLE` | 대기 상태 |
| `2` | `STATE_HOMING` | 원점복귀 중 |
| `3` | `STATE_MOVING` | 이동 중 |
| `4` | `STATE_ERROR` | 에러 발생 |
| `5` | `STATE_ESTOP` | 비상정지 상태 |
| `6` | `STATE_DISABLED` | Disable 상태, 선택 구현 |

---

## 8. Error Codes

| 값 | 이름 | 의미 |
|---:|---|---|
| `0` | `ERR_NONE` | 정상 |
| `1` | `ERR_INVALID_CMD` | 잘못된 명령, target board, motor id, homing 전 move 등 |
| `2` | `ERR_LIMIT_DETECTED` | 리미트 감지 |
| `3` | `ERR_DRIVER_FAULT` | MCP2515, TMC, 드라이버 fault |
| `4` | `ERR_HOMING_FAIL` | 원점복귀 실패 |
| `5` | `ERR_QUEUE_FULL` | trajectory queue full |
| `6` | `ERR_RESERVED` | 예약 |

---

## 9. Homing and Move Policy

Board1은 오픈루프 스텝모터 보드이므로 원점복귀 완료 전에는 위치 명령을 실행하지 않습니다.

권장 순서:

```text
1. 중앙서버가 전체 Enable 전송
2. 중앙서버가 Board1 또는 전체 Homing Start 전송
3. Board1이 1~4축 원점복귀 수행
4. Board1이 0x201 status로 Homing Done Bits 송신
5. 중앙서버가 Board1 1~4축 homing 완료 확인
6. 이후 Board1 0x101 trajectory 전송 허용
```

위치 명령 수신 시 내부 조건:

```c
if (enabled == 0 || state == STATE_ESTOP) {
    error_code = ERR_INVALID_CMD;
    send_status_0x201();
    return;
}

if (homing_done[local_motor_id] == 0) {
    error_code = ERR_INVALID_CMD;
    send_status_0x201();
    return;
}
```

---

## 10. Queue and Error Policy

Board1 trajectory queue 크기는 32개를 권장합니다.

Queue full 상태에서 새 `0x101` 명령이 오면:

```text
1. 새 명령은 저장하지 않음, Drop Tail
2. 기존 queue 내용은 유지
3. ERR_QUEUE_FULL 설정
4. STATE_ERROR 전환
5. 0x201 status 즉시 송신
6. Clear Error 전까지 추가 move 명령 무시
```

---

## 11. 중앙서버 매핑 주의사항

Board1과 Board2는 같은 로봇팔 내부의 축을 나누어 제어합니다.
따라서 중앙서버는 전체 Axis ID를 그대로 CAN payload에 넣지 말고, 반드시 담당 보드와 local Motor ID로 변환해야 합니다.

```text
전체 Axis ID 0 → Board1, CAN ID 0x101, local Motor ID 0
전체 Axis ID 1 → Board1, CAN ID 0x101, local Motor ID 1
전체 Axis ID 2 → Board1, CAN ID 0x101, local Motor ID 2
전체 Axis ID 3 → Board1, CAN ID 0x101, local Motor ID 3
전체 Axis ID 4 → Board2, CAN ID 0x102, local Motor ID 0
```

Homing / Clear Error도 동일합니다.

```text
Board1 3축 homing:
Target Board = 1
Target Local Motor ID = 2

Board2 5축 homing:
Target Board = 2
Target Local Motor ID = 0
```

---

## 12. C Constant Example

```c
#define CAN_ID_ESTOP        0x001
#define CAN_ID_ENABLE       0x010
#define CAN_ID_HOMING       0x020
#define CAN_ID_CLEAR_ERROR  0x030

#define CAN_ID_BOARD1_MOVE  0x101
#define CAN_ID_BOARD1_STAT  0x201

#define BOARD_ID_ALL_LEGACY 0x00
#define BOARD_ID_1          0x01
#define BOARD_ID_2          0x02
#define BOARD_ID_3          0x03
#define BOARD_ID_ALL        0xFF

#define BOARD1_MOTOR_COUNT  4
#define MOTOR_ALL           0xFF

#define STATE_INIT          0
#define STATE_IDLE          1
#define STATE_HOMING        2
#define STATE_MOVING        3
#define STATE_ERROR         4
#define STATE_ESTOP         5
#define STATE_DISABLED      6

#define ERR_NONE            0
#define ERR_INVALID_CMD     1
#define ERR_LIMIT_DETECTED  2
#define ERR_DRIVER_FAULT    3
#define ERR_HOMING_FAIL     4
#define ERR_QUEUE_FULL      5
#define ERR_RESERVED        6
```

---

## 13. Board1 Test Command Summary

```bash
# 전체 Enable
cansend can0 010#01FF000000000000

# Board1 전체 homing
cansend can0 020#01FF000000000000

# Board1 1축 homing
cansend can0 020#0100000000000000

# Board1 1축 30.00도 이동, 50ms
cansend can0 101#80B80B0000E8030A

# Board1 전체 error clear
cansend can0 030#01FF000000000000

# 전체 ESTOP
cansend can0 001#0100000000000000

# Status 확인
candump can0
```

---

## 14. 최종 요약

```text
1. Board1은 로봇팔 1~4축을 담당한다.
2. Board2는 같은 로봇팔의 5축을 담당한다.
3. Board1 위치 명령은 0x101, status는 0x201, actual position feedback은 0x301이다.
4. 0x101 payload Motor ID는 Board1 local Motor ID 0~3이다.
5. Homing 0x020과 Clear Error 0x030은 Target Board + Target Local Motor ID 구조를 사용한다.
6. Board1은 Target Board가 1, 0, 255일 때만 공통 명령을 처리한다.
7. 기존처럼 0x020 / 0x030의 Byte0을 Motor ID로 해석하면 Board2와 충돌하므로 금지한다.
8. 중앙서버는 전체 Axis ID를 Board ID + local Motor ID로 변환해서 CAN frame을 생성한다.
```

# Board2 CAN Protocol FINAL — Integrated Version

## 0. 문서 목적

이 문서는 **전체 CAN 통합 기준에 맞춘 Board2 최종 프로토콜 수정본**입니다.

Board2는 같은 로봇팔 내부의 5축 스텝모터를 담당합니다. Board1은 같은 로봇팔의 1~4축을 담당하므로, Board1과 Board2는 중앙서버 / MoveIt2 관점에서는 하나의 로봇팔 제어계로 묶어서 동작해야 합니다.

이번 최종본의 핵심은 다음입니다.

```text
1. Board2는 전체 시스템 기준 Axis ID 4, 즉 로봇팔 5축을 담당한다.
2. 하지만 Board2 CAN payload 내부 Motor ID는 0으로 고정한다.
3. 전체 Axis ID와 Board 내부 Local Motor ID를 분리한다.
4. Homing / Clear Error는 Target Board + Target Local Motor ID 구조로 처리한다.
5. Board1과 같은 CAN bus에 있어도 0x020 / 0x030 공통 명령이 충돌하지 않도록 한다.
```

---

## 1. 전체 통합 기준

### 1.1 Board 역할

| Board | 역할 | 전체 Axis ID | Board 내부 Local Motor ID | Move CAN ID | Status CAN ID | Position Feedback CAN ID |
|---|---|---:|---:|---:|---:|---:|
| Board1 | 로봇팔 1~4축 스텝모터 | 0~3 | 0~3 | `0x101` | `0x201` | `0x301` |
| Board2 | 로봇팔 5축 스텝모터 | 4 | 0 | `0x102` | `0x202` | `0x302` |
| Board3 | 그리퍼 서보 | gripper | 0~8 | `0x103` | `0x203` | `0x303` |

Board2의 핵심 매핑은 다음과 같습니다.

```text
전체 시스템 기준 Axis ID: 4
MoveIt2 Joint: joint5
담당 보드: Board2
Board ID: 2
Board2 local Motor ID: 0
Move CAN ID: 0x102
Status CAN ID: 0x202
```

중앙서버는 joint5 / 전체 Axis ID 4를 다음처럼 변환해야 합니다.

```text
MoveIt2 joint5 target angle
→ 전체 Axis ID 4
→ Board2 담당 확인
→ CAN ID 0x102 선택
→ payload local Motor ID 0 사용
```

---

## 2. Board ID와 Broadcast 규칙

공통 제어 명령에서 사용하는 Target Board 값은 아래와 같이 통일합니다.

| 값 | 의미 |
|---:|---|
| `0x00` | 전체 보드 broadcast, legacy 호환용 |
| `0x01` | Board1 |
| `0x02` | Board2 |
| `0x03` | Board3 |
| `0xFF` | 전체 보드 broadcast, 최종 권장값 |

최종 중앙서버에서는 전체 명령을 보낼 때 `0xFF`를 권장합니다.
기존 전체 프로토콜과 호환이 필요한 경우 `0x00`도 전체 broadcast로 허용합니다.

---

## 3. Board2 CAN ID

| CAN ID | 방향 | 용도 | 처리 방식 |
|---:|---|---|---|
| `0x001` | 중앙서버 → 전체 보드 | Emergency Stop | Board2 처리 |
| `0x010` | 중앙서버 → 전체 보드 | Enable / Disable | Board2 처리 |
| `0x020` | 중앙서버 → 전체 보드 | Homing Start | Target Board 확인 후 처리 |
| `0x030` | 중앙서버 → 전체 보드 | Clear Error | Target Board 확인 후 처리 |
| `0x102` | 중앙서버 → Board2 | Board2 trajectory point | Board2 전용 처리 |
| `0x202` | Board2 → 중앙서버 | Board2 status | Board2 전용 송신 |
| `0x302` | Board2 → 중앙서버 | Board2 actual position feedback | Board2 전용 송신 |

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
| 0 | Control & Local Motor ID | `uint8_t` | 상위 4bit flags, 하위 4bit local motor id |
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
Exec Rel  Step Rsv  | Local Motor ID
```

| Bit | 이름 | 의미 |
|---:|---|---|
| 7 | Execute | `1`: 명령 실행 또는 queue push, `0`: 무시 |
| 6 | Relative | `1`: 현재 위치 기준 상대 이동 |
| 5 | Step Mode | `0`: Target Pos는 0.01도 단위 angle, `1`: Target Pos는 step |
| 4 | Reserved | 현재 미사용, 반드시 `0` |
| 3~0 | Local Motor ID | Board2에서는 `0`만 유효 |

Board2 일반 절대 각도 명령은 항상 다음 값을 사용합니다.

```text
Execute = 1
Relative = 0
Step Mode = 0
Reserved = 0
Local Motor ID = 0

Byte0 = 0x80
```

중요:

```text
기존 Axis ID 4 방식인 Byte0 = 0x84는 사용하지 않는다.
Board2는 CAN ID 0x102로 보드를 구분하고, payload Motor ID는 0으로 사용한다.
```

### 4.2 Target Pos

Target Pos는 `int32_t` little endian입니다.

| Step Mode | Target Pos 의미 |
|---:|---|
| `0` | 0.01도 단위 angle |
| `1` | step 수 |

예시:

| 실제 각도 | 전송 값 |
|---:|---:|
| `30.00 deg` | `3000` |
| `7.50 deg` | `750` |
| `-15.50 deg` | `-1550` |

### 4.3 각도 → step 변환

Board2는 출력축 기준 각도를 수신하고, STM32 내부에서 감속비를 반영해 step으로 변환합니다.

```c
step = angle_raw * BOARD2_GEAR_RATIO * MOTOR_STEPS_PER_REV * MICROSTEP / 36000;
```

기본값:

```c
#define BOARD2_AXIS_ID          4
#define BOARD2_LOCAL_MOTOR_ID   0
#define BOARD2_GEAR_RATIO       120
#define MOTOR_STEPS_PER_REV     200
#define MICROSTEP               16
```

예시:

```text
7.50 deg → angle_raw = 750
step = 750 × 120 × 200 × 16 / 36000
     = 8000 steps
```

### 4.4 Speed

`Speed`는 `uint16_t` little endian입니다.

| 항목 | 단위 |
|---|---|
| Speed | 0.01도/s |

초기 구현에서는 Speed를 queue에 저장하되, 실제 1ms 보간 계산에는 사용하지 않을 수 있습니다.
실제 이동 시간은 `Duration`이 결정합니다.

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

Board2 5축, 즉 local Motor ID 0을 7.50도로 50ms 동안 이동:

```text
CAN ID = 0x102
Byte0 = 0x80
Target Pos = 750 = 0x000002EE little endian = EE 02 00 00
Speed = 1000 = 0x03E8 little endian = E8 03
Duration = 10 = 0A
```

```bash
cansend can0 102#80EE020000E8030A
```

잘못된 예시:

```bash
# 사용 금지: payload Motor ID를 전체 Axis ID 4로 넣은 예전 방식
cansend can0 102#84EE020000E8030A
```

---

## 5. 공통 제어 명령

## 5.1 Emergency Stop, CAN ID `0x001`

Emergency Stop은 안전 명령이므로 전체 보드 broadcast로 처리합니다.

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | ESTOP | `1` |
| 1~7 | Reserved | `0` |

예시:

```bash
cansend can0 001#0100000000000000
```

Board2 수신 시 동작:

```text
1. STEP 출력 정지
2. 모터 드라이버 Disable
3. trajectory queue clear
4. state = STATE_ESTOP
5. error_code = ERR_NONE 또는 ERR_ESTOP 정책값
6. 0x202 status 즉시 송신
```

ESTOP 해제는 Clear Error가 아니라 `0x010 Enable=1`에서 처리합니다.

---

## 5.2 Enable / Disable, CAN ID `0x010`

Enable / Disable은 기본적으로 전체 보드에 적용합니다.

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | Enable | `0`: Disable, `1`: Enable |
| 1 | Target Board | `0x00` 또는 `0xFF`: 전체, `0x01`: Board1, `0x02`: Board2, `0x03`: Board3 |
| 2~7 | Reserved | `0` |

호환성을 위해 Byte1이 `0x00`이면 전체 broadcast로 처리합니다.

예시:

```bash
# 전체 Enable, 기존 명령 호환
cansend can0 010#0100000000000000

# 전체 Enable, 최종 권장 broadcast
cansend can0 010#01FF000000000000

# Board2만 Enable
cansend can0 010#0102000000000000

# 전체 Disable
cansend can0 010#00FF000000000000
```

Board2 처리 조건:

```text
Target Board == 0x00 또는 0x02 또는 0xFF
```

Enable 수신 시:

```text
1. ESTOP flag 해제
2. error_code = ERR_NONE
3. motor enable pin 활성화
4. state = STATE_IDLE
5. 0x202 status 즉시 송신
```

Disable 수신 시:

```text
1. queue clear
2. STEP 출력 정지
3. motor disable
4. state = STATE_IDLE 또는 STATE_DISABLED 정책값
5. 0x202 status 즉시 송신
```

---

## 5.3 Homing Start, CAN ID `0x020`

`0x020`은 여러 보드가 함께 수신하는 공통 CAN ID입니다.
따라서 Board2는 반드시 Target Board를 확인한 뒤 처리합니다.

### 5.3.1 Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Board | `uint8_t` | `0x00` 또는 `0xFF`: 전체, `0x02`: Board2 |
| 1 | Target Local Motor ID | `uint8_t` | Board2에서는 `0` 또는 `255` |
| 2 | Homing Mode | `uint8_t` | 현재 `0`만 사용 |
| 3~7 | Reserved | - | `0` |

### 5.3.2 Board2 처리 조건

Board2는 아래 조건을 모두 만족할 때만 Homing을 수행합니다.

```text
Target Board == 0x00 또는 0x02 또는 0xFF
Target Local Motor ID == 0 또는 255
Homing Mode == 0
enabled == 1
state != STATE_ESTOP
```

조건이 맞지 않으면 `ERR_INVALID_CMD`를 설정하고 `0x202` status를 즉시 송신합니다.

### 5.3.3 Homing 명령 예시

```bash
# 전체 보드 전체 축 homing, legacy 호환
cansend can0 020#00FF000000000000

# 전체 보드 전체 축 homing, 최종 권장
cansend can0 020#FFFF000000000000

# Board2 5축 homing, local Motor ID 0
cansend can0 020#0200000000000000

# Board2 전체 homing, Board2는 단일 모터이므로 위 명령과 동일 효과
cansend can0 020#02FF000000000000
```

주의:

```text
Board2의 Homing Byte1은 전체 Axis ID 4가 아니라 Board2 local Motor ID 0으로 사용한다.
중앙서버가 전체 Axis ID 4를 Board2 local Motor ID 0으로 변환해야 한다.
```

---

## 5.4 Clear Error, CAN ID `0x030`

`0x030`도 공통 CAN ID입니다.
Board2는 Target Board를 확인한 뒤 처리합니다.

### 5.4.1 Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Board | `uint8_t` | `0x00` 또는 `0xFF`: 전체, `0x02`: Board2 |
| 1 | Target Local Motor ID | `uint8_t` | Board2에서는 `0` 또는 `255` |
| 2~7 | Reserved | - | `0` |

### 5.4.2 Board2 처리 조건

```text
Target Board == 0x00 또는 0x02 또는 0xFF
Target Local Motor ID == 0 또는 255
```

Clear Error 수신 시:

```text
1. error_code = ERR_NONE
2. fault 관련 내부 flag clear
3. 필요 시 queue 관련 error flag clear
4. enabled == 1이면 state = STATE_IDLE
5. enabled == 0이면 state = STATE_INIT 또는 STATE_DISABLED 정책값
6. 0x202 status 즉시 송신
```

단, ESTOP 상태는 Clear Error만으로 해제하지 않습니다.
ESTOP 해제는 `0x010 Enable=1`에서 처리합니다.

### 5.4.3 Clear Error 명령 예시

```bash
# 전체 보드 전체 error clear
cansend can0 030#FFFF000000000000

# Board2 단일 모터 error clear
cansend can0 030#0200000000000000

# Board2 전체 error clear
cansend can0 030#02FF000000000000
```

---

## 6. Board2 Status, CAN ID `0x202`

Board2는 100ms마다 status를 송신합니다.
또한 ESTOP, Enable, Disable, Homing 완료, Clear Error, Queue Full 같은 주요 이벤트에서도 즉시 한 번 송신합니다.

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State | 현재 보드 상태 |
| 1 | Error Code | 현재 error code |
| 2 | Homing Done | `0`: 미완료, `1`: 완료 |
| 3 | Moving Local Motor ID | 이동 또는 homing 중이면 `0`, 없으면 `255` |
| 4 | Limit Status | 5축 limit switch 상태, `0`: inactive, `1`: active |
| 5 | Queue Free | trajectory queue 남은 슬롯 수 |
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
Homing 완료
이동 중인 모터 없음
Limit inactive
Queue free 32
Enabled
Reserved 0
```

---

## 7. State Values

| 값 | 이름 | 설명 |
|---:|---|---|
| `0` | `STATE_INIT` | 초기화 중 |
| `1` | `STATE_IDLE` | 대기 상태 |
| `2` | `STATE_HOMING` | 원점복귀 중 |
| `3` | `STATE_MOVING` | 이동 중 |
| `4` | `STATE_ERROR` | 에러 발생 |
| `5` | `STATE_ESTOP` | 비상정지 상태 |
| `6` | `STATE_DISABLED` | Disable 상태, 선택 구현 |

---

## 8. Error Codes

| 값 | 이름 | 의미 |
|---:|---|---|
| `0` | `ERR_NONE` | 정상 |
| `1` | `ERR_INVALID_CMD` | 잘못된 명령, target board, motor id, homing 전 move 등 |
| `2` | `ERR_LIMIT_DETECTED` | 리미트 감지 |
| `3` | `ERR_DRIVER_FAULT` | MCP2515, TMC, 드라이버 fault |
| `4` | `ERR_HOMING_FAIL` | 원점복귀 실패 |
| `5` | `ERR_QUEUE_FULL` | trajectory queue full |
| `6` | `ERR_RESERVED` | 예약 |

---

## 9. Homing and Move Policy

Board2는 오픈루프 스텝모터 보드이므로 원점복귀 완료 전에는 위치 명령을 실행하지 않습니다.

권장 순서:

```text
1. 중앙서버가 전체 Enable 전송
2. 중앙서버가 Board2 또는 전체 Homing Start 전송
3. Board2가 5축 원점복귀 수행
4. Board2가 0x202 status로 Homing Done = 1 송신
5. 중앙서버가 Board2 5축 homing 완료 확인
6. 이후 Board2 0x102 trajectory 전송 허용
```

위치 명령 수신 시 내부 조건:

```c
if (enabled == 0 || state == STATE_ESTOP) {
    error_code = ERR_INVALID_CMD;
    send_status_0x202();
    return;
}

if (homing_done == 0) {
    error_code = ERR_INVALID_CMD;
    send_status_0x202();
    return;
}
```

---

## 10. Queue and Error Policy

Board2 trajectory queue 크기는 32개를 권장합니다.

Queue full 상태에서 새 `0x102` 명령이 오면:

```text
1. 새 명령은 저장하지 않음, Drop Tail
2. 기존 queue 내용은 유지
3. ERR_QUEUE_FULL 설정
4. STATE_ERROR 전환
5. 0x202 status 즉시 송신
6. Clear Error 전까지 추가 move 명령 무시
```

---

## 11. 중앙서버 매핑 주의사항

Board1과 Board2는 같은 로봇팔 내부의 축을 나누어 제어합니다.
따라서 중앙서버는 전체 Axis ID를 그대로 CAN payload에 넣지 말고, 반드시 담당 보드와 local Motor ID로 변환해야 합니다.

```text
전체 Axis ID 0 → Board1, CAN ID 0x101, local Motor ID 0
전체 Axis ID 1 → Board1, CAN ID 0x101, local Motor ID 1
전체 Axis ID 2 → Board1, CAN ID 0x101, local Motor ID 2
전체 Axis ID 3 → Board1, CAN ID 0x101, local Motor ID 3
전체 Axis ID 4 → Board2, CAN ID 0x102, local Motor ID 0
```

Board2와 관련된 핵심 규칙:

```text
Move command 0x102 Byte0 = 0x80
Homing 0x020 Byte0 = 0x02, Byte1 = 0x00 또는 0xFF
Clear Error 0x030 Byte0 = 0x02, Byte1 = 0x00 또는 0xFF
```

---

## 12. C Constant Example

```c
#define CAN_ID_ESTOP        0x001
#define CAN_ID_ENABLE       0x010
#define CAN_ID_HOMING       0x020
#define CAN_ID_CLEAR_ERROR  0x030

#define CAN_ID_BOARD2_MOVE  0x102
#define CAN_ID_BOARD2_STAT  0x202

#define BOARD_ID_ALL_LEGACY 0x00
#define BOARD_ID_1          0x01
#define BOARD_ID_2          0x02
#define BOARD_ID_3          0x03
#define BOARD_ID_ALL        0xFF

#define BOARD2_AXIS_ID          4
#define BOARD2_LOCAL_MOTOR_ID   0
#define BOARD2_GEAR_RATIO       120
#define MOTOR_ALL               0xFF

#define STATE_INIT          0
#define STATE_IDLE          1
#define STATE_HOMING        2
#define STATE_MOVING        3
#define STATE_ERROR         4
#define STATE_ESTOP         5
#define STATE_DISABLED      6

#define ERR_NONE            0
#define ERR_INVALID_CMD     1
#define ERR_LIMIT_DETECTED  2
#define ERR_DRIVER_FAULT    3
#define ERR_HOMING_FAIL     4
#define ERR_QUEUE_FULL      5
#define ERR_RESERVED        6
```

---

## 13. Board2 Test Command Summary

```bash
# 전체 Enable
cansend can0 010#01FF000000000000

# Board2 5축 homing
cansend can0 020#0200000000000000

# Board2 전체 homing
cansend can0 020#02FF000000000000

# Board2 5축 7.50도 이동, 50ms
cansend can0 102#80EE020000E8030A

# Board2 단일 모터 error clear
cansend can0 030#0200000000000000

# Board2 전체 error clear
cansend can0 030#02FF000000000000

# 전체 ESTOP
cansend can0 001#0100000000000000

# Status 확인
candump can0
```

---

## 14. 최종 요약

```text
1. Board2는 로봇팔 5축을 담당한다.
2. 전체 Axis ID는 4이지만, Board2 payload local Motor ID는 0이다.
3. Board2 위치 명령은 0x102, status는 0x202, actual position feedback은 0x302이다.
4. 0x102 일반 절대 이동 명령의 Byte0은 0x80이다.
5. 기존 0x84 방식은 사용하지 않는다.
6. Homing 0x020과 Clear Error 0x030은 Target Board + Target Local Motor ID 구조를 사용한다.
7. Board2는 Target Board가 2, 0, 255일 때만 공통 명령을 처리한다.
8. 중앙서버는 전체 Axis ID 4를 Board2 local Motor ID 0으로 변환해서 CAN frame을 생성한다.
```

# Board3 CAN Protocol FINAL — Integrated Version

## 0. 문서 목적

이 문서는 **전체 CAN 통합 기준에 맞춘 Board3 최종 프로토콜 수정본**입니다.

Board3는 로봇팔 끝단의 3-finger gripper를 담당합니다. Board1과 Board2는 같은 로봇팔의 스텝모터 1~5축을 나누어 제어하고, Board3는 그리퍼 서보 9개를 제어합니다.

이번 최종본의 핵심은 다음입니다.

```text
1. Board3 명령 CAN ID는 0x103, status CAN ID는 0x203으로 고정한다.
2. 0x103 frame 하나는 서보 1개 명령이다.
3. 그리퍼 1회 동작은 Motor ID 0~8의 9개 frame이 모두 모였을 때만 성립한다.
4. Homing / Clear Error는 Board1, Board2와 같은 Target Board + Target Local Motor ID 구조를 사용한다.
5. Board3 homing은 limit switch 원점 탐색이 아니라 모든 gripper joint target angle을 0.00도로 보내는 home posture 명령이다.
6. Board3 실제 위치 피드백은 0x303 3프레임 압축 구조로 송신한다.
```

---

## 1. 전체 통합 기준

### 1.1 Board 역할

| Board | 역할 | 전체 Axis ID | Board 내부 Local Motor ID | Move CAN ID | Status CAN ID | Position Feedback CAN ID |
|---|---|---:|---:|---:|---:|---:|
| Board1 | 로봇팔 1~4축 스텝모터 | 0~3 | 0~3 | `0x101` | `0x201` | `0x301` |
| Board2 | 로봇팔 5축 스텝모터 | 4 | 0 | `0x102` | `0x202` | `0x302` |
| Board3 | 3-finger gripper 서보 | gripper | 0~8 | `0x103` | `0x203` | `0x303` |

Board3는 로봇팔 본체 축이 아니라 말단 그리퍼를 담당합니다.

```text
중앙서버 / Raspberry Pi
  ↓ CAN
Board3 STM32
  ↓ UART1 + TTL 변환보드
SCS0009 서보 9개
```

---

## 2. Board ID와 Broadcast 규칙

공통 제어 명령에서 사용하는 Target Board 값은 아래와 같이 통일합니다.

| 값 | 의미 |
|---:|---|
| `0x00` | 전체 보드 broadcast, legacy 호환용 |
| `0x01` | Board1 |
| `0x02` | Board2 |
| `0x03` | Board3 |
| `0xFF` | 전체 보드 broadcast, 최종 권장값 |

최종 중앙서버에서는 전체 명령을 보낼 때 `0xFF`를 권장합니다.
기존 전체 프로토콜과 호환이 필요한 경우 `0x00`도 전체 broadcast로 허용합니다.

---

## 3. Board3 CAN ID

| CAN ID | 방향 | 용도 | 처리 방식 |
|---:|---|---|---|
| `0x001` | 중앙서버 → 전체 보드 | Emergency Stop | Board3 처리 |
| `0x010` | 중앙서버 → 전체 보드 | Enable / Disable | Board3 처리 |
| `0x020` | 중앙서버 → 전체 보드 | Homing Start / Home Posture | Target Board 확인 후 처리 |
| `0x030` | 중앙서버 → 전체 보드 | Clear Error | Target Board 확인 후 처리 |
| `0x103` | 중앙서버 → Board3 | Gripper Servo Command | Board3 전용 처리 |
| `0x203` | Board3 → 중앙서버 | Board3 Status | Board3 전용 송신 |
| `0x303` | Board3 → 중앙서버 | Gripper Actual Position Feedback | Board3 전용 송신 |

---

## 4. `0x103` Board3 Gripper Servo Command

`0x103` frame 하나는 서보 1개 명령입니다.
Board3는 서보 9개를 담당하므로, 그리퍼 전체 1회 동작은 `0x103` frame 9개로 구성됩니다.

```text
0x103 Motor ID 0 명령
0x103 Motor ID 1 명령
0x103 Motor ID 2 명령
0x103 Motor ID 3 명령
0x103 Motor ID 4 명령
0x103 Motor ID 5 명령
0x103 Motor ID 6 명령
0x103 Motor ID 7 명령
0x103 Motor ID 8 명령
```

```text
CAN frame 1개 = 서보 1개 명령
CAN frame 9개 = gripper 전체 1회 command set
```

---

## 5. `0x103` Payload 구조

```text
CAN ID = 0x103
DLC = 8
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Local Motor ID | `uint8_t` | Bit7 Execute, Bit3~0 local motor id |
| 1 | Target Position LSB | `int32_t` 일부 | 0.01도 단위, little endian |
| 2 | Target Position | `int32_t` 일부 | 0.01도 단위, little endian |
| 3 | Target Position | `int32_t` 일부 | 0.01도 단위, little endian |
| 4 | Target Position MSB | `int32_t` 일부 | 0.01도 단위, little endian |
| 5 | Speed LSB | `uint16_t` 일부 | 현재 미사용, `0` 권장 |
| 6 | Speed MSB | `uint16_t` 일부 | 현재 미사용, `0` 권장 |
| 7 | Duration | `uint8_t` | 5ms 단위 tick |

---

## 6. Byte0 Control & Local Motor ID

```text
Bit7 Bit6 Bit5 Bit4 | Bit3 Bit2 Bit1 Bit0
Exec Rsv  Rsv  Rsv  | Local Motor ID
```

| Bit | 이름 | 의미 |
|---:|---|---|
| 7 | Execute | `1`: staging 대상으로 처리 |
| 6 | Reserved | 현재 미사용, 반드시 `0` |
| 5 | Reserved | 현재 미사용, 반드시 `0` |
| 4 | Reserved | 현재 미사용, 반드시 `0` |
| 3~0 | Local Motor ID | `0~8`만 유효 |

예상 Byte0 값:

| Local Motor ID | Byte0 |
|---:|---:|
| 0 | `0x80` |
| 1 | `0x81` |
| 2 | `0x82` |
| 3 | `0x83` |
| 4 | `0x84` |
| 5 | `0x85` |
| 6 | `0x86` |
| 7 | `0x87` |
| 8 | `0x88` |

수신부 해석 예시:

```c
uint8_t execute  = (data[0] & 0x80U) ? 1U : 0U;
uint8_t reserved = data[0] & 0x70U;
uint8_t motor_id = data[0] & 0x0FU;
```

유효 조건:

```text
execute == 1
reserved == 0
motor_id <= 8
```

---

## 7. Local Motor ID와 Servo ID 매핑

통신 프로토콜에서는 local Motor ID `0~8`을 사용합니다.
실제 SCS0009 Servo ID는 `1~9`로 매핑합니다.

| Local Motor ID | Joint 이름 | SCS0009 Servo ID |
|---:|---|---:|
| 0 | `finger_1_base_joint` | 1 |
| 1 | `finger_1_middle_joint` | 2 |
| 2 | `finger_1_tip_joint` | 3 |
| 3 | `finger_2_base_joint` | 4 |
| 4 | `finger_2_middle_joint` | 5 |
| 5 | `finger_2_tip_joint` | 6 |
| 6 | `finger_3_base_joint` | 7 |
| 7 | `finger_3_middle_joint` | 8 |
| 8 | `finger_3_tip_joint` | 9 |

코드 매핑:

```c
static const uint8_t motor_id_to_servo_id[9] = {
    1, 2, 3,
    4, 5, 6,
    7, 8, 9
};
```

---

## 8. Target Position 단위

`Byte1~4`는 `int32_t` little endian입니다.
단위는 0.01도입니다.

| 실제 각도 | CAN 값 |
|---:|---:|
| `30.00°` | `3000` |
| `90.00°` | `9000` |
| `-15.00°` | `-1500` |
| `0.00°` | `0` |

중요:

```text
Target Position은 SCS0009 servo position 값이 아니다.
Target Position은 중앙서버가 보낸 0.01도 단위 각도값이다.
```

따라서 `g_cmd`에는 서보 position 값이 아니라 0.01도 단위 각도값을 저장합니다.

```text
g_cmd.target_pos_001deg[0~8] = 0.01도 단위 각도값
```

실제 SCS0009 position 값으로 변환하는 작업은 servo adapter에서 수행합니다.

---

## 9. Duration 단위와 처리 방식

`Byte7`은 duration입니다.
Board3 CAN 프로토콜에서 duration은 5ms 단위 원본 tick입니다.

```text
duration_ms = duration_5ms × 5
```

예시:

| CAN Byte7 | 실제 시간 |
|---:|---:|
| 20 | 100ms |
| 40 | 200ms |
| 100 | 500ms |

`g_cmd`에는 CAN에서 받은 원본 tick 값을 그대로 저장합니다.

```text
g_cmd.duration_5ms = CAN Byte7 원본값
```

제어팀은 Feetech 함수 호출 직전에 ms 단위로 변환합니다.

```c
uint16_t duration_ms = (uint16_t)g_cmd.duration_5ms * 5U;
```

---

## 10. Speed 필드

`Byte5~6`은 Speed 필드로 남겨두지만, 현재 버전에서는 사용하지 않습니다.

```text
Speed = 0 권장
```

처리 정책:

```text
수신은 하지만 동작 계산에는 사용하지 않음
```

향후 속도 제어가 필요하면 프로토콜 v2에서 재정의합니다.

---

## 11. 9개 Frame Staging 구조

Board3는 `0x103` frame을 받자마자 바로 제어팀에 넘기지 않습니다.
먼저 local Motor ID별 staging buffer에 저장합니다.

```text
Motor ID 0 → staging_buffer[0]
Motor ID 1 → staging_buffer[1]
...
Motor ID 8 → staging_buffer[8]
```

9개가 모두 모였을 때만 하나의 gripper 동작 명령으로 처리합니다.

```text
9개 모두 수신 성공
→ command set 유효성 검사
→ g_cmd에 복사
→ g_cmd.is_new_cmd = 1
```

일부만 들어온 경우에는 제어팀으로 넘기지 않고 폐기합니다.

---

## 12. Staging 성공 조건

하나의 command set은 아래 조건을 모두 만족해야 합니다.

| 조건 | 설명 |
|---|---|
| Motor ID 범위 | 모든 frame의 Motor ID가 `0~8` |
| Execute bit | 모든 frame의 Execute bit가 `1` |
| Reserved bit | Byte0의 Bit6~4가 모두 `0` |
| 중복 Motor ID 없음 | 같은 Motor ID가 한 command set 안에서 두 번 오면 안 됨 |
| 9개 frame 모두 수신 | Motor ID `0~8`이 각각 한 번씩 수신되어야 함 |
| Timeout 내 수신 | 첫 frame 이후 timeout 안에 9개가 모두 도착해야 함 |
| Duration 동일 | 9개 frame의 Duration 값이 모두 같아야 함 |
| Enable 상태 | `enabled == 1` |
| ESTOP 아님 | `state != STATE_ESTOP` |

하드웨어 연결 전 기본 timeout:

```text
GRIPPER_STAGING_TIMEOUT_MS = 100ms
```

최종 목표 timeout:

```text
20ms
```

하드웨어 연결 전에는 디버깅 안정성을 위해 100ms를 기본값으로 사용하고, 추후 통신 안정화 후 20ms로 줄입니다.

---

## 13. 잘못된 Command Set 처리

아래 상황에서는 전체 command set을 폐기합니다.

```text
- Motor ID가 0~8 범위 밖
- 같은 Motor ID가 중복 수신됨
- 9개 frame 중 일부만 들어옴
- timeout 발생
- 9개 frame의 duration이 서로 다름
- Execute bit가 0
- Reserved bit가 0이 아님
- ESTOP 상태
- Disable 상태
```

폐기 시 처리:

```text
1. staging buffer clear
2. g_cmd.is_new_cmd는 올리지 않음
3. error_code 설정
4. 0x203 status 즉시 송신
```

---

## 14. g_cmd 전달 구조

정상 command set이 완성되면 통신부는 `g_cmd`를 다음처럼 채웁니다.

```text
g_cmd.target_pos_001deg[0] = Motor ID 0 target angle
g_cmd.target_pos_001deg[1] = Motor ID 1 target angle
...
g_cmd.target_pos_001deg[8] = Motor ID 8 target angle

g_cmd.duration_5ms = 공통 duration 원본값
g_cmd.is_new_cmd = 1
```

제어부는 `g_cmd.is_new_cmd == 1`을 확인한 뒤 값을 읽고, 처리가 끝나면 `g_cmd.is_new_cmd = 0`으로 clear합니다.

---

## 15. 공통 제어 명령

## 15.1 Emergency Stop, CAN ID `0x001`

Emergency Stop은 안전 명령이므로 전체 보드 broadcast로 처리합니다.

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | ESTOP | `1` |
| 1~7 | Reserved | `0` |

예시:

```bash
cansend can0 001#0100000000000000
```

Board3 수신 시 동작:

```text
1. staging buffer clear
2. g_cmd.is_new_cmd = 0
3. 실제 서보 stop 또는 안전 처리, 하드웨어 연결 전에는 상태만 처리
4. state = STATE_ESTOP
5. error_code = ERR_ESTOP
6. enabled = 0
7. 0x203 status 즉시 송신
```

ESTOP 해제는 Clear Error가 아니라 `0x010 Enable=1`에서 처리합니다.

---

## 15.2 Enable / Disable, CAN ID `0x010`

Enable / Disable은 기본적으로 전체 보드에 적용합니다.

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | Enable | `0`: Disable, `1`: Enable |
| 1 | Target Board | `0x00` 또는 `0xFF`: 전체, `0x01`: Board1, `0x02`: Board2, `0x03`: Board3 |
| 2~7 | Reserved | `0` |

호환성을 위해 Byte1이 `0x00`이면 전체 broadcast로 처리합니다.

예시:

```bash
# 전체 Enable, 기존 명령 호환
cansend can0 010#0100000000000000

# 전체 Enable, 최종 권장 broadcast
cansend can0 010#01FF000000000000

# Board3만 Enable
cansend can0 010#0103000000000000

# 전체 Disable
cansend can0 010#00FF000000000000
```

Board3 처리 조건:

```text
Target Board == 0x00 또는 0x03 또는 0xFF
```

Enable 수신 시:

```text
enabled = 1
error_code = ERR_NONE
state = STATE_IDLE
staging buffer clear
0x203 status 즉시 송신
```

Disable 수신 시:

```text
enabled = 0
g_cmd.is_new_cmd = 0
staging buffer clear
state = STATE_DISABLED
0x203 status 즉시 송신
```

---

## 15.3 Homing Start / Home Posture, CAN ID `0x020`

Board3는 `0x020 Homing Start`를 물리적인 limit switch 원점 탐색이 아니라, gripper를 미리 정의된 home posture로 보내는 명령으로 사용합니다.

현재 home posture는 모든 local Motor ID의 목표 각도를 0.00도로 정의합니다.

```text
home_position_001deg = 0
home_position_deg = 0.00°
```

### 15.3.1 Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Board | `uint8_t` | `0x00` 또는 `0xFF`: 전체, `0x03`: Board3 |
| 1 | Target Local Motor ID | `uint8_t` | Board3에서는 `255` 권장, 전체 gripper homing만 사용 |
| 2 | Homing Mode | `uint8_t` | 현재 `0`만 사용 |
| 3~7 | Reserved | - | `0` |

### 15.3.2 Board3 처리 조건

Board3는 아래 조건을 모두 만족할 때만 Homing을 수행합니다.

```text
Target Board == 0x00 또는 0x03 또는 0xFF
Target Local Motor ID == 255
Homing Mode == 0
enabled == 1
state != STATE_ESTOP
```

조건이 맞지 않으면 `ERR_INVALID_CMD` 또는 `ERR_DISABLED`, `ERR_ESTOP`을 설정하고 `0x203` status를 즉시 송신합니다.

### 15.3.3 Homing 명령 예시

```bash
# 전체 보드 전체 homing, legacy 호환
cansend can0 020#00FF000000000000

# 전체 보드 전체 homing, 최종 권장
cansend can0 020#FFFF000000000000

# Board3 gripper home posture
cansend can0 020#03FF000000000000
```

### 15.3.4 처리 동작

정상 Homing Start 수신 시 Board3는 내부적으로 아래와 같은 `g_cmd`를 생성합니다.

```text
1. staging buffer clear
2. g_cmd.target_pos_001deg[0~8] = 0
3. g_cmd.duration_5ms = 100
4. g_cmd.is_new_cmd = 1
5. state = STATE_MOVING 또는 제어 처리 중 상태
6. 0x203 status 즉시 송신
```

즉, Board3 homing은 아래 동작과 같습니다.

```text
Motor ID 0~8 전체를 0.00도 home posture로 이동
duration = 100 × 5ms = 500ms
```

주의:

```text
Board3의 homing은 Board1/Board2처럼 limit switch를 찾는 물리적 원점 탐색이 아니다.
Board3의 homing은 모든 gripper joint target angle을 0.00도로 보내는 home posture 명령이다.
실제 SCS0009 position 변환은 servo adapter에서 처리한다.
```

---

## 15.4 Clear Error, CAN ID `0x030`

`0x030`도 공통 CAN ID입니다.
Board3는 Target Board를 확인한 뒤 처리합니다.

### 15.4.1 Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Board | `uint8_t` | `0x00` 또는 `0xFF`: 전체, `0x03`: Board3 |
| 1 | Target Local Motor ID | `uint8_t` | Board3에서는 `255` 권장 |
| 2~7 | Reserved | - | `0` |

### 15.4.2 Board3 처리 조건

```text
Target Board == 0x00 또는 0x03 또는 0xFF
Target Local Motor ID == 255
```

Clear Error 수신 시:

```text
1. error_code = ERR_NONE
2. fault = 0
3. fault_motor_id = 255
4. staging buffer clear
5. enabled == 1이면 state = STATE_IDLE
6. enabled == 0이면 state = STATE_DISABLED
7. 0x203 status 즉시 송신
```

단, ESTOP 상태는 Clear Error만으로 해제하지 않습니다.
ESTOP 해제는 `0x010 Enable=1`에서 처리합니다.

### 15.4.3 Clear Error 명령 예시

```bash
# 전체 보드 전체 error clear
cansend can0 030#FFFF000000000000

# Board3 error clear
cansend can0 030#03FF000000000000
```

---

## 16. Board3 Status, CAN ID `0x203`

```text
CAN ID = 0x203
DLC = 8
방향 = Board3 → 중앙서버
```

### 16.1 송신 조건

Board3는 아래 상황에서 status를 송신합니다.

```text
1. 100ms마다 주기 송신
2. Enable / Disable 수신 시 즉시 송신
3. ESTOP 수신 시 즉시 송신
4. Clear Error 수신 시 즉시 송신
5. Homing command 생성 시 즉시 송신
6. 정상 command set 완성 시 즉시 송신
7. command set 폐기 또는 error 발생 시 즉시 송신
```

### 16.2 Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | State | `uint8_t` | 현재 Board3 상태 |
| 1 | Error Code | `uint8_t` | 현재 error code |
| 2 | Ready | `uint8_t` | 제어 가능 상태면 `1` |
| 3 | Staging Count | `uint8_t` | 현재 staging된 frame 개수 `0~9` |
| 4 | Fault | `uint8_t` | fault 있으면 `1` |
| 5 | Buffer Free | `uint8_t` | `9 - staging_count` |
| 6 | Enabled | `uint8_t` | `0`: disabled, `1`: enabled |
| 7 | Fault Motor ID | `uint8_t` | fault motor id, 없으면 `255` |

예시:

```text
0x203 01 00 01 00 00 09 01 FF
```

해석:

```text
STATE_IDLE
ERR_NONE
Ready
현재 staging count 0
Fault 없음
Buffer free 9
Enabled
Fault motor 없음
```

### 16.3 Board3 Actual Position Feedback, CAN ID `0x303`

`0x203`은 ready/error/enable/staging 상태 판단용으로 유지하고,
실제 그리퍼 각도는 별도 `0x303` feedback으로 송신한다.

```text
CAN ID = 0x303
DLC = 8
방향 = Board3 → 중앙서버
구조 = 3프레임 압축 피드백
```

Board3 local motor id는 코드 기준 `0~8`이다. 문서나 기구팀 표현에서
모터 1~9라고 부를 경우 아래 표와 같이 각각 local motor id `0~8`에
대응한다.

| Byte0 group index | Byte1~2 | Byte3~4 | Byte5~6 | Byte7 |
|---:|---|---|---|---|
| `0x01` | local motor `0` 현재 각도 | local motor `1` 현재 각도 | local motor `2` 현재 각도 | 상태/에러 flag |
| `0x02` | local motor `3` 현재 각도 | local motor `4` 현재 각도 | local motor `5` 현재 각도 | 상태/에러 flag |
| `0x03` | local motor `6` 현재 각도 | local motor `7` 현재 각도 | local motor `8` 현재 각도 | 상태/에러 flag |

각도 값은 `int16_t` little endian, 0.01도 단위다.

| 실제 각도 | 전송 값 |
|---:|---:|
| `30.00 deg` | `3000` |
| `-15.50 deg` | `-1550` |

중앙서버는 이 값을 radian으로 변환해 그리퍼 `/joint_states` 위치에 반영한다.

Byte7 상태/에러 flag는 한 group 안의 3개 모터 상태를 압축한다.

| Bit | 의미 |
|---:|---|
| 0~1 | group 안 첫 번째 모터 상태 |
| 2~3 | group 안 두 번째 모터 상태 |
| 4~5 | group 안 세 번째 모터 상태 |
| 6 | Valid flag. `1`일 때만 중앙서버가 위치값으로 사용 |
| 7 | 해당 group 안 fault 발생. `ERROR`가 있을 때만 `1`, `CONTACT_HOLD`에서는 `0` |

각 모터 상태 2-bit 값은 아래처럼 사용한다.

| 값 | 이름 | 의미 | 중앙서버 판단 |
|---:|---|---|---|
| `00` | `OK` | 목표 도달 또는 정상 정지, 부하 감지 없음 | 정상 |
| `01` | `MOVING` | 이동 중 | 정상 진행 중 |
| `10` | `CONTACT_HOLD` | 부하/접촉 감지로 더 닫지 않고 현재 각도 유지, 물체 파지 상태 | 정상 파지 상태, fault 아님 |
| `11` | `ERROR` | 서보 통신 오류, 과전류, 비정상 끼임 등 실패 상태 | fault/error |

중앙서버는 `0x01`, `0x02`, `0x03` 세 group이 모두 valid로 모였을 때만
하나의 9축 위치 snapshot으로 반영한다. 일부 group이 누락되거나 너무 늦게
들어오면 이전 group과 섞지 않고 폐기한다.

Board3는 20ms마다 `0x303` frame 3개를 송신한다. 각 주기는 group
`0x01 -> 0x02 -> 0x03` 순서로 구성한다.

---

## 17. State Values

| 값 | 이름 | 설명 |
|---:|---|---|
| `0` | `STATE_INIT` | 초기화 중 |
| `1` | `STATE_IDLE` | 대기 상태 |
| `2` | `STATE_STAGING` | 9개 frame 수집 중 |
| `3` | `STATE_MOVING` | 명령 처리 또는 이동 중 |
| `4` | `STATE_ERROR` | 에러 발생 |
| `5` | `STATE_ESTOP` | 비상정지 상태 |
| `6` | `STATE_DISABLED` | Disable 상태 |

---

## 18. Error Code Values

| 값 | 이름 | 설명 |
|---:|---|---|
| `0` | `ERR_NONE` | 정상 |
| `1` | `ERR_INVALID_CMD` | 잘못된 명령 |
| `2` | `ERR_INVALID_MOTOR_ID` | Motor ID 범위 오류 |
| `3` | `ERR_DUPLICATE_MOTOR_ID` | 같은 command set 안에서 Motor ID 중복 |
| `4` | `ERR_STAGING_TIMEOUT` | 9개 frame 수집 timeout |
| `5` | `ERR_DURATION_MISMATCH` | 9개 frame의 duration 불일치 |
| `6` | `ERR_ANGLE_RANGE` | 목표 각도 한계 초과, 하드웨어 연결 후 적용 |
| `7` | `ERR_SERVO_COMM` | SCS0009 통신 오류, 하드웨어 연결 후 적용 |
| `8` | `ERR_SERVO_FAULT` | 과부하 또는 servo fault, 하드웨어 연결 후 적용 |
| `9` | `ERR_ESTOP` | ESTOP 상태 |
| `10` | `ERR_DISABLED` | Disable 상태에서 command 수신 |

---

## 19. 하드웨어 연결 전 테스트 기준

하드웨어 연결 전에는 실제 서보가 움직이는지 확인하지 않습니다.

성공 기준:

```text
1. 0x103 frame 9개를 Motor ID 0~8로 파싱한다.
2. staging_count가 0~9로 증가한다.
3. 9개 duration이 같을 때만 command set이 성립한다.
4. 정상 command set이면 g_cmd.is_new_cmd가 1이 된다.
5. g_cmd.target_pos_001deg[0~8]에 각도값이 들어간다.
6. g_cmd.duration_5ms에 CAN Byte7 원본값이 들어간다.
7. 잘못된 command set은 g_cmd로 전달되지 않는다.
8. 0x203 status가 주기적으로 또는 이벤트마다 송신된다.
9. 0x020 Homing Start 수신 시 g_cmd.target_pos_001deg[0~8]이 모두 0으로 설정된다.
10. Homing duration은 g_cmd.duration_5ms = 100, 즉 500ms로 설정된다.
11. 0x020 / 0x030에서 Target Board가 3, 0, 255일 때만 Board3가 처리한다.
```

---

## 20. 하드웨어 연결 후 보정할 항목

아래 항목은 실제 그리퍼/서보 연결 후 보정합니다.

```text
- 각 Motor ID별 direction
- 각 Motor ID별 step_per_deg 또는 degree-to-position scale
- 각 Motor ID별 min_goal / max_goal
- home posture 0.00도가 실제 SCS0009 position에서 안전한 자세인지 확인
- 목표 각도 범위 검사
- 실제 부하 threshold
- 실제 fault 조건
- Feetech_Read_Pos_Load() 주기
- 9개 서보를 100ms마다 모두 읽을 수 있는지
- Sync Write 사용 여부
```

---

## 21. C Constant Example

```c
#define CAN_ID_ESTOP        0x001
#define CAN_ID_ENABLE       0x010
#define CAN_ID_HOMING       0x020
#define CAN_ID_CLEAR_ERROR  0x030

#define CAN_ID_BOARD3_MOVE  0x103
#define CAN_ID_BOARD3_STAT  0x203

#define BOARD_ID_ALL_LEGACY 0x00
#define BOARD_ID_1          0x01
#define BOARD_ID_2          0x02
#define BOARD_ID_3          0x03
#define BOARD_ID_ALL        0xFF

#define BOARD3_MOTOR_COUNT  9
#define MOTOR_ALL           0xFF
#define GRIPPER_HOMING_DURATION_5MS 100

#define STATE_INIT          0
#define STATE_IDLE          1
#define STATE_STAGING       2
#define STATE_MOVING        3
#define STATE_ERROR         4
#define STATE_ESTOP         5
#define STATE_DISABLED      6

#define ERR_NONE                 0
#define ERR_INVALID_CMD          1
#define ERR_INVALID_MOTOR_ID     2
#define ERR_DUPLICATE_MOTOR_ID   3
#define ERR_STAGING_TIMEOUT      4
#define ERR_DURATION_MISMATCH    5
#define ERR_ANGLE_RANGE          6
#define ERR_SERVO_COMM           7
#define ERR_SERVO_FAULT          8
#define ERR_ESTOP                9
#define ERR_DISABLED             10
```

---

## 22. Board3 Test Command Summary

```bash
# 전체 Enable
cansend can0 010#01FF000000000000

# Board3만 Enable
cansend can0 010#0103000000000000

# Board3 home posture
cansend can0 020#03FF000000000000

# Board3 error clear
cansend can0 030#03FF000000000000

# 전체 ESTOP
cansend can0 001#0100000000000000

# Status 확인
candump can0
```

### 22.1 Gripper 9개 frame 예시

아래 예시는 9개 Motor ID 모두 target angle 0.00도, duration 100ms로 보내는 구조입니다.

```bash
cansend can0 103#8000000000000014
cansend can0 103#8100000000000014
cansend can0 103#8200000000000014
cansend can0 103#8300000000000014
cansend can0 103#8400000000000014
cansend can0 103#8500000000000014
cansend can0 103#8600000000000014
cansend can0 103#8700000000000014
cansend can0 103#8800000000000014
```

해석:

```text
Byte0 = 0x80~0x88, Execute=1, Motor ID=0~8
Byte1~4 = 0, target angle 0.00도
Byte5~6 = 0, speed unused
Byte7 = 0x14 = 20 × 5ms = 100ms
```

---

## 23. 최종 요약

```text
1. Board3는 로봇팔 그리퍼 서보 9개를 담당한다.
2. Board3 명령 ID는 0x103, status ID는 0x203이다.
3. 0x103 frame 하나는 서보 1개 명령이다.
4. Motor ID 0~8의 9개 frame이 모두 모였을 때만 하나의 gripper command set으로 처리한다.
5. 일부 frame만 들어오거나 잘못된 frame이 있으면 전체 command set을 폐기한다.
6. g_cmd에는 servo position이 아니라 0.01도 단위 각도값을 넣는다.
7. g_cmd.duration_5ms에는 CAN Byte7 원본값을 넣는다.
8. Homing 0x020과 Clear Error 0x030은 Target Board + Target Local Motor ID 구조를 사용한다.
9. Board3는 Target Board가 3, 0, 255일 때만 공통 명령을 처리한다.
10. Board3 homing은 물리적 원점 탐색이 아니라 모든 gripper joint를 0.00도로 보내는 home posture 명령이다.
```
