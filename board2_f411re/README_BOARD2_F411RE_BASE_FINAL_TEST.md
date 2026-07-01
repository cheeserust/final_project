# Board2 STM32F411RE CAN-only Test Firmware — Base Joint Final Protocol

이 코드는 STM32F411RE + MCP2515 조합에서 Board2 최종 통합 프로토콜을 CAN 통신만으로 테스트하기 위한 펌웨어입니다.

## 1. Board2 역할

```text
Board2 = 베이스 1축 / base_joint
서버 Global Joint ID = 0
Board2 Local Motor ID = 0
Move CAN ID = 0x102
Status CAN ID = 0x202
Position Feedback CAN ID = 0x302
```

기존 `0x202` status frame은 그대로 유지하고, MoveIt2 `/joint_states` actual position용 current position feedback만 `0x302`로 추가 송신합니다.

## 2. 핀 연결

기존에 사용하던 STM32F411RE 핀을 그대로 사용합니다.

| 기능 | STM32F411RE 핀 |
|---|---|
| MCP2515 SCK | PB13 |
| MCP2515 MISO | PB14 |
| MCP2515 MOSI | PB15 |
| MCP2515 CS | PB12 |
| MCP2515 INT | PB4 |
| LED | PA5 |

## 3. 구현된 CAN ID

| CAN ID | 방향 | 기능 |
|---:|---|---|
| `0x001` | RPi → STM32 | Emergency Stop broadcast |
| `0x010` | RPi → STM32 | Enable / Disable broadcast |
| `0x020` | RPi → STM32 | Board1 + Board2 stepper homing broadcast |
| `0x030` | RPi → STM32 | Clear Error broadcast |
| `0x102` | RPi → STM32 | Board2 base trajectory command |
| `0x202` | STM32 → RPi | Board2 status |
| `0x302` | STM32 → RPi | Board2 current position feedback |

## 4. Position Feedback `0x302`

```text
CAN ID = 0x302
DLC = 8
송신 주기 = 100ms
```

| Byte | 내용 |
|---:|---|
| Byte0 | Local Motor ID, Board2는 항상 0 |
| Byte1 | Flags |
| Byte2~5 | current_pos_001deg, int32 little-endian |
| Byte6 | error/fault code |
| Byte7 | sequence counter |

Flags:

```text
bit0 = position valid
bit1 = homed / ready
bit2 = moving
bit3 = target reached
bit4~7 = reserved 0
```

예상 candump:

```text
초기 disabled / not homed: 302#0001D8DCFFFF00XX
homing 완료 / idle:       302#000BD8DCFFFF00XX
30.00도 도달 후:          302#000BB80B000000XX
이동 또는 homing 중:      302#0005........00XX
```

`XX`는 sequence counter라서 송신할 때마다 증가합니다.

## 5. 테스트 명령

```bash
# 전체 Enable
cansend can0 010#0100000000000000

# Board1 + Board2 전체 스텝모터 homing
cansend can0 020#FF00000000000000

# Board2 base_joint 30.00도 이동, 50ms
cansend can0 102#80B80B0000E8030A

# Board2 base_joint home -90.00도 이동, 50ms
cansend can0 102#80D8DCFFFFE8030A

# 전체 Clear Error
cansend can0 030#FF00000000000000

# 전체 ESTOP, payload는 현재 사용하지 않음
cansend can0 001#0000000000000000
```

확인:

```bash
candump can0,202:7FF
candump can0,302:7FF
```

## 6. 코드 동작

이 테스트 코드는 실제 모터 드라이버 없이 CAN 프로토콜만 확인하는 용도입니다.

```text
- STEP/DIR 출력 없음
- homing은 20ms 동안 simulated 처리
- homing 완료 후 current_pos_001deg = -9000, 즉 -90.00도 home으로 설정
- move는 1ms tick으로 simulated linear interpolation
- 0x202 status는 100ms 주기 송신
- 0x302 position feedback도 100ms 주기 송신
- 100ms 주기 이벤트에서는 0x202를 먼저 보내고 0x302를 직후 보냄
```

## 7. 주의사항

```text
- payload에는 Board ID를 넣지 않습니다.
- Board2는 local motor id 0만 허용합니다.
- Board2 일반 절대 이동 명령 Byte0은 0x80입니다.
- 0x81~0x8F는 Board2에서 잘못된 motor id입니다.
- Board2 base_joint limit은 -90~180도, raw -9000~18000입니다.
- Board2 gear ratio는 20입니다.
- Board2 home position은 -90도, raw -9000입니다.
```
