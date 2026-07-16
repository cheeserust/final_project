# VicPinky Arm UART Protocol

작성일: 2026-07-06

이 문서는 기존 `ARM_CAN_PROTOCOL.md`를 UART 통신으로 옮길 때 팀 공통으로
맞춰야 할 v1 프로토콜 기준이다.

핵심 방향은 단순하다.

```text
기존 CAN payload 의미는 최대한 유지한다.
UART에서는 CAN ID가 없으므로 UART frame wrapper에 board_id와 msg_id를 넣는다.
보드 firmware의 motion/status 처리 로직은 msg_id + payload 기준으로 재사용한다.
```

즉, 기존의 `0x101`, `0x102`, `0x103`, `0x201` 같은 CAN ID는 UART에서
`Message ID`로 사용한다.

---

## 1. 적용 범위

대상 보드는 기존과 같다.

| Board | 역할 | UART Board ID | 기존 Move Message ID | 기존 Status Message ID | 기존 Feedback Message ID |
|---|---|---:|---:|---:|---:|
| Board1 | `arm_joint_1~3 + base_joint` stepper | `0x01` | `0x101` | `0x201` | `0x301` |
| Board2 | `arm_joint_4` stepper | `0x02` | `0x102` | `0x202` | `0x302` |
| Board3 | gripper servo 9개 | `0x03` | `0x103` | `0x203` | `0x303` |
| Broadcast | 전체 또는 공통 명령 | `0xFF` | 명령별 사용 | - | - |

UART로 바꾸더라도 아래 payload 구조는 기존 CAN 문서와 동일하게 유지한다.

```text
0x001 ESTOP
0x010 Enable / Disable
0x020 Board1+Board2 stepper homing
0x023 Board3 gripper home posture
0x030 Clear Error
0x101 Board1 4-axis move frame
0x102 Board2 arm_joint_4 move frame
0x103 Board3 9-servo move frame
0x201/0x202/0x203 Status
0x301/0x302/0x303 Position feedback
```

---

## 2. 물리 연결 원칙

UART는 CAN처럼 여러 장치가 자연스럽게 한 버스에서 중재되는 통신이 아니다.
그래서 물리 연결을 먼저 정해야 한다.

### 2.1 권장 1안: 보드별 독립 UART

가장 구현이 쉽고 안정적인 방식이다.

```text
PC/RPi UART0 또는 USB-UART 1개 -> Board1
PC/RPi UART1 또는 USB-UART 1개 -> Board2
PC/RPi UART2 또는 USB-UART 1개 -> Board3
```

이 방식에서는 각 보드가 자기 포트로만 응답하므로 periodic status/feedback을
기존 CAN처럼 보내도 된다.

### 2.2 가능 2안: RS-485 half-duplex 공유 버스

한 쌍의 선으로 여러 보드를 묶어야 한다면 TTL UART를 그대로 묶지 말고
RS-485 transceiver를 사용한다.

이 방식에서는 동시에 여러 보드가 송신하면 충돌이 나므로 아래 원칙을 지킨다.

```text
PC/RPi가 master다.
보드는 master가 요청한 경우에만 응답한다.
periodic feedback은 끄고 poll 방식으로 받는다.
broadcast command에는 ACK를 요구하지 않는다.
```

### 2.3 전기적 기본값

| 항목 | 권장값 |
|---|---|
| Logic level | 3.3V TTL 또는 USB-UART/USB-CDC |
| Baudrate | `921600` 권장, 최소 `460800` 권장 |
| UART format | `8N1` |
| Flow control | 없음 |
| Byte order | little endian |

`115200`도 짧은 테스트는 가능하지만, Board3 9-frame staging과 feedback까지
같이 쓰기에는 여유가 작다. 처음 구현은 `921600 8N1`을 기준으로 맞춘다.

---

## 3. UART Frame Format

모든 UART packet은 아래 binary frame을 사용한다.

```text
SOF0 SOF1 VER TYPE FLAGS SEQ BOARD_ID MSG_ID_L MSG_ID_H LEN PAYLOAD... CRC_L CRC_H
```

| Offset | 크기 | 필드 | 값 / 설명 |
|---:|---:|---|---|
| 0 | 1 | `SOF0` | `0xAA` |
| 1 | 1 | `SOF1` | `0x55` |
| 2 | 1 | `VER` | 프로토콜 버전. v1은 `0x01` |
| 3 | 1 | `TYPE` | frame type |
| 4 | 1 | `FLAGS` | option bit |
| 5 | 1 | `SEQ` | sequence counter, `0~255` wrap |
| 6 | 1 | `BOARD_ID` | `0x01`, `0x02`, `0x03`, `0xFF` |
| 7 | 2 | `MSG_ID` | 기존 CAN ID를 little endian으로 저장 |
| 9 | 1 | `LEN` | payload 길이. v1 기본 `0` 또는 `8`, 최대 `64` |
| 10 | N | `PAYLOAD` | message별 payload |
| 10+N | 2 | `CRC16` | CRC-16/CCITT-FALSE, little endian |

8-byte command payload를 보내는 경우 전체 UART frame 길이는 `20 byte`다.

### 3.1 TYPE

| 값 | 이름 | 방향 | 의미 |
|---:|---|---|---|
| `0x01` | `COMMAND` | PC/RPi -> Board | 제어 명령 또는 trajectory command |
| `0x02` | `FEEDBACK` | Board -> PC/RPi | status, position feedback, event |
| `0x03` | `ACK` | Board -> PC/RPi | command 정상 수신/반영 |
| `0x04` | `NACK` | Board -> PC/RPi | command 거절 |
| `0x05` | `HEARTBEAT` | 양방향 | 링크 확인용 |

### 3.2 FLAGS

| Bit | 이름 | 의미 |
|---:|---|---|
| 0 | `ACK_REQ` | `COMMAND` 수신 후 ACK/NACK 필요 |
| 1 | `RETRY` | 이전과 같은 `SEQ`를 재전송 중 |
| 2 | `BROADCAST_ACK_ALLOWED` | 독립 UART 포트에서만 사용 가능 |
| 3~7 | Reserved | 반드시 `0` |

일반 명령은 `ACK_REQ=1`을 권장한다. 단, RS-485 공유 버스에서
`BOARD_ID=0xFF` broadcast를 보낼 때는 여러 보드가 동시에 ACK하지 않도록
`ACK_REQ=0`으로 보낸다.

### 3.3 CRC

CRC는 `VER`부터 `PAYLOAD` 마지막 byte까지 계산한다.
`SOF0`, `SOF1`, `CRC16` 자신은 계산에 포함하지 않는다.

```text
Algorithm: CRC-16/CCITT-FALSE
Polynomial: 0x1021
Initial value: 0xFFFF
RefIn: false
RefOut: false
XorOut: 0x0000
Storage: little endian
```

수신 측은 CRC 오류 frame에 대해 NACK를 보내지 않는다. CRC가 틀린 frame은
누가 보낸 것인지 신뢰할 수 없으므로 조용히 버리고 다음 `0xAA 0x55`를 찾는다.

---

## 4. Sequence와 ACK/NACK

### 4.1 기본 규칙

송신자는 frame을 보낼 때마다 `SEQ`를 1씩 증가시킨다.
단, timeout으로 같은 command를 재전송할 때는 같은 `SEQ`를 유지하고
`FLAGS.RETRY=1`을 세트한다.

보드는 `COMMAND`를 정상 수신하면 같은 `SEQ`, 같은 `MSG_ID`로 `ACK`를 보낸다.
거절하면 같은 `SEQ`, 같은 `MSG_ID`로 `NACK`를 보낸다.

```text
Host COMMAND: TYPE=0x01, SEQ=0x31, BOARD_ID=0x02, MSG_ID=0x010
Board ACK   : TYPE=0x03, SEQ=0x31, BOARD_ID=0x02, MSG_ID=0x010
Board NACK  : TYPE=0x04, SEQ=0x31, BOARD_ID=0x02, MSG_ID=0x010
```

### 4.2 ACK payload

ACK frame은 `LEN=2`를 사용한다.

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | Result | 정상은 `0x00` |
| 1 | Queue Free / Detail | queue 명령이면 남은 queue, 아니면 `0xFF` |

### 4.3 NACK payload

NACK frame은 `LEN=2`를 사용한다.

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | Reason Code | 아래 표 기준 |
| 1 | Detail | 문제 motor id, expected length 등 보드별 상세값 |

NACK reason code:

| 값 | 이름 | 의미 |
|---:|---|---|
| `0x01` | `UNSUPPORTED_VERSION` | `VER` 불일치 |
| `0x02` | `INVALID_LENGTH` | `LEN` 또는 payload 길이 오류 |
| `0x03` | `UNSUPPORTED_MSG_ID` | 처리하지 않는 `MSG_ID` |
| `0x04` | `BOARD_MISMATCH` | 자기 board_id가 아닌 command |
| `0x05` | `INVALID_PAYLOAD` | payload field 값 오류 |
| `0x06` | `NOT_ENABLED` | enable 전 command |
| `0x07` | `ESTOP_ACTIVE` | ESTOP 상태 |
| `0x08` | `QUEUE_FULL` | command queue full |
| `0x09` | `STAGING_ERROR` | staging 순서, duration, 중복 motor 오류 |
| `0x0A` | `BUSY` | 지금 처리 불가 |

### 4.4 중복 command 처리

ACK가 유실되면 host가 같은 `SEQ`로 재전송할 수 있다.
보드는 최근 성공한 command의 `SEQ + MSG_ID + CRC`를 짧게 기억한다.

같은 command가 다시 오면 motion queue에 두 번 넣지 말고 이전 ACK만 다시 보낸다.
이 처리는 trajectory command에서 특히 중요하다.

권장 cache:

```text
board별 최근 accepted command 8개
보관 시간 1초
key = SEQ + MSG_ID + CRC16
```

---

## 5. Message ID와 Payload

UART의 `MSG_ID`는 기존 CAN ID와 같은 값을 사용한다.
payload는 기존 CAN payload 8 byte를 그대로 넣는다.

### 5.1 공통 제어 명령

| MSG_ID | BOARD_ID | 이름 | Payload |
|---:|---:|---|---|
| `0x001` | `0xFF` | ESTOP | `01 00 00 00 00 00 00 00` |
| `0x010` | `0xFF` 또는 개별 board | Enable | `01 00 00 00 00 00 00 00` |
| `0x010` | `0xFF` 또는 개별 board | Disable | `00 00 00 00 00 00 00 00` |
| `0x020` | `0xFF`, `0x01`, `0x02` | Stepper homing | `FF 00 00 00 00 00 00 00` |
| `0x023` | `0x03` 또는 `0xFF` | Gripper home posture | `FF 00 duration 00 00 00 00 00` |
| `0x030` | `0xFF` 또는 개별 board | Clear Error | `FF 00 00 00 00 00 00 00` |

주의:

```text
0x020은 Board1+Board2 stepper homing이다.
Board3 gripper home posture는 0x023을 사용한다.
```

### 5.2 Board1 move, MSG_ID `0x101`

```text
BOARD_ID = 0x01
TYPE = COMMAND
LEN = 8
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | flags + Board1 local motor id |
| 1~4 | Target Pos | `int32_t` | 0.01도 단위, little endian |
| 5~6 | Speed | `uint16_t` | 0.01도/s, 초기 firmware에서는 미사용 가능 |
| 7 | Duration | `uint8_t` | 5ms tick |

Byte0:

```text
Bit7 Bit6 Bit5 Bit4 | Bit3 Bit2 Bit1 Bit0
Exec Rel  Step Rsv  | Motor ID
```

일반 절대 각도 명령:

| Motor ID | Byte0 |
|---:|---:|
| `0` | `0x80` |
| `1` | `0x81` |
| `2` | `0x82` |
| `3` | `0x83` |

Board1은 기존과 동일하게 4-frame staging을 사용한다.

```text
Motor ID 0 -> 1 -> 2 -> 3
첫 frame 이후 20ms 안에 4개 frame 수신
4개 frame의 Duration 동일
Execute=1
Reserved bit=0
안 움직이는 축도 현재 목표 위치를 다시 보내야 함
```

### 5.3 Board2 move, MSG_ID `0x102`

```text
BOARD_ID = 0x02
TYPE = COMMAND
LEN = 8
```

Payload는 Board1과 같은 구조다. Board2 일반 절대 각도 명령은 항상:

```text
Byte0 = 0x80
```

예시 payload:

```text
arm_joint_4를 30.00도로 50ms 이동
80 B8 0B 00 00 E8 03 0A
```

### 5.4 Board3 servo command, MSG_ID `0x103`

```text
BOARD_ID = 0x03
TYPE = COMMAND
LEN = 8
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | Bit7 Execute, Bit3~0 Motor ID |
| 1~4 | Target Position | `int32_t` | 0.01도 단위, little endian |
| 5~6 | Target Load | `uint16_t` | `0~1023`, little endian |
| 7 | Duration | `uint8_t` | 5ms tick |

Board3는 기존과 동일하게 9-frame staging을 사용한다.

```text
Motor ID 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8
9개 frame의 Duration 동일
Execute=1
Reserved bit=0
중복 Motor ID 없음
```

load 기능이 없는 기존 firmware와 임시 테스트할 때는 Byte5~6을 `00 00`으로
보내도 된다.

---

## 6. Status와 Feedback

보드가 보내는 status/feedback도 기존 CAN payload를 그대로 사용한다.

| MSG_ID | BOARD_ID | TYPE | 주기 | Payload |
|---:|---:|---|---|---|
| `0x201` | `0x01` | `FEEDBACK` | 100ms + event | Board1 status 8 byte |
| `0x202` | `0x02` | `FEEDBACK` | 100ms + event | Board2 status 8 byte |
| `0x203` | `0x03` | `FEEDBACK` | 100ms + event | Board3 status 8 byte |
| `0x301` | `0x01` | `FEEDBACK` | 구현별 | Board1 motor position 8 byte |
| `0x302` | `0x02` | `FEEDBACK` | 구현별 | Board2 motor position 8 byte |
| `0x303` | `0x03` | `FEEDBACK` | 20ms cycle | Board3 compressed position group 8 byte |

독립 UART 포트 방식에서는 보드가 기존처럼 주기적으로 feedback을 보내도 된다.
RS-485 공유 버스 방식에서는 periodic feedback을 끄고 poll/slot 방식으로 바꾼다.

---

## 7. 송신 순서와 Timing

UART에서는 CAN arbitration이 없으므로 host가 송신 순서를 명확히 보장해야 한다.

권장 baudrate `921600` 기준으로 8-byte payload UART frame 하나는 약 `0.22ms`
정도 걸린다. 초기 안정화 단계에서는 frame 사이에 `0.5ms~1.0ms` gap을 둔다.

### 7.1 Arm 중심 trajectory 순서

팔 관절 시작 시점 차이를 줄이는 목적이면 아래 순서를 권장한다.

```text
Board1 0x101 motor 0
Board1 0x101 motor 1
Board1 0x101 motor 2
Board1 0x101 motor 3
Board2 0x102 motor 0
Board3 0x103 motor 0~8
```

### 7.2 Gripper까지 같은 batch로 묶는 순서

Board3 9-frame staging 시간이 가장 길기 때문에 gripper까지 한 trajectory point로
묶고 싶으면 긴 staging 그룹을 먼저 보낸다.

```text
Board3 0x103 motor 0~8
Board1 0x101 motor 0~3
Board2 0x102 motor 0
```

현재 v1에는 전 보드 공통 `prepare/commit` 명령이 없다.
따라서 완전히 같은 tick에서 모든 보드가 시작하는 것은 보장하지 않는다.
정확한 동시 시작이 필요해지면 UART protocol v2에서 `PREPARE_TRAJECTORY`와
`COMMIT_TRAJECTORY`를 추가한다.

---

## 8. Raw Frame 예시

### 8.1 전체 Enable command

의미:

```text
TYPE=COMMAND
ACK_REQ=1
SEQ=0x10
BOARD_ID=0xFF
MSG_ID=0x010
PAYLOAD=01 00 00 00 00 00 00 00
```

UART raw bytes:

```text
AA 55 01 01 01 10 FF 10 00 08 01 00 00 00 00 00 00 00 E4 07
```

### 8.2 Board2 arm_joint_4 30.00도, 50ms 이동

의미:

```text
TYPE=COMMAND
ACK_REQ=1
SEQ=0x11
BOARD_ID=0x02
MSG_ID=0x102
PAYLOAD=80 B8 0B 00 00 E8 03 0A
```

UART raw bytes:

```text
AA 55 01 01 01 11 02 02 01 08 80 B8 0B 00 00 E8 03 0A F4 06
```

### 8.3 Board2 ACK 예시

의미:

```text
TYPE=ACK
SEQ=0x11
BOARD_ID=0x02
MSG_ID=0x102
PAYLOAD=00 20
```

`00`은 정상, `20`은 queue free 32를 의미한다.

UART raw bytes:

```text
AA 55 01 03 00 11 02 02 01 02 00 20 57 DF
```

---

## 9. Parser 구현 규칙

수신 parser는 state machine으로 구현한다.

```text
1. 0xAA 찾기
2. 다음 byte가 0x55인지 확인
3. 고정 header 8 byte 수신
4. LEN이 64 이하인지 확인
5. PAYLOAD LEN byte 수신
6. CRC 2 byte 수신
7. CRC 검증
8. VER, TYPE, BOARD_ID, MSG_ID, LEN, PAYLOAD 검증
9. command handler로 전달
```

수신 중 오류 처리:

| 상황 | 처리 |
|---|---|
| SOF 불일치 | 다음 `0xAA`까지 버림 |
| `LEN > 64` | frame 폐기 후 resync |
| CRC 불일치 | NACK 없이 폐기 |
| 지원하지 않는 `VER` | NACK `UNSUPPORTED_VERSION` |
| 자기 board가 아닌 `BOARD_ID` | 폐기. ACK/NACK 없음 |
| broadcast command | 자기 대상이면 처리 |
| payload 검증 실패 | NACK `INVALID_PAYLOAD` |

---

## 10. Host 구현 방향

PC/RPi 쪽은 기존 `CanFrame(can_id, data)` 개념을 재사용하면 된다.

권장 구조:

```text
기존 can_protocol.py
  pack_enable() -> CanFrame(can_id=0x010, data=8 bytes)
  pack_position_command() -> CanFrame(can_id=0x101/0x102, data=8 bytes)

새 uart_transport.py
  CanFrame.can_id -> UART MSG_ID
  board_id는 MSG_ID 또는 기존 board mapping으로 결정
  UART frame으로 wrap해서 serial port에 write
  UART feedback을 unwrap해서 CanFrame처럼 상위 parser에 전달
```

기존 parser도 크게 바꾸지 않는다.

```text
MSG_ID 0x201/0x202/0x203 -> 기존 unpack_status()
MSG_ID 0x301/0x302 -> 기존 unpack_motor_position_feedback()
MSG_ID 0x303 -> 기존 unpack_board3_position_feedback()
```

### 10.1 ACK timeout 권장값

| 항목 | 권장값 |
|---|---:|
| Direct UART ACK timeout | `20ms` |
| RS-485 ACK timeout | `30ms` |
| Retry count | `3` |
| Inter-frame gap | `0.5ms`부터 시작 |
| Retry backoff | `5ms`, `10ms`, `20ms` |

ESTOP은 ACK를 기다리기 전에 즉시 송신하고, 필요하면 같은 frame을 짧은 간격으로
2~3회 반복 송신한다. ESTOP은 idempotent하게 처리해야 한다.

---

## 11. Board firmware 구현 방향

각 보드는 아래 순서로 구현한다.

```text
1. UART RX ring buffer 추가
2. frame parser state machine 추가
3. CRC-16/CCITT-FALSE 검증 추가
4. BOARD_ID filter 추가
5. MSG_ID switch-case 추가
6. 기존 CAN command handler를 msg_id + payload handler로 분리
7. ACK/NACK 송신 추가
8. status/feedback 송신을 UART frame wrapper로 변경
9. retry duplicate command 방지 cache 추가
```

보드별 최소 처리 MSG_ID:

| Board | 반드시 처리할 command |
|---|---|
| Board1 | `0x001`, `0x010`, `0x020`, `0x030`, `0x101` |
| Board2 | `0x001`, `0x010`, `0x020`, `0x030`, `0x102` |
| Board3 | `0x001`, `0x010`, `0x023`, `0x030`, `0x103` |

보드별 송신 MSG_ID:

| Board | 반드시 송신할 feedback |
|---|---|
| Board1 | `0x201`, `0x301` |
| Board2 | `0x202`, `0x302` |
| Board3 | `0x203`, `0x303` |

---

## 12. Bring-Up 순서

### 12.1 링크 단독 확인

1. 보드 하나만 연결한다.
2. host에서 `HEARTBEAT` 또는 `Enable` command를 보낸다.
3. ACK가 같은 `SEQ`, 같은 `MSG_ID`로 돌아오는지 확인한다.
4. CRC 오류 frame을 일부러 보내고 보드가 무시하는지 확인한다.
5. 잘못된 `BOARD_ID` frame을 보내고 보드가 무시하는지 확인한다.

### 12.2 보드별 기능 확인

Board2부터 확인한다.

```text
1. Board2 Enable
2. Board2 Homing
3. Board2 0x102 단일 move
4. 0x202 status 확인
5. 0x302 feedback 확인
```

그 다음 Board1 staging을 확인한다.

```text
0x101 motor 0
0x101 motor 1
0x101 motor 2
0x101 motor 3
20ms 이내
duration 동일
```

마지막으로 Board3 staging을 확인한다.

```text
0x103 motor 0
0x103 motor 1
...
0x103 motor 8
duration 동일
중복 motor id 없음
```

### 12.3 전체 연결 확인

독립 UART 포트 방식:

```text
1. Board1 포트만 연결
2. Board2 포트만 연결
3. Board3 포트만 연결
4. Board1 + Board2
5. Board1 + Board2 + Board3
```

RS-485 공유 버스 방식:

```text
1. periodic feedback 송신 OFF
2. master poll command로 Board1 status 요청
3. master poll command로 Board2 status 요청
4. master poll command로 Board3 status 요청
5. command 중에는 다른 보드가 송신하지 않는지 logic analyzer로 확인
```

---

## 13. v1 최종 체크리스트

```text
1. UART는 AA 55 SOF로 시작한다.
2. CRC는 VER부터 PAYLOAD까지 CRC-16/CCITT-FALSE로 계산한다.
3. multibyte 값은 little endian이다.
4. 기존 CAN ID는 UART MSG_ID로 유지한다.
5. 기존 CAN payload 8 byte 의미는 그대로 유지한다.
6. UART wrapper에는 BOARD_ID를 넣는다.
7. Board1=0x01, Board2=0x02, Board3=0x03, Broadcast=0xFF다.
8. 일반 command는 ACK_REQ=1을 권장한다.
9. CRC 오류 frame에는 NACK하지 않는다.
10. retry command는 같은 SEQ로 보내고 RETRY flag를 세운다.
11. 보드는 duplicate retry를 motion queue에 두 번 넣지 않는다.
12. Board1 0x101은 4-frame staging을 유지한다.
13. Board3 0x103은 9-frame staging을 유지한다.
14. shared RS-485에서는 periodic feedback을 끄고 poll/slot 방식으로 받는다.
15. 정확한 전 보드 동시 시작이 필요하면 v2에서 prepare/commit을 추가한다.
```
