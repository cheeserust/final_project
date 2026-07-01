# Board1 STM32F411CEU6 PA9/PA10 Integrated Protocol Test Firmware

이 폴더는 첨부된 통합 프로토콜에 맞춘 Board1 전용 STM32F411CEU6 테스트 코드입니다.

## 적용 기준

- Board1 담당 축: 팔 2~5축
- Board1 local Motor ID: 0~3
- Board2 담당 축: 베이스 1축
- Board1 Move CAN ID: `0x101`
- Board1 Status CAN ID: `0x201`
- Board1 Position Feedback CAN ID: `0x301`
- Payload에 Board ID 없음

## MCP2515 핀

```text
MCP2515 SCK  = PB13
MCP2515 MISO = PB14
MCP2515 MOSI = PB15
MCP2515 CS   = PA9
MCP2515 INT  = PA10
LED          = PC13, active-low
```

## Board1 기구 파라미터

| Board1 Motor ID | 실제 축 | Gear ratio | Motor full steps/rev | Home raw |
|---:|---|---:|---:|---:|
| 0 | 팔 2축 | 20 | 200 | -9000 |
| 1 | 팔 3축 | 50 | 200 | -8000 |
| 2 | 팔 4축 | 30 | 200 | -9000 |
| 3 | 팔 5축 | 120 | 48 | -17000 |

## 0x101 이동 명령

Board1의 `0x101`은 한 frame이 한 축을 바로 움직이는 구조가 아닙니다.

```text
Motor ID 0 -> 1 -> 2 -> 3
4개 frame을 20ms 안에 수신
4개 frame Duration 동일
4번째 frame 수신 후 4축 trajectory point 1개로 queue push
4축을 같은 1ms tick에서 동시에 시작
```

## 0x201 status

100ms마다 송신합니다. 주요 이벤트 발생 시 즉시 한 번 송신합니다.

`Byte5 Queue Free`는 내부 point queue 기준이 아니라 외부 `0x101` command slot 기준입니다.

```text
내부 4축 point queue = 8개
한 point = 0x101 frame 4개
Queue Free = 남은 point 수 * 4
범위 = 0~32
```

## 0x301 position feedback

100ms마다 local Motor ID `0 -> 1 -> 2 -> 3` 순서로 4개 frame을 송신합니다.

```text
Byte0   = Local Motor ID
Byte1   = Flags
Byte2~5 = current_pos_001deg, int32 little-endian
Byte6   = error/fault code
Byte7   = sequence counter
```

## 빌드 / 플래시

```bash
make clean
make
make flash-stlink
```

## SocketCAN 설정

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
```

## 테스트 순서

```bash
# 1. status / feedback 확인
candump can0

# 2. 전체 Enable
cansend can0 010#0100000000000000

# 3. Board1 + Board2 전체 스텝모터 homing
cansend can0 020#FF00000000000000

# 4. Board1 position feedback 디코딩
candump can0,301:7FF | python3 decode_board1_feedback.py

# 5. 4축 trajectory point 전송
python3 send_board1_4axis_point.py can0 3000 -8000 -9000 -17000 --speed 1000 --duration 10
```

주의: 위 예시는 Motor 0만 30.00도로 보내고, 나머지는 home raw 값을 유지하는 예시입니다. 안 움직이는 축도 생략하면 안 됩니다.

## 구현 범위

현재 코드는 CAN 통신/프로토콜/queue/feedback 검증용입니다.

구현됨:

```text
- MCP2515 CAN RX/TX
- PA9 CS, PA10 INT
- 0x101 4-frame staging
- 0x201 status
- 0x301 current position feedback
- simulated homing
- simulated 1ms trajectory interpolation
```

아직 실제 모터 구동은 하지 않습니다.

```text
- STEP/DIR 실제 pulse 출력 미구현
- 실제 limit switch homing 미구현
- 실제 driver fault pin 처리 미구현
```

자세한 프로토콜은 `Board1_CAN_Protocol_Integrated_FINAL.md`를 확인하세요.
