# Board3 Gripper CAN Pre-Hardware Firmware - STM32F411RE + 0x303 Position Feedback

이 프로젝트는 첨부된 기존 `board3_f411re` 코드 구조를 유지한 상태에서, 수정된 Board3 최종 프로토콜과 `0x303` 고속 현재 위치 피드백을 반영한 STM32F411RE 전용 테스트 펌웨어입니다.

실제 SCS0009 서보/그리퍼를 연결하지 않은 상태에서도 CAN 통신과 프로토콜 로직을 확인할 수 있도록 구성했습니다. 정상 `0x103` command set이 완성되면 실제 Feetech 제어 대신 가상 실행을 수행하고, 목표 각도를 현재 위치 피드백 값으로 복사해 `0x303`에서 확인할 수 있게 했습니다.

---

## 1. 유지한 코드 구조

원본 zip과 동일하게 단일 프로젝트 구조를 유지했습니다.

```text
board3_f411re_0x303/
├── Makefile
├── main.c
├── gripper_shared.h
├── mcp2515.c
├── spi.c
├── uart.c
├── led.c
├── timer.c
├── systick.c
├── clock.c
└── 기타 STM32/CMSIS 파일
```

핵심 수정 파일은 `main.c`, `gripper_shared.h`, `README_BOARD3_F411RE_PRE_HW.md`입니다.

---

## 2. 테스트 보드 / 핀

| 항목 | 내용 |
|---|---|
| MCU | STM32F411RE |
| CAN Controller | MCP2515 |
| CAN Transceiver | MCP2551 / TJA1050 등 |
| Debug UART | USART2 PA2/PA3, 115200 |
| LED | PA5 active-high / Nucleo LD2 |
| 실제 서보 | 현재 미연결 테스트 기준 |

MCP2515 배선은 업로드된 원본 코드 기준을 유지했습니다.

| MCP2515 | STM32F411RE | 기능 |
|---|---:|---|
| SCK | PB13 | SPI2_SCK |
| MISO / SO | PB14 | SPI2_MISO |
| MOSI / SI | PB15 | SPI2_MOSI |
| CS / NSS | PB12 | GPIO output |
| INT | PB4 | EXTI4 active-low + polling fallback |
| VCC | 3.3V | 전원 |
| GND | GND | 공통 GND |

주의: PB4/PB12은 MCP2515 CS/INT로 사용하므로 UART1 디버그용으로 사용하면 안 됩니다. 디버그 출력은 USART2 PA2/PA3 기준입니다.

---

## 3. 반영된 CAN ID

| CAN ID | 방향 | 의미 | 구현 |
|---:|---|---|---|
| `0x001` | 서버/RPi → Board3 | Emergency Stop | 처리 |
| `0x010` | 서버/RPi → Board3 | Enable / Disable broadcast | 처리 |
| `0x020` | 서버/RPi → Board1 + Board2 | Arm Homing broadcast | Board3는 무시 |
| `0x023` | 서버/RPi → Board3 | Gripper Home Posture | 처리 |
| `0x030` | 서버/RPi → Board3 | Clear Error broadcast | 처리 |
| `0x103` | 서버/RPi → Board3 | Gripper Servo Command | 9-frame staging 처리 |
| `0x203` | Board3 → 서버/RPi | Board3 Status | 100ms + 이벤트 송신 |
| `0x303` | Board3 → 서버/RPi | Current Position Feedback | 20ms 송신 |

---

## 4. 0x203과 0x303 역할 분리

```text
0x203 = 상태/에러/heartbeat
- 100ms 주기
- Enable, Disable, ESTOP, Clear Error, Home, Command 완료, Error 시 즉시 송신
- payload: state, error_code, ready, staging_count, fault, buffer_free, enabled, fault_motor_id

0x303 = MoveIt용 현재 위치 피드백
- 20ms 주기
- 3개 CAN frame으로 9개 Motor ID 현재 위치 전송
- int16_t, 0.01도 단위, little-endian
```

0x303은 0x203보다 짧은 주기로 동작합니다. 코드에서는 0x203 heartbeat tick과 0x303 feedback tick이 같은 순간에 몰리지 않도록 0x303 송신을 10ms offset으로 발생시켰습니다. 주기는 그대로 20ms입니다.

---

## 5. 0x303 Position Feedback 구조

```text
CAN ID = 0x303
DLC = 8
주기 = 20ms
1 cycle = 3 frames
```

| Frame index / Byte0 | Byte1~2 | Byte3~4 | Byte5~6 | Byte7 |
|---:|---|---|---|---|
| `0x01` | Motor 0 current pos | Motor 1 current pos | Motor 2 current pos | `0x00` Reserved |
| `0x02` | Motor 3 current pos | Motor 4 current pos | Motor 5 current pos | `0x00` Reserved |
| `0x03` | Motor 6 current pos | Motor 7 current pos | Motor 8 current pos | `0x00` Reserved |

각 위치값은 다음 형식입니다.

```text
자료형: int16_t
단위: 0.01도
엔디안: Little Endian
표현 범위: -327.68도 ~ +327.67도
```

예:

```text
30.00도 = 3000 = 0x0BB8 → payload에서는 B8 0B
-15.50도 = -1550 → int16 two's complement little-endian
```

현재 v1.1에서는 Byte7을 `0x00 Reserved`로 둡니다. 서버 참고 문서의 상태/에러 flag bitmap은 추후 확장용으로 남겨두었습니다.

---

## 6. 0x103 Gripper Command

`0x103` frame 하나는 Motor ID 1개 명령입니다. Board3 전체 gripper 동작은 Motor ID `0~8`의 9개 frame이 모두 모였을 때만 성립합니다.

```text
CAN ID = 0x103
DLC = 8
payload에 Board ID 없음
```

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | Execute + Motor ID | Bit7 Execute, Bit3~0 Motor ID |
| 1~4 | Target Position | int32_t, 0.01도, little-endian |
| 5~6 | Speed | 현재 미사용, 0 권장 |
| 7 | Duration | 5ms 단위 tick |

유효 Byte0 예시는 다음과 같습니다.

| Motor ID | Byte0 |
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

---

## 7. 0x023 Gripper Home

Board3 gripper home posture는 `0x023`을 사용합니다. `0x020`은 Board1/Board2 arm homing broadcast이므로 Board3에서는 무시합니다.

```bash
# Board3 gripper 전체 home posture, 기본 duration 500ms
cansend can0 023#FF00000000000000

# duration = 100 x 5ms = 500ms 명시
cansend can0 023#FF00640000000000
```

Home posture는 모든 Motor ID `0~8` 목표 각도를 `0.00도`로 만드는 가상 command를 생성합니다.

---

## 8. 빌드 / 플래시

```bash
cd board3_f411re_0x303
make clean
make
st-flash write ./rom_0x08000000.bin 0x08000000
```

이 환경에는 `arm-none-eabi-gcc`가 없어서 제가 컨테이너 내부에서 실제 ARM 빌드까지 확인하지는 못했습니다. 코드 구조는 기존 Makefile과 원본 프로젝트 구조를 그대로 유지했습니다.

---

## 9. CAN 설정

Linux / Raspberry Pi 쪽 예시입니다.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can0 up
candump can0
```

0x203만 보고 싶으면:

```bash
candump can0,203:7FF
```

0x303만 보고 싶으면:

```bash
candump can0,303:7FF
```

---

## 10. 기본 테스트 순서

### 10.1 Enable

최종 프로토콜 payload:

```bash
cansend can0 010#0100000000000000
```

현재 수정본은 최종 프로토콜 검증을 위해 legacy payload를 허용하지 않습니다.
아래 형식은 사용하지 마세요.

```bash
# 사용하지 않음
cansend can0 010#01
cansend can0 010#0301
```

### 10.2 Gripper Home

```bash
cansend can0 023#FF00000000000000
```

예상:

```text
0x203 status 즉시 송신
0x303 position feedback에서 9개 motor 현재 위치가 0.00도로 갱신
```

### 10.3 9개 frame command set

아래는 Motor ID 0~8 모두 target=0.00도, duration=100 tick입니다.

```bash
cansend can0 103#8000000000000064
cansend can0 103#8100000000000064
cansend can0 103#8200000000000064
cansend can0 103#8300000000000064
cansend can0 103#8400000000000064
cansend can0 103#8500000000000064
cansend can0 103#8600000000000064
cansend can0 103#8700000000000064
cansend can0 103#8800000000000064
```

정상이라면 UART에 `STAGING OK`와 `VIRTUAL GRIPPER EXEC`가 출력되고, 이후 `0x303`에서 현재 위치값이 반영됩니다.

### 10.4 30도 명령 예시

30.00도는 0.01도 단위로 `3000`, little-endian 32비트로 `B8 0B 00 00`입니다.

```bash
# Motor 0만 30.00도 예시. 실제 command set은 0~8 모두 보내야 실행됨.
cansend can0 103#80B80B0000000064
```

---

## 11. 구현 메모

- `gripper_shared.h`에 `CAN_ID_BOARD3_POSITION 0x303U`를 추가했습니다.
- `gripper_shared.h`에 `GRIPPER_POSITION_FEEDBACK_PERIOD_MS 20U`를 추가했습니다.
- `GripperState`에 `int16_t current_pos_001deg[GRIPPER_MOTOR_COUNT]`를 추가했습니다.
- `main.c`에 `send_position_feedback_0x303()`를 추가했습니다.
- 가상 실행에서는 `g_cmd.target_pos_001deg[]`를 `g_state.current_pos[]`와 `g_state.current_pos_001deg[]`에 반영합니다.
- 실제 서보 연결 후에는 `current_pos_001deg[]`를 목표값 복사가 아니라 Feetech 실제 위치 읽기값을 각도로 역변환한 값으로 갱신해야 합니다.
