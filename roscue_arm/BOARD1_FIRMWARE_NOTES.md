# Board1 STM32F411CEU6 Firmware Notes

## 포함 기능

- 팔 2~4축 + 베이스 1축 STEP/DIR 제어
- TMC5160/TMC2240 Software SPI 초기 설정
- MCP2515 SPI2 CAN 수신/송신
- 리밋스위치 Homing
- HAL 미사용, CMSIS 레지스터 직접 접근

## MCP2515 핀

| 기능 | 핀 | 실제 번호 |
|---|---|---|
| SCK | PB13 | scl |
| SO / MISO | PB14 | sda |
| SI / MOSI | PB15 | 납떔 |
| CS | PA9 | tx1 |
| INT | PA10 | rx1 |


## 보드2 STM32F411CEU6
현재 STM32 Board2는 베이스 제어에 사용하지 않습니다. 베이스축은 Board1 Motor ID 3으로 이동했습니다.
Arduino Board2는 CAN `0x102/0x202/0x302`로 로봇팔 5축을 제어합니다.

DIR PB6
STEP PB7
MISO PB0
CS4 PB2
CLK PB5
MOSI PB1
MOTEN PB3
LIM4 PC8

b10


// 보드2-2
CAN모듈
| SCK | PB13 |
| SO / MISO | PB14 |
| SI / MOSI | PB15 |
| CS | PA9 |
| INT | PA10 |




## UART 디버그 TTL 핀

| 보드 | UART | TX | RX |
|---|---|---|---|
| Board1 | USART2 | PA2 | PA3 |
| Board2 | USART2 | PA2 | PA3 |

외부 USB-TTL은 STM32 TX -> USB-TTL RX, STM32 RX -> USB-TTL TX, GND -> GND로 연결합니다. VCC/5V/3V3은 연결하지 않습니다.

## 리밋스위치 핀

| 축 | 핀 |
|---|---|
| LIM1 | PA7 |
| LIM2 | PA15 |
| LIM3 | PB4 |
| LIM4 | PB12 |

## Board1 축 매핑과 감속비

| Board1 Motor ID | 실제 축 | 감속비 | 모터 full steps/rev |
|---:|---:|---:|---:|
| 0 | 2축 | 20 | 200 |
| 1 | 3축 | 50 | 200 |
| 2 | 4축 | 30 | 200 |
| 3 | 베이스 1축 | 20 | 200 |

Board1 Motor ID 0~2는 TMC5160, Motor ID 3은 TMC2240을 사용합니다.
Board1 Motor ID 3의 리미트 스위치는 PB12(LIM4)입니다.

Arduino Board2는 팔 5축을 담당하며 감속비는 120, 모터 full steps/rev는 48입니다.

## CAN ID

| CAN ID | 용도 |
|---:|---|
| 0x001 | Emergency Stop |
| 0x010 | Enable / Disable |
| 0x020 | Homing Start |
| 0x030 | Clear Error |
| 0x101 | Board1 Arm/Base Axis Move |
| 0x201 | Board1 Status |

## 주의

1. MCP2515 모듈은 8MHz 크리스탈 기준 500kbps CAN 설정을 사용합니다.
2. TMC5160/TMC2240 전류 설정값은 테스트용으로 낮게 넣었습니다. 실제 모터/Rsense에 맞게 조정해야 합니다.
3. PA15, PB3, PB4는 JTAG 관련 기능과 겹칠 수 있습니다. SWD만 쓰는 환경에서 테스트하세요.
4. PC14/PC15는 고속 출력에 약하므로 Board1 Motor ID 1 축 STEP 속도는 낮은 속도부터 테스트하세요.
5. 리밋스위치는 `입력핀 - 스위치 - GND` 방식이며 내부 Pull-up을 사용합니다.

## CAN 명령 예시

### Enable

ID: 0x010

| Byte0 |
|---|
| 1 |

### Homing 전체 축

ID: 0x020

| Byte0 | Byte1 |
|---|---|
| 255 | 0 |

### 팔 2축 30도 이동

ID: 0x101

| Byte0 | Byte1~4 | Byte5~6 | Byte7 |
|---|---|---|---|
| 0x80 | 3000 | speed | 10 |

Byte0은 `Bit7 Execute`, `Bit6 Relative`, `Bit5 Step Mode`, `Bit4 Reserved`, `Bit3~0 Motor ID`입니다. `0x80`은 Execute=1, Motor ID=0입니다.

`Target Angle`은 0.01도 단위입니다. `Byte7=10`은 50ms입니다.
