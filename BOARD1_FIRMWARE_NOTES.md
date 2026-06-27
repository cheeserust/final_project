# Board1 STM32F411CEU6 Firmware Notes

## 포함 기능

- 1~4축 STEP/DIR 제어
- TMC5160 Software SPI 초기 설정
- MCP2515 SPI2 CAN 수신/송신
- 리밋스위치 Homing
- HAL 미사용, CMSIS 레지스터 직접 접근

## MCP2515 핀

| 기능 | 핀 |
|---|---|
| SCK | PB13 |
| SO / MISO | PB14 |
| SI / MOSI | PB15 |
| CS | PB12 |
| INT | PB4 |

## CAN ID

| CAN ID | 용도 |
|---:|---|
| 0x001 | Emergency Stop |
| 0x010 | Enable / Disable |
| 0x020 | Homing Start |
| 0x030 | Clear Error |
| 0x101 | Board1 Axis Move |
| 0x201 | Board1 Status |

## 주의

1. MCP2515 모듈의 크리스탈이 16MHz인지 확인하세요. 8MHz면 코드의 CNF 값을 바꿔야 합니다.
2. TMC5160 전류 설정값은 테스트용으로 낮게 넣었습니다. 실제 모터/Rsense에 맞게 조정해야 합니다.
3. PB3, PB4는 JTAG 관련 기능과 겹칠 수 있습니다. SWD만 쓰는 환경에서 테스트하세요.
4. PC14/PC15는 고속 출력에 약하므로 2축 STEP 속도는 낮은 속도부터 테스트하세요.
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

### 1축 30도 이동

ID: 0x101

| Byte0 | Byte1~4 | Byte5~6 | Byte7 |
|---|---|---|---|
| 0x80 | 3000 | speed | 10 |

Byte0은 `Bit7 Execute`, `Bit6 Relative`, `Bit5 Step Mode`, `Bit4 Reserved`, `Bit3~0 Motor ID`입니다. `0x80`은 Execute=1, Motor ID=0입니다.

`Target Angle`은 0.01도 단위입니다. `Byte7=10`은 50ms입니다.
