# Board1 / Board2 / Board3 CAN Protocol and STM32 Implementation

## 1. Board ID and CAN ID

서버/RPi 쪽에서는 `board_id`로 대상 보드를 고르고, 실제 CAN frame에는 아래 CAN ID를 사용합니다.
payload에는 별도 Board ID를 넣지 않습니다.

| Board ID | 대상 | Move CAN ID | Status CAN ID | Payload Motor ID |
|---:|---|---:|---:|---|
| `1` | Board1, 1~4축 | `0x101` | `0x201` | `0~3` |
| `2` | Board2, 5축 | `0x102` | `0x202` | `0` |
| `3` | Board3, 서보 9개 | `0x103` | `0x203` | `0~8` |

서버 내부의 global joint id와 CAN payload의 Motor ID는 분리합니다.

| Global Joint ID | 실제 축 | Board ID | Move CAN ID | Payload Motor ID |
|---:|---|---:|---:|---:|
| `0` | 1축 | `1` | `0x101` | `0` |
| `1` | 2축 | `1` | `0x101` | `1` |
| `2` | 3축 | `1` | `0x101` | `2` |
| `3` | 4축 | `1` | `0x101` | `3` |
| `4` | 5축 | `2` | `0x102` | `0` |
| `5` | 서보 1 | `3` | `0x103` | `0` |
| `6` | 서보 2 | `3` | `0x103` | `1` |
| `7` | 서보 3 | `3` | `0x103` | `2` |
| `8` | 서보 4 | `3` | `0x103` | `3` |
| `9` | 서보 5 | `3` | `0x103` | `4` |
| `10` | 서보 6 | `3` | `0x103` | `5` |
| `11` | 서보 7 | `3` | `0x103` | `6` |
| `12` | 서보 8 | `3` | `0x103` | `7` |
| `13` | 서보 9 | `3` | `0x103` | `8` |

즉 Board2 5축 명령은 `CAN ID=0x102`, `Motor ID=0`으로 보냅니다.
Board3 서보 명령은 `CAN ID=0x103`, `Motor ID=0~8`로 보냅니다.

## 2. Board1 CAN ID

| CAN ID | 방향 | 용도 |
|---:|---|---|
| `0x001` | RPi -> STM32 | Emergency Stop |
| `0x010` | RPi -> STM32 | Enable / Disable |
| `0x020` | RPi -> STM32 | Homing Start |
| `0x030` | RPi -> STM32 | Clear Error |
| `0x101` | RPi -> STM32 | Board1 motor trajectory point |
| `0x201` | STM32 -> RPi | Board1 status response |

Board1은 `0x101` 위치 명령에서 Motor ID `0~3`만 처리합니다.

MoveIt2 trajectory point 하나는 Motor ID `0 -> 1 -> 2 -> 3` 순서의 네 CAN frame으로 전송해야 합니다. STM32는 네 frame을 staging한 뒤 하나의 4축 point로 queue에 넣고, 4축 segment를 같은 1ms tick에서 동시에 시작합니다.

## 3. Board2 CAN ID

| CAN ID | 방향 | 용도 |
|---:|---|---|
| `0x001` | RPi -> STM32 | Emergency Stop |
| `0x010` | RPi -> STM32 | Enable / Disable |
| `0x020` | RPi -> STM32 | Homing Start |
| `0x030` | RPi -> STM32 | Clear Error |
| `0x102` | RPi -> STM32 | Board2 motor trajectory point |
| `0x202` | STM32 -> RPi | Board2 status response |

Board2는 `0x102` 위치 명령에서 Motor ID `0`만 처리합니다.

Board2는 5축 단일 축 보드로 사용합니다. 서버의 global joint id는 `4`이지만, CAN payload 안에서는 Board2 로컬 Motor ID `0`으로 보냅니다.

Board2는 한 trajectory point당 CAN frame 하나만 전송하면 됩니다.

## 4. Board3 CAN ID

| CAN ID | 방향 | 용도 |
|---:|---|---|
| `0x001` | RPi -> STM32 | Emergency Stop |
| `0x010` | RPi -> STM32 | Enable / Disable |
| `0x030` | RPi -> STM32 | Clear Error |
| `0x103` | RPi -> STM32 | Board3 servo command |
| `0x203` | STM32 -> RPi | Board3 status response |

Board3는 `0x103` 명령에서 Motor ID `0~8`로 서보 1~9번을 구분합니다.

서버의 global servo id는 `5~13`이지만, CAN payload 안에서는 Board3 로컬 Motor ID `0~8`로 보냅니다.

Board3는 서보 9개 전체를 CAN frame 하나에 담지 않습니다. Classic CAN payload는 8바이트이므로, `0x103` frame 하나는 서보 하나의 명령만 담습니다.

9개 서보를 같은 trajectory point에서 동시에 움직여야 하면 Motor ID `0 -> 1 -> ... -> 8` 순서의 아홉 CAN frame을 전송하고, Board3 펌웨어에서 staging한 뒤 9개 서보를 같은 tick에서 동시에 시작합니다.

## 5. RPi -> STM32 Position Command, CAN ID `0x101` / `0x102`

8바이트 고정 길이입니다.

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | 상위 4bit flags, 하위 4bit motor id |
| 1 | Target Pos LSB | `int32_t` 일부 | little endian |
| 2 | Target Pos | `int32_t` 일부 | little endian |
| 3 | Target Pos | `int32_t` 일부 | little endian |
| 4 | Target Pos MSB | `int32_t` 일부 | little endian |
| 5 | Speed LSB | `uint16_t` 일부 | little endian |
| 6 | Speed MSB | `uint16_t` 일부 | little endian |
| 7 | Duration | `uint8_t` | 5ms 단위 |

### Control & Motor ID (Byte 0)

```text
Bit7 Bit6 Bit5 Bit4 | Bit3 Bit2 Bit1 Bit0
Exec Rel  Step Rsv  | Motor ID
```

| Bit | 이름 | 의미 |
|---:|---|---|
| 7 | Execute | `1`: 명령 실행/queue push, `0`: 무시 |
| 6 | Relative | MoveIt2 Bridge에서는 `0` 고정. `1`은 invalid |
| 5 | Step Mode | MoveIt2 Bridge에서는 `0` 고정. Target Pos는 0.01도 단위 angle |
| 4 | Reserved | 현재 미사용, `0` |
| 3~0 | Motor ID | Board1은 `0~3`, Board2는 `0`만 유효 |

RPi 쪽 Byte0 생성 예:

```c
uint8_t byte0 =
    ((execute   ? 1 : 0) << 7) |
    ((relative  ? 1 : 0) << 6) |
    ((step_mode ? 1 : 0) << 5) |
    (motor_id & 0x0F);
```

STM32 쪽 구현은 아래 방식으로 언패킹합니다.

```c
uint8_t motor_id = RxData[0] & 0x0F;
uint8_t flags    = RxData[0] >> 4;

uint8_t execute   = (flags & 0x08) ? 1 : 0;
uint8_t relative  = (flags & 0x04) ? 1 : 0;
uint8_t step_mode = (flags & 0x02) ? 1 : 0;
```

### Target Pos (Byte 1~4)

MoveIt2 Bridge 경로에서는 angle만 사용합니다.

| Step Mode | Target Pos 의미 |
|---:|---|
| `0` | 0.01도 단위 angle. 예: `3000` = `30.00 deg` |
| `1` | 현재 4축 staging 경로에서는 invalid |

angle mode에서 STM32는 아래 정수 공식으로 step 변환합니다.

```c
step = angle_raw * gear_ratio[axis] * 200 * 16 / 36000;
```

현재 Board1 gear ratio:

| Axis / Motor ID | Gear ratio |
|---:|---:|
| 0 | 20 |
| 1 | 20 |
| 2 | 75 |
| 3 | 30 |

Board2 gear ratio는 5축 기구 사양에 맞춰 Board2 펌웨어에서 별도로 정의합니다.

### Speed (Byte 5~6)

`Speed`는 현재 STM32 초기 구현에서 queue에 저장하지만 1ms 보간 계산에는 사용하지 않습니다. 실제 이동 시간은 `Duration`이 결정합니다.

### Duration (Byte 7)

`Duration`은 5ms 단위입니다.

5ms, 10ms, 15ms, 20ms, ...

```text
duration_ms = Byte7 * 5
```

MoveIt2 trajectory point를 50ms 간격으로 보낼 경우:

```text
Byte7 = 10
```

`Byte7=0`이면 STM32 내부에서 최소 `1ms` segment로 처리합니다.

Board1에서 같은 4축 point에 속한 Motor ID `0~3` frame은 모두 같은 Duration을 가져야 합니다. Duration이 하나라도 다르면 staging을 폐기하고 `ERR_INVALID_CMD`를 보고합니다.

Board2는 단일 축이므로 frame 하나의 Duration만 사용합니다.

### Position Command Example

Motor 0을 30.00도로 50ms 동안 이동:

| Byte | 값 |
|---:|---:|
| 0 | `0x80` |
| 1~4 | `3000` as `int32_t little endian` |
| 5~6 | speed as `uint16_t little endian` |
| 7 | `10` |

`0x80`은 Execute=1, Relative=0, Step Mode=0, Motor ID=0입니다.

실제 MoveIt2 point는 아래처럼 네 frame을 연속 전송해야 합니다.

| 순서 | Motor ID | Byte0 예시 |
|---:|---:|---:|
| 1 | 0 | `0x80` |
| 2 | 1 | `0x81` |
| 3 | 2 | `0x82` |
| 4 | 3 | `0x83` |

안 움직이는 축도 생략하지 말고 해당 point의 목표 위치를 그대로 보내야 합니다. `0`을 보내면 0도로 이동하라는 의미입니다.

Board2 5축을 30.00도로 50ms 동안 이동:

| Byte | 값 |
|---:|---:|
| 0 | `0x80` |
| 1~4 | `3000` as `int32_t little endian` |
| 5~6 | speed as `uint16_t little endian` |
| 7 | `10` |

이 frame의 CAN ID는 `0x102`입니다.

## 6. Board3 Servo Command, CAN ID `0x103`

Board3 서보 9개는 `0x103`을 사용합니다.

Classic CAN payload는 8바이트이므로 9개 서보 값을 frame 하나에 모두 넣을 수 없습니다. Board3는 서보당 1 frame을 사용합니다.

권장 포맷은 Board1/Board2 위치 명령과 동일하게 유지합니다.

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | 상위 4bit flags, 하위 4bit servo motor id `0~8` |
| 1 | Target Pos LSB | `int32_t` 일부 | little endian |
| 2 | Target Pos | `int32_t` 일부 | little endian |
| 3 | Target Pos | `int32_t` 일부 | little endian |
| 4 | Target Pos MSB | `int32_t` 일부 | little endian |
| 5 | Speed LSB | `uint16_t` 일부 | little endian |
| 6 | Speed MSB | `uint16_t` 일부 | little endian |
| 7 | Duration | `uint8_t` | 5ms 단위 |

Target Pos는 서보 각도를 0.01도 단위로 넣습니다. 예: `9000` = `90.00 deg`.

서보 Motor ID:

| Payload Motor ID | 대상 |
|---:|---|
| `0` | 서보 1 |
| `1` | 서보 2 |
| `2` | 서보 3 |
| `3` | 서보 4 |
| `4` | 서보 5 |
| `5` | 서보 6 |
| `6` | 서보 7 |
| `7` | 서보 8 |
| `8` | 서보 9 |

동시 제어가 필요한 경우:

| 순서 | Servo Motor ID | Byte0 예시 |
|---:|---:|---:|
| 1 | 0 | `0x80` |
| 2 | 1 | `0x81` |
| 3 | 2 | `0x82` |
| 4 | 3 | `0x83` |
| 5 | 4 | `0x84` |
| 6 | 5 | `0x85` |
| 7 | 6 | `0x86` |
| 8 | 7 | `0x87` |
| 9 | 8 | `0x88` |

Board3 staging 규칙:

- Motor ID는 `0 -> 1 -> ... -> 8` 순서로 받습니다.
- 첫 frame 수신 후 20ms 안에 아홉 frame이 모두 들어와야 합니다.
- 아홉 frame의 Duration은 모두 같아야 합니다.
- Execute=1, Relative=0, StepMode=0, Reserved=0이어야 합니다.
- 위 조건을 만족하면 Board3는 9개 서보 point를 queue에 넣고 같은 tick에서 동시에 시작합니다.

Board3 펌웨어 작성 시 추가로 정해야 할 항목:

- torque enable/disable 같은 서보 전용 제어 명령을 별도 CAN ID로 뺄지 결정
- status `0x203`에 각 서보 상태를 bit field로 보고할지 결정

## 7. RPi -> STM32 Control Commands

### Emergency Stop, CAN ID `0x001`

Payload는 현재 사용하지 않습니다.

수신 즉시 STM32는:

- 모든 STEP 출력 정지
- 모터 disable
- trajectory queue clear
- 상태를 ESTOP으로 전환
- 해당 보드 status 즉시 송신, Board1은 `0x201`, Board2는 `0x202`, Board3는 `0x203`

### Enable / Disable, CAN ID `0x010`

| Byte | 값 | 의미 |
|---:|---:|---|
| 0 | `0` | Disable |
| 0 | `1` | Enable |

Enable 수신 시 STM32는 ESTOP flag와 error를 해제하고 공통 motor enable 핀을 enable 상태로 둡니다.

Disable 수신 시 STM32는 queue clear, step 정지, motor disable을 수행합니다.

### Homing Start, CAN ID `0x020`

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Motor ID | Board1은 `0~3`, Board2는 `0`, 또는 전체 축 `255` |
| 1 | Homing Mode | 현재 `0`만 사용 |

조건:

- Enable 상태가 아니면 무시합니다.
- ESTOP 상태이면 무시합니다.
- mode가 `0`이 아니면 `ERR_INVALID_CMD`로 처리합니다.

Homing 완료 시 해당 축의 `current_step`과 `target_step`은 0으로 설정되고 `homing_done`이 1이 됩니다.

### Clear Error, CAN ID `0x030`

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Motor ID | Board1은 `0~3`, Board2는 `0`, 또는 전체 `255` |

현재 구현에서는 error code를 `ERR_NONE`으로 초기화합니다. ESTOP 자체는 Clear Error만으로 해제하지 않고, `0x010 Enable=1`에서 해제합니다.

## 8. STM32 -> RPi Status, CAN ID `0x201` / `0x202` / `0x203`

STM32는 100ms마다 status를 송신하고, ESTOP/Enable/Homing/Clear Error/Queue Full 같은 주요 이벤트에서도 즉시 한 번 송신합니다.

| Board ID | Status CAN ID |
|---:|---:|
| `1` | `0x201` |
| `2` | `0x202` |
| `3` | `0x203` |

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State | 현재 보드 상태 |
| 1 | Error Code | 현재 error code |
| 2 | Homing Done Bits | Board1: bit0~3 = axis0~3 homing done, Board2: bit0 = axis0 homing done, Board3: servo ready bits 또는 reserved |
| 3 | Moving Motor ID | 이동 또는 homing 중인 첫 motor id, 없으면 `255` |
| 4 | Limit Status Bits | Board1: bit0~3 = axis0~3 limit active, Board2: bit0 = axis0 limit active, Board3: servo fault bits 또는 reserved |
| 5 | Queue Free | trajectory queue 남은 슬롯 수 |
| 6 | Enabled | 공통 motor enable 상태. `0`: disabled, `1`: enabled |
| 7 | Reserved | 현재 `0` |

### State Values

| 값 | 이름 |
|---:|---|
| 0 | `STATE_INIT` |
| 1 | `STATE_IDLE` |
| 2 | `STATE_HOMING` |
| 3 | `STATE_MOVING` |
| 4 | `STATE_ERROR` |
| 5 | `STATE_ESTOP` |

### Error Codes

| 값 | 이름 | 의미 |
|---:|---|---|
| 0 | `ERR_NONE` | 정상 |
| 1 | `ERR_INVALID_CMD` | 잘못된 명령, motor id, homing 전 move 등 |
| 2 | `ERR_LIMIT_DETECTED` | 예약됨 |
| 3 | `ERR_DRIVER_FAULT` | MCP2515 init 실패 등 driver fault |
| 4 | `ERR_HOMING_FAIL` | 예약됨 |
| 5 | `ERR_QUEUE_FULL` | trajectory queue full, 새 명령 drop |
| 6 | `ERR_RESERVED` | 예약됨 |

## 9. Queue and Error Policy

Board1 STM32 trajectory queue는 외부 status 기준으로 32개의 `0x101` command slot입니다. 내부 구현은 4축 point queue 8개이며, 한 4축 point가 command slot 4개를 사용합니다.

Board2 STM32 trajectory queue는 단일 축 point queue로 구성합니다. 외부 status의 Queue Free는 `0x102` command slot 기준으로 보고합니다.

Queue full 상태에서 새 위치 명령이 오면:

- 새 명령은 저장하지 않습니다. 즉 Drop Tail입니다.
- 기존 queue 내용은 유지합니다.
- `ERR_QUEUE_FULL`을 설정합니다.
- 해당 보드 status를 즉시 송신합니다.
- error가 clear되기 전까지 추가 move 명령은 무시합니다.

Board1 staging 규칙:

- Motor ID는 반드시 `0 -> 1 -> 2 -> 3` 순서여야 합니다.
- 첫 frame 수신 후 20ms 안에 네 frame이 모두 들어와야 합니다.
- 네 frame의 Duration은 모두 같아야 합니다.
- Execute=1, Relative=0, StepMode=0, Reserved=0이어야 합니다.
- 위 조건을 만족하지 않으면 staging을 폐기하고 `ERR_INVALID_CMD`를 즉시 보고합니다.

Board2 staging 규칙:

- 한 point당 `0x102` frame 하나만 사용합니다.
- Motor ID는 `0`이어야 합니다.
- Execute=1, Relative=0, StepMode=0, Reserved=0이어야 합니다.
- 위 조건을 만족하지 않으면 `ERR_INVALID_CMD`를 즉시 보고합니다.

Board3 command 규칙:

- Motor ID는 `0~8`이어야 합니다.
- 9개 서보 전체를 frame 하나에 담지 않고, 서보당 `0x103` frame 하나를 사용합니다.
- 동시 제어가 필요하면 Motor ID `0 -> 1 -> ... -> 8` 순서의 9개 frame을 staging합니다.
- 잘못된 Motor ID나 지원하지 않는 flags는 `ERR_INVALID_CMD`로 보고합니다.

상위 제어기 권장 동작:

1. `STATE_ERROR` 또는 `ERR_QUEUE_FULL` 감지
2. trajectory 송신 중단
3. 필요 시 `0x001 ESTOP` 또는 `0x010 Disable`
4. queue를 비운 뒤 다시 시작하려면 Enable/Homing/trajectory 재동기화
5. `0x030 Clear Error` 후 재전송

## 10. STM32 Implementation Summary

현재 STM32 쪽에 구현된 내용:

- MCP2515 SPI2 기반 CAN 송수신
- CAN ID `0x001`, `0x010`, `0x020`, `0x030`, `0x101`, `0x201`
- `0x101` 위치 명령 4개를 staging 후 4축 trajectory point queue push
- Queue full Drop Tail 및 즉시 status 응답
- TIM3 1ms trajectory 선형 보간
- TIM2 10us STEP/DIR pulse 생성
- Homing 및 limit switch debounce
- 100ms 주기 status 송신
- PC용 간단 프로토콜 테스트 `board1_virtual_can_test.c`

아직 초기 구현에서 제한적으로 처리하는 내용:

- `Speed`는 4축 point에 저장하지만 보간 계산에는 사용하지 않습니다.
- CAN 수신은 MCP2515 INT pin polling 방식입니다.
- 고급 속도 프로파일, acceleration/deceleration은 아직 구현하지 않았습니다.

Board2 펌웨어를 만들 때 Board1 구현에서 바꿀 핵심:

- 위치 명령 CAN ID를 `0x102`로 변경
- 상태 응답 CAN ID를 `0x202`로 변경
- 축 개수를 1개로 축소
- 유효 Motor ID를 `0`만 허용
- Board1의 4축 staging 로직 제거
- 5축 gear ratio, STEP/DIR/EN/LIMIT pin을 Board2 하드웨어에 맞게 정의

Board3 펌웨어를 만들 때 정해야 할 핵심:

- 위치/서보 명령 CAN ID는 `0x103`
- 상태 응답 CAN ID는 `0x203`
- 서보 개수는 9개
- 유효 Motor ID는 `0~8`
- 9개 서보 전체 명령은 `0x103` frame 9개를 staging해서 처리
- 서보 enable, fault, ready 상태를 `0x203`에 어떻게 담을지 확정
