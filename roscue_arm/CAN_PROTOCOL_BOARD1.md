# Board1 / Board2 / Board3 CAN Protocol and STM32 Implementation

## 1. Board ID and CAN ID

서버/RPi 쪽에서는 `board_id`로 대상 보드를 고르고, 실제 CAN frame에는 아래 CAN ID를 사용합니다.
payload에는 별도 Board ID를 넣지 않습니다.

| Board ID | 대상 | Move CAN ID | Status CAN ID | Position Feedback CAN ID | Payload Motor ID |
|---:|---|---:|---:|---:|---|
| `1` | Board1, 팔 2~4축 + 베이스 1축 | `0x101` | `0x201` | `0x301` | `0~3` |
| `2` | Arduino Board2, 팔 5축 | `0x102` | `0x202` | `0x302` | `0` |
| `3` | Board3, 서보 9개 | `0x103` | `0x203` | `0x303` | `0~8` |

서버 내부의 global joint id와 CAN payload의 Motor ID는 분리합니다.

| Global Joint ID | 실제 축 | Board ID | Move CAN ID | Payload Motor ID |
|---:|---|---:|---:|---:|
| `0` | 베이스 1축 | `1` | `0x101` | `3` |
| `1` | 팔 2축 | `1` | `0x101` | `0` |
| `2` | 팔 3축 | `1` | `0x101` | `1` |
| `3` | 팔 4축 | `1` | `0x101` | `2` |
| `4` | 팔 5축 | `2` | `0x102` | `0` |
| `5` | 서보 1 | `3` | `0x103` | `0` |
| `6` | 서보 2 | `3` | `0x103` | `1` |
| `7` | 서보 3 | `3` | `0x103` | `2` |
| `8` | 서보 4 | `3` | `0x103` | `3` |
| `9` | 서보 5 | `3` | `0x103` | `4` |
| `10` | 서보 6 | `3` | `0x103` | `5` |
| `11` | 서보 7 | `3` | `0x103` | `6` |
| `12` | 서보 8 | `3` | `0x103` | `7` |
| `13` | 서보 9 | `3` | `0x103` | `8` |

즉 베이스 명령은 Board1의 `CAN ID=0x101`, `Motor ID=3`으로 보냅니다.
Arduino Board2의 팔 5축 명령은 `CAN ID=0x102`, `Motor ID=0`으로 보냅니다.
Board3 서보 명령은 `CAN ID=0x103`, `Motor ID=0~8`로 보냅니다.

## 2. Arm Joint Limits and Home Position

각도 값은 CAN payload의 Target Pos와 같은 `0.01도` 단위 raw angle로 변환해서 사용합니다.

| Joint | Board ID | Payload Motor ID | Min deg | Max deg | Home deg | Min raw | Max raw | Home raw |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `base_joint` | `1` | `3` | `-90` | `180` | `-90` | `-9000` | `18000` | `-9000` |
| `arm_joint_1` / 팔 2축 | `1` | `0` | `-85` | `90` | `-86.5` | `-8500` | `9000` | `-8650` |
| `arm_joint_2` / 팔 3축 | `1` | `1` | `-78.1` | `80` | `-78.1` | `-7810` | `8000` | `-7810` |
| `arm_joint_3` / 팔 4축 | `1` | `2` | `-91.5` | `90` | `-91.5` | `-9150` | `9000` | `-9150` |
| `arm_joint_4` / 팔 5축 | `2` | `0` | `-90` | `90` | `-90` | `-9000` | `9000` | `-9000` |

Board1 펌웨어는 팔 2~4축과 베이스 1축의 limit/home을 적용합니다.
Board1 local Motor ID 3의 리미트 스위치는 PB12(LIM4)입니다.
Arduino Board2 펌웨어는 팔 5축의 limit/home을 적용합니다.

## 3. Board1 CAN ID

| CAN ID | 방향 | 용도 |
|---:|---|---|
| `0x001` | RPi -> STM32 | Emergency Stop |
| `0x010` | RPi -> STM32 | Enable / Disable |
| `0x020` | RPi -> STM32 | Homing Start |
| `0x030` | RPi -> STM32 | Clear Error |
| `0x101` | RPi -> STM32 | Board1 motor trajectory point |
| `0x201` | STM32 -> RPi | Board1 status response |
| `0x301` | STM32 -> RPi | Board1 current position feedback |

Board1은 `0x101` 위치 명령에서 Motor ID `0~3`만 처리합니다. Board1 local Motor ID `0~2`는 실제 팔 `2~4축`, Motor ID `3`은 베이스축입니다.

MoveIt2 trajectory point 하나는 Motor ID `0 -> 1 -> 2 -> 3` 순서의 네 CAN frame으로 전송해야 합니다. STM32는 네 frame을 staging한 뒤 하나의 4축 point로 queue에 넣고, 4축 segment를 같은 1ms tick에서 동시에 시작합니다.

## 4. Board2 CAN ID

| CAN ID | 방향 | 용도 |
|---:|---|---|
| `0x001` | RPi -> STM32 | Emergency Stop |
| `0x010` | RPi -> STM32 | Enable / Disable |
| `0x020` | RPi -> STM32 | Homing Start |
| `0x030` | RPi -> STM32 | Clear Error |
| `0x102` | RPi -> Arduino | Arduino Board2 arm axis 5 trajectory point |
| `0x202` | STM32 -> RPi | Board2 status response |
| `0x302` | STM32 -> RPi | Board2 current position feedback |

Arduino Board2는 `0x102` 위치 명령에서 Motor ID `0`만 처리합니다.
Arduino Board2는 팔 5축 단일 축 보드로 사용합니다.

## 5. Board3 CAN ID

| CAN ID | 방향 | 용도 |
|---:|---|---|
| `0x001` | RPi -> STM32 | Emergency Stop |
| `0x010` | RPi -> STM32 | Enable / Disable |
| `0x030` | RPi -> STM32 | Clear Error |
| `0x103` | RPi -> STM32 | Board3 servo command |
| `0x203` | STM32 -> RPi | Board3 status response |
| `0x303` | STM32 -> RPi | Board3 current position feedback |

Board3는 `0x103` 명령에서 Motor ID `0~8`로 서보 1~9번을 구분합니다.

서버의 global servo id는 `5~13`이지만, CAN payload 안에서는 Board3 로컬 Motor ID `0~8`로 보냅니다.

Board3는 서보 9개 전체를 CAN frame 하나에 담지 않습니다. Classic CAN payload는 8바이트이므로, `0x103` frame 하나는 서보 하나의 명령만 담습니다.

9개 서보를 같은 trajectory point에서 동시에 움직여야 하면 Motor ID `0 -> 1 -> ... -> 8` 순서의 아홉 CAN frame을 전송하고, Board3 펌웨어에서 staging한 뒤 9개 서보를 같은 tick에서 동시에 시작합니다.

## 6. RPi -> STM32/Arduino Position Command, CAN ID `0x101` / `0x102`

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
| 6 | Relative | `1`: 마지막으로 queue commit에 성공한 trajectory endpoint 기준 상대 이동 |
| 5 | Step Mode | `0`: Target Pos는 0.01도 단위 angle, `1`: Target Pos는 step |
| 4 | Reserved | 현재 미사용, `0` |
| 3~0 | Motor ID | Board1은 `0~3`, Arduino Board2는 `0`만 유효 |

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

| Step Mode | Target Pos 의미 |
|---:|---|
| `0` | 0.01도 단위 angle. 예: `3000` = `30.00 deg` |
| `1` | step 수 |

angle mode에서 STM32는 아래 정수 공식으로 step 변환합니다.

```c
step = angle_raw * gear_ratio[axis] * motor_steps_per_rev[axis] * 16 / 36000;
```

Relative target은 명령 수신 순간의 물리적 current position이 아니라 축별 planned endpoint에 더합니다. Planned endpoint는 완성된 다축 point가 queue push에 성공한 뒤에만 갱신되며, staging 오류나 queue full로 commit에 실패하면 바뀌지 않습니다. Absolute point도 commit 성공 시 다음 Relative 명령의 planned 기준점을 갱신합니다. Queue clear, 치명적 stop, Homing 완료, raw jog 완료처럼 계획과 실제 위치를 다시 맞춰야 하는 경우에는 planned endpoint를 current position으로 동기화합니다.

현재 Board1 gear ratio:

| Board1 Motor ID | 실제 축 | Gear ratio | Motor full steps/rev |
|---:|---:|---:|---:|
| 0 | 2축 | 20 | 200 |
| 1 | 3축 | 50 | 200 |
| 2 | 4축 | 30 | 200 |
| 3 | 베이스 1축 | 20 | 200 |

Board1 local Motor ID 3은 베이스로 사용하므로, 팔 5축은 Arduino Board2에서 처리합니다.

### Speed (Byte 5~6)

`Speed`는 현재 STM32 초기 구현에서 queue에 저장하지만 1ms 보간 계산에는 사용하지 않습니다. 실제 이동 시간은 `Duration`을 기준으로 하되, 펌웨어의 step/s 안전 제한보다 빠른 명령이면 내부에서 더 긴 시간으로 늘려 실행합니다.

### Duration (Byte 7)

`Duration`은 5ms 단위입니다.

5ms, 10ms, 15ms, 20ms, ...

```text
total_move_ticks = Byte7 * 5
```

MoveIt2 trajectory point를 50ms 간격으로 보낼 경우:

```text
Byte7 = 10
```

`Byte7=0`이면 STM32 내부에서 최소 `1ms` segment로 처리합니다.

펌웨어는 `MOTION_MAX_STEP_RATE_SPS` 기본값 `1000 step/s`를 넘지 않도록 segment duration을 자동으로 늘립니다. 예를 들어 큰 각도 이동을 `Byte7=10`으로 보내면 요청 시간은 50ms이지만, 실제 microstep 수가 많으면 더 천천히 이동합니다.

Board1에서 같은 4축 point에 속한 Motor ID `0~3` frame은 모두 같은 Duration을 가져야 합니다. Duration이 하나라도 다르면 staging을 폐기하고 `ERR_INVALID_CMD`를 보고합니다.

Arduino Board2는 단일 축이므로 frame 하나의 Duration만 사용합니다.

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

베이스를 30.00도로 50ms 동안 이동하려면 Board1 point의 4번째 frame에 Motor ID 3 목표값을 넣습니다.

| Byte | 값 |
|---:|---:|
| 0 | `0x83` |
| 1~4 | `3000` as `int32_t little endian` |
| 5~6 | speed as `uint16_t little endian` |
| 7 | `10` |

이 frame의 CAN ID는 `0x101`입니다. 같은 trajectory point에서는 Motor ID `0`, `1`, `2` frame도 기존 팔 목표 위치로 함께 보내야 합니다.

## 7. Board3 Servo Command, CAN ID `0x103`

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

## 8. RPi -> STM32 Control Commands

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

### Arm Homing Broadcast, CAN ID `0x020`

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Target Motor | `0xFF`: arm 전체 homing |
| 1 | Homing Mode | 현재 `0`만 사용 |

조건:

- Board1 팔 2~4축과 Board1 베이스축이 동시에 처리하는 전체 스텝모터 homing broadcast입니다.
- payload에는 Target Board를 넣지 않습니다.
- 현재 최종 기준에서는 `Byte0 = 0xFF`만 사용합니다.
- Enable 상태가 아니거나 ESTOP 상태이면 `ERR_INVALID_CMD`로 처리합니다.
- mode가 `0`이 아니면 `ERR_INVALID_CMD`로 처리합니다.

Homing 완료 시 Board1/Board2는 각 local motor의 `homing_done`을 1로 설정하고 status의 축별 axis flags bit0(Position Valid / Homed)으로 보고합니다.

### Clear Error Broadcast, CAN ID `0x030`

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Target Motor | `0xFF`: 전체 error clear |

치명적 error는 기존처럼 모션/queue를 정리하고 error code를 `ERR_NONE`으로 초기화합니다. `ERR_QUEUE_FULL`은 비치명적 latch이므로 Clear Error를 받으면 clear 요청을 등록합니다. TIM3가 기존 queue의 완전 소진을 확인한 뒤 latch를 해제하고, status Byte1이 `ERR_NONE`으로 바뀐 status를 즉시 보내 ack합니다. ESTOP 자체는 Clear Error만으로 해제하지 않고, `0x010 Enable=1`에서 해제합니다.

## 9. STM32 -> RPi Status, CAN ID `0x201` / `0x202` / `0x203`

STM32는 100ms마다 status를 송신하고, ESTOP/Enable/Homing/Clear Error/Queue Full 같은 주요 이벤트에서도 즉시 한 번 송신합니다.

MCP2515의 세 TX buffer가 모두 busy이면 진행 중인 frame을 취소하지 않습니다. 최신 status 한 개를 pending으로 유지하고 main loop에서 비동기로 재시도하며, 재시도 전에 새 status가 생성되면 pending 내용을 최신 상태로 갱신합니다. Status Sequence는 MCP2515 TX buffer 적재에 성공했을 때만 증가합니다.

| Board ID | Status CAN ID |
|---:|---:|
| `1` | `0x201` |
| `2` | `0x202` |
| `3` | `0x203` |

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State | 현재 보드 상태 |
| 1 | Error Code | 현재 error code. `ERR_QUEUE_FULL`은 기존 모션을 중단하지 않는 비치명적 latch |
| 2 | Axis Flags 0/1 | Board1/Board2 compact status. low nibble = axis0 flags, high nibble = axis1 flags. Board2는 axis0만 사용하므로 high nibble은 `0` |
| 3 | Axis Flags 2/3 | Board1/Board2 compact status. low nibble = axis2 flags, high nibble = axis3 flags. Board2는 `0` |
| 4 | Limit Status Bits | Board1: bit0~3 = axis0~3 limit active, Board2: bit0 = axis0 limit active, Board3: servo fault bits 또는 reserved |
| 5 | Queue Free | trajectory queue 남은 슬롯 수 |
| 6 | Enabled | 공통 motor enable 상태. `0`: disabled, `1`: enabled |
| 7 | Status Sequence | Board1/Board2 compact status 송신 순서 counter. Board3: reserved `0` |

Board1/Board2 axis flags는 축당 4bit입니다.

| Bit | 이름 | 의미 |
|---:|---|---|
| 0 | Position Valid / Homed | `1`: homing 완료, current position 값을 actual position으로 사용 가능 |
| 1 | Ready | `1`: homed + enabled + 치명적 error 없음. `ERR_QUEUE_FULL` latch 중에도 기존 queue 실행을 위해 유지될 수 있음 |
| 2 | Moving | `1`: 이동 또는 homing 중 |
| 3 | Target Reached | `1`: 목표 위치 도달 |

### State Values

| 값 | 이름 |
|---:|---|
| 0 | `STATE_INIT` |
| 1 | `STATE_IDLE` |
| 2 | `STATE_HOMING` |
| 3 | `STATE_MOVING` |
| 4 | `STATE_ERROR` |
| 5 | `STATE_ESTOP` |
| 6 | `STATE_DISABLED` |

### Error Codes

| 값 | 이름 | 의미 |
|---:|---|---|
| 0 | `ERR_NONE` | 정상 |
| 1 | `ERR_INVALID_CMD` | 잘못된 명령, motor id, homing 전 move 등 |
| 2 | `ERR_LIMIT_SWITCH_DETECTED` | 예약됨 |
| 3 | `ERR_DRIVER_FAULT` | MCP2515 init 실패 등 driver fault |
| 4 | `ERR_HOMING_FAIL` | 예약됨 |
| 5 | `ERR_QUEUE_FULL` | 비치명적 queue overflow latch. 새 명령 drop, 기존 queue 계속 실행 |
| 6 | `ERR_RESERVED` | 예약됨 |

## 10. STM32 -> RPi Current Position Feedback, CAN ID `0x301` / `0x302` / `0x303`

MoveIt2 `/joint_states`의 actual position 입력을 위해 별도 current position feedback frame을 송신합니다.

Board1 펌웨어와 Arduino Board2 펌웨어는 100ms 주기마다 compact position frame 1개를 송신합니다.

TX buffer가 busy인 동안에는 최신 position frame 한 개만 pending으로 유지합니다. 오래된 position을 누적하지 않고 최신 측정값으로 덮어쓴 뒤 비동기로 재시도합니다.

| Board ID | Position Feedback CAN ID | 송신 frame |
|---:|---:|---|
| `1` | `0x301` | Board1 local motor `0~3` compact, 1 frame |
| `2` | `0x302` | Arduino Board2 arm axis 5 compact, 1 frame |
| `3` | `0x303` | Board3 local motor `0~8`, 9 frames |

아래 payload 표는 Board1 `0x301` / Arduino Board2 `0x302` compact format입니다. Board1 axis3 슬롯(byte6~7)은 베이스 현재각입니다. Arduino Board2는 axis0 슬롯(byte0~1)에 팔 5축 현재각을 담고 byte2~7은 `0`으로 예약합니다. Board3는 별도 구현 전까지 기존 per-axis position feedback format을 유지합니다.

DLC는 8입니다.

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Axis0 Current Pos LSB | `int16_t` 일부 | little endian, 단위 `0.01도` |
| 1 | Axis0 Current Pos MSB | `int16_t` 일부 | little endian |
| 2 | Axis1 Current Pos LSB | `int16_t` 일부 | little endian, 단위 `0.01도` |
| 3 | Axis1 Current Pos MSB | `int16_t` 일부 | little endian |
| 4 | Axis2 Current Pos LSB | `int16_t` 일부 | little endian, 단위 `0.01도` |
| 5 | Axis2 Current Pos MSB | `int16_t` 일부 | little endian |
| 6 | Axis3 Current Pos LSB | `int16_t` 일부 | little endian, 단위 `0.01도` |
| 7 | Axis3 Current Pos MSB | `int16_t` 일부 | little endian |

`current_pos_001deg`는 모터 step 값이 아니라 중앙서버 / MoveIt2 joint 기준 출력축 각도입니다. 단위는 command `target_pos`와 같은 0.01도입니다.

```text
30.00 deg  -> 3000
-15.50 deg -> -1550
```

Board1/Board2 STM32 내부 변환은 현재 step position을 출력축 각도로 역변환합니다.

```c
current_pos_001deg = current_step * 36000 / (gear_ratio[motor_id] * motor_steps_per_rev[motor_id] * 16);
```

Board1 예시:

```text
CAN ID = 0x301
axis0 = 3000  = B8 0B
axis1 = -1550 = F2 F9
axis2 = 0     = 00 00
axis3(base) = 9000  = 28 23
```

Arduino Board2 예시:

```text
CAN ID = 0x302
axis0(arm axis 5) = 3000 = B8 0B
axis1~3 reserved = 00 00 00 00 00 00
payload = B8 0B 00 00 00 00 00 00
```

## 11. Queue and Error Policy

Board1 STM32 trajectory queue의 `TRAJECTORY_QUEUE_SIZE`는 CAN move frame(`0x101`) 기준의 논리적 queue 예산이며, 시간(ms)이나 바이트 수가 아닙니다. Board1은 Motor ID `0 -> 1 -> 2 -> 3`의 4개 frame을 하나의 동기화된 trajectory point로 staging합니다.

현재 구현 기준으로는 다음과 같습니다.

- `TRAJECTORY_QUEUE_SIZE = 128`
- 내부 point 배열: `128 / 4 = 32`개 point 슬롯
- 원형 큐의 empty/full 구분을 위해 1개 슬롯을 비우므로 실제 대기 가능: 31개 point
- 외부 `0x101` command frame 기준 실제 대기 가능: `31 x 4 = 124`개 frame
- Status Byte5 `Queue Free`는 위 command frame 기준의 남은 슬롯 수를 보고하며, 빈 큐의 최대값은 124입니다.

현재 실행 중인 point는 queue에서 꺼내 실행하므로, queue에 대기 중인 point 수와 현재 실행 중인 point는 별도로 보아야 합니다.

Arduino Board2 trajectory queue는 단일 축 point queue로 구성합니다. 외부 status의 Queue Free는 `0x102` command slot 기준으로 보고합니다.

Queue full 상태에서 새 위치 명령이 오면:

- 새 명령은 저장하지 않습니다. 즉 Drop Tail입니다.
- 기존 queue 내용은 유지합니다.
- 별도 queue overflow latch를 설정하고 status Byte1에 `ERR_QUEUE_FULL`을 보고합니다.
- `STATE_ERROR`로 전환하지 않으며 현재 모션과 기존 queue는 계속 실행합니다.
- 해당 보드 status를 즉시 송신합니다.
- latch가 clear되기 전까지 추가 move 및 homing 명령은 무시합니다.
- `0x030 Clear Error`는 비동기 clear 요청으로 등록되며, TIM3가 기존 queue 소진을 확인한 뒤 latch를 해제합니다.
- 상위 제어기는 status Byte1이 `ERR_NONE`으로 바뀐 응답을 clear ack로 사용합니다.
- 중간 point가 유실된 trajectory를 이어 붙이지 않고, 상위 제어기는 최신 current position을 기준으로 새 trajectory를 생성해야 합니다.

### Queue full 트러블슈팅

`Queue Free=0` 또는 `ERR_QUEUE_FULL`이 발생하면 상위 제어기가 STM32의 소비 속도보다 빠르게 `0x101` frame을 전송하고 있는 상태입니다.

1. 새 move frame 전송을 중지합니다.
2. 기존 queue가 실행되어 `Queue Free`가 증가하는지 status(`0x201`)로 확인합니다.
3. `0x030 Clear Error`를 전송하고 `ERR_QUEUE_FULL` 해제를 확인합니다.
4. Queue full 시 drop된 frame이 있을 수 있으므로, 끊긴 trajectory를 그대로 이어 보내지 말고 최신 current position(`0x301`) 기준으로 trajectory를 다시 생성합니다.

`Queue Free`가 계속 증가하지 않으면 모터 enable, homing/limit 상태, 치명적 error, 그리고 각 point를 Motor ID `0 -> 1 -> 2 -> 3` 순서의 4개 frame으로 완성해서 보내고 있는지 확인합니다.

Board1 staging 규칙:

- Motor ID는 반드시 `0 -> 1 -> 2 -> 3` 순서여야 합니다.
- 첫 frame 수신 후 20ms 안에 네 frame이 모두 들어와야 합니다.
- 네 frame의 Duration은 모두 같아야 합니다.
- Execute=1, Reserved=0이어야 합니다. Relative와 Step Mode는 Byte0 정의에 따라 해석합니다.
- 위 조건을 만족하지 않으면 staging을 폐기하고 `ERR_INVALID_CMD`를 즉시 보고합니다.

Arduino Board2 staging 규칙:

- 한 point당 `0x102` frame 하나만 사용합니다.
- Motor ID는 `0`이어야 합니다.
- Execute=1, Reserved=0이어야 합니다. Relative와 Step Mode는 Byte0 정의에 따라 해석합니다.
- 위 조건을 만족하지 않으면 `ERR_INVALID_CMD`를 즉시 보고합니다.

Board3 command 규칙:

- Motor ID는 `0~8`이어야 합니다.
- 9개 서보 전체를 frame 하나에 담지 않고, 서보당 `0x103` frame 하나를 사용합니다.
- 동시 제어가 필요하면 Motor ID `0 -> 1 -> ... -> 8` 순서의 9개 frame을 staging합니다.
- 잘못된 Motor ID나 지원하지 않는 flags는 `ERR_INVALID_CMD`로 보고합니다.

상위 제어기 권장 동작:

1. `ERR_QUEUE_FULL` 감지 즉시 trajectory 송신 중단
2. STM32가 기존 queue를 계속 실행하는 동안 status 감시
3. `0x030 Clear Error`로 overflow clear 요청
4. `STATE_IDLE`이고 status Byte1이 `ERR_NONE`으로 바뀐 ack 확인
5. 최신 current position을 기준으로 trajectory를 다시 생성해 전송

`ERR_QUEUE_FULL` 이외의 치명적 오류는 `STATE_ERROR`로 처리하며 기존 오류 복구 절차를 사용합니다. CAN ID와 8-byte status/command 형식은 변경되지 않습니다.

## 12. STM32 Implementation Summary

현재 STM32 쪽에 구현된 내용:

- MCP2515 SPI2 기반 CAN 송수신
- CAN ID `0x001`, `0x010`, `0x020`, `0x030`, `0x101`, `0x201`, `0x301`
- `0x101` 위치 명령 4개를 staging 후 팔 2~4축 + 베이스 1축 trajectory point queue push
- Queue full 비치명적 latch, Drop Tail, 기존 queue 계속 실행 및 즉시 status 응답
- MCP2515 TX busy 시 기존 frame 보존, 최신 status/position coalescing 및 non-blocking 재시도
- TIM3 1ms trajectory 선형 보간
- TIM2 10us STEP/DIR pulse 생성
- Homing 및 limit switch debounce
- 100ms 주기 compact status 및 compact current position feedback 송신
- PC용 간단 프로토콜 테스트 `board1_virtual_can_test.c`

아직 초기 구현에서 제한적으로 처리하는 내용:

- `Speed`는 4축 point에 저장하지만 보간 계산에는 사용하지 않습니다.
- CAN 수신은 MCP2515 INT pin polling 방식입니다.
- 고급 속도 프로파일, acceleration/deceleration은 아직 구현하지 않았습니다.

Arduino Board2 팔 5축 펌웨어 핵심:

- 위치 명령 CAN ID는 `0x102`
- 상태 응답 CAN ID는 `0x202`
- 현재 위치 feedback CAN ID는 `0x302`
- 유효 Motor ID는 `0`만 허용
- 팔 5축 gear ratio는 `120`, motor full steps/rev는 `48`

Board3 펌웨어를 만들 때 정해야 할 핵심:

- 위치/서보 명령 CAN ID는 `0x103`
- 상태 응답 CAN ID는 `0x203`
- 서보 개수는 9개
- 유효 Motor ID는 `0~8`
- 9개 서보 전체 명령은 `0x103` frame 9개를 staging해서 처리
- 서보 enable, fault, ready 상태를 `0x203`에 어떻게 담을지 확정
