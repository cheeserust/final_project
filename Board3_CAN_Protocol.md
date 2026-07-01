# Board3 CAN Protocol FINAL v1.1 — 0x303 20ms Position Feedback Added

## 0. 문서 목적

이 문서는 기존 **Board3 전용 CAN Protocol FINAL — No Payload Board ID Version**을 기준으로, MoveIt 기반 제어에 필요한 **모터 현재 위치 고속 피드백**을 추가한 Board3 최종 프로토콜입니다.

핵심 변경점은 다음과 같습니다.

```text
기존:
  0x203 Status frame만 100ms 주기로 송신

변경 후:
  0x203 Status frame은 그대로 100ms 주기 Heartbeat / 이벤트 상태 보고용으로 유지
  0x303 Position Feedback frame을 새로 추가하여 20ms 주기로 현재 위치 피드백 송신
```

Board3는 3-finger gripper의 SCS0009 서보 9개를 담당합니다.

```text
중앙서버 / Raspberry Pi / ROS 2 MoveIt
  ↓ CAN
Board3 STM32
  ↓ UART1 + TTL 변환보드
SCS0009 서보 9개
```

이 문서에서도 기존 최종 통합 기준을 유지합니다.

```text
서버/RPi 쪽에서는 내부 board_id로 대상 보드를 고르지만,
실제 CAN frame payload에는 별도 Board ID를 넣지 않는다.
보드 구분은 CAN ID로 한다.
```

---

## 1. 최종 변경 요약

| 구분 | 기존 Board3 프로토콜 | 변경 후 Board3 프로토콜 |
|---|---|---|
| Gripper 명령 | `0x103`, 9개 frame staging | 동일 |
| Gripper home posture | `0x023` | 동일 |
| Board status | `0x203`, 100ms 주기 | 동일, 위치 피드백 용도로 사용하지 않음 |
| 현재 위치 피드백 | 없음 | `0x303` 추가 |
| 위치 피드백 주기 | 없음 | 20ms |
| 위치 피드백 구성 | 없음 | 3개 CAN frame으로 9개 모터 위치 압축 전송 |
| 위치 데이터 단위 | 없음 | `int16_t`, 0.01도 단위 |
| 위치 데이터 엔디안 | 없음 | Little Endian |

중요한 설계 의도는 다음과 같습니다.

```text
0x203 = Board3가 살아있는지, enable인지, error인지, staging 상태가 어떤지 확인하는 저속 상태 프레임
0x303 = MoveIt / joint_states 갱신을 위한 고속 현재 위치 피드백 프레임
```

따라서 `0x203`의 100ms 주기를 20ms로 줄이는 방식이 아니라, **역할이 다른 CAN ID `0x303`을 새로 추가하는 방식**으로 구성합니다.

---

## 2. Board3 역할과 매핑

| 항목 | 값 |
|---|---|
| 담당 보드 | Board3 |
| 담당 장치 | 3-finger gripper |
| Servo 개수 | 9개 |
| Board3 local Motor ID | `0~8` |
| 실제 SCS0009 Servo ID | `1~9` |
| Gripper Command CAN ID | `0x103` |
| Gripper Home CAN ID | `0x023` |
| Status CAN ID | `0x203` |
| Position Feedback CAN ID | `0x303` |

---

## 3. Board3 CAN ID 최종 목록

| CAN ID | 방향 | 용도 | 주기 / 발생 조건 | payload Board ID 사용 여부 |
|---:|---|---|---|---|
| `0x001` | 중앙서버/RPi → 전체 보드 | Emergency Stop | 이벤트 | 없음 |
| `0x010` | 중앙서버/RPi → 전체 보드 | Enable / Disable broadcast | 이벤트 | 없음 |
| `0x020` | 중앙서버/RPi → Board1 + Board2 | Arm Homing broadcast | 이벤트, Board3는 무시 | 없음 |
| `0x023` | 중앙서버/RPi → Board3 | Gripper Home Posture | 이벤트 | 없음 |
| `0x030` | 중앙서버/RPi → 전체 보드 | Clear Error broadcast | 이벤트 | 없음 |
| `0x103` | 중앙서버/RPi → Board3 | Gripper Servo Command | 명령 발생 시 | 없음 |
| `0x203` | Board3 → 중앙서버/RPi | Board3 Status / Heartbeat | 100ms + 이벤트 | 없음 |
| `0x303` | Board3 → 중앙서버/RPi | Motor Current Position Feedback | 20ms | 없음 |

중요:

```text
Board3는 0x020 Arm Homing을 기본적으로 처리하지 않는다.
Board3 gripper home posture는 0x023을 사용한다.
Board3 위치 피드백은 0x203이 아니라 0x303을 사용한다.
```

---

## 4. 주기 설계

## 4.1 `0x203` Status 주기

`0x203`은 기존과 동일하게 **100ms 주기**로 송신합니다.

추가로 아래 이벤트가 발생하면 즉시 한 번 송신합니다.

```text
1. Enable / Disable 수신
2. ESTOP 수신
3. Clear Error 수신
4. 정상 command set 완성
5. command set 폐기 또는 error 발생
6. Gripper Home 0x023 수신
```

`0x203`은 위치 피드백용이 아닙니다. 8바이트 안에 9개 모터의 현재 위치를 모두 넣을 수 없고, MoveIt 제어 주기에도 100ms는 느리기 때문에 위치 피드백은 `0x303`으로 분리합니다.

## 4.2 `0x303` Position Feedback 주기

`0x303`은 **20ms 주기**로 송신합니다.

```text
1 feedback cycle = 20ms마다 1회
1 feedback cycle = 0x303 frame 3개
0x303 frame 3개 = Motor ID 0~8 현재 위치 전체
```

즉, 20ms마다 아래 3개 frame이 순서대로 전송됩니다.

```text
0x303 index 0x01 → Motor ID 0, 1, 2 현재 위치
0x303 index 0x02 → Motor ID 3, 4, 5 현재 위치
0x303 index 0x03 → Motor ID 6, 7, 8 현재 위치
```

CAN bus 관점에서는 다음과 같습니다.

```text
20ms마다 3 frame
1초당 50 feedback cycle
1초당 150개의 0x303 CAN frame
```

---

## 5. `0x103` Gripper Servo Command Frame

`0x103` frame 하나는 **서보 1개 명령**입니다. Board3는 서보 9개를 담당하므로, gripper 한 번의 동작은 `0x103` frame 9개로 구성됩니다.

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
CAN frame 9개 = gripper 전체 1회 동작 command set
```

---

## 6. `0x103` Payload 구조

```text
CAN ID = 0x103
DLC = 8
payload에 Board ID 없음
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | Bit7 Execute, Bit3~0 Motor ID |
| 1 | Target Position LSB | `int32_t` 일부 | 0.01도 단위, little endian |
| 2 | Target Position | `int32_t` 일부 | 0.01도 단위, little endian |
| 3 | Target Position | `int32_t` 일부 | 0.01도 단위, little endian |
| 4 | Target Position MSB | `int32_t` 일부 | 0.01도 단위, little endian |
| 5 | Speed LSB | `uint16_t` 일부 | 현재 v1.1 미사용, `0` 권장 |
| 6 | Speed MSB | `uint16_t` 일부 | 현재 v1.1 미사용, `0` 권장 |
| 7 | Duration | `uint8_t` | 5ms 단위 tick |

---

## 7. `0x103` Byte0 Control & Motor ID

```text
Bit7 Bit6 Bit5 Bit4 | Bit3 Bit2 Bit1 Bit0
Exec Rsv  Rsv  Rsv  | Motor ID
```

| Bit | 이름 | 의미 |
|---:|---|---|
| 7 | Execute | `1`: staging 대상으로 처리 |
| 6 | Reserved | 현재 미사용, 반드시 `0` |
| 5 | Reserved | 현재 미사용, 반드시 `0` |
| 4 | Reserved | 현재 미사용, 반드시 `0` |
| 3~0 | Motor ID | `0~8`만 유효 |

예상 Byte0 값:

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

수신부 해석 예시:

```c
uint8_t execute = (data[0] & 0x80U) ? 1U : 0U;
uint8_t reserved = data[0] & 0x70U;
uint8_t motor_id = data[0] & 0x0FU;
```

유효 조건:

```text
execute == 1
reserved == 0
motor_id <= 8
enabled == 1
state != STATE_ESTOP
```

---

## 8. Motor ID와 Servo ID / Joint 매핑

통신 프로토콜에서는 Motor ID `0~8`을 사용합니다. 실제 SCS0009 Servo ID는 `1~9`로 확정합니다.

| Motor ID | Joint 이름 | Servo ID | `0x303` feedback 위치 |
|---:|---|---:|---|
| 0 | `finger_1_base_joint` | 1 | index `0x01`, Byte1~2 |
| 1 | `finger_1_middle_joint` | 2 | index `0x01`, Byte3~4 |
| 2 | `finger_1_tip_joint` | 3 | index `0x01`, Byte5~6 |
| 3 | `finger_2_base_joint` | 4 | index `0x02`, Byte1~2 |
| 4 | `finger_2_middle_joint` | 5 | index `0x02`, Byte3~4 |
| 5 | `finger_2_tip_joint` | 6 | index `0x02`, Byte5~6 |
| 6 | `finger_3_base_joint` | 7 | index `0x03`, Byte1~2 |
| 7 | `finger_3_middle_joint` | 8 | index `0x03`, Byte3~4 |
| 8 | `finger_3_tip_joint` | 9 | index `0x03`, Byte5~6 |

코드 매핑:

```c
static const uint8_t motor_id_to_servo_id[9] = {
    1, 2, 3,
    4, 5, 6,
    7, 8, 9
};
```

---

## 9. Target Position 단위

`0x103`의 `Byte1~4`는 `int32_t` little endian입니다.

단위는 **0.01도**입니다.

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

SCS0009의 실제 position 값으로 변환하는 작업은 servo adapter에서 수행합니다.

---

## 10. Duration 단위와 처리 방식

`0x103`의 `Byte7`은 duration입니다.

Board3 CAN 프로토콜에서 duration은 **5ms 단위 원본 tick**입니다.

```text
duration_ms = duration_5ms × 5
```

예:

| CAN Byte7 | 실제 시간 |
|---:|---:|
| 20 | 100ms |
| 40 | 200ms |
| 100 | 500ms |

`g_cmd`에는 CAN에서 받은 원본 tick 값을 그대로 저장합니다.

```text
g_cmd.duration_5ms = CAN Byte7 원본값
```

제어팀은 `Feetech_Set_Position_Time()`을 호출하기 직전에 ms 단위로 변환합니다.

```c
uint16_t duration_ms = (uint16_t)g_cmd.duration_5ms * 5U;
```

---

## 11. 9개 Frame Staging 구조

Board3는 `0x103` frame을 받자마자 바로 제어팀에 넘기지 않습니다. 먼저 Motor ID별 staging buffer에 저장합니다.

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
GRIPPER_STAGING_TIMEOUT_FINAL_MS = 20ms
```

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

# 14. `0x303` Motor Current Position Feedback

## 14.1 목적

`0x303`은 MoveIt 기반 제어에서 `/joint_states`를 빠르게 갱신하기 위한 **현재 위치 피드백 전용 CAN ID**입니다.

`0x203` status frame은 100ms 주기로 유지하고, 실제 관절 위치는 `0x303`으로 20ms마다 별도 송신합니다.

```text
0x203: 상태 / 에러 / ready / staging / fault / enabled
0x303: Motor ID 0~8 현재 위치
```

## 14.2 송신 주기

```text
CAN ID = 0x303
DLC = 8
방향 = Board3 → 중앙서버/RPi
송신 주기 = 20ms
1회 송신 cycle = 3 frame
```

송신 순서는 반드시 아래 순서를 권장합니다.

```text
1. index 0x01 frame 송신
2. index 0x02 frame 송신
3. index 0x03 frame 송신
```

서버는 `index 0x03`까지 수신하면 Motor ID 0~8의 위치가 모두 갱신되었다고 보고 `/joint_states` publish를 수행할 수 있습니다.

## 14.3 위치 데이터 단위

`0x303`의 위치 데이터는 `int16_t`입니다.

```text
단위 = 0.01도
자료형 = int16_t
엔디안 = Little Endian
표현 범위 = -327.68° ~ +327.67°
```

예:

| 실제 각도 | 정수값 | Hex |
|---:|---:|---:|
| `30.00°` | `3000` | `0x0BB8` |
| `-15.50°` | `-1550` | `0xF9F2` |
| `0.00°` | `0` | `0x0000` |

Little Endian이므로 `30.00° = 3000 = 0x0BB8`은 CAN payload에서 아래처럼 들어갑니다.

```text
LSB = 0xB8
MSB = 0x0B
```

## 14.4 `0x303` Payload 구조

```text
CAN ID = 0x303
DLC = 8
payload에 Board ID 없음
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Feedback Frame Index | `uint8_t` | `0x01`, `0x02`, `0x03` |
| 1 | Position A LSB | `int16_t` 일부 | 0.01도 단위, little endian |
| 2 | Position A MSB | `int16_t` 일부 | 0.01도 단위, little endian |
| 3 | Position B LSB | `int16_t` 일부 | 0.01도 단위, little endian |
| 4 | Position B MSB | `int16_t` 일부 | 0.01도 단위, little endian |
| 5 | Position C LSB | `int16_t` 일부 | 0.01도 단위, little endian |
| 6 | Position C MSB | `int16_t` 일부 | 0.01도 단위, little endian |
| 7 | Reserved / Group Flag | `uint8_t` | v1.1에서는 `0x00` 기본, 향후 flag 확장 가능 |

## 14.5 `0x303` Frame Index별 모터 매핑

| Byte0 Index | 포함 Motor ID | Byte1~2 | Byte3~4 | Byte5~6 | Byte7 |
|---:|---|---|---|---|---|
| `0x01` | Motor ID `0,1,2` | Motor 0 현재 각도 | Motor 1 현재 각도 | Motor 2 현재 각도 | Reserved / Group Flag |
| `0x02` | Motor ID `3,4,5` | Motor 3 현재 각도 | Motor 4 현재 각도 | Motor 5 현재 각도 | Reserved / Group Flag |
| `0x03` | Motor ID `6,7,8` | Motor 6 현재 각도 | Motor 7 현재 각도 | Motor 8 현재 각도 | Reserved / Group Flag |

주의:

```text
기존 서버 참고 문서에서는 모터 1~3, 4~6, 7~9라고 표현되어 있지만,
Board3 내부 프로토콜의 local Motor ID 기준으로는 0~2, 3~5, 6~8이다.
실제 Servo ID로는 1~3, 4~6, 7~9에 대응된다.
```

## 14.6 Byte7 Reserved / Group Flag 정책

현재 v1.1 최종 구현에서는 개발 부담을 줄이기 위해 `Byte7 = 0x00`을 기본값으로 사용합니다.

```text
Byte7 = 0x00
```

단, 서버 참고안처럼 향후 group 단위 flag가 필요하면 아래 비트맵으로 확장할 수 있습니다.

| Bit | 의미 | 현재 v1.1 적용 |
|---:|---|---|
| bit0~1 | 그룹 내 1번째 motor 상태 | Reserved |
| bit2~3 | 그룹 내 2번째 motor 상태 | Reserved |
| bit4~5 | 그룹 내 3번째 motor 상태 | Reserved |
| bit6 | Frame data valid | Reserved |
| bit7 | Group fault | Reserved |

v1.1 서버 파서는 Byte7이 `0x00`이어도 정상 위치 데이터로 처리해야 합니다.

---

## 15. `0x303` 송신 데이터 출처

`0x303`은 목표 위치가 아니라 **현재 위치**를 보냅니다.

Board3 내부 공유 구조체 기준:

```c
g_state.current_pos_001deg[0~8]
```

의 값이 `0x303`으로 송신됩니다.

역할 분리는 다음과 같습니다.

```text
[그리퍼 제어 코드]
1. SCS0009 현재 위치를 읽음
2. raw servo position 또는 내부 step 값을 실제 joint angle로 변환
3. degree × 100 형태의 int16_t 값으로 변환
4. g_state.current_pos_001deg[0~8] 갱신

[CAN 송신 코드]
1. 20ms 주기 타이머 또는 task에서 g_state.current_pos_001deg[0~8] snapshot
2. 3개 frame으로 압축 packing
3. CAN ID 0x303으로 순서대로 송신
```

중요:

```text
g_cmd.target_pos_001deg[] = 중앙서버가 보낸 목표 각도

g_state.current_pos_001deg[] = 실제 서보 피드백으로부터 계산한 현재 각도
```

따라서 MoveIt의 현재 관절 상태에는 `g_state.current_pos_001deg[]` 기반 값이 들어가야 합니다.

하드웨어 연결 전 CAN 통신 테스트 단계에서는 `current_pos_001deg[]`를 전부 0으로 두거나, 수신한 target을 임시로 mirror하여 송신할 수 있습니다. 다만 최종 동작에서는 반드시 실제 servo feedback 기반 현재 위치로 갱신해야 합니다.

---

## 16. `0x303` Packing 예시 코드

```c
#define CAN_ID_BOARD3_POSITION              0x303U
#define GRIPPER_POSITION_FEEDBACK_PERIOD_MS 20U

static inline void pack_i16_le(uint8_t *data, uint8_t offset, int16_t value)
{
    data[offset]     = (uint8_t)(value & 0xFF);
    data[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

void Send_Position_Feedback_0x303(void)
{
    int16_t pos[GRIPPER_MOTOR_COUNT];

    /* 권장: interrupt/callback과 main loop가 동시에 g_state를 접근할 수 있으므로
       짧은 critical section에서 snapshot을 뜬다. */
    for (uint8_t i = 0; i < GRIPPER_MOTOR_COUNT; i++) {
        pos[i] = g_state.current_pos_001deg[i];
    }

    uint8_t txData[8];

    /* Frame 1: Motor ID 0,1,2 */
    txData[0] = 0x01;
    pack_i16_le(txData, 1, pos[0]);
    pack_i16_le(txData, 3, pos[1]);
    pack_i16_le(txData, 5, pos[2]);
    txData[7] = 0x00;
    CAN_Send_StdId(CAN_ID_BOARD3_POSITION, txData, 8);

    /* Frame 2: Motor ID 3,4,5 */
    txData[0] = 0x02;
    pack_i16_le(txData, 1, pos[3]);
    pack_i16_le(txData, 3, pos[4]);
    pack_i16_le(txData, 5, pos[5]);
    txData[7] = 0x00;
    CAN_Send_StdId(CAN_ID_BOARD3_POSITION, txData, 8);

    /* Frame 3: Motor ID 6,7,8 */
    txData[0] = 0x03;
    pack_i16_le(txData, 1, pos[6]);
    pack_i16_le(txData, 3, pos[7]);
    pack_i16_le(txData, 5, pos[8]);
    txData[7] = 0x00;
    CAN_Send_StdId(CAN_ID_BOARD3_POSITION, txData, 8);
}
```

권장 구현 방식:

```text
타이머 interrupt 안에서 CAN 송신을 직접 길게 수행하지 말고,
20ms flag만 세운 뒤 main loop 또는 CAN TX task에서 3 frame을 송신하는 방식 권장.
```

이유:

```text
MCP2515 SPI 송신, CAN mailbox 대기, retry 등이 interrupt 안에서 길어지면
다른 제어 루프나 수신 interrupt에 영향을 줄 수 있기 때문이다.
```

---

## 17. 서버 / ROS 2 파싱 방식

서버는 `0x303`을 수신하면 Byte0 index를 기준으로 Motor ID 그룹을 판별합니다.

```cpp
if (can_id == 0x303) {
    uint8_t index = data[0];

    int16_t p0 = (int16_t)((data[2] << 8) | data[1]);
    int16_t p1 = (int16_t)((data[4] << 8) | data[3]);
    int16_t p2 = (int16_t)((data[6] << 8) | data[5]);

    switch (index) {
        case 0x01:
            pos_001deg[0] = p0;
            pos_001deg[1] = p1;
            pos_001deg[2] = p2;
            got_group_1 = true;
            break;

        case 0x02:
            pos_001deg[3] = p0;
            pos_001deg[4] = p1;
            pos_001deg[5] = p2;
            got_group_2 = true;
            break;

        case 0x03:
            pos_001deg[6] = p0;
            pos_001deg[7] = p1;
            pos_001deg[8] = p2;
            got_group_3 = true;
            break;

        default:
            return;
    }

    if (got_group_1 && got_group_2 && got_group_3) {
        for (int i = 0; i < 9; i++) {
            double deg = pos_001deg[i] / 100.0;
            /* MoveIt / JointState 사용 단위에 맞게 변환한다.
               일반적인 revolute joint라면 radian으로 변환하여 publish한다. */
            joint_state.position[i] = deg * M_PI / 180.0;
        }

        joint_state_pub->publish(joint_state);
        got_group_1 = got_group_2 = got_group_3 = false;
    }
}
```

서버 구현 권장 사항:

```text
1. 0x203 status parser와 0x303 position parser를 분리한다.
2. 0x303 frame index가 0x01, 0x02, 0x03이 아닌 경우 무시한다.
3. 3개 group이 모두 들어왔을 때만 joint_states를 갱신한다.
4. 0x303 수신 timeout이 발생하면 joint state stale로 표시한다.
5. ROS JointState position 단위는 MoveIt/URDF 설정에 맞춰 최종 변환한다.
```

---

## 18. `0x303` CAN frame 예시

## 18.1 모든 모터 현재 위치가 0.00도인 경우

```bash
candump 예시:
can0  303   [8]  01 00 00 00 00 00 00 00
can0  303   [8]  02 00 00 00 00 00 00 00
can0  303   [8]  03 00 00 00 00 00 00 00
```

`cansend` 예시:

```bash
cansend can0 303#0100000000000000
cansend can0 303#0200000000000000
cansend can0 303#0300000000000000
```

## 18.2 Motor 0 = 30.00도, Motor 1 = -15.50도, Motor 2 = 0.00도인 경우

```text
Motor 0 = 30.00도  → 3000  → 0x0BB8 → B8 0B
Motor 1 = -15.50도 → -1550 → 0xF9F2 → F2 F9
Motor 2 = 0.00도   → 0     → 0x0000 → 00 00
```

Frame 1:

```bash
cansend can0 303#01B80BF2F9000000
```

candump:

```text
can0  303   [8]  01 B8 0B F2 F9 00 00 00
```

---

## 19. `0x203` Board3 Status

```text
CAN ID = 0x203
DLC = 8
방향 = Board3 → 중앙서버/RPi
주기 = 100ms
```

## 19.1 송신 조건

Board3는 아래 상황에서 status를 송신합니다.

```text
1. 100ms마다 주기 송신
2. Enable / Disable 수신 시 즉시 송신
3. ESTOP 수신 시 즉시 송신
4. Clear Error 수신 시 즉시 송신
5. 정상 command set 완성 시 즉시 송신
6. command set 폐기 또는 error 발생 시 즉시 송신
7. Gripper Home 0x023 수신 시 즉시 송신
```

## 19.2 Payload 구조

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

중요:

```text
0x203에는 9개 모터의 현재 위치값을 넣지 않는다.
현재 위치는 반드시 0x303으로 송신한다.
```

---

## 20. State 값

| 값 | 이름 | 설명 |
|---:|---|---|
| 0 | `STATE_INIT` | 초기화 중 |
| 1 | `STATE_IDLE` | 대기 상태 |
| 2 | `STATE_STAGING` | 9개 frame 수집 중 |
| 3 | `STATE_MOVING` | 명령 처리 또는 이동 중 |
| 4 | `STATE_ERROR` | 에러 발생 |
| 5 | `STATE_ESTOP` | 비상정지 상태 |
| 6 | `STATE_DISABLED` | Disable 상태 |

---

## 21. Error Code 값

| 값 | 이름 | 설명 |
|---:|---|---|
| 0 | `ERR_NONE` | 정상 |
| 1 | `ERR_INVALID_CMD` | 잘못된 명령 |
| 2 | `ERR_INVALID_MOTOR_ID` | Motor ID 범위 오류 |
| 3 | `ERR_DUPLICATE_MOTOR_ID` | 같은 command set 안에서 Motor ID 중복 |
| 4 | `ERR_STAGING_TIMEOUT` | 9개 frame 수집 timeout |
| 5 | `ERR_DURATION_MISMATCH` | 9개 frame의 duration 불일치 |
| 6 | `ERR_ANGLE_RANGE` | 목표 각도 한계 초과, 하드웨어 연결 후 적용 |
| 7 | `ERR_SERVO_COMM` | SCS0009 통신 오류, 하드웨어 연결 후 적용 |
| 8 | `ERR_SERVO_FAULT` | 과부하 또는 servo fault, 하드웨어 연결 후 적용 |
| 9 | `ERR_ESTOP` | ESTOP 상태 |
| 10 | `ERR_DISABLED` | Disable 상태에서 command 수신 |

---

## 22. `0x023` Gripper Home Posture

Board3 gripper home posture는 `0x023`을 사용합니다.

```text
CAN ID = 0x023
DLC = 8
payload에 Board ID 없음
```

Board1/Board2의 `0x020 Arm Homing`과 구분하기 위해 별도 CAN ID를 사용합니다.

## 22.1 Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Motor | `uint8_t` | `0xFF`: 전체 gripper home posture |
| 1 | Home Mode | `uint8_t` | 현재 `0`만 사용 |
| 2 | Duration | `uint8_t` | 5ms 단위. `0`이면 기본값 100 사용 |
| 3~7 | Reserved | - | `0` |

현재 최종 통합 기준에서는 `Byte0 = 0xFF`, `Byte1 = 0`을 기본으로 사용합니다.

## 22.2 기본 home posture

Board3 home posture는 모든 Motor ID의 목표 각도를 **0.00도**로 설정하는 동작입니다.

```text
home_position_001deg = 0
home_position_deg = 0.00°
```

정상 `0x023` 수신 시 Board3는 내부적으로 아래와 동일한 `g_cmd`를 생성합니다.

```text
g_cmd.target_pos_001deg[0~8] = 0
g_cmd.duration_5ms = 100 또는 Byte2 값
g_cmd.is_new_cmd = 1
```

기본 home duration:

```text
GRIPPER_HOME_DURATION_5MS = 100
home_duration_ms = 100 × 5ms = 500ms
```

예시:

```bash
# Board3 gripper 전체 home posture, 기본 duration 500ms
cansend can0 023#FF00000000000000

# Board3 gripper 전체 home posture, duration 100 x 5ms = 500ms 명시
cansend can0 023#FF00640000000000
```

주의:

```text
Board3의 home posture는 limit switch를 찾는 물리적 원점 탐색이 아니다.
Board3는 0x020 Arm Homing을 기본적으로 무시한다.
```

---

## 23. 공통 제어 명령

## 23.1 Emergency Stop, CAN ID `0x001`

Emergency Stop은 전체 보드 broadcast입니다.

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | ESTOP | `1` |
| 1~7 | Reserved | `0` |

예시:

```bash
cansend can0 001#0100000000000000
```

Board3 처리:

```text
1. staging buffer clear
2. g_cmd.is_new_cmd = 0
3. state = STATE_ESTOP
4. error_code = ERR_ESTOP
5. enabled = 0
6. 0x203 status 즉시 송신
7. 0x303 위치 피드백은 구현 정책에 따라 계속 송신하거나 중단 가능
```

권장:

```text
ESTOP 상태에서도 서버가 마지막 위치를 stale 처리할 수 있도록 0x303은 계속 20ms 송신하되,
servo feedback이 불가능한 fault 상태라면 0x203의 fault/error로 상태를 명확히 알린다.
```

## 23.2 Enable / Disable, CAN ID `0x010`

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

Board3는 `0x010`을 수신하면 항상 처리합니다.

Enable:

```text
enabled = 1
error_code = ERR_NONE
state = STATE_IDLE
staging buffer clear
0x203 status 즉시 송신
```

Disable:

```text
enabled = 0
g_cmd.is_new_cmd = 0
staging buffer clear
state = STATE_DISABLED
0x203 status 즉시 송신
```

`0x303` 정책:

```text
Disable 상태에서도 현재 위치 모니터링이 필요하면 0x303은 계속 송신 가능하다.
단, servo torque off 등으로 실제 위치 읽기가 불가능하면 0x203 fault/error 또는 stale 처리 정책을 서버와 맞춘다.
```

## 23.3 Arm Homing Broadcast, CAN ID `0x020`

`0x020`은 Board1/Board2 로봇팔 전체 homing 명령입니다.

Board3 기본 정책:

```text
Board3는 0x020을 무시한다.
```

즉 아래 명령은 Board1/Board2만 처리하고, Board3는 처리하지 않습니다.

```bash
cansend can0 020#FF00000000000000
```

Board3 home posture가 필요하면 `0x023`을 사용합니다.

## 23.4 Clear Error Broadcast, CAN ID `0x030`

`0x030`은 전체 보드 Clear Error broadcast입니다.

```text
payload에 Target Board를 넣지 않는다.
Board1, Board2, Board3가 동시에 error clear를 수행한다.
```

### Payload 구조

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Target Motor | `uint8_t` | `0xFF`: 전체 error clear |
| 1~7 | Reserved | - | `0` |

현재 최종 통합 기준에서는 `Byte0 = 0xFF`만 사용합니다.

Board3 처리 조건:

```text
CAN ID == 0x030
Target Motor == 0xFF
```

Clear Error 수신 시:

```text
error_code = ERR_NONE
fault = 0
fault_motor_id = 255
staging buffer clear

if enabled == 1:
    state = STATE_IDLE
else:
    state = STATE_DISABLED

0x203 status 즉시 송신
```

ESTOP 해제는 Clear Error가 아니라 Enable에서 처리합니다.

예시:

```bash
# 전체 보드 error clear
cansend can0 030#FF00000000000000
```

---

## 24. 헤더 파일 추가 / 수정 사항

첨부된 `gripper_shared_HW_X.h` 기준으로 이미 반영된 항목과 추가로 정리해야 할 항목은 다음과 같습니다.

## 24.1 이미 추가된 항목

```c
#define CAN_ID_BOARD3_POSITION 0x303U
```

`GripperState` 내부:

```c
int16_t current_pos_001deg[GRIPPER_MOTOR_COUNT];
```

이 배열은 `0x303` 송신부가 직접 읽는 현재 위치 배열입니다.

## 24.2 추가 권장 항목

현재 요구사항이 20ms이므로 주기 상수를 명시적으로 추가하는 것을 권장합니다.

```c
#define GRIPPER_POSITION_FEEDBACK_PERIOD_MS 20U
```

그리고 헤더 주석 중 `current_pos_001deg` 설명에 `10ms마다 송신`으로 되어 있는 부분은 아래처럼 수정해야 합니다.

```c
/*
 * CAN 통신 코드가 이 배열을 읽어 0x303 3프레임으로 압축하여 20ms마다 송신한다.
 */
int16_t current_pos_001deg[GRIPPER_MOTOR_COUNT];
```

## 24.3 Legacy 주석 정리 권장

첨부된 헤더 파일 일부 주석에는 과거 방식인 `Target Board ID`, `Byte0 == 3 또는 255` 설명이 남아 있습니다.

현재 최종 프로토콜은 다음 기준입니다.

```text
실제 CAN payload에는 Board ID를 넣지 않는다.
보드 구분은 CAN ID로 한다.
0x010 Enable/Disable은 Byte0 = Enable 값만 사용한다.
0x030 Clear Error는 Byte0 = 0xFF 전체 motor clear로 사용한다.
```

따라서 헤더의 레거시 주석은 코드 동작과 혼동되지 않도록 최종 프로토콜 기준으로 정리하는 것을 권장합니다.

---

## 25. 개발 구현 사항

## 25.1 CAN 수신부

기존 기능 유지:

```text
0x001 ESTOP 처리
0x010 Enable / Disable 처리
0x020 Board3에서는 무시
0x023 Gripper Home 처리
0x030 Clear Error 처리
0x103 Gripper command 9-frame staging 처리
```

수신부에서 `0x303`은 처리하지 않습니다. `0x303`은 Board3가 송신하는 feedback frame입니다.

## 25.2 Gripper 제어부

추가 구현 필요:

```text
1. 각 SCS0009 servo의 현재 위치를 읽는다.
2. raw servo position을 실제 joint angle로 변환한다.
3. degree × 100 값을 int16_t로 변환한다.
4. g_state.current_pos_001deg[motor_id]에 저장한다.
```

예시:

```c
float current_deg = ServoRawToJointDeg(motor_id, raw_pos);
int32_t scaled = (int32_t)(current_deg * 100.0f);

if (scaled > 32767) scaled = 32767;
if (scaled < -32768) scaled = -32768;

g_state.current_pos_001deg[motor_id] = (int16_t)scaled;
```

## 25.3 CAN 송신부

추가 구현 필요:

```text
1. 20ms 주기 타이머 또는 task 생성
2. 20ms마다 g_state.current_pos_001deg[0~8] snapshot
3. 0x303 frame 3개 packing
4. index 0x01, 0x02, 0x03 순서로 송신
```

권장:

```text
status 송신 timer와 position feedback timer를 분리한다.

last_status_ms 기준:
  100ms마다 Send_Status_0x203()

last_position_ms 기준:
  20ms마다 Send_Position_Feedback_0x303()
```

## 25.4 서버 / 중앙서버

추가 구현 필요:

```text
1. CAN ID 0x303 parser 추가
2. Byte0 index 기준으로 3개 group 조립
3. int16 little endian을 0.01도 단위 각도로 복원
4. Motor ID 0~8 순서대로 joint state 배열 갱신
5. 3개 group이 모두 모이면 /joint_states publish
6. MoveIt/URDF 단위에 맞춰 degree → radian 변환 여부 적용
7. 일정 시간 0x303이 들어오지 않으면 position feedback stale 처리
```

---

## 26. 하드웨어 연결 전 테스트 기준

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
8. 0x203 status가 100ms 주기 또는 이벤트마다 송신된다.
9. 0x303 position feedback이 20ms마다 3개 frame씩 송신된다.
10. 0x303 index가 0x01 → 0x02 → 0x03 순서로 반복된다.
11. 0x303 payload의 각 위치값이 int16 little endian으로 정상 파싱된다.
12. 0x023 Gripper Home 수신 시 g_cmd.target_pos_001deg[0~8]이 모두 0으로 설정된다.
13. 0x020 Arm Homing은 Board3에서 무시된다.
14. 0x010 Enable / Disable은 Board3도 전체 broadcast로 처리한다.
15. 0x030 Clear Error는 Board3도 전체 broadcast로 처리한다.
```

candump 예상 형태:

```text
can0  203   [8]  ...              # 약 100ms마다
can0  303   [8]  01 xx xx xx xx xx xx 00
can0  303   [8]  02 xx xx xx xx xx xx 00
can0  303   [8]  03 xx xx xx xx xx xx 00
```

20ms 주기 검증:

```text
0x303 index 0x01 frame 간 시간 차이 ≈ 20ms
0x303 index 0x02 frame 간 시간 차이 ≈ 20ms
0x303 index 0x03 frame 간 시간 차이 ≈ 20ms
```

---

## 27. 하드웨어 연결 후 보정할 항목

아래 항목은 실제 그리퍼/서보 연결 후 보정합니다.

```text
- 각 Motor ID별 direction
- 각 Motor ID별 step_per_deg
- 각 Motor ID별 min_goal / max_goal
- home posture 0.00도가 실제 SCS0009 position에서 안전한 자세인지 확인
- 목표 각도 범위 검사
- 실제 부하 threshold
- 실제 fault 조건
- SCS0009 현재 위치 read 주기
- 9개 servo 현재 위치를 20ms 주기로 모두 읽을 수 있는지 확인
- 20ms마다 실제 read가 불가능할 경우, servo read 주기와 CAN feedback 주기를 분리할지 결정
- Sync Read 또는 순차 Read 방식 결정
- 0x303 송신이 0x103 command 수신 / 0x203 status 송신과 충돌하지 않는지 확인
```

중요:

```text
20ms마다 0x303을 송신하는 것과,
20ms마다 실제 servo 9개의 raw position을 모두 새로 읽는 것은 다른 문제이다.

최종적으로는 servo 통신 속도가 충분한지 확인해야 한다.
읽기 속도가 부족하면, servo read는 가능한 최고 주기로 수행하고,
0x303은 가장 최신 g_state.current_pos_001deg[] snapshot을 20ms마다 송신하는 구조로 분리한다.
```

---

## 28. C Constant Example

```c
#define CAN_ID_ESTOP                       0x001U
#define CAN_ID_ENABLE                      0x010U
#define CAN_ID_ARM_HOMING                  0x020U
#define CAN_ID_GRIPPER_HOME                0x023U
#define CAN_ID_CLEAR_ERROR                 0x030U

#define CAN_ID_BOARD3_SERVO_CMD            0x103U
#define CAN_ID_BOARD3_STAT                 0x203U
#define CAN_ID_BOARD3_POSITION             0x303U

#define BOARD3_SERVO_COUNT                 9U
#define MOTOR_ALL                          0xFFU

#define GRIPPER_STATUS_PERIOD_MS           100U
#define GRIPPER_POSITION_FEEDBACK_PERIOD_MS 20U

#define STATE_INIT                         0U
#define STATE_IDLE                         1U
#define STATE_STAGING                      2U
#define STATE_MOVING                       3U
#define STATE_ERROR                        4U
#define STATE_ESTOP                        5U
#define STATE_DISABLED                     6U

#define ERR_NONE                           0U
#define ERR_INVALID_CMD                    1U
#define ERR_INVALID_MOTOR_ID               2U
#define ERR_DUPLICATE_MOTOR_ID             3U
#define ERR_STAGING_TIMEOUT                4U
#define ERR_DURATION_MISMATCH              5U
#define ERR_ANGLE_RANGE                    6U
#define ERR_SERVO_COMM                     7U
#define ERR_SERVO_FAULT                    8U
#define ERR_ESTOP                          9U
#define ERR_DISABLED                       10U
```

---

## 29. Board3 Test Command Summary

```bash
# 전체 Enable
cansend can0 010#0100000000000000

# Board3 gripper 전체 home posture
cansend can0 023#FF00000000000000

# 전체 보드 error clear
cansend can0 030#FF00000000000000

# 전체 ESTOP
cansend can0 001#0100000000000000

# Status / Position feedback 확인
candump can0
```

`0x103` 9개 frame 예시 형식:

```bash
# Motor ID 0, 0.00deg, speed 0, duration 100ms
cansend can0 103#8000000000000014

# Motor ID 1, 0.00deg, speed 0, duration 100ms
cansend can0 103#8100000000000014

# ... Motor ID 8까지 같은 duration으로 전송
```

`0x303` position feedback 수동 테스트 예시:

```bash
# Motor ID 0~8 현재 위치 0.00deg로 가정
cansend can0 303#0100000000000000
cansend can0 303#0200000000000000
cansend can0 303#0300000000000000
```

---

## 30. 최종 요약

```text
1. Board3는 gripper 서보 9개를 담당한다.
2. Board3 명령 ID는 0x103, status ID는 0x203, position feedback ID는 0x303이다.
3. 0x103 payload에는 Board ID를 넣지 않고 local Motor ID 0~8만 넣는다.
4. 0x103은 9개 frame staging 후 하나의 gripper command set으로 처리한다.
5. Board3 gripper home posture는 0x023을 사용한다.
6. Board3는 0x020 Arm Homing을 기본적으로 무시한다.
7. Enable/Disable 0x010은 전체 보드 broadcast이며 payload Board ID가 없다.
8. Clear Error 0x030은 전체 보드 broadcast이며 payload Board ID가 없다.
9. 0x203은 100ms 주기의 상태/heartbeat frame이다.
10. 0x203에는 9개 모터 위치값을 넣지 않는다.
11. 0x303은 20ms 주기의 현재 위치 feedback frame이다.
12. 0x303은 3개 frame으로 Motor ID 0~8의 현재 위치를 압축 송신한다.
13. 0x303 위치값은 int16_t, 0.01도 단위, little endian이다.
14. 서버는 0x303 index 0x01, 0x02, 0x03을 모아 /joint_states를 갱신한다.
15. MoveIt에서 사용할 때는 서버가 URDF/JointState 단위에 맞게 degree 값을 변환한다.
16. 기존 Target Board 방식인 010#01FF..., 020#03FF..., 030#03FF... 는 사용하지 않는다.
17. 서버/RPi 내부 board_id는 CAN ID 선택용이고, 실제 CAN payload에는 Board ID를 넣지 않는다.
```
