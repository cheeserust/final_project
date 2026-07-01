# Board2 HW Stepper Integration Patch — 2026-07-01

이 버전은 기존 Board2 CAN protocol checked 버전에 모터 제어팀 코드에서 해결 가능한 실제 스텝모터 제어 요소를 1축 Board2 구조에 맞춰 통합한 버전입니다.

## 추가/변경 사항

- `0x020` homing을 simulated homing에서 실제 limit switch 기반 homing으로 변경
- STEP/DIR GPIO pulse 출력 추가
- limit switch debounce 추가
- `0x202` status Byte4 limit bit를 실제 limit input에서 갱신
- `0x302` position feedback이 STEP pulse로 갱신된 `g_current_step` 기준으로 송신되도록 유지
- homing 후 limit switch에서 빠져나갈 수 있도록, limit switch가 눌려 있어도 home 반대 방향 이동은 허용
- 이동 중 home 방향으로 limit switch가 stable 감지되면 queue/motion clear 후 `ERR_LIMIT_SWITCH_DETECTED` 처리
- `0x001` ESTOP도 strict final payload 형식으로 검사하도록 변경

## 기본 핀 매핑

현재 기본값은 `main.c` 상단의 `Board2 real STEP/DIR + limit switch hardware pin map` 블록에 있습니다.
실제 하드웨어 배선이 다르면 이 블록만 수정하면 됩니다.

| 기능 | 핀 |
|---|---|
| STEP | PC0 |
| DIR | PC1 |
| LIMIT | PC2 |

MCP2515 핀은 기존 그대로입니다.

| 기능 | 핀 |
|---|---|
| MCP2515 SCK | PB13 |
| MCP2515 MISO | PB14 |
| MCP2515 MOSI | PB15 |
| MCP2515 CS | PB12 |
| MCP2515 INT | PB4 |

## Limit switch 기본 배선

기본 설정은 NO + GND + 내부 Pull-up입니다.

```text
GPIO 입력핀 ---- 리미트 스위치 COM
GND ---------- 리미트 스위치 NO
```

이 경우 평소 GPIO=1, 눌림 GPIO=0이므로:

```c
#define LIMIT_SWITCH_ACTIVE_LEVEL 0U
```

## 남아 있는 실제 하드웨어 보정 항목

- 실제 STEP/DIR/LIMIT 핀 매핑 확인
- 모터 방향이 반대면 DIR polarity 또는 `BOARD2_HOME_DIR` 보정
- `HW_MAX_STEP_PULSES_PER_MS` 속도 cap 보정
- 고속/정밀 trajectory가 필요하면 10us timer 기반 step scheduler로 세분화 권장
- driver enable/fault 핀은 아직 미연결
