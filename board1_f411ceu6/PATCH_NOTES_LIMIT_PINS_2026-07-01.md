# Board1 Limit Pin Patch - 2026-07-01

## 반영한 핀

최종 보드 역할 기준:

- Board2 = 1축 / base_joint
- Board1 = 2~5축 / arm joints

따라서 새로 받은 리미트 핀 중 Board1에 해당하는 항목만 반영했습니다.

| 실제 축 | Board1 local motor id | Limit pin | 상태 |
|---|---:|---|---|
| 2축 | 0 | PA15 | 반영 |
| 3축 | 1 | PB4 | 반영 |
| 4축 | 2 | PB12 | 반영 |
| 5축 | 3 | 미제공 | 미반영 |

PA7은 1축/base축 리미트이므로 Board1이 아니라 Board2에 반영했습니다.

## 안전 처리

Board1은 프로토콜상 2~5축 4개 축을 homing해야 합니다. 현재 5축 limit pin이 없으므로 `0x020` homing은 `ERR_HOMING_FAIL`로 거절하도록 유지했습니다.

이유: 5축 limit pin 없이 homing을 시작하면 해당 축이 무한히 home 방향으로 움직일 수 있기 때문입니다.

5축 limit pin이 확정되면 `main.c`의 아래 항목을 수정하면 됩니다.

```c
#define BOARD1_LIMIT_SWITCH_ASSIGNED_MASK 0x0FU

/* motor 3: arm 5-axis */ {GPIOB, 7U, GPIOB, 6U, <LIMIT_PORT>, <LIMIT_PIN>, GPIOB, 2U},
```
