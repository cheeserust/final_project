# Board2 Limit Pin Patch - 2026-07-01

## 반영한 핀

Board2는 최종 프로토콜 기준으로 1축/base_joint를 담당합니다.

| 실제 축 | Board2 local motor id | Limit pin | 상태 |
|---|---:|---|---|
| 1축 / base_joint | 0 | PA7 | 반영 |

STEP/DIR 핀은 이번 이미지에 포함되지 않았으므로 기존 기본값을 유지했습니다.

| 기능 | 핀 |
|---|---|
| STEP | PC0 |
| DIR | PC1 |
| LIMIT | PA7 |

리미트 스위치 배선 기본값은 NO + GND + 내부 Pull-up입니다.
