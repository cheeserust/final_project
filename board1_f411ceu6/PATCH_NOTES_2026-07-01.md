# Patch Notes — 2026-07-01

## Board1 수정 내용

최종 CAN 프로토콜 검증을 위해 하드웨어 없이 바로 수정/검증 가능한 부분을 반영했습니다.

### 1. 공통 명령 payload strict 검사 추가

아래 명령은 이제 DLC 8 및 reserved byte 0 조건을 검사합니다.

```text
0x001 ESTOP       : 001#0100000000000000
0x010 Enable      : 010#0100000000000000
0x010 Disable     : 010#0000000000000000
0x020 Arm Homing  : 020#FF00000000000000
0x030 Clear Error : 030#FF00000000000000
```

payload에 Board ID를 넣는 legacy 형식은 허용하지 않습니다.

### 2. Board1 4축 joint limit 검사 추가

`0x101` move command 수신 시 최종 목표 위치가 아래 범위를 벗어나면 `ERR_INVALID_CMD`로 거부합니다.

| Motor ID | 실제 축 | Min raw | Max raw |
|---:|---|---:|---:|
| 0 | 팔 2축 | -9000 | 9000 |
| 1 | 팔 3축 | -8000 | 8000 |
| 2 | 팔 4축 | -9000 | 9000 |
| 3 | 팔 5축 | -17000 | 17000 |

angle mode와 step mode 모두 최종 target step을 출력축 raw angle로 역변환하여 검사합니다.

### 3. 그대로 유지한 부분

- `0x101` 4-frame staging 구조
- 20ms staging timeout
- internal 8-point queue / external 32 command slot status
- simulated homing
- simulated motion
- `0x201` status 100ms
- `0x301` position feedback 100ms
