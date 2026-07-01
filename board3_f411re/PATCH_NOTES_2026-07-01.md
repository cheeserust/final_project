# Patch Notes — 2026-07-01

## Board3 수정 내용

최종 CAN 프로토콜 검증을 위해 하드웨어 없이 바로 수정/검증 가능한 부분을 반영했습니다.

### 1. legacy payload Board ID 허용 제거

아래 legacy 형식은 더 이상 허용하지 않습니다.

```text
010#0301
010#FF01
010#01
030#0300000000000000
```

### 2. 공통 명령 payload strict 검사 추가

아래 명령은 이제 DLC 8 및 reserved byte 0 조건을 검사합니다.

```text
0x001 ESTOP       : 001#0100000000000000
0x010 Enable      : 010#0100000000000000
0x010 Disable     : 010#0000000000000000
0x030 Clear Error : 030#FF00000000000000
```

### 3. 그대로 유지한 부분

- `0x020` Arm Homing은 Board3에서 무시
- `0x023` Gripper Home Posture
- `0x103` 9-frame staging
- duplicate motor id 검사
- duration mismatch 검사
- 100ms staging timeout
- `0x203` status 100ms
- `0x303` position feedback 20ms, 3-frame 압축 구조
- 하드웨어 연결 전 target mirror 방식의 virtual feedback
