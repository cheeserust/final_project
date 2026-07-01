# Patch Notes — 2026-07-01

## Board2 확인 내용

Board2 코드는 이미 최종 Board2 프로토콜 기준으로 아래 항목이 반영되어 있어 코드 수정은 하지 않았습니다.

- `0x010` Enable/Disable: DLC 8, Byte1~7 reserved 0 검사
- `0x020` Stepper Homing: DLC 8, Byte0 = 0xFF, Byte1 = 0, Byte2~7 reserved 0 검사
- `0x030` Clear Error: DLC 8, Byte0 = 0xFF, Byte1~7 reserved 0 검사
- `0x102` base_joint move command
- base_joint limit `-9000 ~ 18000` 검사
- `0x202` status 100ms
- `0x302` position feedback 100ms

참고: Board2 프로토콜 문서 기준 `0x001` ESTOP payload는 현재 사용하지 않는 것으로 되어 있어 기존 동작을 유지했습니다.
