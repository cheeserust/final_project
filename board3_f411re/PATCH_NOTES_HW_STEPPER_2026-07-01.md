# Board3 Note — 2026-07-01

사용자가 제공한 모터 제어팀 코드는 STEP/DIR 스텝모터와 리미트스위치용 코드입니다.
Board3는 SCS0009 서보 9개를 UART/TTL로 제어하는 보드이므로 해당 코드의 STEP/DIR/limit switch 로직을 적용하지 않았습니다.

Board3는 이전 strict protocol 버전을 그대로 유지합니다.

남은 실제 하드웨어 작업은 별도 SCS0009 UART 제어 코드에서 처리해야 합니다.

- Servo ID 1~9 통신 확인
- 현재 위치 read
- raw servo position → joint angle 변환
- `g_state.current_pos_001deg[0~8]`를 실제 feedback 값으로 갱신
- `0x303`은 현재처럼 20ms마다 3 frame 송신 유지
