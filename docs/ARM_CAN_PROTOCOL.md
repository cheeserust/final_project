# VicPinky Arm CAN Protocol

이 문서는 서버가 사용하는 현재 hybrid wire contract다. Board1은 Goal V3 direct
joint-goal이고, Arduino Board2는 legacy V2 position command를 사용한다. Board3
`0x103/0x203/0x303`은 기존 gripper protocol을 유지한다. Board1에는 V3
`duration_ms`를 보내고 Board2에는 legacy 5 ms duration tick을 보낸다.

## Joint mapping

| Joint | Board | Move ID | Local motor | Raw limit (0.01°) |
|---|---:|---:|---:|---:|
| `base_joint` | 1 | `0x101` | 3 | `-9000..18000` |
| `arm_joint_1` | 1 | `0x101` | 0 | `-8650..9000` |
| `arm_joint_2` | 1 | `0x101` | 1 | `-7810..8000` |
| `arm_joint_3` | 1 | `0x101` | 2 | `-9150..9000` |
| `arm_joint_4` | 2 | `0x102` | 0 | `-9000..9000` |

서버 입력 배열은 반드시 joint name으로 매핑한다. Board1 송신 순서는 local
motor `0,1,2,3`, 그 다음 Board2 motor `0`이다.

## Board1 Goal V3 frame

Board1은 `0x101`, DLC는 8이다.

| Byte | 의미 |
|---:|---|
| 0 | `0x90 | local_motor_id` (Execute=1, V3=1, Relative=0, Step=0) |
| 1~4 | absolute target `int32`, little-endian, 0.01° |
| 5 | `goal_id` uint8 |
| 6~7 | `duration_ms` uint16 little-endian, `1..65535` |

Board1의 4개 frame은 같은 goal ID와 duration을 사용한다. 범위 밖 target 또는
V3 duration은 clamp하거나 wrap하지 않고 goal 전체를 거절한다.

예: `goal_id=0x2A`, `duration_ms=5000`:

```text
101#90B80B00002A8813  arm_joint_1 = 30.00°
101#91F2F9FFFF2A8813  arm_joint_2 = -15.50°
101#92000000002A8813  arm_joint_3 = 0.00°
101#93282300002A8813  base_joint  = 90.00°
```

## Board2 Arduino legacy V2 frame

Board2는 `0x102`, DLC는 8이며 V3 goal ID를 wire에 싣지 않는다.

| Byte | 의미 |
|---:|---|
| 0 | `0x80 | local_motor_id` (Execute=1, Relative=0, Step=0), motor ID는 0 |
| 1~4 | absolute target `int32`, little-endian, 0.01° |
| 5~6 | legacy speed, 서버는 0으로 송신하며 현재 firmware에서는 무시 |
| 7 | duration tick: `ceil(duration_ms / 5)`, `1..255`로 clamp |

예를 들어 `arm_joint_4=10.00°`, `duration_ms=5000`이면 duration tick은 1000이
아니라 최대값 255로 clamp되어 `102#80E80300000000FF`를 보낸다. 따라서 1275 ms
초과 요청에서는 Board1의 요청 시간과 Board2의 요청 시간이 같지 않다. 기존 미션의
2~4초 요청을 거절하지 않기 위해 서버는 이 legacy 한계를 로그/문서로 알리고
Board2 tick만 255로 제한한다. Board2에는 `0x402` ACK가 없고 `0x040`
START/CANCEL도 구현되어 있지 않다.

## READY, START, CANCEL

Control ID는 `0x040`, DLC 8이다.

```text
START  = 01 <goal_id> 00 00 00 00 00 00
CANCEL = 02 <goal_id> 00 00 00 00 00 00
```

정상 순서는 Board1 4-frame staging → Board1 READY → Board2 legacy frame 1개 →
Board1 START 한 번 → Board1 STARTED → 양쪽 Status 완료다. READY 전에 Board2
frame이나 START를 보내지 않는다.

Board1 READY가 250 ms 안에 없으면 같은 ID/value/duration으로 4개 frame 전체를
재전송하며 최대 200회다. Board1의 full-mask DUPLICATE는 READY ACK 유실 복구로
취급한다. CANCEL은 미송신 batch 제거와 현재 writer 완료 뒤 Board1에 전송하고
Board1 CANCELLED를 기다린다. Board2는 CANCEL을 처리하지 않으므로 이미 받은
legacy target은 계속 수행한다. 서버는 그 뒤 Board2의 fresh IDLE/reached 및
`queueFreeCount=32`까지 기다려 새 goal을 격리한다. 다음 goal은 새 V3 ID를
사용한다.

서버 시작/재연결 시 임의 goal ID로 CANCEL probe를 보내지 않는다. 대신 양쪽의
fresh status가 각각의 형식으로 검증될 때까지 Move를 차단한다. 재시작 뒤 Board1의
`goal_slot_free=0`인 소유자 불명 goal은 자동 CANCEL하지 않으며, 실제 새 goal에
BUSY가 오면 status와 원문 ACK를 기록하고 즉시 실패로 돌려준다.

## ACK/NACK

Board1 `0x401`, DLC 8이다. Board2 legacy firmware는 ACK를 송신하지 않는다.

| Byte | 의미 |
|---:|---|
| 0 | protocol version, 반드시 3 |
| 1 | result: READY=0, STARTED=1, DUPLICATE=2, BUSY=3, STAGING_TIMEOUT=4, CONFLICT=5, CANCELLED=6, INVALID=7 |
| 2 | goal ID |
| 3 | received mask: Board1 `0..0x0F` |
| 4 | state snapshot |
| 5 | reserved 0 |
| 6~7 | duration echo uint16 little-endian |

BUSY goal은 queue하거나 timeout retry하지 않는다. STAGING_TIMEOUT은 누락 mask를
기록하고 Board1 전체 batch를 재전송한다. CONFLICT/INVALID는 동일 payload를
재시도하지 않고, 서버가 소유한 동일 goal에 한해 Board1 CANCEL로 부분 staging을
정리한다. 모든 ACK는 board/version/goal/duration/mask를 함께 검증하며 stale ACK는
현재 goal 증거를 덮지 않는다. READY/STARTED/CANCELLED 판정도 Board1 ACK에만
적용한다.

## Status `0x201/0x202`

| Byte | 의미 |
|---:|---|
| 0 | state: INIT=0, IDLE=1, HOMING=2, MOVING=3, ERROR=4, ESTOP=5, DISABLED=6 |
| 1 | error |
| 2~3 | 축별 4-bit flags: valid=bit0, ready=bit1, moving=bit2, reached=bit3 |
| 4 | limit bits |
| 5 | Board1 `goal_slot_free`는 0/1, Board2 `queueFreeCount`는 0..32 |
| 6 | enabled |
| 7 | status sequence |

Board1 Byte5는 V3 goal slot이라 0 또는 1이어야 한다. Board2 Byte5는 legacy queue
free count이므로 정상 유휴 상태가 32이며 `0..32`를 허용한다. 각 status frame의
state/error/masks/slot/sequence를 하나의 원자적 snapshot으로 저장한다. Board1
mask는 `0..0x0F`, Board2는 `0..0x01`을 넘을 수 없다.

Board1/2의 `error > 6` 또는 Board1의 `(limit bits & 0xF0) != 0`은
프로토콜상 불가능한 status로 간주해 해당 frame만 폐기한다. 이 frame은
최신 정상 status와 수신 시각을 덮어쓰지 않으며, 단일 비정상 frame으로
goal을 즉시 ABORT하지 않는다. 다만 정상 status가 communication timeout 이상
끊기면 기존 heartbeat timeout으로 중단한다.

완료는 Board1 STARTED 뒤 다음 조건을 모두 만족해야 한다.

- Board1/2 state IDLE, error 0, moving mask 0
- Board1 reached `0x0F`, Board2 reached `0x01`
- Board1 `goal_slot_free=1`, Board2 `queueFreeCount=32`
- 양쪽 heartbeat가 1초 이내

요청 duration보다 실제 이동이 길어도 정상 MOVING heartbeat가 계속되면 기다린다.
`duration + grace`만으로 CANCEL하지 않는다.

## Single writer, latest target, E-stop

ROS/웹 callback은 SocketCAN에 직접 쓰지 않는다. 서비스, Board1/2 goal,
Board3 streamer 모두 단일 serialized writer를 지나며 각 `send()`의 full-frame
성공을 확인한다. `ENOBUFS/EAGAIN`은 frame 단위로 제한 재시도하고 short write는
실패다. READY clock은 writer가 해당 board batch를 모두 보낸 직후 시작한다.

웹 수동 arm 입력은 latest-target-wins다. 활성 target과 같으면 dedupe하고, 다른
target이면 Board1 CANCELLED를 기다리는 동안 pending target 하나만 갱신한 뒤
마지막 target만 새 goal ID로 보낸다. 다만 Board2 legacy에는 CANCEL/ACK가 없으므로
이미 받아들인 Board2 target은 취소되지 않고 끝까지 수행한다. Board2가 완전히
IDLE이 된 뒤 마지막 pending target을 보내므로 동시 preemption은 제공하지 않지만
이전 Board2 동작 뒤에 새 명령이 몰래 queue되지는 않는다.

retry 횟수와 모든 runtime timeout은
`src/arm_can_bridge/config/retry_timeout.yaml` 한 파일에서 조정한다. 운영 기본값은
Board1 READY 250 ms × 200회이며, Board2 legacy에는 READY retry를 적용하지
않는다. BUSY/INVALID/CONFLICT에도 retry profile을 적용하지 않는다. 파일명에
`_ms`가 붙은 값의 단위는 millisecond다. 중앙 미션의 단계별 timeout/retry는 별도 실행 계층인
`src/mission_manager/config/action_servers.yaml`에서 action별로 조정한다.

E-stop은 `001#0100000000000000`을 writer 최우선으로 보낸다. 아직 송신되지 않은
goal/START/CANCEL을 제거하고 active goal을 `ABORTED_BY_ESTOP`으로 끝내지만
`010#00...` Disable은 보내지 않는다. 이는 안전 인증 STO가 아닌 software
powered-hold이며 기존 enable/holding torque 상태를 유지한다. encoder 없는
stepper는 급정지 때 탈조할 수 있으므로 표시 위치는 명령 기반 추정값이다.
Enable 또는 Clear Error 뒤 이전 goal은 자동 재시작하지 않는다.

## Position feedback

`0x301`은 signed int16 little-endian 4개로 motor0→`arm_joint_1`, motor1→
`arm_joint_2`, motor2→`arm_joint_3`, motor3→`base_joint`이다. `0x302`의 첫
int16은 `arm_joint_4`이고 나머지는 reserved다. 단위는 0.01°다.

## Board3 불변 범위

Board3 gripper는 기존 `0x103/0x203/0x303` 계약과 9-frame staging을 유지한다.
Board2와 Board3는 공용 `pack_position_command()`를 사용하지만 Board3의 Byte5~6
target-load와 9-frame staging은 Board2 계약과 별개다. Board1만 전용
`pack_arm_goal_v3()`를 사용한다.

## ROS API 안전 경고

팔 입력은 `/arm_controller/execute_joint_goal`의 `ExecuteArmGoal` Action이다.
motion 필드는 joint name 5개, 최종 radians 5개, `duration_ms` 하나다. 웹 경로는
진단용 `request_id`, `web_created_unix_ms`, `gui_received_unix_ms`도 채워 T0/T1을
CAN T2~T7 기록과 연결한다. 최종각 직행은 MoveIt 충돌 회피 waypoint를 보존하지
않으므로, 검증된 named joint pose와 서버 joint limit 검사를 통과한 목표에만
사용한다. MoveIt 계획의 마지막 point를 호환 경로로 재사용하지 않는다. Board3만 기존
`/gripper_controller/follow_joint_trajectory`를 유지한다.
