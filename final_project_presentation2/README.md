# final_project_presentation2

독립형 ROS 2 패키지입니다. 기존 GUI, 미션 노드, Nav2 코드를 import하거나
실행하지 않습니다. 로봇의 하드웨어 ROS 인터페이스만 사용합니다.

## 경로와 마커

로봇은 마커 1과 3 사이의 같은 직선 경로만 왕복하며 자동 코너 회전이나
끝점 180도 회전을 하지 않습니다.

```text
놓기 C / dropoff (3) +----------------+ 집기 A / pickup (1)
```

- 놓기 방향: 전방 카메라로 마커 3을 보며 `A → C` 전진
- 집기 방향: 후방 카메라로 마커 1을 보며 `C → A` 후진
- 사용하는 ArUco 마커는 ID 1(후방)과 ID 3(전방)뿐입니다.
- 실제 검은 정사각형 한 변은 `5 cm`이며 설정값은
  `aruco.marker_size_m: 0.05`입니다.
- 기본 정지거리는 마커 1에서 `0.25 m`, 마커 3에서 `0.20 m`입니다.
  이는 총 이동거리가 아니라 도착 시 카메라와 마커 사이의 정지 거리입니다.
- 출발점 마커는 보이면 상태만 기록하고, 보이지 않거나 자세 허용범위를
  벗어나도 출발을 막지 않습니다. 실제 이동은 반대편 목적지 마커로
  제어합니다.
- 목적지 마커가 처음부터 보이지 않으면 알려진 직선을 저속으로 탐색하고,
  보인 뒤에도 좌우·Yaw 조향 없이 같은 직선 방향으로 이동합니다. 마커는
  목표 거리 도달 여부만 판단하며 가까워지면 정지합니다.
- 탐색 시간 초과 시 정지하고 오류 잠금 없이 IDLE로 돌아가며, 목적지 재개
  체크포인트가 있으면 그대로 보존합니다.

## 실행 구성

중앙 PC:

```bash
ros2 launch final_project_presentation2 final_project_presentation2.launch.py
```

Pinky 컴퓨터:

```bash
ros2 launch final_project_presentation2 final_project_presentation2_watchdog.launch.py
```

중앙 PC와 Pinky 모두 v2 패키지를 설치하고 위 v2 watchdog을 사용해야 합니다.
원본 패키지와 v2 패키지는 공통 하드웨어 `/cmd_vel`을 사용하므로 동시에
실행하지 마십시오.

웹 UI 기본 주소는 중앙 PC 자체의 `http://127.0.0.1:8080`입니다.
외부 PC에서 접속해야 할 때만 통합 JSON의 `web.host`를 변경하고
중앙 노드를 재시작하십시오. v1 HTTP API에는 인증이 없으므로, 이 경우
신뢰할 수 있는 분리된 LAN에서 단 하나의 조작 대시보드만 사용하십시오.

watchdog만 `/cmd_vel`을 publish해야 합니다. Nav2 및 다른 `/cmd_vel`
publisher는 이 데모 중 중지하십시오. 중앙 노드는 고정 토픽
`/final_project_presentation2/cmd_vel_raw`을 사용하고, Pinky watchdog은
중앙 노드가 이 raw 토픽의 유일한 publisher인지 계속 검사합니다. raw 또는
`/cmd_vel` publisher 독점 조건이 깨지면 watchdog은 0을 출력하고 중앙
노드는 진행 중인 주행을 오류 정지합니다. raw 명령은 source timestamp가
있는 depth-1 `TwistStamped`이며, Pinky 시각 기준 기본 0.25초보다 오래된
큐 재생 명령도 거부합니다.
watchdog launch는 프로세스 종료 시 재시작하지만, 베이스 드라이버 자체의
명령 timeout을 대체할 수는 없습니다. 실기 시연에서는 드라이버 측 timeout과
물리 E-stop을 반드시 함께 사용하십시오.

중앙 PC와 Pinky의 시간은 chrony 또는 NTP로 동기화하는 것이 좋습니다.
odom과 JointState는 ROS header timestamp를 검증하지만, 데모 카메라는
드라이버의 0/불규칙 timestamp 때문에 영구 차단되지 않도록 로컬 수신 시각으로
freshness를 판단합니다. 한 프레임 또는 마커 후보가 깨지면 그것만 건너뛰고
다음 프레임과 같은 프레임의 다른 후보는 계속 처리합니다.

## 통합 설정

권위 설정은 패키지와 같은 폴더 안의 다음 JSON 파일입니다.

```text
src/final_project_presentation2/config/final_project_presentation2.json
```

`colcon build --symlink-install`로 한 번 빌드하면 실행 노드도 이 소스 JSON을
직접 읽고 씁니다. 이후 파라미터 수정에는 재빌드가 필요 없으며, 웹 UI의
`설정 다시 불러오기`를 누르면 됩니다. topic 이름과 `web.host`/`web.port`는
노드 생성 시 연결되므로 변경 후 중앙 노드를 재시작해야 합니다.
일반 `colcon build`로 install 복사본이 생기면 이전 값으로 조용히 실행하지
않고 시작을 거부하며 `--symlink-install` 재빌드를 안내합니다.

JSON에는 주석과 마지막 쉼표를 넣을 수 없습니다. `.bak` 또는 날짜별 백업은
자동 생성하지 않습니다. 저장 중에만 임시 파일을 사용하고 atomic replace가
끝나면 제거합니다. 노드는 위 JSON 하나만 읽고 쓰며 다른 위치 탐색, 설정
복사 또는 형식 변환을 하지 않습니다.

마커 ID/크기/정지거리, 카메라 보정, 직선 제어 gain/속도, 수동 회전 설정,
모든 timeout, watchdog 제한, 팔·그리퍼 범위, 카테고리, 저장 동작,
워크플로우를 이 파일 하나에서 관리합니다. 저장 변경은 revision 검사, 파일
lock, 임시 파일, `fsync`, atomic replace를 사용합니다. 잘못된 설정에서는
nonzero 주행을 허용하지 않습니다.

목표 마커가 출발 위치에서 아직 보이지 않을 때는 A-C 직선 경로를
`motion_control.acquire_creep_mps` 속도로 천천히 탐색합니다. 정확한
카메라와 ID가 검출되면 거리 전용 정지 제어로 전환하며, 다른 ID와 반대편
카메라는 계속 무시합니다. 탐색 한도는 `timeouts.acquire_creep_sec`이고,
시간 초과 시 주행을 멈춘 뒤 오류 잠금 없이 재시도 가능한 IDLE 상태로
돌아갑니다. 카메라 영상이 stale이면 탐색 이동도 정지합니다. 기존 live
설정에 두 필드가 없으면 각각 `0.03 m/s`, `60 s`를 기본값으로 사용합니다.

이 탐색 구간은 odom으로 고정 거리를 재는 이동이 아니라 목적지 마커가 보일
때까지 `linear_direction × acquire_creep_mps`를 내는 저속 직진입니다. 현재
`acquire_creep_mps`는 `0.03 m/s`입니다. 마커가 보인 뒤에는 P 감속이나 조향
없이 각 마커의 `max_linear_mps: 0.08`로 계속 직진하고, 측정 거리가
`target_distance_m + distance_tolerance_m` 이하가 되면 정지합니다. 마커별
제한을 지우면
`motion_control.max_linear_mps: 0.08`이 대신 적용되고, 모든 명령의 최종 안전
상한은 `safety.max_linear_mps: 0.12`입니다.

두 마커는 `completion_mode: distance_only`, 최초 유효 검출 요구량은
`stable_detections: 1`입니다. 검출 후 한두 프레임이 빠질 때 속도가 0으로
끊기지 않도록 최대 `marker_loss_sec: 0.35 s` 동안 마지막 직선 방향과
`max_linear_mps`를 유지합니다. 그 안에 다시 검출되지 않으면 즉시 정지하고
해당 이동을 실패 처리합니다. 카메라 영상 자체가 stale이면 이 유예와 관계없이
0 명령을 냅니다.

경로 설정은 front/rear, 출발·목적지 조합, 왕복 방향의 의미적 일관성을
검사하지 않습니다. 서로 반대가 아닌 방향이나 route 카메라와 목적지 마커
카메라가 다른 조합도 설정 로딩을 막지 않습니다. 런타임 예외를 피하기 위한
최소 형식만 검사하므로 카메라는 `front/rear`, 출발·목적지는 존재하는 마커,
`linear_direction`은 `1/-1` 중 하나여야 합니다. 실제 접근에 사용하는
카메라는 `route.*.camera`가 아니라 목적지의 `markers.*.camera`이고, 직진
부호는 해당 route의 `linear_direction`에서 가져옵니다.

현재 거리 전용 모드에서는 `lateral_kp`, `yaw_kp`, `linear_gate_angle_deg`,
`steering_output_scale`을 주행 명령에 사용하지 않습니다. 이 값들은 마커의
`completion_mode`를 다시 `full_pose`로 선택할 때만 좌우·Yaw 정렬 제어에
사용됩니다. `alignment_hysteresis_ratio`는 목표 거리 정지 유지 중 검출값이
경계에서 흔들려도 다시 출발하지 않도록 하는 데 계속 사용합니다.

실제 인쇄 마커가 5 cm라면 `marker_size_m`만 `0.05`로 바꾸는 것이 맞습니다.
PnP 위치값은 마커 한 변 설정에 선형 비례하므로 실제 5 cm 마커를 이전
`0.10` 설정으로 쓰면 거리와 좌우 위치를 약 2배로 잘못 추정합니다. 현재
설정에서는 정면 마커를 가정할 때 전방 카메라의 마커 3은 정지거리 0.20 m에서
약 177 px (`708.85 × 0.05 / 0.20`), 후방 카메라의 마커 1은 정지거리
0.25 m에서 약 172 px (`859.91 × 0.05 / 0.25`) 폭으로 보입니다.

`target_distance_m` 0.25/0.20 m, 미터 단위 tolerance, 카메라 K/distortion은
마커 크기와 별개의 실제 거리·카메라 보정값이므로 자동으로 절반으로 바꾸지
않았습니다. 작은 마커는 먼 거리에서 픽셀 폭과 검출 여유가 줄어드므로 실제
장비에서 획득 거리와 흔들림만 다시 확인하십시오. 현재 기본값은 고정 보정을
쓰도록 `use_camera_info: false`이며, `true`로 바꾸면 수신한 CameraInfo가
템플릿 보정값을 대체합니다.

## 팔 동작과 워크플로우

UI에서 카테고리를 추가/이름변경/삭제하고, 팔 5축 및 그리퍼 9축 동작을
저장할 수 있습니다. 동작은 팔만, 그리퍼만, 또는 둘 다 포함할 수 있습니다.
둘 다 포함하면 동시에 전송하며 한쪽 실패 시 다른 쪽도 취소합니다.
같은 카테고리 이름은 대소문자와 연속 공백 차이까지 정규화해 거부합니다.
카테고리를 삭제하면 그 안의 동작과 워크플로우는 미분류로 이동합니다.

로봇팔 보드의 Enable, Home, Disable, Clear, ESTOP은 UI에서 명시적으로
수동 호출합니다. 저장 동작이나 워크플로우가 이 명령을 자동 호출하지
않습니다. 확인 팝업은 Disable에만 표시되고 다른 명령은 즉시 전송됩니다.
같은 영역에서 `/arm_board/status`를 1초마다 갱신해 Board 1/2/3의 State,
Enabled, Home/Ready, Error, Fault, status age와 권장 조작을 표시합니다.
서비스 이름은 `topics.arm_*_service`, 일반 서비스 timeout과 Home timeout은
각각 `timeouts.arm_service_sec`, `timeouts.arm_home_sec`에서 수정합니다.
이 항목이 없는 이전 live config도 기존 서비스 이름과 timeout 기본값으로
계속 실행됩니다.

STOP은 취소 요청 전송만으로 완료 처리하지 않고 각 액션의 실제 종료 상태를
확인합니다. 통신 오류 때문에 종료를 확인할 수 없으면 정상 대기로 돌아가지
않으며, 해당 중앙 노드를 재시작해야 다음 동작을 실행할 수 있습니다.

데모 기본값은 `safety.require_arm_fault_clear: false`입니다. 따라서 새 노드는
`/arm_board/status`의 일시적인 이상값으로 명령을 미리 차단하지 않고 기존
action 서버에 목표를 바로 보냅니다. 실제 실행 허용 여부는 기존
`arm_can_bridge`가 최종 판단합니다. 이 패키지는 enable, disable, home,
clear-error, ESTOP 서비스를 자동 호출하지 않습니다. 엄격한 사전 검사가
필요한 환경에서만 이 값을 `true`로 변경하십시오.

새 워크플로우에서는 다음 네 단계만 사용합니다.

- `POSE`
- `GO_DROPOFF`
- `WAIT_SECONDS`
- `GO_PICKUP`

주행 안전 자세 예약과 강제 검사는 제거했습니다. 초기 동작 목록은 비어 있고
`next_pose_id`가 1이므로 사용자가 저장하는 첫 동작이 ID 1이 됩니다. 물건
하나의 흐름은 원하는 `POSE`들을 직접 배열한 뒤 `GO_DROPOFF`, 놓기 `POSE`,
`GO_PICKUP`을 필요한 위치에 넣으면 됩니다. 놓은 뒤 별도의 사용자 확인 없이
다음 단계인 집기 위치 복귀를 바로 시작합니다. 이전 버전에 저장된
`WAIT_RETURN_CONFIRM` 단계는 설정 호환을 위해 읽지만 실행 시 자동 통과합니다.

따라서 팔이 뻗어 있거나 JointState가 수신되지 않아도 베이스 주행과 수동
회전은 안전 자세 검사로 차단되지 않습니다. 주행 전에 팔을 차체 안쪽의 운반
자세로 넣는 단계는 사용자가 직접 만든 시퀀스에 포함하고 실물 간섭을 확인해야
합니다.

저장 동작 ID는 워크플로우에서 원하는 순서로 반복·재배치할 수 있습니다.
팔 동작만 실행한다면 `POSE 3 → POSE 1 → POSE 2`도 그대로 가능합니다.
베이스 이동까지 넣을 때는 `POSE 3 → POSE 1 → POSE 2 → GO_DROPOFF →
POSE 2 → GO_PICKUP`처럼 한 번에 실행할 수 있습니다.
저장 동작 ID, ArUco 마커 ID 1·3, 하드웨어 Board ID 1·2·3은 서로 다른 ID
체계입니다.

미분류 카테고리는 삭제할 수 없습니다. 카테고리를 삭제하면 그 안의 동작과
워크플로우만 미분류로 이동합니다. 워크플로우에서 참조하는 동작 ID는 삭제할
수 없습니다.

## 검증

```bash
colcon build --symlink-install --packages-select final_project_presentation2
colcon test --packages-select final_project_presentation2
colcon test-result --verbose
```

실제 주행 전에는 바퀴를 띄우거나 충분한 안전 공간에서 다음 순서로 시험하는
것이 좋습니다: watchdog stale 정지, STOP, 수동 ±1° 회전, 마커 1·3 단일
구간, 목적지 거리 정지 임계값, 사용자가 만든 팔 시퀀스, 전체 직선 왕복
워크플로우. 모든 실기 시험에는 물리 E-stop 조작자를 별도로 두십시오.
