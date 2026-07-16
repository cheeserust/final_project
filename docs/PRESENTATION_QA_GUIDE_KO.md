# Scorpy 프로젝트 발표·데모 Q&A 가이드

작성 기준: 2026-07-14

## 0. 이 문서의 기준과 읽는 법

이 문서는 사용자가 제공한 다음 두 압축본을 실제 실행 코드와 설정 파일 중심으로 분석해 작성했다.

- Scorpy-driving_server_pj (1).zip
- Scorpy-driving_pinky (2).zip

임베디드 파트는 작업공간에 함께 있는 Board2 Arduino 펌웨어와 gripper_firmware_0710_moving_fix.zip도 확인했다. 다만 제공 자료에는 로봇팔 제조용 CAD 원본, PCB 회로도·Layout·Gerber·BOM, Board1 펌웨어가 없다. 따라서 이 문서에서 다음 표기를 구분한다.

- [코드 확인]: 제공 코드·설정에서 직접 확인한 사실
- [실기 확인]: 코드만으로는 알 수 없어 실제 로봇이나 측정 기록으로 확인해야 하는 사항
- [주의]: 발표에서 과장하면 안 되거나 데모 전에 반드시 점검할 사항

발표에서는 “설계상 목표”, “코드에 구현된 조건”, “실기로 측정한 성능”을 섞지 않는 것이 중요하다. 예를 들어 목표 이동 거리가 27 cm라는 것과 실제 바닥 기준 오차가 몇 cm라는 것은 서로 다른 답이다.

---

## 1. 내일 데모 전에 가장 먼저 확인할 사항

### 1.1 마커 기반 팔 동작의 실기 실행 경로

[주의] 정확히 제공된 서버 압축본의 hardware 모드에서는 MoveIt으로 계산한 마커 기반 waypoint를 실제 Board1·2로 실행하는 경로가 의도적으로 막혀 있다. task_executor_node의 plan_and_optionally_execute 함수는 계획 후 다음 오류로 작업을 중단한다.

    MoveIt waypoint execution is disabled by the Board1/2 V3 contract.

영향은 다음과 같다.

- named joint pose인 home, ready, tray 이동 등은 직접 관절 목표로 보낼 수 있다.
- ArUco 54를 이용한 물체 접근·파지는 첫 waypoint에서 중단된다.
- ArUco 50~53을 이용한 엘리베이터 버튼 누르기도 pre-press waypoint에서 중단된다.
- 따라서 이 압축본 그대로라면 전체 미션은 3단계 PICK_OBJECT_TO_TRAY에서 멈출 가능성이 높다.

발표 전 반드시 “실제 데모 PC에 설치된 빌드가 이 압축본과 같은가”, “별도 검증 브랜치나 직접 관절 자세 방식으로 이 부분을 보완했는가”를 확인해야 한다. 확인하지 않은 상태에서 “전체 29단계가 실기에서 완전 자율로 동작한다”고 말하면 안 된다.

### 1.2 팔 보정값의 현장 검증 상태

[코드 확인] fixed_poses.yaml은 calibration.complete가 true지만 field_verified는 false다. 파일 주석에도 최종 버튼·pick 자세를 실제 로봇에서 검증하지 못했다고 적혀 있다. 실행 코드는 field_verified=false를 차단 조건으로 쓰지 않으므로, 시작된다고 해서 안전한 자세라는 뜻은 아니다.

### 1.3 맵 전환 뒤 3초 대기 미적용

[주의] 최종 mission_flow.yaml에는 맵 전환 후 start_delay_sec: 3.0이 적혀 있으나, 제공 압축본의 유일한 /nav/go_to 어댑터는 이 값을 읽지 않는다. 따라서 중앙 코드 기준으로는 명시적 3초 대기가 적용되지 않는다. Pinky의 맵 전환 서버 내부 1.5초 안정화는 있지만, “맵 전환 후 총 3초를 기다린다”고 발표하면 정확하지 않다.

### 1.4 비상정지의 범위

- GUI의 ESTOP은 /arm_board/estop을 호출하는 팔 보드용 소프트웨어 정지다.
- 이동 베이스 전체를 정지시키는 GUI E-stop은 확인되지 않았다.
- Board2는 명령·큐를 멈추지만 드라이버 enable을 유지한다.
- Board3 그리퍼는 ESTOP/Disable 때 서보 torque-off나 즉시 hold 패킷을 실제로 보내는 코드가 확인되지 않았다.
- 직접 /cmd_vel을 내는 정렬·직진·승하차 구간에는 Nav2 collision avoidance가 적용되지 않는다.

[실기 확인] 데모 운영자는 베이스와 팔의 물리 전원 차단 수단을 각각 알고 있어야 한다.

### 1.5 발표 수치의 단일 기준

오래된 문서와 테스트 코드에 1.35 m, 1.37 m, 1.45 m, 35 cm, 60 cm, 160 cm 같은 값이 남아 있다. 최종 전체 미션 YAML 기준값은 다음과 같다.

| 항목 | 최종 설정값 |
|---|---:|
| 엘리베이터 앞 ID 20 정렬 거리 | 1.27 m |
| 정렬 측면 목표 | 0 m |
| 정렬 안정 유지 | 3초 |
| 팔 ready 시작 뒤 베이스 출발 지연 | 2초 |
| 문 쪽 추가 접근 | 0.27 m, 0.15 m/s |
| 문을 향한 회전 | 좌 80도 |
| ID 10 기준 탑승 목표 | 50 cm |
| ID 4·5 기준 하차 목표 | 70 cm |
| 배송 위치 회전 | 좌 180도 |
| 하차 뒤 자세 전환 | 시간 기반 좌 90도 |

### 1.6 설정·테스트 불일치

- 최종 전체 mission_flow.yaml에는 배송지 좌 180도 회전이 있다.
- 제공된 mission_flow_driving_only.yaml에는 이 단계가 빠져 있다.
- 중앙 관련 선택 테스트 결과는 67개 통과, 1개 실패였고 이 불일치가 실패 원인이다.
- driving-only 구성에는 operator confirm 단계가 있지만 해당 launch가 operator console을 자동으로 올리지 않는다.

최종 전체 launch를 쓰는지, driving-only launch를 쓰는지 발표 전에 구분해야 한다.

---

## 2. 프로젝트를 20초와 60초로 설명하는 법

### 20초 답변

“Scorpy는 이동형 로봇과 5축 로봇팔, 3지 9축 그리퍼를 결합한 ROS 2 기반 층간 배송 시스템입니다. 4층 402 위치에서 물체를 집어 트레이에 싣고, 자율주행과 비전 정렬로 엘리베이터를 이용해 5층 배송 위치에 전달한 뒤 4층으로 복귀하는 미션을 목표로 했습니다. 중앙 서버는 전체 순서를 조율하고, 주행·비전·팔·그리퍼 제어는 각 전문 노드와 제어보드가 수행합니다.”

### 60초 답변

“장거리 층내 이동은 LiDAR, 바퀴 odometry, Nav2와 AMCL을 사용하고, 엘리베이터 앞과 내부처럼 정밀 상대 위치가 필요한 구간은 ArUco 마커 기반 visual servoing으로 전환합니다. 중앙 PC는 29단계 미션을 ROS 2 Action으로 순차 실행하며 feedback, timeout, retry, cancel을 관리합니다. Raspberry Pi는 베이스, LiDAR, 카메라, Nav2와 엘리베이터 관련 Action 서버를 실행합니다. 팔 명령은 중앙의 SocketCAN 브리지를 통해 세 제어보드로 전달되고, 그리퍼는 9개의 직렬 서보를 동기 제어합니다. 브라우저 GUI에서는 미션 진행률, 지도와 위치, 보드 상태, 로그, 수동 제어를 확인할 수 있습니다.”

### 이 프로젝트의 핵심 설계 포인트

1. 한 가지 위치 추정 방식만 고집하지 않고 구간에 따라 Nav2와 마커 상대제어를 전환했다.
2. 중앙 조율 계층과 실제 제어 계층을 분리했다.
3. 시간이 긴 작업은 ROS 2 Action으로 통일해 feedback, cancel, result를 다뤘다.
4. 층별 지도를 분리하고 실제 하차 후 map과 AMCL 초기 위치를 전환했다.
5. 팔과 그리퍼를 CAN 기반 분산 제어보드로 구성했다.

---

## 3. 전체 시스템 큰 그림

### 3.1 시스템 경계

    브라우저
      │ HTTP/JSON
      ▼
    중앙 PC
      ├─ Web GUI
      ├─ Mission Manager
      ├─ Nav2 어댑터
      ├─ MoveIt·팔 작업 서버
      └─ SocketCAN 팔 브리지
             │ CAN 500 kbps
             ├─ Board1: 팔 4축
             ├─ Board2: 팔 1축
             └─ Board3: 그리퍼 9축

    Raspberry Pi
      ├─ ZLAC 베이스 모터·encoder odometry
      ├─ LiDAR와 scan filter
      ├─ 전방·후방 카메라
      ├─ Nav2·AMCL·map server
      └─ 정렬·문·승하차·층확인·맵전환 Action 서버

중앙 PC와 Raspberry Pi는 같은 ROS 2 DDS 네트워크에 있어야 한다. 최종 배포에서 /nav/go_to 서버는 중앙 PC의 vicpinky_nav_adapter 하나이며, 이 서버가 Pinky의 /navigate_to_pose로 전달한다.

### 3.2 누가 무엇을 책임지는가

| 계층 | 책임 | 책임지지 않는 것 |
|---|---|---|
| GUI | 사용자 입력, 상태 시각화, 로그, 충돌하는 조작 차단 | 실시간 모터 제어, 물리 안전 보증 |
| Mission Manager | 단계 순서, timeout, retry, cancel 전파, 전체 진행률 | 센서 해석, 모터 폐루프 |
| 전문 Action 서버 | 한 작업의 센서 판단과 제어 | 전체 미션 순서 |
| CAN 브리지 | ROS 관절 목표 검증, 프레임 변환·순서, 상태 조립 | 기구 강도, 물리 전원 차단 |
| 보드 펌웨어 | 로컬 모터·서보 구동, 상태·오류 | 전역 경로, 미션 의미 |
| Nav2 | 지도 안의 장거리 경로와 장애물 costmap | 엘리베이터 버튼 앞 정밀 정렬 |
| 마커 제어 | 짧은 구간 상대 위치 정렬 | 전역 지도 기반 경로 계획 |

### 3.3 ROS 2 통신 방식을 나눈 이유

- Action: 주행, 정렬, 팔 작업처럼 오래 걸리고 feedback·cancel·result가 필요한 작업
- Service: enable, home, clear, status처럼 짧은 요청과 즉시 응답
- Topic: LiDAR, 카메라, odometry, mission status, heartbeat처럼 계속 흐르는 상태

공통 하위 Action인 RunTask는 task_id, target_name, target_floor, marker_id와 extra_json을 사용한다. 공통 인터페이스라 확장하기 쉽지만 extra_json은 컴파일 시 타입 검사를 받지 않으므로 각 서버가 필수 키와 범위를 런타임에 검증해야 한다.

---

## 4. 최종 29단계 미션

### 4.1 단계별 흐름

| 단계 | 상태 | 실제 의미 |
|---:|---|---|
| 1 | ARM_HOMING | 팔 원점 복귀 |
| 2 | ARM_READY_AT_PICKUP | pickup ready 자세 |
| 3 | PICK_OBJECT_TO_TRAY | ID 54 물체를 집어 트레이에 놓기 |
| 4 | GO_TO_ELEVATOR_FRONT | Nav2로 4층 엘리베이터 앞 이동 |
| 5 | ALIGN_ELEVATOR_TAG | ID 20 기준 1.27 m 정렬 |
| 6 | PRESS_ELEVATOR_CALL_BUTTON | ID 50 호출 버튼 |
| 7 | READY_AND_APPROACH_ELEVATOR_4F | 팔 ready와 27 cm 접근 병행 |
| 8 | FACE_ELEVATOR_4F | 좌 80도 회전 |
| 9 | WAIT_ELEVATOR_OPEN | 정면 LiDAR로 문 열림 확인 |
| 10 | ENTER_ELEVATOR | ID 10 기준 승차 |
| 11 | PRESS_5F_BUTTON | ID 52, 5층 버튼 |
| 12 | WAIT_5F | ID 5로 5층 확인 |
| 13 | EXIT_ELEVATOR | ID 5 기준 하차 |
| 14 | SWITCH_5F_MAP | 5층 map과 AMCL seed 전환 |
| 15 | GO_TO_TARGET_PLACE | Nav2로 object_place 이동 |
| 16 | ROTATE_AT_DELIVERY | 좌 180도 회전 |
| 17 | DELIVER_OBJECT_FROM_TRAY | 트레이 물체를 배송 위치에 놓기 |
| 18 | RETURN_TO_ELEVATOR | 5층 엘리베이터 앞으로 이동 |
| 19 | ALIGN_ELEVATOR_TAG_RETURN | ID 20 기준 정렬 |
| 20 | PRESS_ELEVATOR_CALL_BUTTON_RETURN | ID 53 호출 버튼 |
| 21 | READY_AND_APPROACH_ELEVATOR_5F | 팔 ready와 27 cm 접근 병행 |
| 22 | FACE_ELEVATOR_5F | 좌 80도 회전 |
| 23 | WAIT_ELEVATOR_OPEN_RETURN | 문 열림 확인 |
| 24 | ENTER_ELEVATOR_RETURN | ID 10 기준 승차 |
| 25 | PRESS_4F_BUTTON | ID 51, 4층 버튼 |
| 26 | WAIT_4F | ID 4로 4층 확인 |
| 27 | EXIT_ELEVATOR_RETURN | ID 4 기준 하차 |
| 28 | SWITCH_4F_MAP | 4층 map과 AMCL seed 전환 |
| 29 | RETURN_HOME | Nav2로 402 복귀 |

### 4.2 마커 ID 치트시트

마커 사전이 두 종류이므로 반드시 구분한다.

| 용도 | 사전 | ID | 의미 |
|---|---|---:|---|
| Pinky 전방 | DICT_4X4_50 | 20 | 엘리베이터 앞 정렬 |
| Pinky 전방 | DICT_4X4_50 | 10 | 캐빈 내부 승차 기준 |
| Pinky 후방 | DICT_4X4_50 | 4 | 4층 확인·하차 |
| Pinky 후방 | DICT_4X4_50 | 5 | 5층 확인·하차 |
| 손목 카메라 | DICT_4X4_100 | 50 | 4층 호출 버튼 |
| 손목 카메라 | DICT_4X4_100 | 51 | 캐빈 4층 버튼 |
| 손목 카메라 | DICT_4X4_100 | 52 | 캐빈 5층 버튼 |
| 손목 카메라 | DICT_4X4_100 | 53 | 5층 호출 버튼 |
| 손목 카메라 | DICT_4X4_100 | 54 | pickup 물체 |
| 손목 카메라 | DICT_4X4_100 | 55 | 예비 |

[주의] 저장소 루트의 단순 aruco_test.py는 DICT_4X4_50을 사용하므로 손목 ID 50~55 검증 도구로 쓰면 안 된다. 최종 손목 detector 설정을 사용해야 한다.

### 4.3 주요 전역 좌표

| 위치 | x | y | yaw |
|---|---:|---:|---:|
| 4층 402 | 2.998 | -12.433 | 0 |
| 4층 elevator_front | 0.470 | -0.455 | 1.983 |
| 5층 elevator_front | 14.93 | 1.90 | 0 |
| 5층 object_place | 2.84 | 1.17 | 0.109 |

map switch가 사용하는 AMCL seed는 일반 Nav 목표와 별도다.

- 4층 seed: 약 (1.20, -0.051, -1.46)
- 5층 seed: 약 (15.77, 1.09, -3.0614)

이 값은 “하차 직후 로봇의 예상 위치”이고 elevator_front Nav 목표와 같은 개념이 아니다. 다만 실제 설치 위치와 맞는지는 현장에서 확인해야 한다.

### 4.4 timeout과 retry

| 작업 | 중앙 timeout | retry |
|---|---:|---:|
| Nav2 이동 | 180초 | 1회 |
| ID 20 정렬 | 120초 | 1회 |
| 팔 execute | 240초 | 없음 |
| 버튼 누르기 | 180초 | 없음 |
| 팔 homing | 180초 | 없음 |
| ready + approach | 90초 | 없음 |
| 직진·회전 | 30초 | 없음 |
| 문 열림 | 60초 | 없음 |
| 탑승·하차 | 각 300초 | 없음 |
| 층 확인 | 150초 | 1회 |
| 맵 전환 | 30초 | 1회 |

실패하면 설정된 횟수만큼만 재시도하고 전체 미션을 중단한다. 자동 rollback이나 중간 checkpoint 재개는 없다. 새 미션은 처음부터 시작하므로 실패 후에는 물체 위치, 팔 자세, 현재 층, map, AMCL 위치를 사람이 먼저 맞춰야 한다.

---

## 5. 모든 팀원이 공통으로 받을 수 있는 질문

### 기초

#### Q1. 이 프로젝트가 해결하려는 문제는 무엇인가요?

“한 층 안에서만 움직이는 모바일 로봇을 넘어, 물체 조작과 엘리베이터 이용을 결합해 서로 다른 층 사이의 배송 과정을 자동화하는 것이 목표입니다.”

#### Q2. 단순 자율주행 로봇과 무엇이 다른가요?

“목적지까지 이동하는 것뿐 아니라 물체를 집고, 엘리베이터 호출·층 선택 버튼을 누르고, 승하차하며, 층별 지도를 바꾼 뒤 물체를 놓는 조작 작업까지 하나의 미션으로 통합했습니다.”

#### Q3. 왜 ROS 2를 사용했나요?

“주행, 비전, 팔, GUI를 독립 노드로 나눠 병렬 개발하기 좋고, Topic·Service·Action을 통해 스트림, 짧은 명령, 장시간 작업을 목적에 맞게 연결할 수 있기 때문입니다.”

#### Q4. 중앙 서버가 모든 모터를 직접 제어하나요?

“아닙니다. 중앙 서버는 순서와 오류 처리를 조율합니다. 실제 주행 제어, 마커 추종, 스텝 생성, 서보 제어는 각각의 전문 노드와 펌웨어가 담당합니다.”

#### Q5. 완전 자율이라고 할 수 있나요?

“설계된 402→5층 배송→402 복귀 시나리오에서는 사람이 단계마다 명령하지 않도록 구성했습니다. 다만 현재 범용 건물·임의 목적지를 모두 처리하는 시스템은 아니고, 마커 배치·층별 지도·고정 시나리오에 의존합니다.”

### 중급

#### Q6. 왜 Nav2와 ArUco를 둘 다 사용하나요?

“Nav2는 지도 위 장거리 이동과 장애물 회피에 적합하고, 버튼 앞이나 엘리베이터 내부는 수 cm 수준의 상대 정렬이 중요합니다. 그래서 전역 이동 후 마커 기반 근거리 제어로 전환했습니다.”

#### Q7. 실패하면 로봇은 어떻게 하나요?

“각 Action이 성공·실패와 원인을 반환하고 중앙 서버가 timeout과 일부 retry를 관리합니다. 재시도 후에도 실패하면 다음 단계로 무리하게 진행하지 않고 미션을 중단하며 GUI와 로그에 마지막 상태를 남깁니다.”

#### Q8. 전체 진행률 50%는 시간도 절반이라는 뜻인가요?

“아닙니다. 완료 단계 수와 현재 단계 progress를 29단계 기준으로 환산한 값입니다. 단계별 실제 시간이 다르기 때문에 시간 비율과는 다릅니다.”

#### Q9. 네트워크가 끊기면 어떻게 되나요?

“ROS 2 Action과 heartbeat가 끊겨 timeout 또는 stale 상태가 될 수 있습니다. 다만 브라우저 연결이 끊겼다고 미션이 자동 취소되는 구조는 아니므로 운영자는 중앙·Pinky 프로세스와 물리 E-stop을 별도로 관리해야 합니다.”

#### Q10. 안전은 어떻게 고려했나요?

“관절 범위, 상태 freshness, enable·homing 조건, Action timeout·cancel, 마커 소실 시 정지, odometry 소실 시 정지 같은 소프트웨어 방어가 있습니다. 하지만 이는 안전 인증 시스템이 아니며 베이스 직접 제어 구간의 collision monitor와 전원 차단형 E-stop은 별도 보완이 필요합니다.”

### 심화

#### Q11. 이 시스템의 가장 큰 기술적 한계는 무엇인가요?

“고정 마커와 고정 층 지도에 의존하고, 엘리베이터 API나 문 안전센서와 직접 연동하지 않습니다. 또한 제공 압축본의 마커 기반 팔 waypoint 실기 실행 경로가 막혀 있어 배포 빌드 확인이 필요합니다.”

#### Q12. 왜 하위 작업을 하나의 공통 RunTask Action으로 만들었나요?

“중앙 실행기는 모든 작업을 같은 방식으로 timeout, feedback, cancel 처리할 수 있어 통합이 단순해집니다. 대신 작업별 extra_json을 각 서버가 엄격히 검증해야 하는 trade-off가 있습니다.”

#### Q13. 시스템의 single point of failure는 무엇인가요?

“Mission Manager와 ROS 2 네트워크가 전체 조율의 중심입니다. 중앙 프로세스가 재시작되면 실행 상태와 Goal Handle을 자동 복구하지 못합니다. 각 펌웨어는 로컬 상태를 유지할 수 있지만 전체 미션 resume은 지원하지 않습니다.”

#### Q14. 성능을 어떤 지표로 평가했나요?

“코드에서 목표·허용오차·timeout은 확인되지만 전체 성공률, 평균 수행시간, 실제 정렬 오차, payload, 배터리 시간은 제공 자료로 확인되지 않습니다. 발표에서는 팀이 실제 측정한 횟수와 결과만 제시해야 합니다.”

#### Q15. 향후 개선 방향은 무엇인가요?

“마커 waypoint를 검증된 실기 궤적으로 연결하고, 베이스 Twist mux·collision monitor·watchdog과 전원 차단형 E-stop을 추가하며, 층 마커 freshness와 map 수렴의 yaw·covariance 검증, checkpoint resume, 엘리베이터 API 연동을 보강할 수 있습니다.”

---

---

## 6. 담당 1 — 로봇팔 펌웨어·기구·PCB 설계

### 6.1 한 문장 역할 설명

“5축 로봇팔의 기구 구조와 관절 한계를 설계하고, 각 모터를 구동하는 제어보드와 homing·상태·오류·CAN 명령 처리를 구현한 파트입니다.”

### 6.2 반드시 알아야 할 큰 틀

#### 팔 자유도와 보드 분담

팔은 base_joint와 arm_joint_1~4의 5축이다. 그리퍼의 9축은 별도다.

| ROS 관절 | 보드 | 로컬 모터 | 코드상 허용 범위 |
|---|---:|---:|---:|
| arm_joint_1 | Board1 | 0 | -86.5° ~ 90° |
| arm_joint_2 | Board1 | 1 | -78.1° ~ 80° |
| arm_joint_3 | Board1 | 2 | -91.5° ~ 90° |
| base_joint | Board1 | 3 | -90° ~ 180° |
| arm_joint_4 | Board2 | 0 | -90° ~ 90° |

CAN의 각도 raw 단위는 0.01도다. ROS 쪽은 radian이므로 브리지가 변환한다. ROS 관절 순서와 보드 모터 번호 순서가 같지 않기 때문에 이름 기반 매핑이 중요하다.

#### Board1과 Board2의 차이

- Board1 4축: Goal V3. 네 목표를 먼저 staging하고 READY ACK 후 START한다.
- Board2 1축: legacy position frame. 명시적 READY·START·CANCEL ACK가 없다.
- 호스트는 Board1 READY를 확인한 뒤 Board2 명령을 보내고 즉시 Board1 START를 보낸다.
- 따라서 Board1 내부 4축은 원자적으로 시작하지만 5축 전체가 하나의 하드웨어 클록으로 완전 동기 시작하는 구조는 아니다.

#### Board2 제공 펌웨어에서 확인한 내용

- Arduino Uno/Nano 계열 + MCP2515 8 MHz, CAN 500 kbps
- TB6600형 STEP/DIR/ENA 인터페이스
- STEP D6, DIR D7, ENA D8, limit D3
- 기어비 120:1, 모터 48 step/rev, microstep 16
- 출력축 1회전당 92,160 step
- 최대 1,250 step/s, queue 32
- encoder 없이 발생시킨 step 수로 위치를 추정하는 open-loop
- 일정 속도 구동이며 요청 시간이 너무 짧으면 물리 최대 속도에 맞춰 실제 시간이 늘어난다.
- 전원 인가 뒤 homing으로 기준점을 잡아야 한다.

[주의] Board2 limit pin은 INPUT_PULLUP인데 코드에서는 HIGH를 눌림으로 해석한다. 외부 반전 회로나 실제 배선 논리를 확인해야 한다. 펌웨어 내부 homing timeout과 TB6600 fault 입력 감시는 확인되지 않았다.

#### 기구 모델에서 확인한 내용

- 모델은 5축 팔과 3지 9축 그리퍼로 구성된다.
- URDF 설계 메타데이터의 전체 질량은 약 8.739 kg이나 실제 조립체 실측값은 별도다.
- 팔 링크 재질 메타데이터에는 알루미늄 6061, PLA+ 등이 섞여 있다.
- 5자유도이므로 임의의 3차원 위치와 임의의 3축 자세를 항상 동시에 만족시키는 일반 6자유도 팔은 아니다.
- MoveIt 설정도 위치 중심 계획이며 orientation constraint가 기본적으로 꺼져 있다.

[실기 확인] payload, 최대 reach, 반복정밀도, backlash, 안전율, 무게중심, 실제 재질은 제조 도면과 시험 결과가 필요하다.

#### PCB에서 발표 전 채워야 할 값

제공 자료에는 PCB 원본이 없으므로 담당자는 아래를 자신의 회로도·BOM으로 채워야 한다.

| 질문 | 팀의 실제 값 |
|---|---|
| Board1 MCU와 펌웨어 버전 | ______ |
| CAN controller·transceiver·oscillator | ______ |
| 입력전압과 각 전원 rail | ______ |
| 모터 드라이버 인터페이스 | ______ |
| fuse·역극성·TVS·과전류 보호 | ______ |
| 120 Ω 종단저항 위치와 on/off 방식 | ______ |
| PCB 층수·크기·revision | ______ |
| connector·pinout·test point | ______ |
| 최대·평균 전류와 발열 시험 | ______ |
| 실제 E-stop 때 driver enable 상태 | ______ |

### 6.3 예상 질문과 답변

#### 기초

**Q. 로봇팔은 몇 축인가요?**

“팔 자체는 base 회전을 포함한 5축이고, 말단의 그리퍼는 세 손가락에 세 관절씩 별도 9축입니다.”

**Q. 왜 전원을 켤 때 homing을 하나요?**

“확인된 Board2는 절대 encoder가 없는 스테퍼 방식이라 전원을 켠 순간의 절대 각도를 모릅니다. limit switch를 찾아 기준 좌표를 정한 뒤에야 명령 각도를 의미 있게 사용할 수 있습니다.”

**Q. 관절이 범위를 넘어가는 것은 어떻게 막나요?**

“GUI 입력, 중앙 브리지, 보드 프로토콜에서 관절 범위를 검사합니다. 다만 소프트웨어 제한과 별개로 실제 hard stop과 limit switch 배치는 기구 안전 설계로 확인해야 합니다.”

**Q. 왜 5축으로 설계했나요?**

“이번 시나리오는 정해진 접근 방향과 버튼·물체 위치를 대상으로 하므로 필요한 작업 범위와 무게·복잡도 사이에서 5축을 선택했습니다. 대신 6축처럼 임의 자세를 모두 만들 수 없다는 제한이 있습니다.”

#### 중급

**Q. 다섯 관절이 동시에 움직이나요?**

“Board1의 네 축은 목표를 모두 staging한 뒤 START로 같이 시작합니다. Board2는 legacy 방식이라 그 직전에 명령을 보내므로 시작 오차를 줄였지만 5축 전체의 완전한 하드웨어 동기 방식은 아닙니다.”

**Q. duration을 주면 정확히 그 시간에 도착하나요?**

“목표 시간으로 사용하지만 모터 최대 속도보다 빠른 명령은 수행할 수 없습니다. Board2는 속도 제한에 맞춰 실제 시간을 늘리며, legacy duration은 5 ms 단위 최대 1,275 ms라는 표현 한계도 있습니다.”

**Q. encoder가 없는데 현재 각도는 어떻게 표시하나요?**

“확인된 Board2는 명령한 step 누적값을 위치로 추정합니다. 중앙 /joint_states는 실제 feedback frame이 있으면 그것을 우선하고 없을 때 명령 추정값을 사용할 수 있습니다. 외력이나 탈조가 생기면 실제와 어긋날 수 있어 재homing이 필요합니다.”

**Q. 비상정지 때 팔이 축 늘어지나요?**

“Board2 코드에서는 queue와 진행 명령을 멈추되 enable을 유지해 holding torque를 남깁니다. 이는 전원을 끊는 STO가 아니라 소프트웨어 정지입니다. 다른 보드의 실제 출력 상태도 실기로 확인해야 합니다.”

**Q. 왜 보드를 둘로 나눴나요?**

“여러 축의 배선과 연산·전류 부담을 분산하고 기존 보드를 재사용하기 위한 구조입니다. 대신 Board1 V3와 Board2 legacy 프로토콜을 함께 조율해야 하는 복잡성이 생겼습니다.”

#### 심화

**Q. CAN 프레임 하나가 빠지면 일부 축만 먼저 움직이지 않나요?**

“Board1은 같은 goal ID와 duration의 네 축이 모두 들어와야 READY가 되고 별도 START를 받아야 움직입니다. 일부만 도착하면 실행하지 않습니다. 다만 Board2 legacy 명령은 같은 수준의 원자적 ACK가 없다는 한계가 있습니다.”

**Q. 이동 중 새 목표가 오면 어떻게 하나요?**

“호스트는 최신 목표 하나만 보류하는 정책을 쓰고 Board1에는 cancel을 보낼 수 있습니다. Board2는 선점 cancel을 확인할 수 없으므로 fresh IDLE, reached, queue free가 확인될 때까지 새 목표를 격리합니다.”

**Q. MoveIt 경로가 그대로 펌웨어로 전달되나요?**

“아닙니다. 현재 V3 실기 계약은 최종 관절 목표와 duration 중심입니다. MoveIt의 중간 waypoint 전체를 Board1·2가 추종하는 trajectory streaming 구조가 아니며, 제공 압축본은 그래서 마커 waypoint 실기 실행을 차단하고 있습니다.”

**Q. payload는 몇 kg인가요?**

“제공 코드에는 신뢰할 수 있는 실측 payload가 없습니다. 모터 토크, 감속비, 링크 무게중심, 최악 자세의 정적·동적 토크와 실제 시험 결과를 근거로 답해야 하며 추정값을 발표 수치로 말하면 안 됩니다.”

**Q. PCB 노이즈와 CAN 안정성은 어떻게 고려했나요?**

답변 틀: “CAN_H/L 차동 배선, 버스 양 끝 120 Ω 종단, 공통 접지와 전원·신호 경로 분리, TVS·디커플링을 적용했습니다. 실제 부품명과 배치, 검증 파형은 저희 회로도 revision ___와 측정 결과 ___를 기준으로 합니다.”

[실기 확인] 실제 설계에 적용된 항목만 남겨 말한다.

**Q. 가장 위험한 고장 모드는 무엇인가요?**

“스테퍼 탈조로 내부 위치와 실제 자세가 달라지는 경우, limit switch 고장 상태의 homing, 통신 단절 후 driver가 어떤 출력을 유지하는지가 중요합니다. 그래서 재homing 절차와 물리 전원 차단 수단이 필요합니다.”

---

## 7. 담당 2 — 로봇 그리퍼 기구설계·3D 모델링

### 7.1 한 문장 역할 설명

“서로 다른 형상의 물체를 안정적으로 감싸 잡고 버튼 작업 자세도 만들 수 있도록 3지 9관절 구조를 설계하고, CAD 형상을 URDF·충돌 모델·TCP로 연결한 파트입니다.”

### 7.2 반드시 알아야 할 큰 틀

#### 구조

- 손가락 3개가 palm 둘레에 약 120도 간격으로 배치된다.
- 각 손가락은 base, middle, tip의 3개 회전관절이다.
- 총 9개 관절을 각각 서보로 구동한다.
- 기계적으로 한 모터에 연동되는 underactuated 구조가 아니라, 9축을 독립 명령하고 load 접촉 판단으로 형상에 맞추는 방식이다.

#### 모델상 주요 값

| 항목 | 설계 모델 값 |
|---|---|
| finger base 관절 범위 | 약 ±70.3° |
| middle 관절 범위 | 약 -137.7° ~ 52.7° |
| tip 관절 범위 | 약 ±111.3° |
| middle 링크 관절 간 거리 | 약 63 mm |
| tip 형상 길이 | 약 58 mm |
| 손가락 base 배치 반경 | 약 47 mm |
| grasp TCP 높이 | palm 기준 약 134.3 mm |

모델 메타데이터상 finger 부품은 PLA+, gripper base는 steel로 적혀 있다. 실제 출력 재료, infill, 출력 방향과 조립품 질량은 담당자가 실물 기록으로 확인해야 한다.

#### grasp 동작

- open profile: 9축을 0도 부근으로 연다.
- object close: 먼저 세 손가락 middle을 약 -100도로 움직이고, 다음 단계에서 tip을 약 -70도로 감싼다.
- 물체 profile effort raw는 700, 단계당 약 1.2초다.
- button profile은 물체 파지와 다른 손가락 자세를 사용한다.
- grasp_tcp_link는 모델의 닫힌 자세를 기준으로 계산된 좌표이며 실물 측정값이라고 단정하면 안 된다.

#### 3D 모델이 소프트웨어에 주는 것

- visual mesh: RViz와 발표 시각화
- collision mesh와 joint limit: MoveIt 계획과 자기충돌 검사
- joint origin·axis: 순기구학과 TF
- grasp TCP·button contact TCP: 물체·버튼 목표 pose 계산
- 카메라 mount: 손목 카메라 관측을 base 좌표로 변환

[주의] 제공 자료에는 Fusion·STEP 같은 제조용 CAD 원본, 공차 도면, 체결부 상세, 출력 조건이 없다. URDF mesh만으로 제조 공차나 강도를 증명할 수 없다.

#### 발표 전 채워야 할 값

| 질문 | 팀의 실제 값 |
|---|---|
| CAD 도구와 최종 revision | ______ |
| 재질·infill·layer·출력 방향 | ______ |
| 체결 나사·insert·bearing 규격 | ______ |
| 허용 물체 크기 범위 | ______ |
| 실제 최대 파지 질량 | ______ |
| 반복 파지 성공률 | ______ |
| 손가락 끝 마찰재 | ______ |
| 간극·backlash·공차 | ______ |
| 파손 또는 변형 시험 | ______ |

### 7.3 예상 질문과 답변

#### 기초

**Q. 왜 손가락을 세 개로 만들었나요?**

“두 손가락보다 원통형이나 비정형 물체를 둘러싸는 접촉점을 늘릴 수 있고, 네 손가락보다 구조와 제어 복잡도를 줄이는 절충안입니다.”

**Q. 그리퍼는 몇 자유도인가요?**

“손가락마다 3관절이고 세 손가락이므로 총 9자유도입니다.”

**Q. 어떤 물체를 잡을 수 있나요?**

“설계 의도는 세 손가락이 단계적으로 감싸 다양한 형상을 잡는 것입니다. 다만 실제 크기·무게 범위는 코드가 아니라 팀의 실험 결과로 답해야 합니다.”

**Q. 버튼은 물체를 잡는 자세로 누르나요?**

“아닙니다. 버튼용 손가락 자세와 button contact TCP를 별도로 정의해 접촉점을 만들고, 팔이 pre-press·press·retreat 경로를 사용합니다.”

#### 중급

**Q. 물체 형상에 어떻게 적응하나요?**

“9축을 독립 명령할 수 있고 middle과 tip을 단계적으로 닫습니다. 펌웨어가 각 서보의 load 임계치를 감지하면 해당 방향을 잠그고 조금 back-off하기 때문에 손가락별 접촉 시점 차이에 대응합니다.”

**Q. 왜 한 번에 닫지 않고 두 단계로 닫나요?**

“먼저 middle 관절로 물체를 둘러싼 뒤 tip으로 추가 접촉하면 물체를 밀어내는 현상을 줄이고 감싸는 파지를 만들기 쉽습니다.”

**Q. URDF와 실제 CAD가 왜 정확히 맞아야 하나요?**

“joint origin이나 링크 길이가 틀리면 카메라에서 계산한 목표와 실제 손끝 위치가 어긋나고, collision model도 잘못된 판단을 합니다. 그래서 CAD 기준 좌표와 실물 조립 오차를 함께 보정해야 합니다.”

**Q. TCP는 무엇인가요?**

“로봇이 작업 기준으로 삼는 말단 좌표입니다. 이 프로젝트는 일반 파지용 grasp TCP와 버튼 접촉용 TCP를 분리해 사용합니다.”

**Q. 출력 방향이 왜 중요한가요?**

“FDM 부품은 layer 방향에 따라 강도가 크게 달라집니다. 손가락이 굽힘을 받을 때 layer 분리가 일어나지 않도록 하중 방향, fillet, 체결부, infill을 같이 설계해야 합니다.”

#### 심화

**Q. 이 그리퍼는 underactuated adaptive gripper인가요?**

“아닙니다. 기계적으로 관절이 수동 연동되는 underactuated 구조가 아니라 9개의 서보를 독립 구동합니다. 적응성은 명령 profile과 load 기반 접촉 처리에서 나옵니다.”

**Q. 파지력을 N 단위로 말할 수 있나요?**

“현재 펌웨어의 load는 서보 내부 raw 추정값입니다. 손끝 힘 N으로 변환하려면 관절 토크 상수, 링크 기구학, 접촉 위치와 로드셀 보정이 필요하므로 실험 없이 N 단위로 답할 수 없습니다.”

**Q. 손가락 강도는 어떻게 검증했나요?**

답변 틀: “최악 접촉점의 굽힘 모멘트와 재료·출력 방향을 고려해 설계했고, ___ kg 반복 파지 ___회에서 변형 ___ mm를 측정했습니다.”

[실기 확인] 계산서나 시험값이 없다면 “현재는 기능 검증 단계이고 정량 강도 검증이 후속 과제”라고 답한다.

**Q. 9축이면 배선 간섭이 크지 않나요?**

“자유도와 형상 적응성이 늘지만 서보 배선, 관절 가동범위, 유지보수 복잡도도 커집니다. 그래서 joint limit과 cable routing, strain relief가 기구 설계의 핵심입니다.”

**Q. 설계에서 가장 큰 오차원은 무엇인가요?**

“3D 출력 공차, 서보 spline 조립각, 링크 유격, TCP 모델 오차가 누적됩니다. software joint zero만 맞추는 것과 실제 손끝 위치를 보정하는 것은 별도 작업입니다.”

---

## 8. 담당 3 — 제어보드 CAN 통신·GUI

### 8.1 한 문장 역할 설명

“ROS 관절 명령을 검증해 SocketCAN 프레임으로 변환하고 세 보드의 상태·위치·오류를 다시 ROS와 GUI에 연결한 통신·감독 파트입니다.”

### 8.2 반드시 알아야 할 큰 틀

#### 통신 흐름

    Web GUI
      → ROS Service 또는 Action
      → arm_can_bridge
      → 단일 CAN writer
      → can0
      → Board1, Board2, Board3
      → 상태·피드백·ACK
      → bridge 상태 조립
      → GUI

- Classic CAN, 표준 11-bit ID, 500 kbps
- 여러 ROS callback이 동시에 프레임을 쓰지 않도록 하나의 writer가 전송 순서를 직렬화한다.
- extended, RTR, error frame은 정상 제어 데이터로 해석하지 않는다.
- 형식 검증을 통과한 최신 상태만 freshness 판단에 사용한다.

#### 주요 CAN ID

| ID | 의미 |
|---:|---|
| 0x001 | 공통 ESTOP |
| 0x010 | enable / disable |
| 0x020 | 팔 homing |
| 0x023 | 그리퍼 homing |
| 0x030 | 오류 clear |
| 0x040 | Board1 V3 START / CANCEL |
| 0x101 / 0x102 / 0x103 | Board1 / Board2 / Board3 위치 명령 |
| 0x201 / 0x202 / 0x203 | 보드 상태 |
| 0x301 / 0x302 / 0x303 | 위치 feedback |
| 0x401 | Board1 V3 ACK |

#### Board1 V3 원자적 시작

1. 같은 goal ID와 duration으로 0x101 네 프레임을 전송한다.
2. received mask가 완성된 READY ACK를 기다린다.
3. 취소 확인이 어려운 Board2 legacy 목표를 전송한다.
4. Board1 START를 보낸다.
5. STARTED ACK와 Board1·2의 완료 상태를 함께 확인한다.

ACK에는 READY, STARTED, DUPLICATE, BUSY, STAGING_TIMEOUT, CONFLICT, CANCELLED, INVALID 같은 결과가 있다. ACK가 유실되면 같은 goal ID의 전체 목표를 다시 보낼 수 있고, 보드는 DUPLICATE로 중복 실행을 막는다.

#### 완료 판정

단순히 duration이 지났다고 성공으로 처리하지 않는다.

- Board1·2가 fresh 상태
- IDLE, error 0, moving false
- 목표 축 reached
- Board1 goal slot free
- Board2 queue free

#### GUI에서 담당 파트가 봐야 할 것

- enable, disable, home, clear, status, arm ESTOP
- 팔 5축·그리퍼 9축 수동 명령
- 상태·error·heartbeat freshness·queue·joint feedback
- 저장 pose와 수동 sequence
- mission, direct nav, sequence, manual motion의 상호 배제
- YAML의 동일 joint limit을 불러와 입력 검증

GUI는 상위 감독 계층이며 hard real-time controller가 아니다. 수동 팔 제어도 최종 joint goal 직접 전송이므로 MoveIt collision-free path를 보장하지 않는다.

### 8.3 예상 질문과 답변

#### 기초

**Q. 왜 CAN을 사용했나요?**

“여러 보드가 두 신호선 버스를 공유할 수 있고 ID 우선순위, CRC, 오류 검출을 제공해 모터 주변처럼 노이즈가 있는 분산 제어에 적합하기 때문입니다.”

**Q. 통신 속도와 형식은 무엇인가요?**

“500 kbps의 Classic CAN이고 표준 11-bit ID를 사용합니다.”

**Q. GUI가 모터를 직접 돌리나요?**

“아닙니다. GUI는 ROS 명령을 보내고 상태를 표시합니다. 브리지가 검증과 CAN 변환을 하고 실제 모터 동작은 보드 펌웨어가 수행합니다.”

**Q. 통신이 끊긴 것은 어떻게 알 수 있나요?**

“상태 프레임의 마지막 유효 수신 시간을 기록해 freshness를 표시하고, stale 상태에서는 새 움직임을 허용하지 않습니다.”

#### 중급

**Q. 왜 Board1 목표를 네 프레임으로 나누나요?**

“Classic CAN payload가 8 byte이므로 네 축 목표를 한 프레임에 담기 어렵습니다. 대신 네 프레임을 모두 staging한 뒤 START해 부분 수신 상태의 실행을 막았습니다.”

**Q. 오래된 ACK가 새 목표를 성공시킬 수 있나요?**

“goal ID, protocol version, duration, received mask와 보드 정보를 현재 목표와 대조하므로 단순히 마지막 ACK라는 이유로 신뢰하지 않습니다.”

**Q. 여러 사용자가 동시에 명령하면 어떻게 하나요?**

“GUI가 mission·sequence·manual 간 동시 실행을 막고 브리지도 활성 motion을 직렬화합니다. 다만 외부 CLI처럼 GUI 밖의 클라이언트까지 GUI lock이 완전히 소유하는 것은 아니므로 데모는 한 GUI에서 조작해야 합니다.”

**Q. callback에서 바로 CAN을 쓰면 안 되나요?**

“동시에 실행되는 callback이 프레임 순서를 섞으면 Board1 staging 사이에 다른 명령이 끼어들 수 있습니다. 단일 writer로 순서와 ESTOP 우선 처리를 보장합니다.”

**Q. Board2에는 ACK가 없는데 성공을 어떻게 압니까?**

“SocketCAN write와 이후 fresh 상태·위치 feedback으로 간접 확인합니다. 명령 수신 자체의 명시적 ACK가 없다는 것은 남는 한계입니다.”

#### 심화

**Q. BUSY면 재시도하면 되지 않나요?**

“다른 goal을 보유한 상태일 수 있어 무작정 cancel이나 재전송하면 다른 작업을 침범할 수 있습니다. 현재는 즉시 실패시키고 상태와 raw ACK를 진단하는 보수적 정책입니다.”

**Q. ESTOP 프레임이 일반 명령 뒤에서 기다리나요?**

“브리지 writer의 우선 경로로 보내고 아직 송신되지 않은 motion·START·CANCEL을 제거합니다. 다만 실제 전원 차단과 각 보드의 물리 출력 정지는 펌웨어·회로가 별도로 보장해야 합니다.”

**Q. CAN 종단은 어떻게 확인하나요?**

“버스 물리 양 끝에 120 Ω씩 두고 전원을 끈 상태에서 CAN_H와 CAN_L 사이가 약 60 Ω인지 확인합니다. 단, 실제 PCB의 종단 위치와 jumper 구성은 담당 회로도로 답해야 합니다.”

**Q. 버스 부하는 충분한가요?**

“500 kbps에서 상태·feedback 주기와 명령 burst를 계산해야 정확히 답할 수 있습니다. 평균 부하뿐 아니라 9축 gripper staging과 0x303 세 프레임 feedback이 겹치는 최악 burst, arbitration 지연, error frame을 측정해야 합니다.”

**Q. GUI 검증을 우회하면 위험한 값을 보낼 수 있지 않나요?**

“그래서 GUI만 믿지 않고 브리지에서도 정확한 joint set, 중복 이름, finite 값, 범위, duration, enable·home·idle·freshness를 다시 검사합니다.”

---

## 9. 담당 4 — 로봇 그리퍼 전장설계·펌웨어

### 9.1 한 문장 역할 설명

“중앙 CAN 명령을 STM32에서 받아 9개의 직렬 서보를 동기 구동하고, load 기반 접촉 감지와 상태·오류 feedback을 구현한 파트입니다.”

### 9.2 반드시 알아야 할 큰 틀

#### 코드에서 확인한 전장·통신 구성

- MCU: STM32F411RE
- CAN controller: MCP2515, 8 MHz, CAN 500 kbps
- MCP2515 SPI2: PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS
- interrupt: PB4 active-low
- debug USART2: 115200 bps
- servo bus: USART1 half-duplex, PA9 단선, 1 Mbps
- source 주석상 외부 4.7~10 kΩ pull-up 필요
- Feetech SCS0009 계열 직렬 서보 9개
- logical motor 0~8 → servo ID 1~9

[실기 확인] servo 정격전압, 9개 동시 peak 전류, 전원공급기, fuse, wire gauge, UART level shifting, transceiver·TVS·termination은 회로도와 BOM이 필요하다.

#### 9축 명령 staging

- 0x103 프레임 9개가 중복 없이 모두 도착해야 실행한다.
- 모든 frame의 duration이 같아야 한다.
- staging timeout은 실제 코드 기준 100 ms다.
- 완성된 목표는 Feetech Sync Write로 9개 서보에 전달한다.
- servo angle scale은 약 300°/1024 step이다.
- 각 servo별 home offset이 있고 허용 step은 20~1000이다.

#### load 기반 접촉 처리

1. 서보의 position, speed, load를 읽는다.
2. load raw가 목표 threshold 이상이면 접촉으로 판단한다.
3. 접촉 방향을 기록하고 약 3 ms 뒤 5 step을 back-off한다.
4. 같은 방향의 추가 조임은 잠그고 반대 방향의 열기는 허용한다.

이것은 정확한 N 단위 force control이 아니라 서보 내부 load 추정값을 이용한 threshold 보호 제어다.

#### 완료·오류

- 목표 허용오차: 30 servo step, 이론상 약 8.8°
- 요청 duration + 1.5초 후에도 미도달이면 timeout 판단
- 접촉 증거가 있으면 CONTACT_HOLD
- 접촉 없이 미도달이면 SERVO_FAULT
- 한 servo가 연속 50회 통신 실패하면 SERVO_COMM
- 상태 0x203, 위치 0x303 세 그룹으로 feedback
- 호스트는 50 ms 안에 모인 세 그룹만 coherent 9축 snapshot으로 사용한다.

#### 중요한 안전 한계

[주의] emergency_stop과 disable 함수는 firmware state와 command를 정리하지만 Feetech torque-disable 또는 즉시 freeze packet을 전송하는 코드가 확인되지 않는다. 따라서 “그리퍼 E-stop이면 물리 torque가 확실히 꺼진다”고 말하면 안 된다.

### 9.3 예상 질문과 답변

#### 기초

**Q. 그리퍼 모터는 몇 개인가요?**

“세 손가락에 세 관절씩 총 9개의 직렬 서보를 사용합니다.”

**Q. 9개를 어떻게 동시에 움직이나요?**

“CAN으로 9개 목표를 모두 staging한 뒤 하나의 Feetech Sync Write 패킷으로 전달해 시작 시점 차이를 줄입니다.”

**Q. 물체를 잡았다는 것을 어떻게 아나요?**

“각 서보가 제공하는 load raw가 설정 threshold를 넘는지 보고 접촉을 판단합니다.”

**Q. 너무 세게 조이지 않나요?**

“접촉 threshold에 도달하면 5 step 정도 뒤로 물러나고 같은 방향 추가 조임을 막습니다. 다만 정확한 힘 제어가 아니라 보호용 threshold 방식입니다.”

#### 중급

**Q. 9개 중 한 frame이 빠지면 어떻게 되나요?**

“실행하지 않습니다. 100 ms 안에 9개 motor가 중복 없이 모두 도착하고 duration도 같아야 commit합니다.”

**Q. load 700은 몇 N인가요?**

“0~1023 범위의 서보 내부 raw 추정값이므로 바로 N으로 변환할 수 없습니다. 실제 손끝 힘은 로드셀을 이용해 관절·자세별로 보정해야 합니다.”

**Q. profile마다 load가 같은가요?**

“아닙니다. open profile은 낮은 값, object close는 700, button profile은 다른 effort를 사용할 수 있고 GUI 수동 기본값도 별도입니다. 펌웨어 내부 700은 CAN load가 0일 때만 쓰는 fallback입니다.”

**Q. 서보 하나가 응답하지 않으면 나머지만 계속 움직이나요?**

“일시 오류는 누적하지만 같은 모터가 연속 50회 실패하면 전체 hold를 시도하고 SERVO_COMM 오류로 전환합니다.”

**Q. 위치 정밀도는 0.293도인가요?**

“그 값은 이론상 1 step 분해능입니다. 현재 완료 허용오차가 30 step, 약 8.8도이고 기어 유격·조립 오차도 있으므로 실제 반복정밀도와는 다릅니다.”

#### 심화

**Q. 왜 CAN과 servo UART를 나눴나요?**

“상위 로봇 네트워크는 여러 보드가 공유하는 CAN으로 통일하고, Board3 내부에서는 서보가 지원하는 1 Mbps half-duplex protocol을 사용해 역할을 분리했습니다.”

**Q. half-duplex 충돌은 어떻게 막나요?**

“송신 시 TX 모드로 패킷을 보낸 뒤 RX로 전환하고 timeout·overrun을 처리합니다. 실제 파형과 pull-up 전압, level 호환은 오실로스코프로 검증해야 합니다.”

**Q. 응답 데이터 무결성은 충분한가요?**

“현재 parser는 header와 ID·error를 확인하지만 checksum과 length 검증이 충분하지 않은 경로가 있어 개선 여지가 있습니다. 오류 주입과 checksum 검사를 강화할 수 있습니다.”

**Q. ESTOP 때 torque가 꺼지나요?**

“제공 코드만으로는 보장할 수 없습니다. MCU는 새 명령을 막지만 servo torque-off packet이 확인되지 않으므로 실기 검증 전에는 전원 차단형 E-stop이라고 표현하지 않습니다.”

**Q. 9개 서보 전원은 어떻게 산정했나요?**

답변 틀: “서보 한 개의 stall·기동 전류와 동시 동작 계수를 기준으로 peak ___ A, continuous ___ A를 산정하고 fuse ___ A, 배선 ___ AWG, 전원 ___ V ___ A를 사용했습니다.”

[실기 확인] 실제 BOM과 측정값으로 빈칸을 채운 뒤 답한다.

---

## 10. 담당 5 — 중앙 서버·GUI

### 10.1 한 문장 역할 설명

“29단계 전체 미션을 순서대로 조율하고 각 하위 Action의 feedback·timeout·retry·cancel을 관리하며, 브라우저에서 주행·팔·보드·로그 상태를 통합 확인하도록 만든 파트입니다.”

### 10.2 반드시 알아야 할 큰 틀

#### 중앙 서버의 역할

Mission Manager는 직접 모터를 제어하지 않는다. 세 YAML을 결합해 실행 단계를 만들고 전문 Action 서버에 위임한다.

- mission_flow.yaml: 29단계 순서와 단계별 파라미터
- locations.yaml: 층, 좌표, marker ID
- action_servers.yaml: 서버 이름, timeout, retry

최종 시나리오는 범용 배송 입력을 모두 받는 형태가 아니라 안전하게 제한한 4층 402 → 5층 object_place → 4층 402, object_1 미션이다. 입력이 이 고정 범위를 벗어나면 Goal을 거절한다.

#### 최종 중앙 launch 구성

- 로봇 모델과 MoveIt move_group
- arm_can_bridge
- 최종 팔 서버 roscue_arm_pick/task_executor_node
- 손목 RealSense와 ArUco detector
- 유일한 /nav/go_to 어댑터
- Mission Manager와 ready_and_approach coordinator
- Web GUI

[주의] arm_task_server는 deprecated 호환 코드다. 최종 팔 서버와 동시에 실행하면 /arm/* 이름이 충돌할 수 있다.

기본값은 can0, execution_mode=hardware, GUI 0.0.0.0:8080, auto_port=false, RViz off다. 8080을 다른 프로세스가 사용하면 자동으로 다른 포트를 찾지 않고 GUI 시작이 실패한다.

#### Goal 수락과 실행

1. GUI가 /api/mission/start를 받는다.
2. ExecuteMission Goal을 /mission/execute로 보낸다.
3. 중앙이 입력 제한, 중복 미션, 설정과 직접 사용하는 12개 Action 서버를 검사한다.
4. 단계마다 RunTask Goal을 보내고 feedback을 전체 progress로 바꾼다.
5. 실패하면 해당 retry만 수행한 뒤 중단한다.
6. MissionStatus, feedback, result, event log를 GUI에 제공한다.

[주의] ready_and_approach 내부에서 쓰는 /base/drive_straight는 직접 사전검사 12개 목록 밖이다. coordinator가 실행 시 최대 5초 기다린다.

#### GUI 구조와 기능

- 브라우저는 ROS 2와 직접 연결하지 않고 Flask HTTP/JSON API를 사용한다.
- 약 0.7초 주기로 snapshot을 polling한다.
- 지도, AMCL, odometry, global·local path, 미션 FSM과 진행률
- 보드 상태, fault, joint feedback, heartbeat
- mission start·cancel, direct navigation, initial pose
- enable·disable·home·clear·arm ESTOP
- 수동 arm·gripper, 저장 pose CRUD, sequence 실행
- 이벤트 로그와 연결 상태

기본 event log는 ~/.ros/vicpinky_gui/event_log.sqlite3, 저장 pose는 ~/.ros/vicpinky_gui/saved_poses.json이다. 저장 pose는 임시 파일·fsync·원자적 replace를 사용한다.

#### 동시 명령 차단

GUI는 mission, direct nav, saved pose sequence, manual motion이 서로 겹치지 않게 차단한다. 하지만 CLI나 다른 클라이언트가 만든 Goal Handle까지 GUI가 소유하는 것은 아니다. 터미널에서 시작한 미션이나 GUI 프로세스 재시작 전 미션은 GUI Cancel Handle이 없을 수 있다.

#### 화면 표시를 과대해석하면 안 되는 항목

- Robot Link ONLINE: Mission Manager heartbeat가 보인다는 뜻이지 CAN·카메라·Nav2까지 모두 정상이라는 뜻은 아니다.
- Mission success: 하위 Action이 success를 보고했다는 뜻이다. 실제 버튼 lamp나 접점 반응을 독립 센서로 검증한 것은 아니다.
- map applied 표시: 새 map message를 이용한 GUI 추론이며 map checksum 검증은 아니다.
- progress 50%: 단계 비율이지 시간 비율이 아니다.
- GUI ESTOP: 팔 보드 대상이며 베이스 전체 E-stop이 아니다.

### 10.3 예상 질문과 답변

#### 기초

**Q. 중앙 서버가 하는 일은 무엇인가요?**

“팔, 주행, 정렬, 엘리베이터 작업의 실행 순서를 정하고 각 작업의 성공·실패·진행률·timeout·cancel을 관리합니다.”

**Q. 미션 순서는 코드에 고정돼 있나요?**

“실행 엔진과 시나리오를 분리했습니다. 순서는 mission_flow, 위치는 locations, timeout·retry는 action_servers YAML에 정의합니다. 다만 최종 데모 입력은 검증 범위인 402와 5층 object_place로 제한했습니다.”

**Q. 브라우저가 ROS 2와 직접 통신하나요?**

“아닙니다. Flask 기반 GUI 노드가 HTTP 요청을 ROS Action·Service로 바꾸고 ROS Topic 상태를 JSON snapshot으로 전달합니다.”

**Q. GUI에서 무엇을 볼 수 있나요?**

“미션 FSM과 진행률, 지도와 위치·경로, 팔 보드와 관절 상태, 연결 heartbeat, event log, 수동 제어 상태를 볼 수 있습니다.”

#### 중급

**Q. 하위 노드가 꺼져 있으면 어떻게 하나요?**

“Goal을 수락하기 전에 최종 미션에서 직접 쓰는 12개 Action 서버의 준비 상태를 검사합니다. 단, server ready는 센서 데이터와 실제 하드웨어까지 정상이라는 뜻은 아닙니다.”

**Q. 팔과 베이스를 어떻게 동시에 움직이나요?**

“ready_and_approach coordinator가 팔 ready Goal을 먼저 보내고 수락 시점에서 2초 뒤 27 cm 직진 Goal을 시작합니다. 둘 다 성공해야 다음 단계로 가고 한쪽이 실패하면 다른 쪽도 취소합니다.”

**Q. Cancel은 하위 작업까지 전달되나요?**

“중앙 실행기가 현재 child Goal에 cancel을 전달하고 늦게 수락된 Goal도 추적해 취소합니다. 실제 모터 정지는 각 하위 서버와 펌웨어가 cancel을 어떻게 처리하는지에 달려 있습니다.”

**Q. 실패한 지점부터 다시 시작할 수 있나요?**

“현재 checkpoint resume은 없습니다. 실패 원인을 기록하고 중단한 뒤 물체·팔·층·map·AMCL 상태를 초기 조건으로 복구하고 처음부터 새 미션을 시작합니다.”

**Q. 왜 팔이나 버튼 작업은 자동 retry하지 않나요?**

“물체를 이미 집었거나 버튼을 이미 눌렀을 수 있어 물리 동작을 무조건 반복하면 더 위험합니다. 현재 자동 retry는 Nav, 정렬, 층 확인, 맵 전환에만 한 번 설정했습니다.”

**Q. GUI가 꺼지면 미션도 멈추나요?**

“브라우저만 닫혀도 Mission Manager가 살아 있으면 미션은 계속될 수 있습니다. GUI ROS 프로세스가 재시작되면 기존 Goal Handle을 잃어 GUI에서 cancel하지 못할 수 있습니다.”

#### 심화

**Q. extra_json을 쓴 이유와 단점은 무엇인가요?**

“하나의 RunTask로 pose, 거리, 속도, 버튼 역할, map 같은 서로 다른 확장값을 전달하기 위해서입니다. 인터페이스 확장은 쉽지만 정적 타입 안전성이 약해 서버별 런타임 검증이 필수입니다.”

**Q. Action Goal 전송 timeout 직후 늦게 수락되면 orphan motion이 생기지 않나요?**

“future done callback으로 늦게 들어온 Goal Handle도 받아 즉시 cancel하도록 정리 로직을 두었습니다.”

**Q. MultiThreadedExecutor가 필요한 이유는 무엇인가요?**

“execute callback이 하위 결과를 기다리는 동안에도 feedback, result, heartbeat와 cancel callback이 계속 처리돼야 하기 때문입니다.”

**Q. 두 미션이 동시에 시작될 수 있나요?**

“Mission Manager가 lock과 mission_active 상태로 두 번째 Goal을 거절하고 GUI도 충돌하는 명령을 막습니다. 외부 클라이언트까지 포함한 운영 규칙은 별도로 한 곳에서 조작하도록 정해야 합니다.”

**Q. GUI에 보안 기능이 있나요?**

“제공 코드에는 인증, 권한 분리, TLS가 보이지 않고 0.0.0.0에 bind합니다. 따라서 신뢰 가능한 폐쇄 실험망에서만 사용해야 합니다.”

**Q. 중앙 서버의 가장 큰 한계는 무엇인가요?**

“메모리 기반 실행 상태라 프로세스 재시작 후 자동 resume이 없고, 하위 Action의 success 계약을 신뢰합니다. 또한 YAML의 map 전환 후 3초 delay가 제공 압축본의 nav adapter에서 소비되지 않는 설정 불일치가 있습니다.”

---

## 11. 담당 6 — 자율주행·엘리베이터 정렬 제어

### 11.1 한 문장 역할 설명

“LiDAR·encoder 기반 Nav2로 층내 장거리 주행을 수행하고, 엘리베이터 앞에서는 ID 20 상대 pose를 이용해 버튼 조작이 가능한 위치까지 정밀 정렬한 파트입니다.”

### 11.2 반드시 알아야 할 큰 틀

#### 베이스

- 차동구동
- wheel radius 0.0825 m
- wheel base 0.475 m
- encoder 4096 pulse/rev
- ZLAC 계열 controller, /dev/motor, Modbus RTU 115200 bps
- 좌우 RPM 제한 ±28, 이론상 최고 선속도 약 0.242 m/s
- /odom과 odom → base_footprint TF 약 30 Hz

차동구동 역기구학은 다음 개념이다.

    left velocity  = v - ωL/2
    right velocity = v + ωL/2

이를 바퀴 반지름으로 나눠 각속도와 RPM으로 바꾼다.

#### Nav2 층내 주행

- 사전 제작 occupancy map
- AMCL: LiDAR와 map으로 위치 추정
- odometry: wheel encoder
- global planner: NavFn
- local controller: DWB
- control frequency: 10 Hz
- max x velocity 0.25 m/s
- max angular velocity 0.5 rad/s
- goal tolerance: xy 0.25 m, yaw 0.1 rad
- local costmap 약 5 m × 5 m, 0.05 m resolution
- LiDAR /scan_filtered를 장애물 costmap에 사용

IMU 융합 코드는 확인되지 않았다. wheel slip은 장거리에서는 AMCL이 일부 보정하지만 짧은 odometry 직진·회전에는 영향을 준다.

#### ID 20 정렬

- 전방 camera /front_camera/image_raw
- DICT_4X4_50, ID 20, marker size 0.10 m
- camera 기준 lateral tvec.x와 depth tvec.z 사용
- target distance 1.27 m, lateral 0 m
- tolerance: lateral ±0.07 m, distance ±0.08 m
- 범위 안에서 연속 3초 유지해야 success
- linear P gain 0.18, angular P gain 0.65
- 최대 v 0.045 m/s, 최대 ω 0.18 rad/s
- lateral error가 0.18 m보다 크면 전진하지 않고 회전부터 한다.
- pose가 1.2초 이상 stale이면 정렬 완료 timer를 지우고 -0.12 rad/s 탐색 회전
- 내부 timeout 90초, 중앙 timeout 120초와 retry 1회

yaw error를 계산·발행하지만 현재 controller는 x와 z만 사용한다. 완전한 3D pose docking이 아니라 평면상 측면·거리 정렬이다. dock용 detector는 카메라 왜곡계수를 0으로 둔다는 한계도 있다.

#### 정렬 뒤 동작

- 호출 버튼 작업
- 팔 ready Goal 수락 2초 뒤 0.27 m를 0.15 m/s로 직진
- odometry 기준 남은 거리 3 cm 이내에서 종료
- odom이 0.5초 이상 stale이면 정지·실패
- 좌 80도 회전, odom yaw 기준 tolerance 3도

#### 안전 경계

Nav2 구간에는 costmap 장애물 회피가 있다. 그러나 dock align, 27 cm 직진, 회전, 승하차는 /cmd_vel을 직접 발행하며 Twist mux·collision monitor가 확인되지 않았다. base motor node에 명시적 /cmd_vel watchdog도 보이지 않는다.

### 11.3 예상 질문과 답변

#### 기초

**Q. 자율주행은 어떤 방식인가요?**

“LiDAR와 encoder odometry를 이용해 AMCL로 위치를 추정하고 Nav2가 지도 위 경로를 계획·추종합니다.”

**Q. 어떤 센서를 사용하나요?**

“LiDAR는 localization과 장애물 costmap·문 확인, encoder는 odometry, 전방 camera는 엘리베이터 앞과 승차 마커, 후방 camera는 층 확인과 하차에 사용합니다.”

**Q. 왜 Nav2만으로 버튼 앞까지 가지 않나요?**

“현재 Nav2 xy goal tolerance가 25 cm라 복도 이동에는 충분하지만 팔이 버튼을 누르는 기준으로는 큽니다. 그래서 근처까지 Nav2로 간 뒤 ID 20 상대좌표로 정밀 정렬합니다.”

**Q. 로봇은 어떻게 좌우로 회전하나요?**

“차동구동이므로 좌우 바퀴 속도 차이를 만들어 회전합니다. 선속도와 각속도를 wheel base와 radius를 이용해 양쪽 RPM으로 변환합니다.”

#### 중급

**Q. 마커가 한 프레임 보이면 정렬 완료인가요?**

“아닙니다. lateral ±7 cm, 거리 ±8 cm 범위 안에서 연속 3초 있어야 완료합니다.”

**Q. 마커를 잃으면 계속 전진하나요?**

“아닙니다. pose가 1.2초보다 오래되면 사용하지 않고 천천히 탐색 회전합니다. 90초 안에 정렬하지 못하면 zero Twist 후 실패합니다.”

**Q. 27 cm를 정확히 간다고 할 수 있나요?**

“명령 목표는 27 cm이고 encoder odometry상 남은 거리가 3 cm 이내면 종료합니다. 실제 바닥 기준 오차는 wheel slip과 calibration의 영향을 받으므로 실측 없이 정확도 수치를 단정하지 않습니다.”

**Q. 장애물은 어떻게 피하나요?**

“Nav2 구간에서는 LiDAR costmap과 DWB를 사용합니다. 직접 /cmd_vel 구간에는 같은 장애물 회피가 없으므로 통제된 엘리베이터 구역과 별도 안전 감시가 필요합니다.”

**Q. Nav2가 실패하면 바로 끝나나요?**

“Nav2 behavior tree의 recovery가 먼저 동작하고, 중앙 /nav/go_to 설정에서 한 번 재시도합니다. 그래도 실패하면 미션을 중단합니다.”

#### 심화

**Q. wheel slip은 어떻게 보정하나요?**

“장거리에서는 AMCL이 LiDAR scan과 map을 이용해 encoder 누적오차를 보정합니다. 27 cm 직진과 80도 회전은 짧은 odometry 기준이라 slip 영향이 남고 IMU 보정은 현재 없습니다.”

**Q. ArUco yaw를 왜 제어에 사용하지 않나요?**

“현재 버튼 작업에 필요한 측면과 거리 오차를 우선 제어한 단순 P controller입니다. yaw는 관측용으로 발행하지만 법선각까지 독립 제어하는 docking은 후속 개선 항목입니다.”

**Q. 전역 좌표와 마커 좌표를 필터로 융합했나요?**

“하나의 filter로 동시에 융합하지 않고 state에 따라 controller를 전환합니다. Nav2가 전역 위치로 근처까지 간 뒤 camera 상대좌표만으로 마지막 정렬을 수행합니다.”

**Q. 왜 lateral error가 크면 전진을 막나요?**

“옆으로 많이 어긋난 채 전진하면 marker가 시야에서 벗어나거나 버튼 접근 자세가 더 나빠질 수 있어 먼저 heading을 바로잡기 위한 조건입니다.”

**Q. MultiThreadedExecutor가 제어에 왜 필요한가요?**

“Action execute loop가 도는 동안 camera, marker, scan, odom callback이 계속 갱신돼야 stale 판단과 폐루프 제어가 정상 동작하기 때문입니다.”

**Q. 현재 가장 큰 주행 안전 한계는 무엇인가요?**

“직접 Twist 구간의 collision monitor·mux와 base watchdog이 코드에서 확인되지 않고, scan filter가 0.55 m보다 가까운 값을 -inf로 제거해 가까운 외부 물체까지 사라질 가능성이 있어 현장 검증이 필요합니다.”

---

## 12. 담당 7 — 자율주행·층간 이동 구현

### 12.1 한 문장 역할 설명

“엘리베이터 문을 확인하고 전·후방 카메라 마커로 승차·층 확인·하차를 수행한 뒤, 목적층 map과 AMCL 위치를 전환해 다시 Nav2 주행으로 이어지게 만든 파트입니다.”

### 12.2 반드시 알아야 할 큰 틀

#### 층간 이동 전체 흐름

    Nav2로 elevator_front
      → ID 20 정렬
      → 호출 버튼
      → 27 cm 접근과 좌 80도
      → LiDAR 문 열림
      → 전방 ID 10으로 승차
      → 목적층 버튼
      → 후방 ID 4 또는 5로 층 확인
      → 같은 층 marker로 하차
      → 좌 90도
      → 해당 층 map load와 AMCL seed
      → Nav2 재개

복귀는 목표층 marker와 map만 5에서 4로 바뀌는 대칭 흐름이다.

#### 문 열림

- /elevator/wait_door_open
- /scan_filtered 정면 ±0.35 rad, 약 ±20도
- range 1.2 m 이상 beam이 한 scan에서 15개 이상이면 open
- Pinky 내부 timeout 30초, 중앙 timeout 60초

이후 ID 10 안정 검출을 다시 요구하므로 LiDAR의 열린 공간과 camera의 캐빈 기준을 이중 gate로 사용한다. 다만 scan timestamp freshness를 별도로 확인하지 않는다.

#### 승차

- 전방 camera, DICT_4X4_50, 캐빈 ID 10
- 15 frame 안정 검출
- solvePnP의 lateral·distance로 10 Hz visual servo
- 최종 목표 50 cm, tolerance ±1.5 cm
- 선속도 약 0.1~0.2 m/s, angular gain 0.5
- pose가 0.75초 이상 stale이면 즉시 zero Twist
- servo timeout 90초

15 frame은 camera FPS가 final launch에 고정되지 않아 정확히 몇 초라고 환산하면 안 된다.

#### 층 확인

- 탑승 후 전방 처리를 끄고 후방 camera를 사용
- 4층 ID 4, 5층 ID 5
- 15 frame 안정 검출 뒤 /tag/floor_id
- /floor/check가 expected floor와 같을 때만 success
- 중앙 timeout 150초, retry 1회

다른 층 ID가 보이면 바로 내리지 않고 기다린다. 다만 floor check는 마지막 floor 값의 timestamp freshness를 별도로 검사하지 않는 한계가 있다.

#### 하차

- /elevator/exit
- 목표층 ID 4·5를 다시 안정 검출
- 후방 marker를 보며 후진 visual servo
- 코드 기본은 60 cm지만 최종 mission YAML이 70 cm로 override
- 목표 도달 뒤 0.4 rad/s로 약 3.93초 좌회전해 90도

마지막 90도는 odometry feedback이 아니라 시간 기반이다. 하차 중 문 edge를 별도 센서로 연속 감시하는 collision monitor는 확인되지 않았다.

#### map switching

1. /map_server/load_map으로 4F 또는 5F map을 load한다.
2. 고정 landing pose를 /initialpose로 1초마다 발행한다.
3. AMCL xy가 seed에서 0.5 m 안인지 최대 10초 확인한다.
4. 내부에서 1.5초 안정화한다.
5. 다음 Nav Goal로 이어진다.

현재 수렴 판정은 xy만 보고 yaw error와 covariance를 확인하지 않는다. YAML의 추가 3초 delay는 제공 중앙 압축본에서 적용되지 않는다.

### 12.3 예상 질문과 답변

#### 기초

**Q. 엘리베이터 안에서도 Nav2를 사용하나요?**

“아닙니다. 좁은 캐빈은 층 map 좌표와 연결하기 어렵기 때문에 ID 10과 층 marker 상대좌표로 승하차합니다.”

**Q. 문이 열린 것은 어떻게 압니까?**

“정면 LiDAR에서 1.2 m 이상 열린 beam이 충분한지 확인하고, 다음 단계에서 캐빈 내부 ID 10이 안정적으로 보이는지도 확인합니다.”

**Q. 현재 층은 어떻게 구분하나요?**

“후방 camera가 landing의 ID 4 또는 ID 5를 15 frame 안정 검출하면 floor ID를 발행하고 목표층과 비교합니다.”

**Q. 층마다 좌표가 다른 문제는 어떻게 해결하나요?**

“실제 하차 후 그 층의 occupancy map을 load하고 landing의 고정 initial pose로 AMCL을 다시 초기화합니다.”

#### 중급

**Q. 잘못된 층에 도착하면 어떻게 되나요?**

“floor check가 expected floor와 일치하지 않아 다음 단계로 넘어가지 않고, exit Action도 목표층 marker를 다시 확인합니다.”

**Q. camera 영상이 끊기면 계속 움직이나요?**

“승하차 pose가 0.75초 이상 갱신되지 않으면 zero Twist로 정지하고 재검출을 기다립니다. 제한시간을 넘기면 실패합니다.”

**Q. 왜 전방과 후방 camera가 모두 필요한가요?**

“승차할 때는 앞쪽 캐빈 ID 10을 보고, 들어간 뒤에는 뒤쪽 landing의 층 marker와 하차 방향을 봐야 하기 때문입니다.”

**Q. map은 언제 바꾸나요?**

“물리적으로 엘리베이터에서 완전히 하차한 뒤 바꿉니다. 캐빈 안에서 먼저 바꾸면 실제 위치와 새 map의 initial pose가 맞지 않습니다.”

**Q. map 전환 성공은 무엇으로 판단하나요?**

“LoadMap 응답만 보지 않고 initial pose를 발행한 뒤 AMCL xy가 seed 0.5 m 안에 들어오는지 확인하고 내부 안정화 시간을 둡니다.”

#### 심화

**Q. 한 번의 marker 오검출로 잘못 내릴 수 있지 않나요?**

“detector에서 15 frame 안정 검출을 요구하고 floor check와 exit가 목표층을 두 번 확인합니다. 다만 마지막 floor 값 freshness 검사가 없다는 한계는 남아 있습니다.”

**Q. 고정 initial pose는 얼마나 정확한가요?**

“현장에서 정한 landing seed입니다. marker pose를 map 좌표로 동적으로 복원하는 방식은 아니므로 실제 하차 위치가 seed와 너무 다르면 AMCL이 불안정할 수 있습니다.”

**Q. 하차 중 문이 닫히면 어떻게 하나요?**

“marker pose가 stale이면 정지하지만 문 edge나 엘리베이터 안전신호를 별도로 연속 감시하는 코드는 확인되지 않았습니다. 통제된 데모 환경과 물리 안전요원이 필요합니다.”

**Q. 왜 하차 뒤 90도는 시간 기반인가요?**

“현재 exit 서버에 단순 자세 전환으로 통합한 구현입니다. 별도 odometry 기반 /base/rotate 서버로 교체하면 slip과 battery 변화에 더 견고하게 만들 수 있습니다.”

**Q. 엘리베이터 API와 연동하나요?**

“아닙니다. 실제 버튼을 팔로 누르고 LiDAR와 marker로 문·층을 판단합니다. 범용 상용 엘리베이터 연동보다는 사람과 유사한 물리 인터페이스를 사용한 방식입니다.”

**Q. map 전환 후 3초 기다리나요?**

“YAML에는 3초 의도가 적혀 있지만 제공 압축본의 nav adapter가 값을 읽지 않아 중앙 기준 추가 3초는 적용되지 않습니다. map switch 서버 내부의 1.5초 안정화만 확인됩니다.”

**Q. 이 파트의 가장 큰 한계는 무엇인가요?**

“마커 설치와 조명·시야에 의존하고, 문 안전신호와 직접 연동하지 않으며, 승하차가 Nav2 collision monitor 밖의 직접 Twist 제어라는 점입니다.”

---

## 13. 파트 사이 통합 질문

### Q1. 물체 marker를 본 뒤 실제 손가락까지 어떤 경로로 연결되나요?

“손목 camera가 ArUco pose를 검출하고 TF로 base 좌표계에 변환합니다. 팔 작업 서버가 물체별 offset을 적용해 접근·파지 pose를 만들고 MoveIt으로 계획한 뒤, 검증된 관절 명령은 CAN bridge를 통해 Board1·2로, 그리퍼 profile은 Board3로 전달하는 구조입니다. 다만 제공 압축본은 MoveIt waypoint를 Board1·2 실기로 넘기는 부분이 차단돼 있어 배포 빌드 확인이 필요합니다.”

### Q2. 버튼을 실제로 눌렀다는 것은 누가 확인하나요?

“팔 Action은 계획된 press·hold·retreat가 성공했는지 보고합니다. 버튼 lamp, 접점, 엘리베이터 controller 응답을 독립적으로 확인하는 센서는 없으므로 물리 버튼 수락까지 폐루프로 확인한 것은 아닙니다.”

### Q3. 중앙 PC와 Raspberry Pi를 왜 분리했나요?

“MoveIt, GUI, 미션 조율처럼 계산량과 운영 인터페이스가 큰 기능은 중앙 PC에 두고, 베이스·LiDAR·camera·Nav2처럼 로봇에 가까운 I/O와 제어는 Raspberry Pi에 두었습니다. ROS 2 네트워크로 Action과 Topic을 연결합니다.”

### Q4. 층내 주행과 층간 이동은 어디서 경계가 나뉘나요?

“층 map 안의 elevator_front까지는 Nav2입니다. 그 이후 ID 20 정렬, 문 확인, 승하차와 층 확인은 marker·LiDAR 기반 전문 Action입니다. 하차와 map switch가 끝나면 다시 Nav2로 돌아갑니다.”

### Q5. 팔 ready와 베이스 접근을 왜 겹쳐 실행하나요?

“엘리베이터 문 앞 준비 시간을 줄이고 정해진 순서를 맞추기 위해 팔 Goal을 먼저 수락한 뒤 2초 후 base를 출발시킵니다. 두 child가 모두 성공해야 회전하므로 단순 fire-and-forget은 아닙니다.”

### Q6. 모든 상태 feedback이 실제 센서값인가요?

“아닙니다. Nav와 camera는 센서 기반이고 Board3도 servo feedback을 읽지만, 확인된 Board2는 step 누적 기반 추정 위치입니다. GUI의 heartbeat나 Action success처럼 소프트웨어 상태를 나타내는 값도 있으므로 각각의 의미를 구분해야 합니다.”

### Q7. 취소 명령 한 번으로 전체가 즉시 멈추나요?

“Mission Manager는 현재 child Action에 cancel을 전파하지만 각 하위 서버와 펌웨어가 이를 처리하는 시간이 필요합니다. GUI arm ESTOP도 베이스 전체 전원을 끄지 않으므로 위험 상황에는 물리 E-stop이 우선입니다.”

### Q8. 왜 map을 엘리베이터 안에서 바꾸지 않나요?

“캐빈 안에서는 층 map상의 물리 위치가 아직 확정되지 않습니다. 목표층 landing으로 실제 하차한 뒤 그 위치에 맞는 seed로 map과 AMCL을 바꾸는 편이 좌표 불일치를 줄입니다.”

### Q9. 가장 어려웠던 통합 지점은 무엇이라고 답하면 좋나요?

“서로 다른 시간척도와 좌표계를 연결한 점입니다. 전역 map 좌표, camera 상대좌표, 팔 base 좌표, CAN raw 각도를 변환하면서도 한 단계 실패가 다음 물리 동작으로 번지지 않게 Action 상태와 timeout을 맞춰야 했습니다.”

### Q10. 프로젝트의 독창성은 무엇인가요?

“개별 기술 하나보다 이동, 물체 조작, 실제 버튼 인터페이스, 엘리베이터 승하차, 층별 localization을 하나의 관찰 가능한 ROS 2 미션으로 통합한 데 의미가 있습니다.”

---

## 14. 담당별 마지막 암기 카드

| 담당 | 반드시 외울 세 가지 | 스스로 먼저 말할 수 있어야 하는 한계 |
|---|---|---|
| 팔 펌웨어·기구·PCB | 팔 5축, Board1 4축+Board2 1축, 0.01° raw와 homing | Board1 원본·PCB·payload 자료 부재, waypoint 실기 경로 |
| 그리퍼 기구·3D | 3손가락×3관절, 약 120° 배치, grasp와 button TCP 분리 | 실제 강도·허용 물체·공차는 실측 필요 |
| CAN·GUI | CAN 500 kbps, 주요 ID, V3 staging→READY→START | Board2 ACK·cancel 부재, GUI는 안전 controller가 아님 |
| 그리퍼 전장·FW | STM32F411RE+MCP2515, servo bus 1 Mbps, 9축 Sync Write | load는 N이 아니며 ESTOP torque-off 미보장 |
| 중앙 서버·GUI | 29단계, 세 YAML 역할, Action feedback·timeout·cancel | 고정 시나리오, resume 없음, GUI ESTOP은 arm만 |
| 주행·정렬 | Nav2+AMCL, ID20 1.27 m/3초, 27 cm+좌80° | 직접 Twist 구간 collision monitor·watchdog 부재 |
| 층간 이동 | LiDAR gate, ID10 승차·ID4/5 층, 하차 후 map switch | door safety 연동 없음, 3초 delay 미적용, 하차 회전 시간 기반 |

---

## 15. 데모 전 실행 체크리스트

### 15.1 가장 먼저: 어떤 코드가 실제 배포됐는지 고정

- [ ] 데모 PC와 Pinky에 설치된 source·install이 제공 압축본과 같은지 확인
- [ ] Board1·Board2·Board3에 올라간 firmware commit 또는 빌드 날짜 기록
- [ ] final_system.launch.py와 final_robot.launch.py 조합인지 확인
- [ ] driving-only가 섞여 있지 않은지 확인
- [ ] workspace를 수정했다면 다시 build·source한 install이 맞는지 확인
- [ ] 모든 팀원이 최종 수치 1.27 m, 3초, 27 cm, 80°, 50 cm, 70 cm를 동일하게 사용

### 15.2 물리 안전

- [ ] 베이스와 팔의 물리 전원 차단 방법을 운영자 두 명 이상이 숙지
- [ ] 로봇 진행 경로와 팔 sweep 영역에서 관객을 분리
- [ ] 엘리베이터 문을 물리적으로 감시할 안전요원 지정
- [ ] 배터리, motor controller, servo 전원과 connector 확인
- [ ] CAN 종단과 공통 접지 확인
- [ ] 팔을 지지하거나 사람 손으로 움직이는 상태에서 enable하지 않기

### 15.3 중앙 PC

- [ ] can0가 올라오고 세 보드 status가 fresh
- [ ] Board1·2 enable, clear, homing, joint feedback 확인
- [ ] Board3 9축 feedback과 open·close profile을 저위험 상태에서 확인
- [ ] 손목 camera와 DICT_4X4_100 ID 50~54 검출 확인
- [ ] GUI port 8080이 비어 있고 페이지가 정상 polling
- [ ] execution_mode=hardware인지 확인
- [ ] fixed pose 현장 보정값과 실물 자세 확인

### 15.4 Pinky

- [ ] motor serial /dev/motor 연결
- [ ] /odom이 정지·전진·회전에 맞게 변함
- [ ] TF map → odom → base_footprint 연결
- [ ] /scan_filtered의 실제 장애물과 문 방향 값 확인
- [ ] 전방·후방 camera device가 뒤바뀌지 않음
- [ ] ID 20, 10, 4, 5 각각 15-frame 또는 정렬 조건 확인
- [ ] Nav2와 AMCL, 4F map·initial pose 확인
- [ ] /cmd_vel zero 후 실제 base가 정지하는지 확인

### 15.5 Action 서버

다음 서버를 시작 전에 확인한다.

    /arm/homing
    /arm/execute
    /arm/press_button
    /nav/go_to
    /navigate_to_pose
    /dock/align
    /mission/ready_and_approach
    /base/drive_straight
    /base/rotate
    /elevator/wait_door_open
    /elevator/board
    /elevator/exit
    /floor/check
    /map/switch

### 15.6 전체 미션 전에 반드시 단독 시험

- [ ] 빈 작업공간에서 arm home과 ready
- [ ] ID 54 검출 후 pickup approach가 실기로 실행되는지
- [ ] ID 50 한 개로 pre-press·press·retreat가 실기로 실행되는지
- [ ] gripper close 뒤 실제 object가 손상되지 않는지
- [ ] ID 20 정렬 뒤 zero Twist
- [ ] 27 cm odometry 직진과 80도 회전
- [ ] ID 10 승차 제어는 실제 엘리베이터가 아닌 안전한 mock 구역에서 먼저 시험
- [ ] 4F↔5F map switch와 AMCL seed

[주의] 두 번째와 세 번째가 제공 압축본의 차단 경로다. 이 단독 시험을 통과하지 않으면 전체 미션을 시작하지 않는다.

### 15.7 데모 운영 규칙

- [ ] 미션은 GUI 한 곳에서만 Start·Cancel
- [ ] CLI와 GUI에서 동시에 Goal을 만들지 않기
- [ ] 각 단계의 상태명과 예상 다음 동작을 한 명이 소리 내어 확인
- [ ] 실패 시 즉시 재시작하지 않고 물리 상태부터 복구
- [ ] 관객 질문을 받는 사람과 E-stop 담당자를 분리
- [ ] 로그와 화면 녹화, 가능하면 고정 camera 영상 확보

---

## 16. 장애 상황별 첫 확인점

| 증상 | 가장 먼저 볼 것 | 정확한 설명 |
|---|---|---|
| Mission Start 즉시 거절 | 고정 입력값, 12개 직접 Action 서버, 이미 실행 중인 Goal | Goal 수락 전 preflight 실패 |
| 3단계 pickup에서 중단 | task executor의 waypoint disabled 오류, 배포 build | 제공 압축본의 알려진 차단 경로 |
| 버튼 앞에서 팔 중단 | pre-press waypoint 실행 경로, field_verified | detection 성공과 실기 실행은 별개 |
| Robot Link LOST | /robot/heartbeat, Mission Manager, DDS network | 전체 하드웨어 고장 표시가 아님 |
| Nav2가 출발하지 않음 | /navigate_to_pose, TF, AMCL, /odom, map | 중앙 /nav/go_to는 어댑터일 뿐 |
| ID 20 정렬 실패 | 사전·ID·10 cm 크기, 조명, camera, pose freshness | 1.2초 stale이면 탐색, 90초 후 실패 |
| 문이 열렸는데 승차 안 함 | /scan_filtered open 조건, 전방 ID 10 15-frame | LiDAR와 camera 두 gate가 모두 필요 |
| 층 도착 후 하차 안 함 | 후방 camera, 목표 ID 4·5, /tag/floor_id | 다른 층이면 의도적으로 기다림 |
| map switch에서 멈춤 | LoadMap service, /amcl_pose, seed와 실제 위치 | xy 0.5 m 수렴 조건 |
| map switch 직후 Nav 불안정 | 제공 adapter의 3초 delay 미적용 | YAML에 있어도 실행 코드가 소비하지 않음 |
| 팔 board stale | can0, termination, status 0x201·202, power | 마지막 정상값을 현재값으로 오해하지 않기 |
| 그리퍼 일부 servo 오류 | servo bus 전원, ID, 50회 comm fault | Board3가 SERVO_COMM으로 전환 가능 |
| GUI Cancel이 안 됨 | Goal을 GUI에서 시작했는지, GUI process 재시작 여부 | 소유한 Goal Handle만 cancel 가능 |
| GUI ESTOP 뒤 base가 움직임 | base 물리 E-stop | GUI ESTOP은 arm board만 대상 |

---

## 17. 발표에서 단정하면 안 되는 표현

| 피해야 할 표현 | 더 정확한 표현 |
|---|---|
| “어떤 건물에서도 완전 자율입니다.” | “마커와 층별 map을 설치한 검증 시나리오를 자동 실행하도록 구성했습니다.” |
| “장애물이 있으면 언제나 멈춥니다.” | “Nav2 구간은 LiDAR costmap을 쓰지만 직접 Twist 구간은 별도 통제가 필요합니다.” |
| “E-stop이 모든 전원을 차단합니다.” | “현재 GUI 정지는 팔 보드 소프트웨어 정지이며 물리 전원 차단은 별도입니다.” |
| “그리퍼를 700 N으로 잡습니다.” | “서보 load raw threshold 700을 사용하며 N 환산은 별도 보정이 필요합니다.” |
| “MoveIt 궤적을 실제 팔이 그대로 추종합니다.” | “현재 V3는 최종 관절 목표 중심이며 waypoint 실기 경로는 확인이 필요합니다.” |
| “팔 위치는 모두 encoder 실측입니다.” | “확인된 Board2는 step 누적 추정이고 feedback 종류는 보드별로 구분해야 합니다.” |
| “정렬 정확도는 7 cm입니다.” | “코드의 lateral 허용범위가 ±7 cm이며 실제 오차 분포는 실측값으로 제시해야 합니다.” |
| “성공률은 높습니다.” | “___회 중 ___회 성공이라는 실제 시험 기록이 있습니다.” |
| “payload는 ___ kg입니다.” | “해당 값은 계산·실측 근거 ___로 검증했습니다.” |
| “ONLINE이면 전부 정상입니다.” | “Mission Manager heartbeat가 정상이고 각 sensor·board 상태는 별도로 확인합니다.” |

모르는 수치를 질문받았을 때는 다음처럼 답한다.

“코드에서 확인되는 명령값과 제한은 여기까지이고, 실제 기구 성능은 실측 자료가 필요한 항목입니다. 확인되지 않은 값을 추정해서 말씀드리지는 않겠습니다.”

---

## 18. 오늘 팀이 함께 채워야 할 실측 한 장

| 항목 | 결과 |
|---|---|
| 실제 배포 server commit·archive | ______ |
| 실제 배포 Pinky commit·archive | ______ |
| Board1·2·3 firmware build | ______ |
| 전체 미션 성공 횟수 / 시도 횟수 | ______ / ______ |
| 평균 전체 수행시간 | ______ |
| ID 20 정렬 실제 lateral·distance 오차 | ______ |
| 27 cm 이동 실제 오차 | ______ |
| 80도·90도·180도 실제 회전 오차 | ______ |
| pickup 성공 횟수 | ______ |
| button 50·51·52·53 각각 성공 여부 | ______ |
| gripper 허용 물체 크기·질량 | ______ |
| 팔 payload·reach·반복정밀도 | ______ |
| 배터리 1회 충전 데모 가능 횟수 | ______ |
| 물리 E-stop 위치와 담당자 | ______ |
| known issue와 우회 절차 | ______ |

---

## 19. 분석 근거와 검증 범위

### 주요 근거 파일

- central_bringup/launch/final_system.launch.py
- mission_manager/config/mission_flow.yaml
- mission_manager/config/action_servers.yaml
- mission_manager/config/locations.yaml
- mission_manager/mission_manager/mission_manager_node.py
- mission_manager/mission_manager/task_executor.py
- mission_manager/mission_manager/ready_and_approach_coordinator.py
- vicpinky_nav_adapter/vicpinky_nav_adapter/nav_adapter_node.py
- vicpinky_gui/vicpinky_gui/gui_node.py
- roscue_arm_pick/roscue_arm_pick/task_executor_node.py
- roscue_arm_pick/config/task_sequence.yaml
- roscue_arm_pick/config/fixed_poses.yaml
- roscue_arm_pick/config/gripper_profiles.yaml
- arm_can_bridge/config/arm_can_bridge.yaml
- docs/ARM_CAN_PROTOCOL.md
- arduino_board2_firmware/board2_axis5/board2_axis5.ino
- gripper_firmware_0710_moving_fix.zip 내부 STM32 source
- Pinky의 final_robot.launch.py, task_servers.launch.py
- Pinky의 bringup.py, Nav2 params, aruco_pose_publisher.py
- Pinky의 elevator_board_off.py, map_switcher_server.py와 관련 Action 서버

### 확인한 것과 확인하지 못한 것

- 두 압축본의 실제 source·launch·YAML·interface를 교차 확인했다.
- server와 Pinky의 vicpinky_interfaces 정의가 동일함을 확인했다.
- 중앙 관련 선택 단위 테스트에서는 설정 불일치 한 건이 검출됐다.
- 실제 로봇, 엘리베이터, camera, CAN bus를 이 분석 환경에서 구동하지 않았다.
- PCB와 제조용 CAD, Board1 firmware가 없어 해당 세부사항은 검증하지 못했다.
- 따라서 본 문서는 코드 리뷰 기반 발표 준비 자료이며 실기 성능 인증서가 아니다.

---

## 20. 발표 직전 1분 요약

1. 목표: 4층 402 물체를 5층에 배송하고 402로 복귀하는 이동·조작 통합 미션
2. 구조: 중앙 PC가 29단계를 조율하고 Pinky와 세 CAN 보드가 실제 제어
3. 주행: Nav2·AMCL 장거리 + ArUco 근거리 정렬
4. 층간: LiDAR 문 gate + ID 10 승차 + ID 4·5 층 확인 + 하차 후 map switch
5. 팔: 5축, Board1 4축 V3 + Board2 1축 legacy
6. 그리퍼: 3지 9축, Sync Write와 load threshold 접촉 처리
7. GUI: 상태·지도·미션·보드·로그 통합, Action cancel과 조작 상호 배제
8. 정확한 수치: 1.27 m, 3초, 27 cm, 좌80°, 탑승 50 cm, 하차 70 cm
9. 정직한 한계: 직접 Twist 안전계층, 물리 E-stop, 정량 실측, map delay, 하드웨어 자료
10. 데모 전 최우선: marker 기반 pickup·button waypoint가 실제 배포 build에서 실행되는지 단독 확인
