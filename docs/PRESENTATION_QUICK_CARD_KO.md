# Scorpy 발표 직전 요약 카드

## 1. 20초 소개

“Scorpy는 이동형 로봇과 5축 팔, 3지 9축 그리퍼를 결합한 ROS 2 기반 층간 배송 시스템입니다. 4층 402에서 물체를 집어 엘리베이터를 타고 5층에 배송한 뒤 4층으로 복귀하는 29단계 미션을 목표로 했습니다. 중앙 서버는 전체 순서를 조율하고, 주행·비전·팔·그리퍼 제어는 전문 노드와 제어보드가 수행합니다.”

## 2. 전체 구조

    브라우저
      → 중앙 PC: GUI → Mission Manager
                    ├→ MoveIt·팔 Action → CAN bridge → Board1·2·3
                    └→ /nav/go_to adapter
      → Raspberry Pi: Nav2·AMCL, LiDAR, 전후방 camera,
                      정렬·문·승하차·층·map Action
      → 베이스와 로봇팔

- 중앙 PC: 순서, timeout, retry, cancel, GUI, 팔
- Raspberry Pi: 베이스 I/O, Nav2, camera·LiDAR, 엘리베이터 동작
- Action: 오래 걸리는 작업
- Service: enable, home, clear 같은 짧은 요청
- Topic: sensor, odom, status, heartbeat

## 3. 29단계를 8묶음으로 기억

1. 팔 homing·ready → ID 54 물체를 트레이에 적재
2. Nav2로 4층 엘리베이터 앞 → ID 20 정렬
3. ID 50 호출 → 팔 ready와 27 cm 접근 → 좌 80도
4. LiDAR 문 확인 → ID 10 승차 → ID 52로 5층 선택
5. ID 5 층 확인·하차 → 5층 map switch
6. object_place 주행 → 좌 180도 → 트레이 물체 배송
7. 5층 엘리베이터 복귀 → ID 53 호출 → 승차 → ID 51로 4층 선택
8. ID 4 층 확인·하차 → 4층 map switch → 402 복귀

## 4. 최종 수치

| 항목 | 값 |
|---|---:|
| ID 20 정렬 | 1.27 m, lateral 0 m |
| 정렬 허용범위 | lateral ±7 cm, distance ±8 cm |
| 완료 조건 | 범위 안 3초 |
| 팔 ready 뒤 base 출발 | 2초 |
| 추가 접근 | 27 cm, 0.15 m/s |
| 문 방향 회전 | 좌 80도 |
| 탑승 | ID 10 기준 50 cm |
| 하차 | ID 4·5 기준 70 cm |
| 배송지 | 좌 180도 |
| 하차 뒤 회전 | 시간 기반 좌 90도 |

## 5. 마커 ID

| 사전 | ID | 역할 |
|---|---:|---|
| DICT_4X4_50 | 20 | 엘리베이터 앞 정렬 |
| DICT_4X4_50 | 10 | 캐빈 내부 승차 |
| DICT_4X4_50 | 4·5 | 층 확인·하차 |
| DICT_4X4_100 | 50 | 4층 호출 |
| DICT_4X4_100 | 51 | 캐빈 4층 |
| DICT_4X4_100 | 52 | 캐빈 5층 |
| DICT_4X4_100 | 53 | 5층 호출 |
| DICT_4X4_100 | 54 | pickup 물체 |

## 6. 담당별 핵심

| 담당 | 핵심 답변 | 반드시 밝힐 한계 |
|---|---|---|
| 팔 FW·기구·PCB | 팔 5축, Board1 4축 V3 + Board2 1축 legacy, homing 후 0.01° raw 제어 | PCB·Board1 원본과 payload 실측 자료 부재 |
| 그리퍼 기구·3D | 3손가락×3관절, 약 120° 배치, grasp TCP와 button TCP 분리 | 강도·물체 범위·공차는 실측 필요 |
| CAN·GUI | Classic CAN 500 kbps, staging→READY→START, 상태 freshness 표시 | Board2는 ACK·선점 cancel 부재 |
| 그리퍼 전장·FW | STM32F411RE, MCP2515, 9개 servo Sync Write, load threshold 접촉 | load는 N이 아니고 ESTOP torque-off 미보장 |
| 중앙 서버·GUI | 세 YAML로 29단계, Action feedback·timeout·retry·cancel | 고정 시나리오, checkpoint resume 없음 |
| 주행·정렬 | Nav2·AMCL 장거리 + ID 20 근거리 P 제어 | 직접 Twist 구간 collision monitor·watchdog 부재 |
| 층간 이동 | LiDAR 문 gate, ID 10 승차, ID 4·5 층, 하차 후 map switch | 문 안전신호 미연동, 하차 회전 시간 기반 |

## 7. 자주 나오는 공통 질문

**왜 ROS 2인가요?**

“주행·비전·팔·GUI를 독립 개발하고 Topic·Service·Action으로 목적에 맞게 연결하기 좋기 때문입니다.”

**왜 Nav2와 ArUco를 같이 쓰나요?**

“Nav2는 전역 장거리 주행, ArUco는 버튼과 엘리베이터의 근거리 상대 정렬에 적합해서 구간별로 전환합니다.”

**실패하면 어떻게 하나요?**

“각 Action이 원인을 반환하고 중앙이 timeout과 지정된 retry를 적용합니다. 그래도 실패하면 다음 단계로 진행하지 않고 중단합니다.”

**완전 자율인가요?**

“마커와 층별 map이 준비된 고정 402→5층→402 시나리오를 단계 개입 없이 수행하도록 구성했습니다. 임의 건물·임의 목적지를 위한 범용 시스템은 아닙니다.”

**안전은 어떻게 처리했나요?**

“범위 검증, freshness, timeout, cancel, marker·odom 소실 시 정지가 있습니다. 다만 안전 인증이나 전체 전원 차단을 뜻하지 않으며 물리 E-stop과 통제 구역이 필요합니다.”

## 8. 데모 전에 반드시 확인

1. 제공 서버 압축본은 marker 기반 MoveIt waypoint를 hardware에서 Board1·2로 실행하지 않고 오류로 중단한다. ID 54 pickup과 ID 50 button을 각각 단독 실기 시험한다.
2. fixed_poses.yaml의 field_verified는 false다. 실제 보정 자세를 확인한다.
3. YAML의 map switch 후 3초 delay는 제공 nav adapter가 읽지 않는다.
4. GUI ESTOP은 arm board만 대상이며 base 전체 E-stop이 아니다.
5. direct Twist 구간에는 Nav2 장애물 회피가 없다.
6. 최종 값은 1.27 m, 3초, 27 cm, 좌80도, 50 cm, 70 cm다.
7. 미션은 GUI 한 곳에서만 시작하고 E-stop 담당자를 별도로 둔다.

## 9. 모르는 수치 질문에 대한 답

“코드에서 확인되는 목표값과 제한은 여기까지이고, 실제 payload·정확도·성공률은 실측 자료가 필요한 항목입니다. 확인되지 않은 값을 추정해서 말씀드리지는 않겠습니다.”
