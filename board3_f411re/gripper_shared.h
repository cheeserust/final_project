#ifndef GRIPPER_SHARED_H
#define GRIPPER_SHARED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 파일명: gripper_shared.h
 * 버전: Board3 Gripper Protocol v1.1 - 0x303 Position Feedback
 *
 * 목적:
 *   이 헤더파일은 Board3의 CAN 통신 코드와 그리퍼 제어 코드가
 *   공통으로 사용하는 상수, 상태값, 에러값, 공유 구조체를 정의한다.
 *
 * 현재 단계:
 *   실제 SCS0009 서보와 그리퍼 하드웨어가 연결되기 전 단계이다.
 *   따라서 실제 위치 보정(direction, step_per_deg, min/max goal 등)은
 *   아직 포함하지 않고, 우선 CAN 수신 → 9개 frame staging →
 *   g_cmd 전달 → g_state status 보고 구조를 맞추는 것이 목적이다.
 *
 *   v1.1 변경 사항:
 *   CAN ID 0x303 고속 현재 위치 피드백을 추가한다.
 *   기존 0x203 status는 100ms 상태/heartbeat로 유지하고,
 *   0x303은 20ms마다 Motor ID 0~8 현재 위치를 3개 frame으로 송신한다.
 *   Board3 gripper home posture는 0x023으로 처리하고, 0x020 Arm Homing은 무시한다.
 *
 * 역할 분리:
 *
 *   [CAN 통신 코드 담당]
 *     1. CAN ID 0x103 수신
 *     2. payload 파싱
 *     3. Motor ID 0~8 검사
 *     4. 9개 frame staging
 *     5. 중복 Motor ID 검사
 *     6. duration 동일성 검사
 *     7. 정상 command set이면 g_cmd에 저장
 *     8. g_state를 이용해 CAN ID 0x203 status 송신
 *     9. g_state.current_pos_001deg[]를 이용해 CAN ID 0x303 위치 피드백 송신
 *
 *   [그리퍼 제어 코드 담당]
 *     1. g_cmd.is_new_cmd 확인
 *     2. g_cmd.target_pos_001deg[0~8] 읽기
 *     3. 0.01도 단위 각도값을 SCS0009 position 값으로 변환
 *     4. Motor ID를 실제 Servo ID로 변환
 *     5. Feetech_Set_Position_Time() 호출
 *     6. ready, fault, current_pos 등을 g_state에 반영
 *
 * 중요:
 *   g_cmd.target_pos_001deg[]에는 SCS0009 position 값이 들어가지 않는다.
 *   이 배열에는 중앙서버가 CAN으로 보낸 "0.01도 단위 각도값"이 들어간다.
 *
 *   예:
 *     3000  =  30.00도
 *     9000  =  90.00도
 *    -1500  = -15.00도
 *
 *   실제 SCS0009 position 0~1023 변환은 그리퍼 제어 코드에서 수행한다.
 * ========================================================================== */


/* ============================================================================
 * 1. Board ID / CAN ID 정의
 * ========================================================================== */

/*
 * Board3 내부 식별용 ID.
 *
 * 최종 프로토콜에서는 실제 CAN payload에 Board ID를 넣지 않고
 * CAN ID로 보드를 구분한다. 현재 main.c 수신부는 legacy Board ID payload를
 * 더 이상 허용하지 않는다.
 */
#define BOARD3_ID                         3U

/*
 * 전체 대상 / 전체 Motor ID 값.
 * 최종 프로토콜에서 0x023 Gripper Home과 0x030 Clear Error의 Byte0은
 * 0xFF를 사용한다.
 */
#define BOARD_ID_ALL                      255U
#define MOTOR_ALL                         BOARD_ID_ALL

/*
 * Emergency Stop 명령 CAN ID.
 *
 * 방향:
 *   중앙서버/RPi → Board3
 *
 * 의미:
 *   비상정지 명령이다.
 *   수신 시 staging buffer를 비우고, g_cmd.is_new_cmd를 0으로 내리고,
 *   상태를 STATE_ESTOP으로 바꾼다.
 *
 * 하드웨어 연결 후:
 *   실제 SCS0009 servo stop / torque disable / unload 방식은
 *   그리퍼 제어 코드에서 구현해야 한다.
 */
#define CAN_ID_BOARD3_ESTOP               0x001U

/*
 * Enable / Disable 명령 CAN ID.
 *
 * 방향:
 *   중앙서버/RPi → Board3
 *
 * 의미:
 *   Board3가 0x103 gripper command를 받을 수 있는지 제어한다.
 *
 * 최종 payload:
 *   Byte0 = Enable 값, 0: Disable, 1: Enable
 *   Byte1~7 = Reserved, 0
 *
 * 주의:
 *   legacy 형식인 010#0301, 010#FF01, 010#01은 허용하지 않는다.
 */
#define CAN_ID_BOARD3_ENABLE              0x010U

/*
 * Arm Homing Broadcast CAN ID.
 *
 * 방향:
 *   중앙서버/RPi → Board1 + Board2
 *
 * 의미:
 *   0x020은 Board1/Board2 로봇팔 homing용 broadcast이다.
 *   Board3는 이 CAN ID를 수신해도 처리하지 않고 무시한다.
 *   Board3 gripper home posture는 0x023을 사용한다.
 */
#define CAN_ID_BOARD3_HOMING              0x020U
#define CAN_ID_BOARD3_GRIPPER_HOME        0x023U  /* FINAL: Board3 gripper home posture */

/*
 * Clear Error 명령 CAN ID.
 *
 * 방향:
 *   중앙서버/RPi → Board3
 *
 * 의미:
 *   g_state.error_code, fault, fault_motor_id 등을 초기화한다.
 *
 * 주의:
 *   ESTOP 해제는 Clear Error가 아니라 Enable 명령에서 처리하는 것을 권장한다.
 */
#define CAN_ID_BOARD3_CLEAR_ERROR         0x030U

/*
 * Board3 gripper command CAN ID.
 *
 * 방향:
 *   중앙서버/RPi → Board3
 *
 * 의미:
 *   SCS0009 서보 1개에 대한 목표 각도 명령이다.
 *
 * 중요:
 *   0x103 frame 하나에는 서보 1개 명령만 들어간다.
 *   Board3는 서보 9개를 담당하므로, gripper 한 번의 동작에는
 *   Motor ID 0~8에 대한 총 9개의 0x103 frame이 필요하다.
 */
#define CAN_ID_BOARD3_CMD                 0x103U

/*
 * Board3 status CAN ID.
 *
 * 방향:
 *   Board3 → 중앙서버/RPi
 *
 * 의미:
 *   Board3의 현재 상태, 에러, ready/fault, staging 상태 등을 보고한다.
 *
 * DLC:
 *   8바이트 고정
 */
#define CAN_ID_BOARD3_STATUS              0x203U

/*
 * Board3 position feedback CAN ID.
 *
 * 방향:
 *   Board3 → 중앙서버/RPi
 *
 * 의미:
 *   MoveIt2 / joint_states 동기화를 위한 현재 위치 고속 피드백이다.
 *   20ms마다 3개의 CAN frame으로 Motor ID 0~8 현재 위치를 전송한다.
 */
#define CAN_ID_BOARD3_POSITION            0x303U


/* ============================================================================
 * 2. Gripper 기본 설정
 * ========================================================================== */

/*
 * Board3가 제어하는 서보 개수.
 *
 * 현재 v1.0에서는 3-finger gripper의 서보 9개를 사용한다.
 */
#define GRIPPER_MOTOR_COUNT               9U

/*
 * CAN 프로토콜에서 사용하는 Motor ID 최소값.
 */
#define GRIPPER_MOTOR_ID_MIN              0U

/*
 * CAN 프로토콜에서 사용하는 Motor ID 최대값.
 *
 * 유효한 Motor ID 범위:
 *   0, 1, 2, 3, 4, 5, 6, 7, 8
 */
#define GRIPPER_MOTOR_ID_MAX              8U

/*
 * 하드웨어 연결 전 기본 staging timeout.
 *
 * 의미:
 *   첫 번째 0x103 frame이 들어온 뒤, 이 시간 안에 Motor ID 0~8의
 *   9개 frame이 모두 들어와야 정상 command set으로 인정한다.
 *
 * 현재 값:
 *   100ms
 *
 * 이유:
 *   하드웨어 연결 전/초기 디버깅 단계에서는 UART debug 출력이나
 *   CAN 송신 간격 때문에 20ms가 빡빡할 수 있으므로 100ms를 기본값으로 둔다.
 */
#define GRIPPER_STAGING_TIMEOUT_MS        100U

/*
 * 최종 목표 staging timeout.
 *
 * 통신 안정화 후 최종적으로 줄일 목표값이다.
 * 실제 최종 코드에서 20ms로 변경할 수 있다.
 */
#define GRIPPER_STAGING_TIMEOUT_FINAL_MS  20U

/*
 * Board3 status 송신 주기.
 *
 * Board3는 100ms마다 CAN ID 0x203으로 상태를 주기 송신한다.
 * 또한 error, enable/disable, ESTOP, command set 완료 같은 주요 이벤트에서도
 * 즉시 status를 한 번 송신하는 것을 권장한다.
 */
#define GRIPPER_STATUS_PERIOD_MS          100U

/*
 * Board3 현재 위치 피드백 송신 주기.
 *
 * Board3는 MoveIt2 기반 제어에서 현재 관절 위치를 더 빠르게 갱신하기 위해
 * CAN ID 0x303으로 20ms마다 위치 피드백을 송신한다.
 * 0x203 status보다 짧은 주기이며, status와 역할을 분리한다.
 */
#define GRIPPER_POSITION_FEEDBACK_PERIOD_MS 20U

/*
 * Board3 homing 시 사용할 home 위치.
 *
 * 단위:
 *   0.01도
 *
 * 현재 정책:
 *   home position = 0.00도
 *   따라서 0.00도 = 0 이다.
 *
 * 주의:
 *   이 값은 SCS0009 position 0이 아니다.
 *   g_cmd.target_pos_001deg[]에 들어가는 각도값이다.
 *   실제 SCS0009 position 변환은 그리퍼 제어 코드에서 수행한다.
 */
#define GRIPPER_HOME_POSITION_001DEG      0L

/*
 * Board3 homing 시 사용할 기본 이동 시간.
 *
 * 단위:
 *   5ms tick
 *
 * 현재 값:
 *   100 × 5ms = 500ms
 *
 * 실제 Feetech 함수 호출 전에는
 *   duration_ms = GRIPPER_HOMING_DURATION_5MS × 5
 * 로 변환한다.
 */
#define GRIPPER_HOMING_DURATION_5MS       100U

/*
 * fault가 발생한 motor가 없음을 나타내는 값.
 *
 * 0x203 status Byte7 = fault_motor_id
 *   - 0~8: 해당 Motor ID에서 fault 발생
 *   - 255: fault motor 없음
 */
#define GRIPPER_NO_FAULT_MOTOR            255U


/* ============================================================================
 * 3. 0x103 Byte0 bit mask 정의
 * ========================================================================== */

/*
 * Execute bit mask.
 *
 * 0x103 payload Byte0의 Bit7이다.
 *
 * 의미:
 *   1이면 이 frame을 staging 대상으로 처리한다.
 *   0이면 실행 명령이 아니므로 무시하거나 ERR_INVALID_CMD로 처리할 수 있다.
 *
 * 예:
 *   Byte0 = 0x80 → Execute=1, Motor ID=0
 *   Byte0 = 0x81 → Execute=1, Motor ID=1
 */
#define GRIPPER_CMD_EXECUTE_MASK          0x80U

/*
 * Reserved bit mask.
 *
 * 0x103 payload Byte0의 Bit6~4이다.
 *
 * 현재 v1.0에서는 사용하지 않으므로 반드시 0이어야 한다.
 * 이 값이 0이 아니면 잘못된 command로 보고 command set을 폐기한다.
 */
#define GRIPPER_CMD_RESERVED_MASK         0x70U

/*
 * Motor ID bit mask.
 *
 * 0x103 payload Byte0의 Bit3~0이다.
 *
 * 이 값으로 Motor ID 0~8을 구분한다.
 */
#define GRIPPER_CMD_MOTOR_ID_MASK         0x0FU


/* ============================================================================
 * 4. State 값 정의
 *
 * 이 값은 0x203 status의 Byte0에 들어간다.
 * ========================================================================== */

/*
 * 초기화 중 상태.
 * 전원 인가 직후 또는 초기 설정 중에 사용한다.
 */
#define STATE_INIT                        0U

/*
 * 대기 상태.
 * Board3가 enable되어 있고, 현재 staging이나 moving이 없는 상태이다.
 */
#define STATE_IDLE                        1U

/*
 * staging 상태.
 * 0x103 frame 9개를 모으는 중인 상태이다.
 *
 * 예:
 *   Motor ID 0, 1, 2까지만 들어온 상태라면
 *   state = STATE_STAGING
 *   staging_count = 3
 *   buffer_free = 6
 */
#define STATE_STAGING                     2U

/*
 * moving 상태.
 * 정상 command set이 g_cmd로 전달되었고,
 * 그리퍼 제어 코드가 해당 명령을 처리 중인 상태이다.
 *
 * 하드웨어 연결 전에는 실제 이동 대신 debug print 단계에서도 사용할 수 있다.
 */
#define STATE_MOVING                      3U

/*
 * error 상태.
 * 잘못된 command set, timeout, duration mismatch, servo fault 등
 * error_code가 ERR_NONE이 아닌 상태이다.
 */
#define STATE_ERROR                       4U

/*
 * ESTOP 상태.
 * Emergency Stop 명령을 받은 상태이다.
 * 이 상태에서는 0x103 move command를 처리하지 않는다.
 */
#define STATE_ESTOP                       5U

/*
 * disabled 상태.
 * Enable/Disable 명령에서 Disable을 받은 상태이다.
 * 이 상태에서는 0x103 move command를 처리하지 않는다.
 */
#define STATE_DISABLED                    6U


/* ============================================================================
 * 5. Error Code 정의
 *
 * 이 값은 0x203 status의 Byte1에 들어간다.
 * ========================================================================== */

/*
 * 정상 상태. 에러 없음.
 */
#define ERR_NONE                          0U

/*
 * 일반적인 잘못된 명령.
 *
 * 예:
 *   - DLC가 8이 아님
 *   - Execute bit가 0인데 실행 명령으로 들어옴
 *   - Reserved bit가 0이 아님
 *   - payload 구조가 잘못됨
 */
#define ERR_INVALID_CMD                   1U

/*
 * Motor ID 범위 오류.
 *
 * Board3는 Motor ID 0~8만 허용한다.
 * Motor ID가 9 이상이면 이 에러를 설정한다.
 */
#define ERR_INVALID_MOTOR_ID              2U

/*
 * 같은 command set 안에서 Motor ID가 중복된 경우.
 *
 * 예:
 *   Motor ID 1 frame이 이미 들어왔는데 또 Motor ID 1 frame이 들어옴.
 *
 * 처리:
 *   전체 command set을 폐기하고 g_cmd.is_new_cmd를 올리지 않는다.
 */
#define ERR_DUPLICATE_MOTOR_ID            3U

/*
 * staging timeout.
 *
 * 첫 번째 0x103 frame 수신 후 timeout 안에 Motor ID 0~8의
 * 9개 frame이 모두 도착하지 않은 경우이다.
 */
#define ERR_STAGING_TIMEOUT               4U

/*
 * duration 불일치.
 *
 * 하나의 gripper command set에 포함된 9개 frame의 duration 값은 모두 같아야 한다.
 * 하나라도 다르면 전체 command set을 폐기한다.
 */
#define ERR_DURATION_MISMATCH             5U

/*
 * 목표 각도 범위 초과.
 *
 * 하드웨어 연결 후 적용할 에러이다.
 * 실제 joint limit 또는 servo min/max goal을 벗어나는 명령이면 설정한다.
 *
 * 하드웨어 연결 전 v1.0에서는 우선 reserved 성격으로 남겨둔다.
 */
#define ERR_ANGLE_RANGE                   6U

/*
 * SCS0009 통신 오류.
 *
 * 하드웨어 연결 후 Feetech 통신 실패, UART timeout, servo 응답 없음 등의 상황에서 사용한다.
 */
#define ERR_SERVO_COMM                    7U

/*
 * servo fault.
 *
 * 하드웨어 연결 후 과부하, servo 내부 error, load threshold 초과 등의 상황에서 사용한다.
 */
#define ERR_SERVO_FAULT                   8U

/*
 * ESTOP 상태에서 명령이 들어온 경우 또는 ESTOP 상태를 표시할 때 사용한다.
 */
#define ERR_ESTOP                         9U

/*
 * Disable 상태에서 0x103 command가 들어온 경우 사용한다.
 */
#define ERR_DISABLED                      10U


/* ============================================================================
 * 6. Motor ID → Servo ID 매핑
 * ========================================================================== */

/*
 * Motor ID:
 *   중앙서버/CAN 프로토콜에서 사용하는 논리 번호이다.
 *   범위는 0~8이다.
 *
 * Servo ID:
 *   실제 SCS0009 서보에 설정된 하드웨어 ID이다.
 *   현재 v1.0에서는 Servo ID 1~9로 확정한다.
 *
 * 매핑:
 *   Motor ID 0 → Servo ID 1
 *   Motor ID 1 → Servo ID 2
 *   ...
 *   Motor ID 8 → Servo ID 9
 *
 * 주의:
 *   실제 서보 ID 설정이 이 매핑과 다르면 엉뚱한 손가락이 움직일 수 있다.
 */
static const uint8_t GRIPPER_MOTOR_ID_TO_SERVO_ID[GRIPPER_MOTOR_COUNT] = {
    1U, 2U, 3U,
    4U, 5U, 6U,
    7U, 8U, 9U
};


/* ============================================================================
 * 7. Servo Home Goal 값
 * ========================================================================== */

/*
 * 각 서보의 원점 기준 position 값이다.
 *
 * 하드웨어 연결 전에는 CAN staging 자체에는 필요하지 않다.
 * 하드웨어 연결 후 0.01도 각도값을 SCS0009 position으로 변환할 때 사용한다.
 *
 * 예:
 *   servo_pos = home_goal[motor_id]
 *             + direction[motor_id] * target_deg * step_per_deg[motor_id]
 *
 * 현재 값:
 *   Servo ID 1 → 520
 *   Servo ID 2 → 520
 *   Servo ID 3 → 500
 *   Servo ID 4 → 500
 *   Servo ID 5 → 520
 *   Servo ID 6 → 511
 *   Servo ID 7 → 520
 *   Servo ID 8 → 508
 *   Servo ID 9 → 530
 */
static const uint16_t GRIPPER_SERVO_HOME_GOAL[GRIPPER_MOTOR_COUNT] = {
    520U, 520U, 500U,
    500U, 520U, 511U,
    520U, 508U, 530U
};


/* ============================================================================
 * 8. GripperCommand 구조체
 * ========================================================================== */

/*
 * GripperCommand
 *
 * 역할:
 *   CAN 통신 코드가 0x103 frame 9개를 모두 수신하고,
 *   유효성 검사를 통과한 뒤 그리퍼 제어 코드에 넘겨주는 최종 명령 구조체이다.
 *
 * 이 구조체는 staging buffer 자체가 아니다.
 *
 * 흐름:
 *   CAN 0x103 frame 9개 수신
 *     ↓
 *   staging buffer에 Motor ID별 저장
 *     ↓
 *   중복 ID, timeout, duration 동일성 검사
 *     ↓
 *   정상 command set이면 GripperCommand에 복사
 *     ↓
 *   is_new_cmd = 1
 *     ↓
 *   그리퍼 제어 코드가 읽어서 처리
 */
typedef struct
{
    /*
     * target_pos_001deg
     *
     * Motor ID 0~8 각각의 목표 각도값이다.
     *
     * 단위:
     *   0.01도
     *
     * 예:
     *   3000  =  30.00도
     *   9000  =  90.00도
     *  -1500  = -15.00도
     *
     * 중요:
     *   이 값은 SCS0009 servo position 값이 아니다.
     *   즉, 3000은 servo position 3000이 아니라 30.00도라는 뜻이다.
     *
     * 사용 위치:
     *   그리퍼 제어 코드에서 이 값을 읽고,
     *   Motor ID별 home goal, direction, step_per_deg를 적용해
     *   SCS0009 position 값으로 변환한다.
     */
    int32_t target_pos_001deg[GRIPPER_MOTOR_COUNT];

    /*
     * duration_5ms
     *
     * CAN 0x103 Byte7 원본값이다.
     *
     * 단위:
     *   5ms tick
     *
     * 예:
     *   duration_5ms = 20  → 100ms
     *   duration_5ms = 40  → 200ms
     *   duration_5ms = 100 → 500ms
     *
     * 정책:
     *   g_cmd에는 CAN 원본값을 그대로 저장한다.
     *   Feetech_Set_Position_Time() 호출 직전에 아래처럼 ms로 변환한다.
     *
     *   uint16_t duration_ms = duration_5ms * 5;
     *
     * 이유:
     *   CAN/g_cmd 인터페이스는 프로토콜 원본 단위를 유지하고,
     *   실제 Feetech 함수 단위 변환은 servo adapter에서 담당하기 위함이다.
     */
    uint8_t duration_5ms;

    /*
     * is_new_cmd
     *
     * 새 gripper command set이 준비되었는지 나타내는 flag이다.
     *
     * 값:
     *   0 = 새 명령 없음
     *   1 = 새 명령 있음
     *
     * CAN 통신 코드:
     *   9개 frame이 모두 정상 수신되고 검사까지 통과한 뒤
     *   target_pos_001deg[], duration_5ms를 모두 채우고
     *   마지막에 is_new_cmd = 1로 설정한다.
     *
     * 그리퍼 제어 코드:
     *   is_new_cmd == 1이면 명령을 local 변수로 복사한 뒤,
     *   처리 시작 전에 is_new_cmd = 0으로 clear한다.
     *
     * 주의:
     *   interrupt/callback과 main loop가 동시에 접근할 수 있으므로,
     *   실제 코드에서는 critical section 또는 interrupt disable 구간이 필요할 수 있다.
     */
    uint8_t is_new_cmd;

} GripperCommand;


/* ============================================================================
 * 9. GripperState 구조체
 * ========================================================================== */

/*
 * GripperState
 *
 * 역할:
 *   Board3의 현재 상태를 저장하는 구조체이다.
 *   CAN 통신 코드는 이 구조체를 이용해 0x203 status frame을 만든다.
 *
 * 0x203 payload 매핑:
 *
 *   Byte0 = state
 *   Byte1 = error_code
 *   Byte2 = ready
 *   Byte3 = staging_count
 *   Byte4 = fault
 *   Byte5 = buffer_free
 *   Byte6 = enabled
 *   Byte7 = fault_motor_id
 *
 * 하드웨어 연결 전:
 *   ready = 1
 *   fault = 0
 *   fault_motor_id = 255
 *   current_pos[]는 0 또는 디버그 값으로 사용 가능
 *
 * 하드웨어 연결 후:
 *   Feetech_Read_Pos_Load() 결과를 바탕으로 ready, fault,
 *   fault_motor_id, current_pos[]를 갱신한다.
 */
typedef struct
{
    /*
     * state
     *
     * Board3의 현재 상태이다.
     * 0x203 status Byte0에 들어간다.
     *
     * 사용 값:
     *   STATE_INIT
     *   STATE_IDLE
     *   STATE_STAGING
     *   STATE_MOVING
     *   STATE_ERROR
     *   STATE_ESTOP
     *   STATE_DISABLED
     */
    uint8_t state;

    /*
     * error_code
     *
     * 현재 에러 코드이다.
     * 0x203 status Byte1에 들어간다.
     *
     * 정상 상태이면 ERR_NONE이다.
     */
    uint8_t error_code;

    /*
     * ready
     *
     * Board3가 gripper command를 받을 준비가 되었는지 나타낸다.
     * 0x203 status Byte2에 들어간다.
     *
     * 값:
     *   0 = 준비 안 됨
     *   1 = 준비됨
     *
     * 하드웨어 연결 전:
     *   구조 테스트를 위해 1로 고정해도 된다.
     *
     * 하드웨어 연결 후:
     *   9개 servo 통신 가능 여부, enable 상태, ESTOP 여부 등을 종합해서 결정한다.
     */
    uint8_t ready;

    /*
     * staging_count
     *
     * 현재 staging buffer에 모인 유효 0x103 frame 개수이다.
     * 0x203 status Byte3에 들어간다.
     *
     * 범위:
     *   0~9
     *
     * 예:
     *   0 = 아직 아무 frame도 안 들어옴
     *   3 = Motor ID 3개 수신 완료
     *   9 = Motor ID 0~8 전체 수신 완료
     */
    uint8_t staging_count;

    /*
     * fault
     *
     * Board3 또는 servo 쪽에 fault가 있는지 나타낸다.
     * 0x203 status Byte4에 들어간다.
     *
     * 값:
     *   0 = fault 없음
     *   1 = fault 있음
     *
     * 하드웨어 연결 전:
     *   0으로 고정 가능
     *
     * 하드웨어 연결 후:
     *   SCS0009 통신 실패, 부하 초과, servo error 등을 감지하면 1로 설정한다.
     */
    uint8_t fault;

    /*
     * buffer_free
     *
     * staging buffer에 남은 빈 slot 수이다.
     * 0x203 status Byte5에 들어간다.
     *
     * 계산:
     *   buffer_free = 9 - staging_count
     *
     * 예:
     *   staging_count = 0 → buffer_free = 9
     *   staging_count = 4 → buffer_free = 5
     *   staging_count = 9 → buffer_free = 0
     *
     * 주의:
     *   Board1의 trajectory queue_free = 32개 queue slot과 다르다.
     *   Board3의 buffer_free는 9개 frame staging buffer 기준이다.
     */
    uint8_t buffer_free;

    /*
     * enabled
     *
     * Board3가 현재 enable 상태인지 나타낸다.
     * 0x203 status Byte6에 들어간다.
     *
     * 값:
     *   0 = disabled
     *   1 = enabled
     *
     * 처리:
     *   0x010 Enable/Disable 명령에 따라 갱신한다.
     *
     * disabled 상태에서는 0x103 command를 g_cmd로 넘기지 않는다.
     */
    uint8_t enabled;

    /*
     * fault_motor_id
     *
     * fault가 발생한 Motor ID를 나타낸다.
     * 0x203 status Byte7에 들어간다.
     *
     * 값:
     *   0~8 = 해당 Motor ID에서 fault 발생
     *   255 = fault motor 없음
     *
     * 하드웨어 연결 전:
     *   255로 고정 가능
     *
     * 하드웨어 연결 후:
     *   특정 servo 통신 실패 또는 과부하 발생 시 해당 Motor ID를 넣는다.
     */
    uint8_t fault_motor_id;

    /*
     * current_pos
     *
     * 각 Motor ID의 현재 위치 피드백 또는 디버그용 위치값이다.
     *
     * 단위:
     *   하드웨어 연결 전에는 임시값 또는 0
     *   하드웨어 연결 후에는 팀에서 정한 단위 사용
     *
     * 주의:
     *   current_pos[9] 전체는 CAN 0x203 status에 직접 들어가지 않는다.
     *   0x203은 8바이트뿐이므로 9개 위치값을 모두 담을 수 없다.
     *
     * 용도:
     *   내부 디버깅
     *   추후 별도 position report CAN ID를 만들 경우 활용 가능
     */
    int32_t current_pos[GRIPPER_MOTOR_COUNT];

    /*
     * current_pos_001deg
     *
     * 각 Motor ID의 현재 위치를 0.01도 단위 int16_t 값으로 변환한 배열이다.
     * CAN 0x303 위치 피드백은 이 배열을 읽어서 20ms마다 3개의 frame으로 전송한다.
     *
     * 단위:
     *   0.01도
     *
     * 예:
     *   3000  =  30.00도
     *  -1550  = -15.50도
     *
     * 범위:
     *   int16_t 기준 -327.68도 ~ +327.67도
     */
    int16_t current_pos_001deg[GRIPPER_MOTOR_COUNT];

} GripperState;


/* ============================================================================
 * 10. 공유 전역 변수 선언
 * ========================================================================== */

/*
 * g_cmd
 *
 * CAN 통신 코드가 쓰고, 그리퍼 제어 코드가 읽는 공유 명령 구조체이다.
 *
 * 접근 주의:
 *   CAN RX interrupt/callback에서 g_cmd를 쓰고,
 *   main loop 또는 servo control loop에서 g_cmd를 읽는다면
 *   데이터가 중간에 섞이지 않도록 critical section이 필요하다.
 *
 * 권장 쓰기 순서:
 *   1. target_pos_001deg[0~8] 모두 저장
 *   2. duration_5ms 저장
 *   3. 마지막에 is_new_cmd = 1
 *
 * 권장 읽기 순서:
 *   1. is_new_cmd 확인
 *   2. local 변수로 전체 복사
 *   3. is_new_cmd = 0
 *   4. local 복사본으로 servo 제어
 */
extern volatile GripperCommand g_cmd;

/*
 * g_state
 *
 * Board3 상태를 나타내는 공유 구조체이다.
 *
 * CAN 통신 코드:
 *   g_state 값을 이용해 0x203 status frame을 구성한다.
 *
 * 그리퍼 제어 코드:
 *   ready, fault, fault_motor_id, current_pos 등을 갱신한다.
 */
extern volatile GripperState g_state;


/* ============================================================================
 * 11. Helper Macro
 * ========================================================================== */

/*
 * staging_count 값으로부터 buffer_free 값을 계산한다.
 *
 * buffer_free = 9 - staging_count
 *
 * staging_count가 9보다 커지는 비정상 상황에서는 0을 반환한다.
 */
#define GRIPPER_BUFFER_FREE_FROM_COUNT(count) \
    ((uint8_t)((GRIPPER_MOTOR_COUNT > (count)) ? (GRIPPER_MOTOR_COUNT - (count)) : 0U))

/*
 * CAN duration_5ms 값을 실제 ms 단위로 변환한다.
 *
 * 예:
 *   GRIPPER_DURATION_5MS_TO_MS(100) → 500
 */
#define GRIPPER_DURATION_5MS_TO_MS(tick) \
    ((uint16_t)((uint16_t)(tick) * 5U))


#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_SHARED_H */
