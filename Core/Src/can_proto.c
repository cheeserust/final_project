#include "../Inc/can_proto.h"
#include "../Inc/gpio.h"
#include "../Inc/mcp2515.h"
#include "../Inc/stepper.h"
#include "../Inc/trajectory.h"

static int32_t get_i32_le(const uint8_t *p)
{
    uint32_t v = ((uint32_t)p[0]) |
                 ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) |
                 ((uint32_t)p[3] << 24);
    return (int32_t)v;  // CAN 데이터 4바이트를 little-endian int32 값으로 변환
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8));  // CAN 데이터 2바이트를 little-endian uint16 값으로 변환
}

void can_send_status(void)
{
    uint8_t data[8];  // 송신할 상태 CAN 프레임 데이터 8바이트

    system_update_state();  // 현재 모터 상태를 전역 상태값에 반영
    data[0] = global_motor_state;           // 현재 시스템 상태
    data[1] = global_motor_error;           // 현재 에러 코드
    data[2] = system_homing_done_bits();    // 각 축의 원점복귀 완료 비트
    data[3] = system_first_moving_axis();   // 움직이는 축 중 첫 번째 축 번호
    data[4] = stepper_limit_status_bits();  // 리미트 스위치 입력 상태 비트
    data[5] = trajectory_free_count();      // 남은 궤적 큐 슬롯 수
    data[6] = system_enabled_status();      // 모터 enable/estop 상태
    data[7] = 0;                            // 예비 바이트
    (void)mcp2515_send_std(CAN_ID_BOARD1_STAT, data, 8U);  // 보드 상태 프레임 송신
}

static void set_all_axis_enabled(uint8_t enabled)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis[i].enabled = enabled;  // 모든 축의 enable 상태를 동일하게 설정
    }
}

static void handle_estop(void)
{
    global_motor_estop = 1;          // 비상정지 상태 진입
    global_motor_enabled = 0;        // 모터 구동 명령 차단
    global_motor_error = ERR_NONE;   // 비상정지는 별도 상태로 보고 에러 코드는 초기화
    set_all_axis_enabled(0);         // 모든 축 비활성화
    trajectory_clear();              // 대기 중인 이동 명령 삭제
    stepper_stop_all();              // 스텝 펄스 출력 정지
    motor_disable();                 // 모터 드라이버 출력 비활성화
    can_send_status();               // 변경된 상태 즉시 송신
}

static void handle_enable(const uint8_t *data, uint8_t len)
{
    if (len < 1U) return;  // enable 값이 없으면 무시

    if (data[0] == 1U) {
        global_motor_estop = 0;         // 비상정지 상태 해제
        global_motor_error = ERR_NONE;  // 기존 에러 코드 초기화
        global_motor_enabled = 1;       // 모터 구동 허용
        set_all_axis_enabled(1);        // 모든 축 활성화
        motor_enable();                 // 모터 드라이버 enable 핀 활성화
    } else {
        global_motor_enabled = 0;  // 모터 구동 금지
        set_all_axis_enabled(0);   // 모든 축 비활성화
        trajectory_clear();        // 남아있는 이동 명령 제거
        stepper_stop_all();        // 진행 중인 스텝 출력 정지
        motor_disable();           // 모터 드라이버 출력 차단
    }
    can_send_status();  // enable 처리 결과 송신
}

static void handle_homing(const uint8_t *data, uint8_t len)
{
    uint8_t target_axis;
    uint8_t mode;

    if (len < 2U) return;  // 축 번호와 모드 값이 모두 있어야 처리
    target_axis = data[0];  // 원점복귀 대상 축 번호
    mode = data[1];         // 원점복귀 모드

    if (!global_motor_enabled || global_motor_estop) return;  // 모터 비활성/비상정지 중이면 무시
    if (mode != 0U) {
        global_motor_error = ERR_INVALID_CMD;  // 지원하지 않는 homing 모드
        can_send_status();                     // 에러 상태 송신
        return;
    }

    global_motor_error = ERR_NONE;      // 새 homing 명령 전 에러 초기화
    trajectory_cancel_staging();        // 조립 중이던 다축 이동 명령 취소
    if (target_axis == HOMING_ALL_AXIS) stepper_start_homing_all();  // 전체 축 원점복귀 시작
    else if (target_axis < AXIS_COUNT) stepper_start_homing(target_axis);  // 지정 축 원점복귀 시작
    else global_motor_error = ERR_INVALID_CMD;  // 존재하지 않는 축 번호
    can_send_status();  // homing 시작 또는 에러 상태 송신
}

static void handle_clear_error(const uint8_t *data, uint8_t len)
{
    uint8_t target_axis = HOMING_ALL_AXIS;  // 기본값은 전체 축 대상

    if (len >= 1U) target_axis = data[0];  // 데이터가 있으면 지정 축만 대상으로 처리
    if (target_axis != HOMING_ALL_AXIS && target_axis >= AXIS_COUNT) {
        global_motor_error = ERR_INVALID_CMD;  // 잘못된 축 번호
        can_send_status();                     // 에러 상태 송신
        return;
    }

    global_motor_error = ERR_NONE;       // 에러 코드 초기화
    trajectory_cancel_staging();         // 에러 중 들어오던 이동 명령 조립 취소
    if (!global_motor_estop) global_motor_state = STATE_IDLE;  // 비상정지가 아니면 대기 상태로 복귀
    can_send_status();                   // 에러 해제 결과 송신
}

static void handle_move(const uint8_t *data, uint8_t len)
{
    TrajectoryPoint point;
    uint8_t execute;
    uint8_t stage_result;

    if (len < 8U) return;  // 이동 명령은 8바이트 프레임만 처리

    point.motor_id = data[0] & 0x0FU;       // 하위 4비트: 축 번호
    point.flags = data[0] >> 4;             // 상위 4비트: 실행/좌표 모드 플래그
    execute = (point.flags & 0x08U) ? 1U : 0U;  // 실행 플래그
    point.target_pos = get_i32_le(&data[1]);    // 목표 위치(raw 각도 단위)
    point.speed = get_u16_le(&data[5]);         // 목표 속도 필드
    point.duration_5ms = data[7];               // 이동 시간(5ms 단위)

    if (!execute) return;  // 실행 플래그가 없으면 무시
    if (!global_motor_enabled || global_motor_estop) return;  // 모터 비활성/비상정지 중이면 무시
    if (global_motor_error != ERR_NONE) return;  // 에러 상태에서는 새 이동 명령 차단
    if (point.motor_id >= AXIS_COUNT) {
        trajectory_cancel_staging();      // 잘못된 프레임으로 조립 중인 명령 취소
        global_motor_error = ERR_INVALID_CMD;  // 존재하지 않는 축 번호
        can_send_status();                // 에러 상태 송신
        return;
    }
    if (!axis[point.motor_id].homing_done) {
        trajectory_cancel_staging();      // homing 전 이동 명령은 폐기
        global_motor_error = ERR_INVALID_CMD;  // 원점복귀 전 이동 금지
        can_send_status();                // 에러 상태 송신
        return;
    }

    stage_result = trajectory_stage_command(&point);  // 축별 프레임을 다축 이동 명령으로 조립
    if (stage_result == TRAJECTORY_STAGE_INVALID) {
        global_motor_error = ERR_INVALID_CMD;  // 프레임 순서/플래그/축 번호 오류
        can_send_status();                     // 에러 상태 송신
        return;
    }
    if (stage_result == TRAJECTORY_STAGE_QUEUE_FULL) {
        global_motor_error = ERR_QUEUE_FULL;  // 궤적 큐가 가득 참
        can_send_status();                    // 에러 상태 송신
        return;
    }
}

void can_process_frame(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (id == CAN_ID_ESTOP) handle_estop();                         // 비상정지 명령 처리
    else if (id == CAN_ID_ENABLE) handle_enable(data, len);          // 모터 enable/disable 명령 처리
    else if (id == CAN_ID_HOMING) handle_homing(data, len);          // 원점복귀 명령 처리
    else if (id == CAN_ID_CLEAR_ERROR) handle_clear_error(data, len); // 에러 해제 명령 처리
    else if (id == CAN_ID_BOARD1_MOVE) handle_move(data, len);       // 이동 명령 처리
}
