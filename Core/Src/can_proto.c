#include "../Inc/can_proto.h"
#include "../Inc/stepper.h"
#include "../Inc/trajectory.h"
#include "../Inc/mcp2515.h"

static int32_t decode_int32_le(const uint8_t *p)
{
    uint32_t v = ((uint32_t)p[0]) |
                 ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) |
                 ((uint32_t)p[3] << 24);
    return (int32_t)v;  // CAN 데이터 4바이트를 little-endian int32 값으로 변환
}

static uint16_t decode_uint16_le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8));  // CAN 데이터 2바이트를 little-endian uint16 값으로 변환
}

static void encode_int32_le(uint8_t *p, int32_t value)
{
    uint32_t v = (uint32_t)value;
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

void can_send_status(void)
{
    uint8_t data[8];  // [8바이트] CAN 송신할 status 데이터 

    system_update_state();  // 현재 모터 상태를 전역 상태값에 반영
    data[0] = global_motor_state;           // 현재 시스템 상태
    data[1] = global_motor_error;           // 현재 에러 코드
    data[2] = system_homing_done_bits();    // 각 축의 원점복귀 완료 비트
    data[3] = system_first_moving_axis();   // 움직이는 축 중 첫 번째 축 번호
    data[4] = stepper_limit_switch_status_bits();  // 리미트 스위치 입력 상태 비트
    data[5] = get_free_axis_command_count();      // 남은 궤적 큐 슬롯 수
    data[6] = system_enabled_status();      // 모터 enable/estop 상태
    data[7] = 0;                            // 예비 바이트
    (void)mcp2515_send_std(CAN_ID_BOARD1_STAT, data, 8);  // 보드 상태 프레임 송신
}

void can_send_position_feedback(uint8_t motor_id)
{
    static uint8_t sequence_counter;
    uint8_t data[8];
    uint8_t flags = 0;
    int32_t current_pos_001deg;

    if (motor_id >= AXIS_COUNT) return;

    current_pos_001deg = step_to_angle(motor_id, axis[motor_id].current_step);

    if (axis[motor_id].homing_done) {
        flags |= (1 << 0);  // position valid
        flags |= (1 << 1);  // homed/ready
    }
    if (axis[motor_id].moving || axis[motor_id].homing) {
        flags |= (1 << 2);  // moving
    }
    if (axis[motor_id].homing_done &&
        !axis[motor_id].moving &&
        !axis[motor_id].homing &&
        axis[motor_id].current_step == axis[motor_id].target_step) {
        flags |= (1 << 3);  // target reached
    }

    data[0] = motor_id;
    data[1] = flags;
    encode_int32_le(&data[2], current_pos_001deg);
    data[6] = global_motor_error;
    data[7] = sequence_counter++;

    (void)mcp2515_send_std(CAN_ID_BOARD1_POS, data, 8);
}

void can_send_position_feedback_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        can_send_position_feedback(i);
    }
}

uint8_t can_decode_frame(uint16_t id, const uint8_t *data, uint8_t len, CanCommand *cmd)
{
    if (id == CAN_ID_ESTOP) {
        cmd->type = CAN_CMD_ESTOP;
        return 1;
    }

    if (id == CAN_ID_ENABLE) {
        if (len < 1) return 0;  // enable 값이 없으면 무시
        cmd->type = CAN_CMD_ENABLE;
        cmd->enable = data[0];
        return 1;
    }

    if (id == CAN_ID_HOMING) {
        if (len < 2) return 0;  // 축 번호와 모드 값이 모두 있어야 처리
        cmd->type = CAN_CMD_HOMING;
        cmd->target_axis = data[0];  // 원점복귀 대상 축 번호
        cmd->homing_mode = data[1];  // 원점복귀 모드
        return 1;
    }

    if (id == CAN_ID_CLEAR_ERROR) {
        if (len < 1) return 0;  // 최종 프로토콜은 Byte0=0xFF 전체 clear를 명시해야 함
        cmd->type = CAN_CMD_CLEAR_ERROR;
        cmd->target_axis = data[0];  // 전체 clear는 0xFF만 허용
        return 1;
    }

    if (id == CAN_ID_BOARD1_MOVE) {
        if (len < 8) return 0;  // 이동 명령은 8바이트 프레임만 처리
        cmd->type = CAN_CMD_MOVE;
        cmd->trajectory_command.motor_id = data[0] & 0x0F;       // 하위 4비트: 축 번호
        cmd->trajectory_command.flags = data[0] >> 4;             // 상위 4비트: 실행/좌표 모드 플래그
        cmd->trajectory_command.target_pos = decode_int32_le(&data[1]);  // 목표 위치(raw 각도 단위)
        cmd->trajectory_command.speed = decode_uint16_le(&data[5]);       // 목표 속도 필드
        cmd->trajectory_command.move_duration_units_from_can = data[7];  // 이동 시간(5ms 단위)
        return 1;
    }

    return 0;
}
