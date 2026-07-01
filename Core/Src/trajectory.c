#include "../Inc/trajectory.h"

static MultiAxisMoveCommand pending_command;  // 미완성 다축 명령
static uint8_t pending_receiving;             // 수신 진행 중
static uint8_t pending_next_motor_id;         // 다음 수신 축
static uint32_t pending_start_ms;             // 수신 시작 시각
static MultiAxisMoveCommand move_circular_queue[MULTI_AXIS_QUEUE_SIZE];  // 실행 대기 중인 다축 이동 명령 원형 큐
static volatile uint8_t move_circular_queue_head;   // 큐에 새 명령을 넣을 위치
static volatile uint8_t move_circular_queue_tail;   // 큐에서 다음 명령을 꺼낼 위치
static volatile uint8_t move_circular_queue_count;  // 큐에 저장된 명령 개수

// Board1 local motor 0~3 controls robot arm axes 2~5.
static const int32_t gear_ratio[AXIS_COUNT] = { 20, 50, 30, 120 };  // 각 축 감속비
static const int32_t motor_steps_per_rev[AXIS_COUNT] = { 200, 200, 200, 48 };  // 5축은 7.5도 스텝모터
static const int32_t angle_min[AXIS_COUNT] = {-9000, -8000, -9000, -17000};  // 각 축 최소 각도, 100 곱한 각도
static const int32_t angle_max[AXIS_COUNT] = {9000, 8000, 9000, 17000};  // 각 축 최대 각도,100 곱한 각도
static const int32_t home_angle[AXIS_COUNT] = {-9000, -8000, -9000, -17000};  // homing 완료 후 논리 home 100 곱한 각도

static uint8_t move_circular_queue_push(const MultiAxisMoveCommand *command)
{
    if (move_circular_queue_count >= MULTI_AXIS_QUEUE_SIZE) return 0;

    move_circular_queue[move_circular_queue_head] = *command;
    move_circular_queue_head = (uint8_t)((move_circular_queue_head + 1) % MULTI_AXIS_QUEUE_SIZE);
    move_circular_queue_count++;
    return 1;
}

static uint8_t move_circular_queue_pop(MultiAxisMoveCommand *command)
{
    if (move_circular_queue_count == 0) return 0;

    *command = move_circular_queue[move_circular_queue_tail];
    move_circular_queue_tail = (uint8_t)((move_circular_queue_tail + 1) % MULTI_AXIS_QUEUE_SIZE);
    move_circular_queue_count--;
    return 1;
}

static void pending_clear(void)
{
    pending_receiving = 0;      // 수신 상태 초기화
    pending_next_motor_id = 0;  // 다음 수신 축 번호 초기화
    pending_start_ms = 0;       // 시작 시간 초기화
}

void trajectory_cancel_pending(void)
{
    pending_clear();  // 수신 중이던 다축 이동 명령 취소
}

void trajectory_clear(void)
{
    move_circular_queue_head = 0;   // 큐 입력 위치 초기화
    move_circular_queue_tail = 0;   // 큐 출력 위치 초기화
    move_circular_queue_count = 0;  // 큐 명령 개수 초기화
    pending_clear();  // 수신 중인 명령도 함께 초기화

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis[i].move_total_time_ms = 0;              // 현재 실행 중인 이동 시간 초기화
        axis[i].move_elapsed_time_ms = 0;            // 이동 진행 시간 초기화
        axis[i].move_step_offset = 0;            // 이동해야 할 스텝 수 초기화
        axis[i].move_start_step = axis[i].current_step;  // 시작 위치를 현재 위치로 맞춤
        axis[i].move_end_step = axis[i].current_step;    // 종료 위치를 현재 위치로 맞춤
        if (!axis[i].homing) axis[i].moving = 0;         // homing 중이 아니면 이동 상태 해제
        axis[i].target_step = axis[i].current_step;      // 목표 위치를 현재 위치로 맞춤
    }
}

// 외부에서 사용함: 남은 수신 가능한 명령 수
uint8_t get_available_axis_command_count(void)
{
    return (uint8_t)((MULTI_AXIS_QUEUE_SIZE - move_circular_queue_count) * AXIS_COUNT);  // 남은 다축 큐를 축별 명령 개수로 환산
}

int32_t angle_to_step(uint8_t axis_id, int32_t angle_raw)
{
    int64_t step_value;  // 곱셈 중 오버플로우를 줄이기 위한 64비트 중간값
    if (axis_id >= AXIS_COUNT) return 0;  // 잘못된 축 번호는 0스텝 처리
    step_value = (int64_t)angle_raw *
                 gear_ratio[axis_id] *
                 motor_steps_per_rev[axis_id] *
                 MICROSTEP;
    return (int32_t)(step_value / 36000);  // 100 곱한 각도를 마이크로스텝으로 변환 360 * 100
}

int32_t step_to_angle(uint8_t axis_id, int32_t step)
{
    int64_t angle_value;
    int64_t steps_per_output_rev;

    if (axis_id >= AXIS_COUNT) return 0;

    angle_value = (int64_t)step * 36000;
    steps_per_output_rev = (int64_t)gear_ratio[axis_id] *
                           motor_steps_per_rev[axis_id] *
                           MICROSTEP;

    if (angle_value >= 0) angle_value += steps_per_output_rev / 2;
    else angle_value -= steps_per_output_rev / 2;

    return (int32_t)(angle_value / steps_per_output_rev);
}

int32_t get_home_angle(uint8_t axis_id)
{
    if (axis_id >= AXIS_COUNT) return 0;
    return home_angle[axis_id];
}

static uint8_t target_step_from_cmd(uint8_t axis_id, int32_t target_pos, uint8_t flags, int32_t *target_step)
{
    uint8_t relative = (flags & 0x04) ? 1 : 0;
    uint8_t step_mode = (flags & 0x02) ? 1 : 0;
    int64_t resolved;
    int32_t min_step;
    int32_t max_step;

    if (axis_id >= AXIS_COUNT) return 0;

    resolved = step_mode ? target_pos : angle_to_step(axis_id, target_pos);
    if (relative) resolved += axis[axis_id].current_step;
    if (resolved < INT32_MIN || resolved > INT32_MAX) return 0;

    *target_step = (int32_t)resolved;

    min_step = angle_to_step(axis_id, angle_min[axis_id]);
    max_step = angle_to_step(axis_id, angle_max[axis_id]);
    if (min_step > max_step) {
        int32_t tmp = min_step;
        min_step = max_step;
        max_step = tmp;
    }

    if (*target_step < min_step) return 0;
    if (*target_step > max_step) return 0;
    return 1;
}

uint8_t trajectory_check_pending_timeout(void)
{
    if (!pending_receiving) return 0;  // 수신 중인 명령이 없으면 정상
    if ((global_tick_ms - pending_start_ms) <= PENDING_TIMEOUT_MS) return 0;  // 제한 시간 이내면 계속 대기

    pending_clear();                    // 시간 초과된 명령 취소
    global_motor_error = ERR_INVALID_CMD;  // 축별 프레임 누락을 잘못된 명령으로 처리
    return 1;                           // 타임아웃 발생
}

uint8_t trajectory_add_pending_command(const CanTrajectoryCommand *command)
{
    uint8_t execute = (command->flags & 0x08) ? 1 : 0;    // 실행 플래그
    uint8_t relative = (command->flags & 0x04) ? 1 : 0;   // 상대좌표 플래그
    uint8_t step_mode = (command->flags & 0x02) ? 1 : 0;  // 스텝 직접 명령 플래그
    uint8_t reserved = (command->flags & 0x01) ? 1 : 0;   // 예약 플래그

    if (trajectory_check_pending_timeout()) return TRAJECTORY_PENDING_INVALID;  // 이전 명령 수신이 시간 초과되면 실패

    if (!execute || reserved || command->motor_id >= AXIS_COUNT) {
        pending_clear();  // 지원하지 않는 플래그 또는 축 번호면 수신 취소
        return TRAJECTORY_PENDING_INVALID;  // 잘못된 명령
    }
    if (!relative && !step_mode &&
        (command->target_pos < angle_min[command->motor_id] ||
         command->target_pos > angle_max[command->motor_id])) {
        pending_clear();  // 축별 동작 범위를 벗어난 목표 각도면 수신 취소
        return TRAJECTORY_PENDING_INVALID;  // 잘못된 명령
    }

    if (!pending_receiving) {
        if (command->motor_id != 0) return TRAJECTORY_PENDING_INVALID;  // 다축 명령은 0번 축부터 순서대로 수신
        pending_clear();  // 새 수신 시작 전 상태 초기화
        pending_receiving = 1;  // 수신 시작
        pending_start_ms = global_tick_ms;  // 시작 시간 저장
        pending_command.move_duration_5ms = command->move_duration_5ms;  // 모든 축에 적용할 이동 시간 저장
    } else {
        if (command->motor_id != pending_next_motor_id) {
            pending_clear();  // 예상한 축 순서가 아니면 수신 취소
            return TRAJECTORY_PENDING_INVALID;  // 잘못된 명령 순서
        }
        if (command->move_duration_5ms != pending_command.move_duration_5ms) {
            pending_clear();  // 축별 이동 시간이 다르면 동기 이동 불가
            return TRAJECTORY_PENDING_INVALID;  // 잘못된 명령
        }
    }

    pending_command.target_pos[command->motor_id] = command->target_pos;  // 해당 축 목표 위치 저장
    pending_command.speed[command->motor_id] = command->speed;            // 해당 축 속도 필드 저장
    pending_command.flags[command->motor_id] = command->flags;            // 해당 축 제어 플래그 저장
    pending_next_motor_id++;                                        // 다음에 받을 축 번호 증가

    if (pending_next_motor_id < AXIS_COUNT) return TRAJECTORY_PENDING_WAITING;  // 아직 모든 축 명령을 받지 못함

    if (!move_circular_queue_push(&pending_command)) {
        pending_clear();  // 큐 입력 실패 시 수신 상태 초기화
        return TRAJECTORY_PENDING_QUEUE_FULL;  // 큐 가득 참
    }

    pending_clear();  // 큐에 넣은 뒤 수신 상태 초기화
    return TRAJECTORY_PENDING_COMMITTED;  // 다축 명령 수신 및 큐 입력 완료
}

static void try_start_next_command(void)
{
    MultiAxisMoveCommand command;  // 큐에서 꺼낸 다음 다축 이동 명령
    int32_t target_step[AXIS_COUNT];
    uint16_t duration_ms;

    if (!global_motor_enabled || global_motor_estop || global_motor_error != ERR_NONE) return;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].homing || axis[i].moving || axis[i].move_total_time_ms != 0) return;
    }
    if (!move_circular_queue_pop(&command)) return;  // 대기 중인 명령이 없으면 종료

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (!target_step_from_cmd(i, command.target_pos[i], command.flags[i], &target_step[i])) {
            global_motor_error = ERR_INVALID_CMD;
            return;
        }
    }

    duration_ms = (uint16_t)command.move_duration_5ms * 5;  // 5ms 단위를 실제 ms로 변환
    if (duration_ms == 0) duration_ms = 1;  // 0ms 명령은 최소 1ms로 보정

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        (void)command.speed[i];  // 현재 보간은 duration 기반이라 speed 필드는 보관만 함
        axis[i].move_start_step = axis[i].current_step;  // 이동 시작 위치
        axis[i].move_end_step = target_step[i];          // 이동 종료 위치
        axis[i].move_step_offset = target_step[i] - axis[i].move_start_step;  // 총 이동 스텝
        axis[i].move_total_time_ms = duration_ms;    // 이동 총 실행 시간
        axis[i].move_elapsed_time_ms = 0;            // 이동 경과 시간 초기화
        axis[i].target_step = axis[i].move_start_step;  // 첫 목표 위치를 시작 위치로 설정
        axis[i].moving = (axis[i].move_step_offset != 0) ? 1 : 0;  // 이동량이 있으면 moving 표시
    }
}

static float s_curve(uint16_t elapsed_time, uint16_t total_time)
{
    if (total_time == 0) return 1.0f;
    if (elapsed_time >= total_time) return 1.0f;

    float t = (float)elapsed_time / (float)total_time;  // 진행률 0.0f ~ 1.0f

    // Ken Perlin의 5차 s커브: https://en.wikipedia.org/wiki/Smoothstep
    return t * t * t * (t * (6.0f * t - 15.0f) + 10.0f);
}

void trajectory_1ms_interrupt(void)
{
    try_start_next_command();  // 실행 가능한 다음 이동 명령 시작

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].homing || axis[i].move_total_time_ms == 0) continue;  // homing 중이거나 실행 중인 이동이 없으면 건너뜀

        if (axis[i].move_elapsed_time_ms < axis[i].move_total_time_ms) {
            
            // s커브로 진행율
            float move_ratio = s_curve(axis[i].move_elapsed_time_ms, axis[i].move_total_time_ms);

            // 선형보간법 Linear interpolation: float lerp(float v0, float v1, float t) { return v0 + t * (v1 - v0); }
            axis[i].target_step = axis[i].move_start_step + (int32_t)(move_ratio *(float)(axis[i].move_end_step - axis[i].move_start_step));
            axis[i].move_elapsed_time_ms++;  // 1ms 추가
            axis[i].moving = (axis[i].move_step_offset != 0) ? 1 : 0;  // 이동량이 있으면 moving 유지

        } else {
            axis[i].target_step = axis[i].move_end_step;  // 마지막 목표 위치를 종료 위치로
            axis[i].move_total_time_ms = 0;                
            axis[i].move_elapsed_time_ms = 0;               
            axis[i].moving = (axis[i].current_step != axis[i].target_step) ? 1 : 0;  // target 남았으면 moving 유지
        }
    }
}
