/*
 * Simple host-side CAN protocol test.
 *
 * build:
 *   gcc -Wall -Wextra board1_virtual_can_test.c -o board1_test
 *
 * run:
 *   ./board1_test
 */

#include <stdint.h>
#include <stdio.h>

#define AXIS_COUNT               4
#define TRAJECTORY_QUEUE_SIZE    32
#define MULTI_AXIS_QUEUE_SIZE    (TRAJECTORY_QUEUE_SIZE / AXIS_COUNT)
#define MOTOR_STEPS_PER_REV      200
#define MICROSTEP                16
#define STAGING_TIMEOUT_MS       20

#define CAN_ID_ESTOP             0x001
#define CAN_ID_ENABLE            0x010
#define CAN_ID_HOMING            0x020
#define CAN_ID_CLEAR_ERROR       0x030
#define CAN_ID_BOARD1_MOVE       0x101

#define STATE_IDLE               1
#define STATE_ERROR              4
#define STATE_ESTOP              5

#define ERR_NONE                 0
#define ERR_INVALID_CMD          1
#define ERR_QUEUE_FULL           5

typedef struct {
    uint8_t motor_id;
    uint8_t flags;
    int32_t target_pos;
    uint16_t speed;
    uint8_t move_duration_units_from_can;
} CanTrajectoryCommand;

typedef struct {
    int32_t target_pos[AXIS_COUNT];
    uint16_t speed[AXIS_COUNT];
    uint8_t move_duration_units;
} MultiAxisMoveCommand;

typedef struct {
    MultiAxisMoveCommand point;
    uint8_t active;
    uint8_t next_motor_id;
    uint32_t start_ms;
} StagingState;

static const int32_t gear_ratio[AXIS_COUNT] = { 20, 20, 75, 30 };
static const int32_t angle_min[AXIS_COUNT] = { -9000, -9000, -8000, -9000 };
static const int32_t angle_max[AXIS_COUNT] = { 18000, 9000, 8000, 9000 };
static MultiAxisMoveCommand queue[MULTI_AXIS_QUEUE_SIZE];
static StagingState staging;
static uint8_t queue_head;
static uint8_t queue_count;
static uint8_t homing_done_bits;
static uint8_t enabled;
static uint8_t state = STATE_IDLE;
static uint8_t error = ERR_NONE;
static uint32_t tick_ms;

static int32_t get_i32_le(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0]) |
                     ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8));
}

static void put_i32_le(uint8_t *p, int32_t v)
{
    uint32_t u = (uint32_t)v;
    // u를 8비트로 쪼개서 4개로 나눔
    p[0] = (uint8_t)u;
    p[1] = (uint8_t)(u >> 8);
    p[2] = (uint8_t)(u >> 16);
    p[3] = (uint8_t)(u >> 24);
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static int32_t angle_raw_to_step(uint8_t axis_id, int32_t angle_raw)
{
    int64_t numerator = (int64_t)angle_raw *
                        gear_ratio[axis_id] *
                        MOTOR_STEPS_PER_REV *
                        MICROSTEP;
    return (int32_t)(numerator / 36000);
}

static uint8_t angle_raw_in_limit(uint8_t axis_id, int32_t angle_raw)
{
    if (axis_id >= AXIS_COUNT) return 0;
    if (angle_raw < angle_min[axis_id]) return 0;
    if (angle_raw > angle_max[axis_id]) return 0;
    return 1;
}

static void send_status(const char *tag)
{
    printf("%-18s state=%u err=%u enabled=%u homing=0x%02X q_free=%u staging=%u next=%u\n",
           tag,
           state,
           error,
           enabled,
           homing_done_bits,
           (unsigned)((MULTI_AXIS_QUEUE_SIZE - queue_count) * AXIS_COUNT),
           staging.active,
           staging.next_motor_id);
}

static void clear_queue(void)
{
    queue_head = 0;
    queue_count = 0;
    staging.active = 0;
    staging.next_motor_id = 0;
    staging.start_ms = 0;
}

static uint8_t queue_push(const MultiAxisMoveCommand *point)
{
    if (queue_count >= MULTI_AXIS_QUEUE_SIZE) return 0;
    queue[queue_head] = *point;
    queue_head = (uint8_t)((queue_head + 1) % MULTI_AXIS_QUEUE_SIZE);
    queue_count++;
    return 1;
}

static uint8_t check_timeout(void)
{
    if (!staging.active) return 0;
    if ((tick_ms - staging.start_ms) <= STAGING_TIMEOUT_MS) return 0;

    staging.active = 0;
    staging.next_motor_id = 0;
    error = ERR_INVALID_CMD;
    state = STATE_ERROR;
    send_status("STAGING_TIMEOUT");
    return 1;
}

static uint8_t stage_point(const CanTrajectoryCommand *point)
{
    uint8_t execute = (point->flags & 0x08) ? 1 : 0;
    uint8_t relative = (point->flags & 0x04) ? 1 : 0;
    uint8_t step_mode = (point->flags & 0x02) ? 1 : 0;
    uint8_t reserved = (point->flags & 0x01) ? 1 : 0;

    if (check_timeout()) return ERR_INVALID_CMD;

    if (!execute || relative || step_mode || reserved || point->motor_id >= AXIS_COUNT ||
        !angle_raw_in_limit(point->motor_id, point->target_pos)) {
        staging.active = 0;
        staging.next_motor_id = 0;
        return ERR_INVALID_CMD;
    }

    if (!staging.active) {
        if (point->motor_id != 0) return ERR_INVALID_CMD;
        staging.active = 1;
        staging.next_motor_id = 0;
        staging.start_ms = tick_ms;
        staging.point.move_duration_units = point->move_duration_units_from_can;
    } else if (point->motor_id != staging.next_motor_id ||
               point->move_duration_units_from_can != staging.point.move_duration_units) {
        staging.active = 0;
        staging.next_motor_id = 0;
        return ERR_INVALID_CMD;
    }

    staging.point.target_pos[point->motor_id] = point->target_pos;
    staging.point.speed[point->motor_id] = point->speed;
    staging.next_motor_id++;

    if (staging.next_motor_id < AXIS_COUNT) return ERR_NONE;
    if (!queue_push(&staging.point)) {
        staging.active = 0;
        staging.next_motor_id = 0;
        return ERR_QUEUE_FULL;
    }

    printf("POINT_COMMIT      move_duration=%u steps=[%ld,%ld,%ld,%ld]\n",
           (unsigned)staging.point.move_duration_units * 5,
           (long)angle_raw_to_step(0, staging.point.target_pos[0]),
           (long)angle_raw_to_step(1, staging.point.target_pos[1]),
           (long)angle_raw_to_step(2, staging.point.target_pos[2]),
           (long)angle_raw_to_step(3, staging.point.target_pos[3]));
    staging.active = 0;
    staging.next_motor_id = 0;
    return ERR_NONE;
}

static void process_frame(uint16_t id, const uint8_t *data, uint8_t len)
{
    if (id == CAN_ID_ESTOP) {
        enabled = 0;
        error = ERR_NONE;
        state = STATE_ESTOP;
        clear_queue();
        send_status("ESTOP");
        return;
    }

    if (id == CAN_ID_ENABLE && len >= 1) {
        enabled = (data[0] == 1) ? 1 : 0;
        if (enabled) {
            error = ERR_NONE;
            state = STATE_IDLE;
        } else {
            clear_queue();
        }
        send_status(enabled ? "ENABLE" : "DISABLE");
        return;
    }

    if (id == CAN_ID_HOMING && len >= 2) {
        if (!enabled || state == STATE_ESTOP || data[1] != 0) return;
        error = ERR_NONE;
        state = STATE_IDLE;
        clear_queue();
        if (data[0] == 255) homing_done_bits = 0x0F;
        else if (data[0] < AXIS_COUNT) homing_done_bits |= (uint8_t)(1 << data[0]);
        else {
            error = ERR_INVALID_CMD;
            state = STATE_ERROR;
        }
        send_status("HOMING");
        return;
    }

    if (id == CAN_ID_CLEAR_ERROR) {
        error = ERR_NONE;
        if (state != STATE_ESTOP) state = STATE_IDLE;
        staging.active = 0;
        staging.next_motor_id = 0;
        send_status("CLEAR_ERROR");
        return;
    }

    if (id == CAN_ID_BOARD1_MOVE && len >= 8) {
        CanTrajectoryCommand point;
        uint8_t result;

        point.motor_id = data[0] & 0x0F;
        point.flags = data[0] >> 4;
        point.target_pos = get_i32_le(&data[1]);
        point.speed = get_u16_le(&data[5]);
        point.move_duration_units_from_can = data[7];

        if (!enabled || state == STATE_ESTOP || error != ERR_NONE) return;
        if (point.motor_id >= AXIS_COUNT) {
            error = ERR_INVALID_CMD;
            state = STATE_ERROR;
            staging.active = 0;
            staging.next_motor_id = 0;
            send_status("MOVE_BAD_AXIS");
            return;
        }
        if (!(homing_done_bits & (1 << point.motor_id))) {
            error = ERR_INVALID_CMD;
            state = STATE_ERROR;
            staging.active = 0;
            staging.next_motor_id = 0;
            send_status("MOVE_REJECT");
            return;
        }
        if (!angle_raw_in_limit(point.motor_id, point.target_pos)) {
            error = ERR_INVALID_CMD;
            state = STATE_ERROR;
            staging.active = 0;
            staging.next_motor_id = 0;
            send_status("MOVE_LIMIT");
            return;
        }

        result = stage_point(&point);
        if (result == ERR_INVALID_CMD) {
            error = ERR_INVALID_CMD;
            state = STATE_ERROR;
            send_status("STAGE_INVALID");
        } else if (result == ERR_QUEUE_FULL) {
            error = ERR_QUEUE_FULL;
            state = STATE_ERROR;
            send_status("QUEUE_FULL");
        } else if (!staging.active) {
            send_status("POINT_QUEUED");
        }
    }
}

static void send_enable(uint8_t on)
{
    uint8_t data[8] = { on };
    process_frame(CAN_ID_ENABLE, data, 1);
}

static void send_homing_all(void)
{
    uint8_t data[8] = { 255, 0 };
    process_frame(CAN_ID_HOMING, data, 2);
}

static void send_move(uint8_t motor_id, uint8_t flags, int32_t target, uint16_t speed, uint8_t move_duration_units_from_can)
{
    uint8_t data[8] = { 0 };
    data[0] = (uint8_t)((flags << 4) | (motor_id & 0x0F));
    put_i32_le(&data[1], target);
    put_u16_le(&data[5], speed);
    data[7] = move_duration_units_from_can;
    process_frame(CAN_ID_BOARD1_MOVE, data, 8);
}

static void send_board1_point(int32_t a0, int32_t a1, int32_t a2, int32_t a3, uint8_t move_duration_units_from_can)
{
    const uint8_t execute = 0x08;
    send_move(0, execute, a0, 0, move_duration_units_from_can);
    send_move(1, execute, a1, 0, move_duration_units_from_can);
    send_move(2, execute, a2, 0, move_duration_units_from_can);
    send_move(3, execute, a3, 0, move_duration_units_from_can);
}

int main(void)
{
    const uint8_t execute = 0x08;
    uint8_t data[8] = { 0 };

    send_status("BOOT");
    send_enable(1);
    send_homing_all();

    send_board1_point(3000, 1000, -500, 0, 10);
    send_board1_point(18100, 0, 0, 0, 10);
    process_frame(CAN_ID_CLEAR_ERROR, data, 1);
    send_move(0, execute, 3100, 0, 10);
    send_move(2, execute, -400, 0, 10);

    process_frame(CAN_ID_CLEAR_ERROR, data, 1);
    send_move(0, execute, 3100, 0, 10);
    send_move(1, execute, 1000, 0, 11);

    process_frame(CAN_ID_CLEAR_ERROR, data, 1);
    send_move(0, execute, 3200, 0, 10);
    tick_ms += 21;
    (void)check_timeout();

    process_frame(CAN_ID_CLEAR_ERROR, data, 1);
    clear_queue();
    for (uint8_t i = 0; i < MULTI_AXIS_QUEUE_SIZE; i++) {
        send_board1_point(100, 100, 100, 100, 10);
    }
    send_board1_point(200, 200, 200, 200, 10);

    process_frame(CAN_ID_ESTOP, data, 0);
    return 0;
}
