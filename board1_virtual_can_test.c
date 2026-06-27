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

#define AXIS_COUNT               4U
#define TRAJECTORY_QUEUE_SIZE    32U
#define MULTI_AXIS_QUEUE_SIZE    (TRAJECTORY_QUEUE_SIZE / AXIS_COUNT)
#define MOTOR_STEPS_PER_REV      200
#define MICROSTEP                16
#define STAGING_TIMEOUT_MS       20U

#define CAN_ID_ESTOP             0x001U
#define CAN_ID_ENABLE            0x010U
#define CAN_ID_HOMING            0x020U
#define CAN_ID_CLEAR_ERROR       0x030U
#define CAN_ID_BOARD1_MOVE       0x101U

#define STATE_IDLE               1U
#define STATE_ERROR              4U
#define STATE_ESTOP              5U

#define ERR_NONE                 0U
#define ERR_INVALID_CMD          1U
#define ERR_QUEUE_FULL           5U

typedef struct {
    uint8_t motor_id;
    uint8_t flags;
    int32_t target_pos;
    uint16_t speed;
    uint8_t duration_5ms;
} TrajectoryPoint;

typedef struct {
    int32_t target_pos[AXIS_COUNT];
    uint16_t speed[AXIS_COUNT];
    uint8_t duration_5ms;
} MultiAxisTrajectoryPoint;

typedef struct {
    MultiAxisTrajectoryPoint point;
    uint8_t active;
    uint8_t next_motor_id;
    uint32_t start_ms;
} StagingState;

static const int32_t gear_ratio[AXIS_COUNT] = { 20, 20, 75, 30 };
static MultiAxisTrajectoryPoint queue[MULTI_AXIS_QUEUE_SIZE];
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

static uint8_t queue_push(const MultiAxisTrajectoryPoint *point)
{
    if (queue_count >= MULTI_AXIS_QUEUE_SIZE) return 0;
    queue[queue_head] = *point;
    queue_head = (uint8_t)((queue_head + 1U) % MULTI_AXIS_QUEUE_SIZE);
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

static uint8_t stage_point(const TrajectoryPoint *point)
{
    uint8_t execute = (point->flags & 0x08U) ? 1U : 0U;
    uint8_t relative = (point->flags & 0x04U) ? 1U : 0U;
    uint8_t step_mode = (point->flags & 0x02U) ? 1U : 0U;
    uint8_t reserved = (point->flags & 0x01U) ? 1U : 0U;

    if (check_timeout()) return ERR_INVALID_CMD;

    if (!execute || relative || step_mode || reserved || point->motor_id >= AXIS_COUNT) {
        staging.active = 0;
        staging.next_motor_id = 0;
        return ERR_INVALID_CMD;
    }

    if (!staging.active) {
        if (point->motor_id != 0U) return ERR_INVALID_CMD;
        staging.active = 1;
        staging.next_motor_id = 0;
        staging.start_ms = tick_ms;
        staging.point.duration_5ms = point->duration_5ms;
    } else if (point->motor_id != staging.next_motor_id ||
               point->duration_5ms != staging.point.duration_5ms) {
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

    printf("POINT_COMMIT      duration_ms=%u steps=[%ld,%ld,%ld,%ld]\n",
           (unsigned)staging.point.duration_5ms * 5U,
           (long)angle_raw_to_step(0U, staging.point.target_pos[0]),
           (long)angle_raw_to_step(1U, staging.point.target_pos[1]),
           (long)angle_raw_to_step(2U, staging.point.target_pos[2]),
           (long)angle_raw_to_step(3U, staging.point.target_pos[3]));
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

    if (id == CAN_ID_ENABLE && len >= 1U) {
        enabled = (data[0] == 1U) ? 1U : 0U;
        if (enabled) {
            error = ERR_NONE;
            state = STATE_IDLE;
        } else {
            clear_queue();
        }
        send_status(enabled ? "ENABLE" : "DISABLE");
        return;
    }

    if (id == CAN_ID_HOMING && len >= 2U) {
        if (!enabled || state == STATE_ESTOP || data[1] != 0U) return;
        error = ERR_NONE;
        state = STATE_IDLE;
        clear_queue();
        if (data[0] == 255U) homing_done_bits = 0x0FU;
        else if (data[0] < AXIS_COUNT) homing_done_bits |= (uint8_t)(1U << data[0]);
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

    if (id == CAN_ID_BOARD1_MOVE && len >= 8U) {
        TrajectoryPoint point;
        uint8_t result;

        point.motor_id = data[0] & 0x0FU;
        point.flags = data[0] >> 4;
        point.target_pos = get_i32_le(&data[1]);
        point.speed = get_u16_le(&data[5]);
        point.duration_5ms = data[7];

        if (!enabled || state == STATE_ESTOP || error != ERR_NONE) return;
        if (!(homing_done_bits & (1U << point.motor_id))) {
            error = ERR_INVALID_CMD;
            state = STATE_ERROR;
            staging.active = 0;
            staging.next_motor_id = 0;
            send_status("MOVE_REJECT");
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
    process_frame(CAN_ID_ENABLE, data, 1U);
}

static void send_homing_all(void)
{
    uint8_t data[8] = { 255U, 0U };
    process_frame(CAN_ID_HOMING, data, 2U);
}

static void send_move(uint8_t motor_id, uint8_t flags, int32_t target, uint16_t speed, uint8_t duration_5ms)
{
    uint8_t data[8] = { 0 };
    data[0] = (uint8_t)((flags << 4) | (motor_id & 0x0FU));
    put_i32_le(&data[1], target);
    put_u16_le(&data[5], speed);
    data[7] = duration_5ms;
    process_frame(CAN_ID_BOARD1_MOVE, data, 8U);
}

static void send_board1_point(int32_t a0, int32_t a1, int32_t a2, int32_t a3, uint8_t duration_5ms)
{
    const uint8_t execute = 0x08U;
    send_move(0U, execute, a0, 0U, duration_5ms);
    send_move(1U, execute, a1, 0U, duration_5ms);
    send_move(2U, execute, a2, 0U, duration_5ms);
    send_move(3U, execute, a3, 0U, duration_5ms);
}

int main(void)
{
    const uint8_t execute = 0x08U;
    uint8_t data[8] = { 0 };

    send_status("BOOT");
    send_enable(1U);
    send_homing_all();

    send_board1_point(3000, 1000, -500, 0, 10U);
    send_move(0U, execute, 3100, 0U, 10U);
    send_move(2U, execute, -400, 0U, 10U);

    process_frame(CAN_ID_CLEAR_ERROR, data, 1U);
    send_move(0U, execute, 3100, 0U, 10U);
    send_move(1U, execute, 1000, 0U, 11U);

    process_frame(CAN_ID_CLEAR_ERROR, data, 1U);
    send_move(0U, execute, 3200, 0U, 10U);
    tick_ms += 21U;
    (void)check_timeout();

    process_frame(CAN_ID_CLEAR_ERROR, data, 1U);
    clear_queue();
    for (uint8_t i = 0; i < MULTI_AXIS_QUEUE_SIZE; i++) {
        send_board1_point(100, 100, 100, 100, 10U);
    }
    send_board1_point(200, 200, 200, 200, 10U);

    process_frame(CAN_ID_ESTOP, data, 0U);
    return 0;
}
