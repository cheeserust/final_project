/*
 * Arduino Uno/Nano Board2 firmware for robot arm axis 5.
 *
 * Libraries:
 * - MCP_CAN_lib by Cory J. Fowler
 * - AccelStepper by Mike McCauley
 *
 * Hardware:
 * - MCP2515: CS D10, INT D2, SPI D11/D12/D13, 8 MHz crystal, 500 kbps CAN
 * - TB6600: STEP D6, DIR D7, ENA D8 active-low, 5 us STEP pulse
 * - Limit switch: D3, INPUT_PULLUP, active-high/inverted
 */

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <AccelStepper.h>
#include <stdint.h>

static const uint8_t PIN_CAN_CS = 10;
static const uint8_t PIN_CAN_INT = 2;
static const uint8_t PIN_STEP = 6;
static const uint8_t PIN_DIR = 7;
static const uint8_t PIN_ENABLE = 8;
static const uint8_t PIN_LIMIT = 3;

static const unsigned long CAN_ID_ESTOP = 0x001;
static const unsigned long CAN_ID_ENABLE = 0x010;
static const unsigned long CAN_ID_HOMING = 0x020;
static const unsigned long CAN_ID_CLEAR_ERROR = 0x030;
static const unsigned long BOARD_MOVE_CAN_ID = 0x102;
static const unsigned long BOARD_STATUS_CAN_ID = 0x202;
static const unsigned long BOARD_POSITION_CAN_ID = 0x302;

static const uint8_t CAN_CTRL_EXECUTE = 0x80;
static const uint8_t CAN_CTRL_RELATIVE = 0x40;
static const uint8_t CAN_CTRL_STEP_MODE = 0x20;
static const uint8_t CAN_CTRL_RESERVED = 0x10;
static const uint8_t CAN_CTRL_MOTOR_MASK = 0x0F;

static const uint8_t STATE_INIT = 0;
static const uint8_t STATE_IDLE = 1;
static const uint8_t STATE_HOMING = 2;
static const uint8_t STATE_MOVING = 3;
static const uint8_t STATE_ERROR = 4;
static const uint8_t STATE_ESTOP = 5;
static const uint8_t STATE_DISABLED = 6;

static const uint8_t ERR_NONE = 0;
static const uint8_t ERR_INVALID_CMD = 1;
static const uint8_t ERR_LIMIT_SWITCH_DETECTED = 2;
static const uint8_t ERR_DRIVER_FAULT = 3;
static const uint8_t ERR_QUEUE_FULL = 5;

static const uint8_t HOMING_ALL_AXIS = 255;
static const int32_t GEAR_RATIO = 120;
static const int32_t MOTOR_STEPS_PER_REV = 48;
static const int32_t MICROSTEP = 16;
static const int32_t MIN_ANGLE_RAW = -9000;
static const int32_t MAX_ANGLE_RAW = 9000;
static const int32_t HOME_ANGLE_RAW = -9000;
static const int8_t HOME_DIR = -1;
static const uint16_t LIMIT_DEBOUNCE_MS = 20;
static const float MOTION_MAX_STEP_RATE_SPS = 1250.0f;
static const float HOMING_STEP_RATE_SPS = 1250.0f;
static const uint8_t QUEUE_SIZE = 32;

struct MotionPoint {
    int32_t targetStep;
    float speedSps;
};

static MCP_CAN CAN0(PIN_CAN_CS);
static AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

static MotionPoint queue[QUEUE_SIZE];
static uint8_t queueHead = 0;
static uint8_t queueCount = 0;

static bool canReady = false;
static bool enabled = false;
static bool estopActive = false;
static bool homed = false;
static bool homingActive = false;
static bool motionActive = false;
static bool statusEvent = false;

static uint8_t state = STATE_INIT;
static uint8_t errorCode = ERR_NONE;
static uint8_t statusSequence = 0;

static int32_t currentStep = 0;
static int32_t activeTargetStep = 0;

static bool limitLastRaw = false;
static bool limitDebounced = false;
static uint32_t limitChangedMs = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastPositionMs = 0;

static int32_t readI32Le(const uint8_t *p)
{
    uint32_t v = ((uint32_t)p[0]) |
                 ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) |
                 ((uint32_t)p[3] << 24);
    return (int32_t)v;
}

static uint16_t readU16Le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8));
}

static void writeI16Le(uint8_t *p, int16_t value)
{
    uint16_t v = (uint16_t)value;
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static int16_t clampI16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static int32_t angleToStep(int32_t angleRaw)
{
    int64_t stepValue = (int64_t)angleRaw *
                        GEAR_RATIO *
                        MOTOR_STEPS_PER_REV *
                        MICROSTEP;
    return (int32_t)(stepValue / 36000);
}

static int32_t stepToAngle(int32_t step)
{
    int64_t stepsPerOutputRev = (int64_t)GEAR_RATIO *
                                MOTOR_STEPS_PER_REV *
                                MICROSTEP;
    int64_t angleValue = (int64_t)step * 36000;

    if (angleValue >= 0) angleValue += stepsPerOutputRev / 2;
    else angleValue -= stepsPerOutputRev / 2;

    return (int32_t)(angleValue / stepsPerOutputRev);
}

static bool frameIsExact8(uint8_t len)
{
    return len == 8;
}

static bool reservedZero(const uint8_t *data, uint8_t startIndex)
{
    for (uint8_t i = startIndex; i < 8; i++) {
        if (data[i] != 0) return false;
    }
    return true;
}

static void requestStatusEvent()
{
    statusEvent = true;
}

static bool limitPressedRaw()
{
    return digitalRead(PIN_LIMIT) == HIGH;
}

static void updateLimitDebounce(uint32_t nowMs)
{
    bool raw = limitPressedRaw();

    if (raw != limitLastRaw) {
        limitLastRaw = raw;
        limitChangedMs = nowMs;
    }

    if ((uint32_t)(nowMs - limitChangedMs) >= LIMIT_DEBOUNCE_MS) {
        limitDebounced = raw;
    }
}

static void setMotorEnabled(bool on)
{
    digitalWrite(PIN_ENABLE, on ? LOW : HIGH);
}

static void syncCurrentStep()
{
    currentStep = (int32_t)stepper.currentPosition();
}

static void clearQueueAndMotion()
{
    syncCurrentStep();
    queueHead = 0;
    queueCount = 0;
    motionActive = false;
    activeTargetStep = currentStep;
    stepper.moveTo(currentStep);
    stepper.setSpeed(0.0f);
}

static void enterError(uint8_t code)
{
    clearQueueAndMotion();
    homingActive = false;
    errorCode = code;
    if (estopActive) state = STATE_ESTOP;
    else state = STATE_ERROR;
    requestStatusEvent();
}

static bool motionCommandAllowed()
{
    if (!enabled) return false;
    if (estopActive) return false;
    if (errorCode != ERR_NONE) return false;
    if (homingActive) return false;
    if (!homed) return false;
    return true;
}

static uint8_t queueFreeCount()
{
    return (uint8_t)(QUEUE_SIZE - queueCount);
}

static int32_t queueReferenceStep()
{
    if (queueCount > 0) {
        uint8_t last = (uint8_t)((queueHead + queueCount - 1) % QUEUE_SIZE);
        return queue[last].targetStep;
    }
    if (motionActive) return activeTargetStep;
    return currentStep;
}

static uint32_t absStepDelta(int32_t a, int32_t b)
{
    int64_t d = (int64_t)a - (int64_t)b;
    if (d < 0) d = -d;
    if (d > 0xFFFFFFFFLL) return 0xFFFFFFFFUL;
    return (uint32_t)d;
}

static bool queuePush(int32_t targetStep, uint8_t duration5ms)
{
    if (queueCount >= QUEUE_SIZE) return false;

    int32_t referenceStep = queueReferenceStep();
    uint32_t delta = absStepDelta(targetStep, referenceStep);
    uint32_t requestedMs = (uint32_t)duration5ms * 5UL;
    uint32_t effectiveMs = requestedMs == 0 ? 1 : requestedMs;

    if (delta > 0) {
        uint32_t requiredMs =
            (uint32_t)((delta * 1000UL + (uint32_t)MOTION_MAX_STEP_RATE_SPS - 1UL) /
                       (uint32_t)MOTION_MAX_STEP_RATE_SPS);
        if (requiredMs > effectiveMs) effectiveMs = requiredMs;
    }

    float speed = delta == 0 ? MOTION_MAX_STEP_RATE_SPS :
                  ((float)delta * 1000.0f) / (float)effectiveMs;
    if (speed > MOTION_MAX_STEP_RATE_SPS) speed = MOTION_MAX_STEP_RATE_SPS;

    uint8_t tail = (uint8_t)((queueHead + queueCount) % QUEUE_SIZE);
    queue[tail].targetStep = targetStep;
    queue[tail].speedSps = speed;
    queueCount++;
    return true;
}

static bool queuePop(MotionPoint *point)
{
    if (queueCount == 0) return false;

    *point = queue[queueHead];
    queueHead = (uint8_t)((queueHead + 1) % QUEUE_SIZE);
    queueCount--;
    return true;
}

static bool resolveTargetStep(int32_t targetRaw,
                              bool relative,
                              bool stepMode,
                              int32_t *targetStep)
{
    int64_t resolved = stepMode ? (int64_t)targetRaw : (int64_t)angleToStep(targetRaw);
    if (relative) resolved += currentStep;

    if (resolved < ((int64_t)-2147483647L - 1L)) return false;
    if (resolved > 2147483647L) return false;

    int32_t resolvedStep = (int32_t)resolved;
    int32_t minStep = angleToStep(MIN_ANGLE_RAW);
    int32_t maxStep = angleToStep(MAX_ANGLE_RAW);

    if (minStep > maxStep) {
        int32_t tmp = minStep;
        minStep = maxStep;
        maxStep = tmp;
    }

    if (resolvedStep < minStep) return false;
    if (resolvedStep > maxStep) return false;

    *targetStep = resolvedStep;
    return true;
}

static uint8_t makeAxisFlags()
{
    uint8_t flags = 0;
    bool moving = homingActive || (motionActive && currentStep != activeTargetStep);

    if (homed) flags |= 0x01;
    if (homed && enabled && !estopActive && state != STATE_ERROR && errorCode == ERR_NONE) {
        flags |= 0x02;
    }
    if (moving) flags |= 0x04;
    if (homed && !moving && !homingActive && currentStep == activeTargetStep) {
        flags |= 0x08;
    }

    return flags;
}

static void sendStatus()
{
    if (!canReady) return;

    uint8_t data[8];
    data[0] = state;
    data[1] = errorCode;
    data[2] = makeAxisFlags() & 0x0F;
    data[3] = 0;
    data[4] = limitPressedRaw() ? 0x01 : 0x00;
    data[5] = estopActive ? 0 : queueFreeCount();
    data[6] = enabled ? 1 : 0;
    data[7] = statusSequence++;

    CAN0.sendMsgBuf(BOARD_STATUS_CAN_ID, 0, 8, data);
}

static void sendPositionFeedback()
{
    if (!canReady) return;

    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    writeI16Le(&data[0], clampI16(stepToAngle(currentStep)));

    CAN0.sendMsgBuf(BOARD_POSITION_CAN_ID, 0, 8, data);
}

static void flushStatusEvent()
{
    if (!statusEvent) return;
    statusEvent = false;
    sendStatus();
}

static void startNextMotionIfNeeded()
{
    if (motionActive || homingActive || errorCode != ERR_NONE || !enabled) return;

    MotionPoint point;
    if (!queuePop(&point)) {
        if (state == STATE_MOVING) state = STATE_IDLE;
        return;
    }

    activeTargetStep = point.targetStep;
    stepper.moveTo(activeTargetStep);
    stepper.setSpeed(point.speedSps);
    motionActive = true;
    state = STATE_MOVING;
    requestStatusEvent();
}

static void finishCurrentMotion()
{
    motionActive = false;
    activeTargetStep = currentStep;
    stepper.moveTo(currentStep);
    stepper.setSpeed(0.0f);
    state = queueCount > 0 ? STATE_MOVING : STATE_IDLE;
    requestStatusEvent();
}

static void serviceMotion()
{
    if (!enabled || estopActive || errorCode != ERR_NONE || homingActive) return;

    startNextMotionIfNeeded();
    if (!motionActive) return;

    syncCurrentStep();
    if (currentStep == activeTargetStep) {
        finishCurrentMotion();
        startNextMotionIfNeeded();
        return;
    }

    bool movingTowardHomeLimit =
        (HOME_DIR > 0) ? (activeTargetStep > currentStep) :
                         (activeTargetStep < currentStep);
    if (movingTowardHomeLimit && limitDebounced) {
        enterError(ERR_LIMIT_SWITCH_DETECTED);
        return;
    }

    stepper.runSpeedToPosition();
    syncCurrentStep();

    if (currentStep == activeTargetStep) {
        finishCurrentMotion();
    }
}

static void finishHoming()
{
    int32_t homeStep = angleToStep(HOME_ANGLE_RAW);

    stepper.setCurrentPosition(homeStep);
    currentStep = homeStep;
    activeTargetStep = homeStep;
    stepper.moveTo(homeStep);
    stepper.setSpeed(0.0f);
    queueHead = 0;
    queueCount = 0;
    motionActive = false;
    homingActive = false;
    homed = true;
    state = STATE_IDLE;
    requestStatusEvent();
}

static void serviceHoming()
{
    if (!homingActive) return;

    if (!enabled || estopActive || errorCode != ERR_NONE) {
        clearQueueAndMotion();
        homingActive = false;
        return;
    }

    if (limitDebounced) {
        finishHoming();
        return;
    }

    state = STATE_HOMING;
    stepper.setSpeed((float)HOME_DIR * HOMING_STEP_RATE_SPS);
    stepper.runSpeed();
    syncCurrentStep();
}

static void handleEstop(uint8_t len, const uint8_t *data)
{
    if (!frameIsExact8(len) || data[0] != 1 || !reservedZero(data, 1)) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    clearQueueAndMotion();
    homingActive = false;
    estopActive = true;
    state = STATE_ESTOP;
    requestStatusEvent();
}

static void handleEnableDisable(uint8_t len, const uint8_t *data)
{
    if (!frameIsExact8(len) || !reservedZero(data, 1)) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    if (data[0] == 1) {
        estopActive = false;
        errorCode = ERR_NONE;
        enabled = true;
        setMotorEnabled(true);
        if (state == STATE_INIT || state == STATE_DISABLED ||
            state == STATE_ERROR || state == STATE_ESTOP) {
            state = STATE_IDLE;
        }
    } else if (data[0] == 0) {
        clearQueueAndMotion();
        homingActive = false;
        enabled = false;
        setMotorEnabled(false);
        state = STATE_DISABLED;
    } else {
        enterError(ERR_INVALID_CMD);
        return;
    }

    requestStatusEvent();
}

static void handleHoming(uint8_t len, const uint8_t *data)
{
    if (!frameIsExact8(len) || data[0] != HOMING_ALL_AXIS ||
        data[1] != 0 || !reservedZero(data, 2)) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    if (!enabled || estopActive || errorCode != ERR_NONE) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    clearQueueAndMotion();
    homed = false;
    homingActive = true;
    state = STATE_HOMING;
    stepper.setSpeed((float)HOME_DIR * HOMING_STEP_RATE_SPS);
    requestStatusEvent();
}

static void handleClearError(uint8_t len, const uint8_t *data)
{
    if (!frameIsExact8(len) || data[0] != HOMING_ALL_AXIS || !reservedZero(data, 1)) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    estopActive = false;
    errorCode = ERR_NONE;
    clearQueueAndMotion();
    state = enabled ? STATE_IDLE : STATE_DISABLED;
    requestStatusEvent();
}

static void handleMove(uint8_t len, const uint8_t *data)
{
    if (!frameIsExact8(len)) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    syncCurrentStep();
    if (!motionCommandAllowed()) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    uint8_t b0 = data[0];
    bool execute = (b0 & CAN_CTRL_EXECUTE) != 0;
    bool relative = (b0 & CAN_CTRL_RELATIVE) != 0;
    bool stepMode = (b0 & CAN_CTRL_STEP_MODE) != 0;
    uint8_t motorId = b0 & CAN_CTRL_MOTOR_MASK;

    if (!execute || (b0 & CAN_CTRL_RESERVED) || motorId != 0) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    int32_t targetRaw = readI32Le(&data[1]);
    (void)readU16Le(&data[5]);
    uint8_t duration5ms = data[7];
    int32_t targetStep = 0;

    if (!resolveTargetStep(targetRaw, relative, stepMode, &targetStep)) {
        enterError(ERR_INVALID_CMD);
        return;
    }

    if (!queuePush(targetStep, duration5ms)) {
        enterError(ERR_QUEUE_FULL);
        return;
    }

    requestStatusEvent();
}

static void handleCanFrame(unsigned long id, uint8_t len, const uint8_t *data)
{
    switch (id) {
    case CAN_ID_ESTOP:
        handleEstop(len, data);
        break;
    case CAN_ID_ENABLE:
        handleEnableDisable(len, data);
        break;
    case CAN_ID_HOMING:
        handleHoming(len, data);
        break;
    case CAN_ID_CLEAR_ERROR:
        handleClearError(len, data);
        break;
    case BOARD_MOVE_CAN_ID:
        handleMove(len, data);
        break;
    default:
        break;
    }
}

static void pollCan()
{
    if (!canReady) return;

    uint8_t budget = 16;
    while (budget-- > 0 && CAN0.checkReceive() == CAN_MSGAVAIL) {
        unsigned long rxId = 0;
        uint8_t len = 0;
        uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

        if (CAN0.readMsgBuf(&rxId, &len, data) == CAN_OK) {
            handleCanFrame(rxId, len, data);
        }
    }
}

void setup()
{
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_LIMIT, INPUT_PULLUP);
    pinMode(PIN_CAN_INT, INPUT_PULLUP);
    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR, LOW);
    setMotorEnabled(false);

    stepper.setMaxSpeed(MOTION_MAX_STEP_RATE_SPS);
    stepper.setMinPulseWidth(5);
    stepper.setCurrentPosition(0);
    activeTargetStep = 0;

    uint32_t now = millis();
    limitLastRaw = limitPressedRaw();
    limitDebounced = limitLastRaw;
    limitChangedMs = now;
    lastStatusMs = now;
    lastPositionMs = now;

    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);
        canReady = true;
        state = STATE_DISABLED;
    } else {
        canReady = false;
        errorCode = ERR_DRIVER_FAULT;
        state = STATE_ERROR;
    }

    sendStatus();
    sendPositionFeedback();
}

void loop()
{
    uint32_t now = millis();

    syncCurrentStep();
    updateLimitDebounce(now);
    pollCan();

    if (homingActive) serviceHoming();
    else serviceMotion();

    syncCurrentStep();

    if ((uint32_t)(now - lastPositionMs) >= 100) {
        lastPositionMs = now;
        sendPositionFeedback();
    }

    if ((uint32_t)(now - lastStatusMs) >= 100) {
        lastStatusMs = now;
        sendStatus();
    }

    flushStatusEvent();
}
