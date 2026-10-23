
 /* Purpose:
 * Handle CAN message encoding and decoding
 * between the STM32 and ODESC motor controller.
 *
 * STATUS: TEMPORARY / NOT HARDWARE-TESTED
 * Assumes ODrive CANSimple-compatible firmware.
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

// Temporary configuration - verify with hardware team
#define ESC_NODE_ID       1u
#define ESC_HEARTBEAT_ID  0x01u
#define ESC_ENCODER_ID    0x09u
#define ESC_SET_VEL_ID    0x0Du

// Represents a classic CAN message
typedef struct {
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
} ESC_CANFrame;

// Stores feedback from the motor controller
typedef struct {
    float position;
    float velocity;

    uint32_t axis_error;
    uint8_t axis_state;

    uint32_t last_heartbeat_ms;
    uint32_t last_encoder_ms;

    bool heartbeat_received;
    bool encoder_received;
} ESC_Telemetry;

// --------------------------------------
// Helper: Convert 4 bytes into a float
// --------------------------------------

static float ESC_BytesToFloat(const uint8_t *data)
{
    uint32_t bits =
        ((uint32_t)data[0]) |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);

    float value;
    memcpy(&value, &bits, sizeof(value));

    return value;
}

// --------------------------------------
// Helper: Convert a float into 4 bytes
// --------------------------------------

static void ESC_FloatToBytes(
    float value,
    uint8_t *data)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));

    for (int i = 0; i < 4; i++) {
        data[i] = (uint8_t)(bits >> (8 * i));
    }
}

// --------------------------------------
// Build a velocity command CAN packet
// --------------------------------------

bool ESC_SetVelocity(
    float velocity,
    float torque_ff,
    ESC_CANFrame *frame)
{
    if (frame == NULL ||
        !isfinite(velocity) ||
        !isfinite(torque_ff))
        return false;

    frame->id =
        (ESC_NODE_ID << 5) | ESC_SET_VEL_ID;

    frame->length = 8;

    // First 4 bytes: target velocity
    ESC_FloatToBytes(velocity, &frame->data[0]);

    // Last 4 bytes: torque feedforward
    ESC_FloatToBytes(torque_ff, &frame->data[4]);

    return true;
}

// --------------------------------------
// Process incoming CAN telemetry
// --------------------------------------

bool ESC_ProcessFrame(
    const ESC_CANFrame *frame,
    ESC_Telemetry *telemetry,
    uint32_t now_ms)
{
    if (frame == NULL || telemetry == NULL)
        return false;

    uint16_t node = frame->id >> 5;
    uint8_t command = frame->id & 0x1F;

    // Ignore messages from other nodes
    if (node != ESC_NODE_ID)
        return false;

    // Heartbeat message
    if (command == ESC_HEARTBEAT_ID &&
        frame->length >= 5 &&
        frame->length <= 8) {

        telemetry->axis_error =
            ((uint32_t)frame->data[0]) |
            ((uint32_t)frame->data[1] << 8) |
            ((uint32_t)frame->data[2] << 16) |
            ((uint32_t)frame->data[3] << 24);

        telemetry->axis_state = frame->data[4];

        telemetry->last_heartbeat_ms = now_ms;
        telemetry->heartbeat_received = true;

        return true;
    }

    // Encoder telemetry message
    if (command == ESC_ENCODER_ID &&
        frame->length == 8) {

        float position =
            ESC_BytesToFloat(&frame->data[0]);

        float velocity =
            ESC_BytesToFloat(&frame->data[4]);

        if (!isfinite(position) || !isfinite(velocity))
            return false;

        telemetry->position = position;
        telemetry->velocity = velocity;

        telemetry->last_encoder_ms = now_ms;
        telemetry->encoder_received = true;

        return true;
    }

    return false;
}

// --------------------------------------
// Check communication health
// --------------------------------------

bool ESC_IsConnected(
    const ESC_Telemetry *telemetry,
    uint32_t now_ms)
{
    if (telemetry == NULL ||
        !telemetry->heartbeat_received)
        return false;

    // Temporary 300 ms heartbeat timeout
    if ((uint32_t)(now_ms -
        telemetry->last_heartbeat_ms) > 300u)
        return false;

    return true;
}
