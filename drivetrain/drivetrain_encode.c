#include "drivetrain_encode.h"

#include <string.h>

/* Little-endian byte pack helpers (protocol doc Part 0). */

static void pack_u16_le(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8) & 0xFFu);
}

/* memcpy into a uint32_t instead of pointer-casting float* -- avoids
 * undefined behavior from breaking C's strict-aliasing rule. */
static void pack_f32_le(uint8_t *dst, float v)
{
    uint32_t bits;

    memcpy(&bits, &v, sizeof(bits));
    dst[0] = (uint8_t)(bits & 0xFFu);
    dst[1] = (uint8_t)((bits >> 8) & 0xFFu);
    dst[2] = (uint8_t)((bits >> 16) & 0xFFu);
    dst[3] = (uint8_t)((bits >> 24) & 0xFFu);
}

drivetrain_status_t drivetrain_encode_estop(const drivetrain_estop_t *msg, uint8_t out[DRIVETRAIN_ESTOP_LEN])
{
    if (msg->estop_reason > DRIVETRAIN_ESTOP_OTHER) {
        return DRIVETRAIN_ERR_RANGE;
    }

    out[0] = (uint8_t)msg->estop_reason;
    out[1] = msg->sequence_number;
    return DRIVETRAIN_OK;
}

drivetrain_status_t drivetrain_encode_command(const drivetrain_command_t *msg, uint8_t out[DRIVETRAIN_COMMAND_LEN])
{
    if (msg->mode > DRIVETRAIN_MODE_ESTOP) {
        return DRIVETRAIN_ERR_RANGE;
    }

    pack_f32_le(&out[0], msg->linear_velocity);
    pack_f32_le(&out[4], msg->angular_velocity);
    out[8] = (uint8_t)msg->mode;
    out[9] = msg->sequence_number;
    out[10] = 0; /* reserved */
    out[11] = 0; /* reserved */
    return DRIVETRAIN_OK;
}

drivetrain_status_t drivetrain_encode_odometry(const drivetrain_odometry_t *msg, uint8_t out[DRIVETRAIN_ODOMETRY_LEN])
{
    pack_f32_le(&out[0], msg->pose_x);
    pack_f32_le(&out[4], msg->pose_y);
    pack_f32_le(&out[8], msg->pose_heading);
    pack_f32_le(&out[12], msg->linear_velocity);
    pack_f32_le(&out[16], msg->angular_velocity);
    out[20] = msg->fault_flags;
    pack_u16_le(&out[21], msg->loop_time_us);
    out[23] = msg->sequence_number;
    return DRIVETRAIN_OK;
}

drivetrain_status_t drivetrain_encode_config(const drivetrain_config_t *msg, uint8_t out[DRIVETRAIN_CONFIG_LEN])
{
    out[0] = msg->param_id;
    pack_f32_le(&out[1], msg->param_value);
    return DRIVETRAIN_OK;
}
