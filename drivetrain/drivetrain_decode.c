#include "drivetrain_decode.h"

#include <string.h>

/* Unpacks 2 little-endian bytes into a uint16. */
static uint16_t unpack_u16_le(const uint8_t *src)
{
    return (uint16_t)(src[0] | ((uint16_t)src[1] << 8));
}

/* Unpacks 4 little-endian bytes into a float. */
static float unpack_f32_le(const uint8_t *src)
{
    uint32_t bits = (uint32_t)src[0]
                   | ((uint32_t)src[1] << 8)
                   | ((uint32_t)src[2] << 16)
                   | ((uint32_t)src[3] << 24);
    float v;

    memcpy(&v, &bits, sizeof(v));
    return v;
}

/* Decodes an Estop message (0x000) from buf into out. */
drivetrain_status_t drivetrain_decode_estop(const uint8_t *buf, uint8_t len, drivetrain_estop_t *out)
{
    if (len != DRIVETRAIN_ESTOP_LEN) {
        return DRIVETRAIN_ERR_LENGTH;
    }
    if (buf[0] > DRIVETRAIN_ESTOP_OTHER) {
        return DRIVETRAIN_ERR_RANGE;
    }

    out->estop_reason = (drivetrain_estop_reason_t)buf[0];
    out->sequence_number = buf[1];
    return DRIVETRAIN_OK;
}

/* Decodes a Command message (0x010) from buf into out. */
drivetrain_status_t drivetrain_decode_command(const uint8_t *buf, uint8_t len, drivetrain_command_t *out)
{
    if (len != DRIVETRAIN_COMMAND_LEN) {
        return DRIVETRAIN_ERR_LENGTH;
    }
    if (buf[8] > DRIVETRAIN_MODE_ESTOP) {
        return DRIVETRAIN_ERR_RANGE;
    }

    out->linear_velocity = unpack_f32_le(&buf[0]);
    out->angular_velocity = unpack_f32_le(&buf[4]);
    out->mode = (drivetrain_mode_t)buf[8];
    out->sequence_number = buf[9];
    /* buf[10..11] reserved, ignored */
    return DRIVETRAIN_OK;
}

/* Decodes an Odometry/Telemetry message (0x020) from buf into out. */
drivetrain_status_t drivetrain_decode_odometry(const uint8_t *buf, uint8_t len, drivetrain_odometry_t *out)
{
    if (len != DRIVETRAIN_ODOMETRY_LEN) {
        return DRIVETRAIN_ERR_LENGTH;
    }

    out->pose_x = unpack_f32_le(&buf[0]);
    out->pose_y = unpack_f32_le(&buf[4]);
    out->pose_heading = unpack_f32_le(&buf[8]);
    out->linear_velocity = unpack_f32_le(&buf[12]);
    out->angular_velocity = unpack_f32_le(&buf[16]);
    out->fault_flags = buf[20];
    out->loop_time_us = unpack_u16_le(&buf[21]);
    out->sequence_number = buf[23];
    return DRIVETRAIN_OK;
}

/* Decodes a Config message (0x030) from buf into out. */
drivetrain_status_t drivetrain_decode_config(const uint8_t *buf, uint8_t len, drivetrain_config_t *out)
{
    if (len != DRIVETRAIN_CONFIG_LEN) {
        return DRIVETRAIN_ERR_LENGTH;
    }

    /* param_id isn't range-checked -- protocol doc 1.3 lists 0=Kp/1=Ki/2=Kd
     * as examples, not a closed set. */
    out->param_id = buf[0];
    out->param_value = unpack_f32_le(&buf[1]);
    return DRIVETRAIN_OK;
}
