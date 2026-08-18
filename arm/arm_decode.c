#include "arm_decode.h"

#include <string.h>

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
arm_status_t arm_decode_estop(const uint8_t *buf, uint8_t len, arm_estop_t *out)
{
    if (len != ARM_ESTOP_LEN) {
        return ARM_ERR_LENGTH;
    }
    if (buf[0] > ARM_ESTOP_OTHER) {
        return ARM_ERR_RANGE;
    }

    out->estop_reason = (arm_estop_reason_t)buf[0];
    out->sequence_number = buf[1];
    return ARM_OK;
}

/* Decodes a Fault Event message (0x005) from buf into out. */
arm_status_t arm_decode_fault_event(const uint8_t *buf, uint8_t len, arm_fault_event_t *out)
{
    if (len != ARM_FAULT_EVENT_LEN) {
        return ARM_ERR_LENGTH;
    }
    if (buf[0] > ARM_JOINT_GRIPPER) {
        return ARM_ERR_RANGE;
    }

    out->joint_id = (arm_joint_id_t)buf[0];
    /* fault_type not range-checked -- doc leaves room for future types. */
    out->fault_type = buf[1];
    out->sequence_number = buf[2];
    return ARM_OK;
}

/* Decodes a Command message (0x010) from buf into out. */
arm_status_t arm_decode_command(const uint8_t *buf, uint8_t len, arm_command_t *out)
{
    if (len != ARM_COMMAND_LEN) {
        return ARM_ERR_LENGTH;
    }
    if (buf[20] > ARM_MODE_ESTOP) {
        return ARM_ERR_RANGE;
    }

    out->shoulder_angle = unpack_f32_le(&buf[0]);
    out->elbow_angle = unpack_f32_le(&buf[4]);
    out->wrist_pitch = unpack_f32_le(&buf[8]);
    out->wrist_roll = unpack_f32_le(&buf[12]);
    out->gripper_position = unpack_f32_le(&buf[16]);
    out->mode = (arm_mode_t)buf[20];
    out->sequence_number = buf[21];
    /* buf[22..23] reserved, ignored */
    return ARM_OK;
}

/* Decodes a Feedback message (0x020) from buf into out. */
arm_status_t arm_decode_feedback(const uint8_t *buf, uint8_t len, arm_feedback_t *out)
{
    if (len != ARM_FEEDBACK_LEN) {
        return ARM_ERR_LENGTH;
    }

    out->shoulder_angle = unpack_f32_le(&buf[0]);
    out->shoulder_velocity = unpack_f32_le(&buf[4]);
    out->elbow_angle = unpack_f32_le(&buf[8]);
    out->elbow_velocity = unpack_f32_le(&buf[12]);
    out->wrist_pitch = unpack_f32_le(&buf[16]);
    out->wrist_pitch_velocity = unpack_f32_le(&buf[20]);
    out->wrist_roll = unpack_f32_le(&buf[24]);
    out->wrist_roll_velocity = unpack_f32_le(&buf[28]);
    out->gripper_position = unpack_f32_le(&buf[32]);
    out->fault_flags = buf[36];
    out->sequence_number = buf[37];
    return ARM_OK;
}

/* Decodes a Config message (0x030) from buf into out. */
arm_status_t arm_decode_config(const uint8_t *buf, uint8_t len, arm_config_t *out)
{
    if (len != ARM_CONFIG_LEN) {
        return ARM_ERR_LENGTH;
    }

    /* param_id isn't range-checked -- open-ended set of tunable parameters. */
    out->param_id = buf[0];
    out->param_value = unpack_f32_le(&buf[1]);
    return ARM_OK;
}
