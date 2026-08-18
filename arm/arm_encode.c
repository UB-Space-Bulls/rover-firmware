#include "arm_encode.h"

#include <string.h>

/* Packs a float into 4 little-endian bytes (memcpy avoids undefined
 * behavior from pointer-casting float* to uint32_t*). */
static void pack_f32_le(uint8_t *dst, float v)
{
    uint32_t bits;

    memcpy(&bits, &v, sizeof(bits));
    dst[0] = (uint8_t)(bits & 0xFFu);
    dst[1] = (uint8_t)((bits >> 8) & 0xFFu);
    dst[2] = (uint8_t)((bits >> 16) & 0xFFu);
    dst[3] = (uint8_t)((bits >> 24) & 0xFFu);
}

/* Encodes an Estop message (0x000) into out. */
arm_status_t arm_encode_estop(const arm_estop_t *msg, uint8_t out[ARM_ESTOP_LEN])
{
    if (msg->estop_reason > ARM_ESTOP_OTHER) {
        return ARM_ERR_RANGE;
    }

    out[0] = (uint8_t)msg->estop_reason;
    out[1] = msg->sequence_number;
    return ARM_OK;
}

/* Encodes a Fault Event message (0x005) into out. */
arm_status_t arm_encode_fault_event(const arm_fault_event_t *msg, uint8_t out[ARM_FAULT_EVENT_LEN])
{
    if (msg->joint_id > ARM_JOINT_GRIPPER) {
        return ARM_ERR_RANGE;
    }

    out[0] = (uint8_t)msg->joint_id;
    out[1] = msg->fault_type;
    out[2] = msg->sequence_number;
    return ARM_OK;
}

/* Encodes a Command message (0x010) into out. */
arm_status_t arm_encode_command(const arm_command_t *msg, uint8_t out[ARM_COMMAND_LEN])
{
    if (msg->mode > ARM_MODE_ESTOP) {
        return ARM_ERR_RANGE;
    }

    pack_f32_le(&out[0], msg->shoulder_angle);
    pack_f32_le(&out[4], msg->elbow_angle);
    pack_f32_le(&out[8], msg->wrist_pitch);
    pack_f32_le(&out[12], msg->wrist_roll);
    pack_f32_le(&out[16], msg->gripper_position);
    out[20] = (uint8_t)msg->mode;
    out[21] = msg->sequence_number;
    out[22] = 0; /* reserved */
    out[23] = 0; /* reserved */
    return ARM_OK;
}

/* Encodes a Feedback message (0x020) into out. */
arm_status_t arm_encode_feedback(const arm_feedback_t *msg, uint8_t out[ARM_FEEDBACK_LEN])
{
    pack_f32_le(&out[0], msg->shoulder_angle);
    pack_f32_le(&out[4], msg->shoulder_velocity);
    pack_f32_le(&out[8], msg->elbow_angle);
    pack_f32_le(&out[12], msg->elbow_velocity);
    pack_f32_le(&out[16], msg->wrist_pitch);
    pack_f32_le(&out[20], msg->wrist_pitch_velocity);
    pack_f32_le(&out[24], msg->wrist_roll);
    pack_f32_le(&out[28], msg->wrist_roll_velocity);
    pack_f32_le(&out[32], msg->gripper_position);
    out[36] = msg->fault_flags;
    out[37] = msg->sequence_number;
    return ARM_OK;
}

/* Encodes a Config message (0x030) into out. */
arm_status_t arm_encode_config(const arm_config_t *msg, uint8_t out[ARM_CONFIG_LEN])
{
    out[0] = msg->param_id;
    pack_f32_le(&out[1], msg->param_value);
    return ARM_OK;
}
