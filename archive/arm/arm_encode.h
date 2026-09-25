#ifndef ARM_ENCODE_H
#define ARM_ENCODE_H

/*
 * Arm bus wire format: struct <-> byte-buffer only, no CAN HAL dependency.
 * Compiles unmodified on both STM32 (paired with FDCAN HAL calls at the
 * call site) and Jetson (paired with SocketCAN at the call site). See
 * docs/jetson-stm protocol.md Part 2 for the spec this mirrors.
 *
 * Note: 0x000 Estop has no field table in the protocol doc's Part 2 (only
 * listed in the ID map). Defined here identically to the drivetrain Estop
 * message (estop_reason + sequence_number) per project decision.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Lets C++ code (e.g. a Jetson ROS2 node) link against these as plain C functions. */

/* CAN IDs (11-bit standard, arm bus) */
#define ARM_ID_ESTOP       0x000u
#define ARM_ID_FAULT_EVENT 0x005u
#define ARM_ID_COMMAND     0x010u
#define ARM_ID_FEEDBACK    0x020u
#define ARM_ID_CONFIG      0x030u

/*
 * Fixed wire lengths (bytes). ARM_FEEDBACK_LEN is 38: the protocol doc's
 * 2.3 states "~39 bytes" but summing its own field table (9 float32 +
 * 2 uint8) gives 38 exactly; the field table is treated as authoritative.
 */
#define ARM_ESTOP_LEN       2u
#define ARM_FAULT_EVENT_LEN 3u
#define ARM_COMMAND_LEN     24u
#define ARM_FEEDBACK_LEN    38u
#define ARM_CONFIG_LEN      5u

/* Result of every encode/decode call */
typedef enum {
    ARM_OK = 0,
    ARM_ERR_LENGTH, /* buf/frame length didn't match the message's fixed size */
    ARM_ERR_RANGE,  /* a field held a value outside its defined enumeration */
} arm_status_t;

typedef enum {
    ARM_ESTOP_MANUAL      = 0,
    ARM_ESTOP_COMM_LOSS   = 1,
    ARM_ESTOP_OVERCURRENT = 2,
    ARM_ESTOP_OTHER       = 3,
} arm_estop_reason_t;

/* joint_id numbering, protocol doc 2.1 */
typedef enum {
    ARM_JOINT_SHOULDER    = 0,
    ARM_JOINT_ELBOW       = 1,
    ARM_JOINT_WRIST_PITCH = 2,
    ARM_JOINT_WRIST_ROLL  = 3,
    ARM_JOINT_GRIPPER     = 4,
} arm_joint_id_t;

typedef enum {
    ARM_MODE_IDLE  = 0,
    ARM_MODE_RUN   = 1,
    ARM_MODE_ESTOP = 2,
} arm_mode_t;

/*
 * Fault Flags bitfield (0x020 fault_flags byte), protocol doc 2.4 -- each
 * macro has a single bit set, so multiple faults can be OR'd together into
 * one byte. Check with `flags & ARM_FAULT_SHOULDER_LIMIT`.
 */
#define ARM_FAULT_SHOULDER_LIMIT (1u << 0)
#define ARM_FAULT_ELBOW_LIMIT    (1u << 1)
#define ARM_FAULT_NOT_HOMED      (1u << 2)

typedef struct {
    arm_estop_reason_t estop_reason;
    uint8_t             sequence_number;
} arm_estop_t;

/*
 * fault_type: only 0 (limit switch hit) is defined so far, doc explicitly
 * leaves "room for future types" -- decode does not range-check this field,
 * unlike joint_id/mode/estop_reason which are closed enumerations.
 */
typedef struct {
    arm_joint_id_t joint_id;
    uint8_t        fault_type;
    uint8_t        sequence_number;
} arm_fault_event_t;

typedef struct {
    float      shoulder_angle;    /* rad */
    float      elbow_angle;       /* rad */
    float      wrist_pitch;       /* rad */
    float      wrist_roll;        /* rad */
    float      gripper_position;
    arm_mode_t mode;
    uint8_t    sequence_number;
} arm_command_t;

typedef struct {
    float   shoulder_angle;
    float   shoulder_velocity;
    float   elbow_angle;
    float   elbow_velocity;
    float   wrist_pitch;
    float   wrist_pitch_velocity;
    float   wrist_roll;
    float   wrist_roll_velocity;
    float   gripper_position; /* echo of last commanded value */
    uint8_t fault_flags;      /* ARM_FAULT_* bitmask */
    uint8_t sequence_number;
} arm_feedback_t;

typedef struct {
    uint8_t param_id;
    float   param_value;
} arm_config_t;

/*
 * `out` is a caller-supplied buffer sized to the matching ARM_*_LEN (e.g.
 * `uint8_t buf[ARM_COMMAND_LEN];`); each function fills it with the bytes
 * to send. Returns ARM_ERR_RANGE if `msg` held an invalid enum value.
 */
arm_status_t arm_encode_estop(const arm_estop_t *msg, uint8_t out[ARM_ESTOP_LEN]);
arm_status_t arm_encode_fault_event(const arm_fault_event_t *msg, uint8_t out[ARM_FAULT_EVENT_LEN]);
arm_status_t arm_encode_command(const arm_command_t *msg, uint8_t out[ARM_COMMAND_LEN]);
arm_status_t arm_encode_feedback(const arm_feedback_t *msg, uint8_t out[ARM_FEEDBACK_LEN]);
arm_status_t arm_encode_config(const arm_config_t *msg, uint8_t out[ARM_CONFIG_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* ARM_ENCODE_H */
