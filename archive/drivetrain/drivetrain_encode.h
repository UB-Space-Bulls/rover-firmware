#ifndef DRIVETRAIN_ENCODE_H
#define DRIVETRAIN_ENCODE_H

/*
 * Drivetrain bus wire format: struct <-> byte-buffer only, no CAN HAL
 * dependency. Compiles unmodified on both STM32 (paired with FDCAN HAL
 * calls at the call site) and Jetson (paired with SocketCAN at the call
 * site). See docs/jetson-stm protocol.md Part 1 for the spec this mirrors.
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
/* Lets C++ code (e.g. a Jetson ROS2 node) link against these as plain C functions. */

/* CAN IDs (11-bit standard, drivetrain bus) */
#define DRIVETRAIN_ID_ESTOP    0x000u
#define DRIVETRAIN_ID_COMMAND  0x010u
#define DRIVETRAIN_ID_ODOMETRY 0x020u
#define DRIVETRAIN_ID_CONFIG   0x030u

/* Fixed wire lengths (bytes), per protocol doc 1.3 */
#define DRIVETRAIN_ESTOP_LEN    2u
#define DRIVETRAIN_COMMAND_LEN  12u
#define DRIVETRAIN_ODOMETRY_LEN 24u
#define DRIVETRAIN_CONFIG_LEN   5u

/* Result of every encode/decode call */
typedef enum {
    DRIVETRAIN_OK = 0,
    DRIVETRAIN_ERR_LENGTH, /* buf/frame length didn't match the message's fixed size */
    DRIVETRAIN_ERR_RANGE,  /* a field held a value outside its defined enumeration */
} drivetrain_status_t;

typedef enum {
    DRIVETRAIN_ESTOP_MANUAL      = 0,
    DRIVETRAIN_ESTOP_COMM_LOSS   = 1,
    DRIVETRAIN_ESTOP_OVERCURRENT = 2,
    DRIVETRAIN_ESTOP_OTHER       = 3,
} drivetrain_estop_reason_t;

typedef enum {
    DRIVETRAIN_MODE_IDLE  = 0,
    DRIVETRAIN_MODE_RUN   = 1,
    DRIVETRAIN_MODE_ESTOP = 2,
} drivetrain_mode_t;

/*
 * Fault Flags bitfield (0x020 fault_flags byte), protocol doc 1.4 -- each
 * macro has a single bit set, so multiple faults can be OR'd together into
 * one byte. Check with `flags & DRIVETRAIN_FAULT_OVERCURRENT`.
 */
#define DRIVETRAIN_FAULT_OVERCURRENT   (1u << 0)
#define DRIVETRAIN_FAULT_COMM_TIMEOUT  (1u << 1)
#define DRIVETRAIN_FAULT_ENCODER_FAULT (1u << 2)
#define DRIVETRAIN_FAULT_CAN_BUS_OFF   (1u << 3)

typedef struct {
    drivetrain_estop_reason_t estop_reason;
    uint8_t                   sequence_number;
} drivetrain_estop_t;

typedef struct {
    float             linear_velocity;  /* m/s */
    float             angular_velocity; /* rad/s */
    drivetrain_mode_t mode;
    uint8_t           sequence_number;
} drivetrain_command_t;

typedef struct {
    float    pose_x;           /* m */
    float    pose_y;           /* m */
    float    pose_heading;     /* rad */
    float    linear_velocity;  /* m/s, measured */
    float    angular_velocity; /* rad/s, measured */
    uint8_t  fault_flags;      /* DRIVETRAIN_FAULT_* bitmask */
    uint16_t loop_time_us;
    uint8_t  sequence_number;
} drivetrain_odometry_t;

typedef struct {
    uint8_t param_id;
    float   param_value;
} drivetrain_config_t;

/*
 * `out` is a caller-supplied buffer sized to the matching DRIVETRAIN_*_LEN
 * (e.g. `uint8_t buf[DRIVETRAIN_COMMAND_LEN];`); each function fills it
 * with the bytes to send. Returns DRIVETRAIN_ERR_RANGE if `msg` held an
 * invalid enum value.
 */
drivetrain_status_t drivetrain_encode_estop(const drivetrain_estop_t *msg, uint8_t out[DRIVETRAIN_ESTOP_LEN]);
drivetrain_status_t drivetrain_encode_command(const drivetrain_command_t *msg, uint8_t out[DRIVETRAIN_COMMAND_LEN]);
drivetrain_status_t drivetrain_encode_odometry(const drivetrain_odometry_t *msg, uint8_t out[DRIVETRAIN_ODOMETRY_LEN]);
drivetrain_status_t drivetrain_encode_config(const drivetrain_config_t *msg, uint8_t out[DRIVETRAIN_CONFIG_LEN]);

#ifdef __cplusplus
}
#endif

#endif /* DRIVETRAIN_ENCODE_H */
