/*
 * Roundtrip tests for the drivetrain and arm encode/decode modules: encode
 * a struct, decode it back, check the values match, and check that
 * malformed input (wrong length / bad enum value) is rejected. Host-only
 * (no STM32/HAL dependency), run with tests/run_tests.sh.
 */
#include <assert.h>
#include <stdio.h>

#include "../drivetrain/drivetrain_encode.h"
#include "../drivetrain/drivetrain_decode.h"
#include "../arm/arm_encode.h"
#include "../arm/arm_decode.h"

static void test_drivetrain_estop(void)
{
    drivetrain_estop_t in = { DRIVETRAIN_ESTOP_OVERCURRENT, 42 };
    uint8_t buf[DRIVETRAIN_ESTOP_LEN];
    drivetrain_estop_t out;

    assert(drivetrain_encode_estop(&in, buf) == DRIVETRAIN_OK);
    assert(drivetrain_decode_estop(buf, DRIVETRAIN_ESTOP_LEN, &out) == DRIVETRAIN_OK);
    assert(out.estop_reason == in.estop_reason);
    assert(out.sequence_number == in.sequence_number);

    assert(drivetrain_decode_estop(buf, DRIVETRAIN_ESTOP_LEN - 1, &out) == DRIVETRAIN_ERR_LENGTH);

    buf[0] = 99; /* not a valid drivetrain_estop_reason_t */
    assert(drivetrain_decode_estop(buf, DRIVETRAIN_ESTOP_LEN, &out) == DRIVETRAIN_ERR_RANGE);
}

static void test_drivetrain_command(void)
{
    drivetrain_command_t in = { -1.5f, 3.25f, DRIVETRAIN_MODE_RUN, 7 };
    uint8_t buf[DRIVETRAIN_COMMAND_LEN];
    drivetrain_command_t out;

    assert(drivetrain_encode_command(&in, buf) == DRIVETRAIN_OK);
    assert(drivetrain_decode_command(buf, DRIVETRAIN_COMMAND_LEN, &out) == DRIVETRAIN_OK);
    assert(out.linear_velocity == in.linear_velocity);
    assert(out.angular_velocity == in.angular_velocity);
    assert(out.mode == in.mode);
    assert(out.sequence_number == in.sequence_number);
    assert(buf[10] == 0 && buf[11] == 0); /* reserved bytes zeroed */

    in.mode = (drivetrain_mode_t)99;
    assert(drivetrain_encode_command(&in, buf) == DRIVETRAIN_ERR_RANGE);
}

static void test_drivetrain_odometry(void)
{
    drivetrain_odometry_t in = {
        1.0f, -2.0f, 3.14159f, 0.5f, -0.25f,
        DRIVETRAIN_FAULT_OVERCURRENT | DRIVETRAIN_FAULT_ENCODER_FAULT,
        65535, 200
    };
    uint8_t buf[DRIVETRAIN_ODOMETRY_LEN];
    drivetrain_odometry_t out;

    assert(drivetrain_encode_odometry(&in, buf) == DRIVETRAIN_OK);
    assert(drivetrain_decode_odometry(buf, DRIVETRAIN_ODOMETRY_LEN, &out) == DRIVETRAIN_OK);
    assert(out.pose_x == in.pose_x);
    assert(out.pose_y == in.pose_y);
    assert(out.pose_heading == in.pose_heading);
    assert(out.linear_velocity == in.linear_velocity);
    assert(out.angular_velocity == in.angular_velocity);
    assert(out.fault_flags == in.fault_flags);
    assert(out.loop_time_us == in.loop_time_us);
    assert(out.sequence_number == in.sequence_number);

    assert(drivetrain_decode_odometry(buf, DRIVETRAIN_ODOMETRY_LEN + 1, &out) == DRIVETRAIN_ERR_LENGTH);
}

static void test_drivetrain_config(void)
{
    drivetrain_config_t in = { 200, -9.5f }; /* param_id outside the doc's example list, still valid */
    uint8_t buf[DRIVETRAIN_CONFIG_LEN];
    drivetrain_config_t out;

    assert(drivetrain_encode_config(&in, buf) == DRIVETRAIN_OK);
    assert(drivetrain_decode_config(buf, DRIVETRAIN_CONFIG_LEN, &out) == DRIVETRAIN_OK);
    assert(out.param_id == in.param_id);
    assert(out.param_value == in.param_value);
}

static void test_arm_estop(void)
{
    arm_estop_t in = { ARM_ESTOP_COMM_LOSS, 5 };
    uint8_t buf[ARM_ESTOP_LEN];
    arm_estop_t out;

    assert(arm_encode_estop(&in, buf) == ARM_OK);
    assert(arm_decode_estop(buf, ARM_ESTOP_LEN, &out) == ARM_OK);
    assert(out.estop_reason == in.estop_reason);
    assert(out.sequence_number == in.sequence_number);
}

static void test_arm_fault_event(void)
{
    arm_fault_event_t in = { ARM_JOINT_ELBOW, 0, 3 };
    uint8_t buf[ARM_FAULT_EVENT_LEN];
    arm_fault_event_t out;

    assert(arm_encode_fault_event(&in, buf) == ARM_OK);
    assert(arm_decode_fault_event(buf, ARM_FAULT_EVENT_LEN, &out) == ARM_OK);
    assert(out.joint_id == in.joint_id);
    assert(out.fault_type == in.fault_type);
    assert(out.sequence_number == in.sequence_number);

    buf[0] = 99; /* not a valid arm_joint_id_t */
    assert(arm_decode_fault_event(buf, ARM_FAULT_EVENT_LEN, &out) == ARM_ERR_RANGE);

    /* fault_type is intentionally NOT range-checked (room for future types) */
    arm_encode_fault_event(&in, buf);
    buf[1] = 250;
    assert(arm_decode_fault_event(buf, ARM_FAULT_EVENT_LEN, &out) == ARM_OK);
    assert(out.fault_type == 250);
}

static void test_arm_command(void)
{
    arm_command_t in = { 0.1f, 0.2f, -0.3f, 0.4f, 0.5f, ARM_MODE_IDLE, 11 };
    uint8_t buf[ARM_COMMAND_LEN];
    arm_command_t out;

    assert(arm_encode_command(&in, buf) == ARM_OK);
    assert(arm_decode_command(buf, ARM_COMMAND_LEN, &out) == ARM_OK);
    assert(out.shoulder_angle == in.shoulder_angle);
    assert(out.elbow_angle == in.elbow_angle);
    assert(out.wrist_pitch == in.wrist_pitch);
    assert(out.wrist_roll == in.wrist_roll);
    assert(out.gripper_position == in.gripper_position);
    assert(out.mode == in.mode);
    assert(out.sequence_number == in.sequence_number);
    assert(buf[22] == 0 && buf[23] == 0); /* reserved bytes zeroed */
}

static void test_arm_feedback(void)
{
    arm_feedback_t in = {
        0.1f, 0.11f, 0.2f, 0.22f, 0.3f, 0.33f, 0.4f, 0.44f, 0.5f,
        ARM_FAULT_SHOULDER_LIMIT | ARM_FAULT_NOT_HOMED, 88
    };
    uint8_t buf[ARM_FEEDBACK_LEN];
    arm_feedback_t out;

    assert(arm_encode_feedback(&in, buf) == ARM_OK);
    assert(arm_decode_feedback(buf, ARM_FEEDBACK_LEN, &out) == ARM_OK);
    assert(out.shoulder_angle == in.shoulder_angle);
    assert(out.shoulder_velocity == in.shoulder_velocity);
    assert(out.elbow_angle == in.elbow_angle);
    assert(out.elbow_velocity == in.elbow_velocity);
    assert(out.wrist_pitch == in.wrist_pitch);
    assert(out.wrist_pitch_velocity == in.wrist_pitch_velocity);
    assert(out.wrist_roll == in.wrist_roll);
    assert(out.wrist_roll_velocity == in.wrist_roll_velocity);
    assert(out.gripper_position == in.gripper_position);
    assert(out.fault_flags == in.fault_flags);
    assert(out.sequence_number == in.sequence_number);
}

static void test_arm_config(void)
{
    arm_config_t in = { 3, 12.75f };
    uint8_t buf[ARM_CONFIG_LEN];
    arm_config_t out;

    assert(arm_encode_config(&in, buf) == ARM_OK);
    assert(arm_decode_config(buf, ARM_CONFIG_LEN, &out) == ARM_OK);
    assert(out.param_id == in.param_id);
    assert(out.param_value == in.param_value);
}

int main(void)
{
    test_drivetrain_estop();
    test_drivetrain_command();
    test_drivetrain_odometry();
    test_drivetrain_config();
    test_arm_estop();
    test_arm_fault_event();
    test_arm_command();
    test_arm_feedback();
    test_arm_config();

    printf("All tests passed.\n");
    return 0;
}
