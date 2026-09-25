/*
 * Roundtrip tests for the drivetrain encode/decode module: encode
 * a struct, decode it back, check the values match, and check that
 * malformed input (wrong length / bad enum value) is rejected. Host-only
 * (no STM32/HAL dependency), run with tests/run_tests.sh.
 */
#include <assert.h>
#include <stdio.h>

#include "../drivetrain/serialization/drivetrain_encode.h"
#include "../drivetrain/serialization/drivetrain_decode.h"

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

int main(void)
{
    test_drivetrain_estop();
    test_drivetrain_command();
    test_drivetrain_odometry();
    test_drivetrain_config();

    printf("All tests passed.\n");
    return 0;
}
