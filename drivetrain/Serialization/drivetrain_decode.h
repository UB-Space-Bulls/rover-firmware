#ifndef DRIVETRAIN_DECODE_H
#define DRIVETRAIN_DECODE_H

#include "drivetrain_encode.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * `buf`/`len` are the raw bytes received off the CAN bus and how many
 * arrived. len must equal the message's fixed DRIVETRAIN_*_LEN or
 * DRIVETRAIN_ERR_LENGTH is returned. Reserved bytes in the wire format are
 * ignored, not validated.
 */
drivetrain_status_t drivetrain_decode_estop(const uint8_t *buf, uint8_t len, drivetrain_estop_t *out);
drivetrain_status_t drivetrain_decode_command(const uint8_t *buf, uint8_t len, drivetrain_command_t *out);
drivetrain_status_t drivetrain_decode_odometry(const uint8_t *buf, uint8_t len, drivetrain_odometry_t *out);
drivetrain_status_t drivetrain_decode_config(const uint8_t *buf, uint8_t len, drivetrain_config_t *out);

#ifdef __cplusplus
}
#endif

#endif /* DRIVETRAIN_DECODE_H */
