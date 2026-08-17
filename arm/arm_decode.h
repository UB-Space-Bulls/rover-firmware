#ifndef ARM_DECODE_H
#define ARM_DECODE_H

#include "arm_encode.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * len must equal the message's fixed ARM_*_LEN or ARM_ERR_LENGTH is
 * returned. Reserved bytes in the wire format are ignored, not validated.
 * fault_event's fault_type is not range-checked (see arm_encode.h note).
 */
arm_status_t arm_decode_estop(const uint8_t *buf, uint8_t len, arm_estop_t *out);
arm_status_t arm_decode_fault_event(const uint8_t *buf, uint8_t len, arm_fault_event_t *out);
arm_status_t arm_decode_command(const uint8_t *buf, uint8_t len, arm_command_t *out);
arm_status_t arm_decode_feedback(const uint8_t *buf, uint8_t len, arm_feedback_t *out);
arm_status_t arm_decode_config(const uint8_t *buf, uint8_t len, arm_config_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ARM_DECODE_H */
