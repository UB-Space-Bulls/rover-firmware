#ifndef HOMING_H
#define HOMING_H

#include <stdbool.h>

typedef enum
{
    HOMING_IDLE,
    HOMING_INITIALIZE,
    HOMING_VERIFY_POSITION,
    HOMING_CALIBRATE,
    HOMING_COMPLETE,
    HOMING_ERROR
} HomingState;

void Homing_Start(void);
void Homing_Update(void);

HomingState Homing_GetState(void);

void Homing_InitializationResult(bool success);
void Homing_PositionVerificationResult(bool valid);
void Homing_CalibrationResult(bool success);

void Homing_Reset(void);

#endif