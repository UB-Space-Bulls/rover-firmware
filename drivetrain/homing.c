#include "homing.h"

static HomingState homingState = HOMING_IDLE;

void Homing_Start(void)
{
    if (homingState == HOMING_IDLE)
    {
        homingState = HOMING_INITIALIZE;
    }
}

void Homing_Update(void)
{
    switch (homingState)
    {
        case HOMING_IDLE:
            // Waiting for a homing/calibration request
            break;

        case HOMING_INITIALIZE:
            // TODO: Initialize the encoder and required hardware
            break;

        case HOMING_VERIFY_POSITION:
            // TODO: Verify that the encoder readings are valid
            break;

        case HOMING_CALIBRATE:
            // TODO: Perform the agreed calibration procedure
            break;

        case HOMING_COMPLETE:
            // Calibration completed successfully
            break;

        case HOMING_ERROR:
            // TODO: Report the error and stop movement if needed
            break;
    }
}

HomingState Homing_GetState(void)
{
    return homingState;
}

void Homing_InitializationResult(bool success)
{
    if (homingState == HOMING_INITIALIZE)
    {
        homingState = success
            ? HOMING_VERIFY_POSITION
            : HOMING_ERROR;
    }
}

void Homing_PositionVerificationResult(bool valid)
{
    if (homingState == HOMING_VERIFY_POSITION)
    {
        homingState = valid
            ? HOMING_CALIBRATE
            : HOMING_ERROR;
    }
}

void Homing_CalibrationResult(bool success)
{
    if (homingState == HOMING_CALIBRATE)
    {
        homingState = success
            ? HOMING_COMPLETE
            : HOMING_ERROR;
    }
}

void Homing_Reset(void)
{
    if (homingState == HOMING_ERROR ||
        homingState == HOMING_COMPLETE)
    {
        homingState = HOMING_IDLE;
    }
}