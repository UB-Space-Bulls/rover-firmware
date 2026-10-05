#ifndef HOMING_H
#define HOMING_H

typedef enum
{
    HOMING_IDLE,
    HOMING_INITIALIZE,
    HOMING_SEEK_HOME,
    HOMING_DETECT_HOME,
    HOMING_BACKOFF,
    HOMING_SLOW_APPROACH,
    HOMING_SET_ZERO,
    HOMING_COMPLETE,
    HOMING_ERROR
} HomingState;

void Homing_Start(void);
void Homing_Update(void);

#endif