
#include "Watchdog.h"
#include <stddef.h>

// Initialize watchdog in a safe state
bool Watchdog_Init(Watchdog *wd,
                   uint32_t now_ms,
                   uint32_t timeout_ms)
{
    if (wd == NULL) {
        return false;
    }

    wd->last_command_ms = now_ms;
    wd->timeout_ms = timeout_ms;
    wd->seen_command = false;

    wd->initialized =
        (timeout_ms > 0u && timeout_ms <= UINT32_MAX / 2u);

    wd->faults = wd->initialized
        ? WATCHDOG_FAULT_NONE
        : WATCHDOG_FAULT_CONFIG;

    return wd->initialized;
}

// Periodically check the time since the last valid command
void Watchdog_Update(Watchdog *wd, uint32_t now_ms)
{
    if (wd == NULL || !wd->initialized) {
        return;
    }

    // Unsigned arithmetic handles the timer wrapping
    // around, assuming updates occur regularly.
    if ((uint32_t)(now_ms - wd->last_command_ms)
        >= wd->timeout_ms) {
        wd->faults |= WATCHDOG_FAULT_COMM_TIMEOUT;
    }
}

// Called only after a command has been validated
bool Watchdog_RecordValidCommand(Watchdog *wd,
                                 uint32_t now_ms)
{
    if (wd == NULL || !wd->initialized) {
        return false;
    }

    // Detect an expired deadline BEFORE accepting
    // this command. Late commands cannot clear faults.
    Watchdog_Update(wd, now_ms);

    if (wd->faults != WATCHDOG_FAULT_NONE) {
        return false;
    }

    wd->last_command_ms = now_ms;
    wd->seen_command = true;

    return true;
}

// Latch an emergency-stop fault
void Watchdog_TriggerEstop(Watchdog *wd)
{
    if (wd != NULL && wd->initialized) {
        wd->faults |= WATCHDOG_FAULT_ESTOP;
    }
}

// Motors may run only if valid commands are arriving
// and no watchdog faults have been detected
bool Watchdog_CanRun(const Watchdog *wd)
{
    return wd != NULL
        && wd->initialized
        && wd->seen_command
        && wd->faults == WATCHDOG_FAULT_NONE;
}

// Return the stored fault flags
uint8_t Watchdog_GetFaults(const Watchdog *wd)
{
    if (wd == NULL || !wd->initialized) {
        return WATCHDOG_FAULT_CONFIG;
    }

    return wd->faults;
}

// Explicitly reset faults after a safety-approved reset
bool Watchdog_Rearm(Watchdog *wd, uint32_t now_ms)
{
    if (wd == NULL || !wd->initialized) {
        return false;
    }

    wd->faults = WATCHDOG_FAULT_NONE;
    wd->last_command_ms = now_ms;
    wd->seen_command = false;

    return true;
}
