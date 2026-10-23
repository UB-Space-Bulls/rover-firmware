
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Provisional timeout from rover protocol
#define WATCHDOG_DEFAULT_TIMEOUT_MS 200u

// Watchdog fault flags
#define WATCHDOG_FAULT_NONE         0u
#define WATCHDOG_FAULT_COMM_TIMEOUT (1u << 0)
#define WATCHDOG_FAULT_ESTOP        (1u << 1)
#define WATCHDOG_FAULT_CONFIG       (1u << 2)

typedef struct {
    uint32_t last_command_ms;
    uint32_t timeout_ms;
    uint8_t faults;
    bool seen_command;
    bool initialized;
} Watchdog;

// Initialize the watchdog
bool Watchdog_Init(Watchdog *wd,
                   uint32_t now_ms,
                   uint32_t timeout_ms);

// Record a successfully validated command
bool Watchdog_RecordValidCommand(Watchdog *wd,
                                 uint32_t now_ms);

// Check whether communication timed out
void Watchdog_Update(Watchdog *wd, uint32_t now_ms);

// Record an emergency stop
void Watchdog_TriggerEstop(Watchdog *wd);

// Check whether watchdog permits motor control
bool Watchdog_CanRun(const Watchdog *wd);

// Retrieve the fault flags
uint8_t Watchdog_GetFaults(const Watchdog *wd);

// Explicitly rearm following authorized safety checks
bool Watchdog_Rearm(Watchdog *wd, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
