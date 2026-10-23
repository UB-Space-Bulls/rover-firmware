#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../drivetrain/safety/Watchdog.h"

static void test_startup_and_timeout(void)
{
    Watchdog wd;
    assert(Watchdog_Init(&wd, 1000u, 200u));
    assert(!Watchdog_CanRun(&wd));
    Watchdog_Update(&wd, 1199u);
    assert(Watchdog_GetFaults(&wd) == WATCHDOG_FAULT_NONE);
    Watchdog_Update(&wd, 1200u);
    assert(Watchdog_GetFaults(&wd) & WATCHDOG_FAULT_COMM_TIMEOUT);
    assert(!Watchdog_CanRun(&wd));
}

static void test_command_refresh_and_latched_timeout(void)
{
    Watchdog wd;
    assert(Watchdog_Init(&wd, 0u, 200u));
    assert(Watchdog_RecordValidCommand(&wd, 0u));
    assert(Watchdog_CanRun(&wd));
    assert(Watchdog_RecordValidCommand(&wd, 150u));
    Watchdog_Update(&wd, 349u);
    assert(Watchdog_CanRun(&wd));
    Watchdog_Update(&wd, 350u);
    assert(!Watchdog_CanRun(&wd));
    assert(!Watchdog_RecordValidCommand(&wd, 351u));
    assert(Watchdog_GetFaults(&wd) & WATCHDOG_FAULT_COMM_TIMEOUT);
}

static void test_late_message_cannot_hide_timeout(void)
{
    Watchdog wd;
    assert(Watchdog_Init(&wd, 0u, 200u));
    assert(Watchdog_RecordValidCommand(&wd, 0u));
    /* No Watchdog_Update call between frames; this one is already too late. */
    assert(!Watchdog_RecordValidCommand(&wd, 250u));
    assert(Watchdog_GetFaults(&wd) & WATCHDOG_FAULT_COMM_TIMEOUT);
}

static void test_estop_and_explicit_rearm(void)
{
    Watchdog wd;
    assert(Watchdog_Init(&wd, 0u, 200u));
    assert(Watchdog_RecordValidCommand(&wd, 10u));
    Watchdog_TriggerEstop(&wd);
    assert(Watchdog_GetFaults(&wd) & WATCHDOG_FAULT_ESTOP);
    assert(!Watchdog_CanRun(&wd));
    assert(!Watchdog_RecordValidCommand(&wd, 20u));
    assert(Watchdog_Rearm(&wd, 30u));
    assert(Watchdog_GetFaults(&wd) == WATCHDOG_FAULT_NONE);
    assert(!Watchdog_CanRun(&wd)); /* Rearming alone does not permit motion. */
    assert(Watchdog_RecordValidCommand(&wd, 31u));
    assert(Watchdog_CanRun(&wd));
}

static void test_clock_rollover(void)
{
    Watchdog wd;
    assert(Watchdog_Init(&wd, UINT32_MAX - 20u, 200u));
    assert(Watchdog_RecordValidCommand(&wd, UINT32_MAX - 10u));
    Watchdog_Update(&wd, 188u); /* 199 ms after the recorded command */
    assert(Watchdog_CanRun(&wd));
    Watchdog_Update(&wd, 189u); /* Exactly 200 ms later */
    assert(!Watchdog_CanRun(&wd));
}

static void test_invalid_config_and_multiple_faults(void)
{
    Watchdog wd;
    assert(!Watchdog_Init(&wd, 0u, 0u));
    assert(!Watchdog_CanRun(&wd));
    assert(Watchdog_GetFaults(&wd) & WATCHDOG_FAULT_CONFIG);
    assert(!Watchdog_Rearm(&wd, 0u));

    assert(Watchdog_Init(&wd, 0u, 200u));
    Watchdog_TriggerEstop(&wd);
    Watchdog_Update(&wd, 200u);
    assert(Watchdog_GetFaults(&wd) & WATCHDOG_FAULT_ESTOP);
    assert(Watchdog_GetFaults(&wd) & WATCHDOG_FAULT_COMM_TIMEOUT);
}

int main(void)
{
    test_startup_and_timeout();
    test_command_refresh_and_latched_timeout();
    test_late_message_cannot_hide_timeout();
    test_estop_and_explicit_rearm();
    test_clock_rollover();
    test_invalid_config_and_multiple_faults();
    puts("Watchdog tests passed.");
    return 0;
}
