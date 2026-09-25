#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t last_message_time;
    uint32_t timeout;
    bool timed_out;
} Watchdog;