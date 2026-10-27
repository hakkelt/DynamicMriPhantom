#pragma once
#include <stdint.h>
// Reading the cycle counter costs a few cycles, so busy-wait loops advance the virtual clock.
uint32_t esp_cpu_get_cycle_count(void);
