#pragma once
#include <stdint.h>
void sim_reg_write(uint32_t reg, uint32_t value);
#define REG_WRITE(r, v) sim_reg_write((r), (v))
