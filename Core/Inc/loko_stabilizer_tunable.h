#pragma once

/* Runtime-tunable PID parameters for the stabilizer.
 * These can be modified via the /PID command from UART.
 * Default values are from loko_config.h. */

extern float stab_level_kp;
extern float stab_level_ki;
extern float stab_level_kd;
