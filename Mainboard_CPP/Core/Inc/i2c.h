#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

/* I2C_HandleTypeDef hi2c1 is defined in i2c_msp.c */

void I2C_BusClear(void);
void I2C1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_H */
