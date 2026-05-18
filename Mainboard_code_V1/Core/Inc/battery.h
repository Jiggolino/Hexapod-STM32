/**
 * LiPo pack voltage measurement + state-of-charge lookup.
 *
 * Voltage path: battery divider → ADC3 (with Vrefint-based Vdda trim and
 * empirical gain/offset calibration). Percentage path: piecewise-linear
 * interpolation over an empirical 0.2 C discharge curve.
 */

#ifndef BATTERY_H
#define BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define BATTERY_CELLS           2U

/** Fully discharged pack voltage [V]  (3.30 V/cell × 2) */
#define BATTERY_VOLTAGE_MIN_F   6.8f

/** Fully charged pack voltage [V]     (4.20 V/cell × 2) */
#define BATTERY_VOLTAGE_MAX_F   8.4f

/** Bind the ADC handle used for the battery divider (and Vrefint — they
 *  share ADC3 on the H7) and run the one-shot Vdda calibration.
 *  Call after HAL_ADCEx_Calibration_Start() in main(). */
void  Battery_Init(ADC_HandleTypeDef *hadc);

/** Calibrated pack voltage [V], biased slightly low for LiPo safety. */
float Battery_GetVoltage(void);

/** Calibrated Vdda in millivolts (for diagnostics). */
uint32_t Battery_GetVddaMv(void);

float   Battery_GetPercentageF(float voltage);
uint8_t Battery_GetPercentage8(float voltage);

#ifdef __cplusplus
}
#endif

#endif /* BATTERY_H */
