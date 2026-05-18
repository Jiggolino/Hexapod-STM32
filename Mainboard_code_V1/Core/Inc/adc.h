#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ADC channel assignments (must match MX_ADCx_Init in main.c) */
#define ADC_CH_CURRENT_RIGHT    ADC_CHANNEL_16   /* ADC1 */
#define ADC_CH_CURRENT_LEFT     ADC_CHANNEL_10   /* ADC2 */
#define ADC_CH_BATTERY          ADC_CHANNEL_5    /* ADC3 */

/* Software averaging on top of the 16× hardware oversampling.
 * Output is 16-bit range (0–65535) due to HW oversampling with no right-shift. */
#define ADC_SW_AVG_N            16u

/**
 * Bind the three ADC handles.  Call after HAL_ADCEx_Calibration_Start().
 */
void ADC_Init(ADC_HandleTypeDef *hadc1_current_right,
              ADC_HandleTypeDef *hadc2_current_left,
              ADC_HandleTypeDef *hadc3_battery);

/**
 * Single conversion on any channel of any ADC.
 * SamplingTime is always ADC_SAMPLETIME_810CYCLES_5 regardless of the
 * channel config stored in the handle.
 */
uint32_t ADC_ReadRaw(ADC_HandleTypeDef *hadc, uint32_t channel);

/**
 * Average of n conversions.  n = 0 returns 0.
 */
uint32_t ADC_ReadRawAvg(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t n);

/* Named single-read helpers (use ADC_SW_AVG_N averages) */
uint32_t ADC_ReadCurrentRight(void);
uint32_t ADC_ReadCurrentLeft(void);
uint32_t ADC_ReadBatteryRaw(void);
uint32_t ADC_ReadVrefintRaw(void);   /* ADC3 internal 1.21 V reference */

/**
 * Calibrated Vdda in millivolts derived from the Vrefint factory cal register.
 * Call once after ADC_Init(); the result is cached internally and also
 * returned here for convenience.
 */
uint32_t ADC_CalibrateVdda(void);

/** Last value returned by ADC_CalibrateVdda() (no new conversion). */
uint32_t ADC_GetVddaMv(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_H */
