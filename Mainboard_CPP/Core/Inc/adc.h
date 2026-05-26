/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */
#define ADC_CH_CURRENT_RIGHT    ADC_CHANNEL_16
#define ADC_CH_CURRENT_LEFT     ADC_CHANNEL_10
#define ADC_CH_BATTERY          ADC_CHANNEL_5
#define ADC_SW_AVG_N            16u
/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */
#include <stdint.h>
void     ADC_Init(ADC_HandleTypeDef *hadc1_current_right,
                  ADC_HandleTypeDef *hadc2_current_left,
                  ADC_HandleTypeDef *hadc3_battery);
uint32_t ADC_ReadRaw(ADC_HandleTypeDef *hadc, uint32_t channel);
uint32_t ADC_ReadRawAvg(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t n);
uint32_t ADC_ReadCurrentRight(void);
uint32_t ADC_ReadCurrentLeft(void);
uint32_t ADC_ReadBatteryRaw(void);
uint32_t ADC_ReadVrefintRaw(void);
uint32_t ADC_CalibrateVdda(void);
uint32_t ADC_GetVddaMv(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

