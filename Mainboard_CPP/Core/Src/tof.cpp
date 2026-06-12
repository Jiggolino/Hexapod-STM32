/**
 * @file  tof.c
 * @brief VL53L1X Time-of-Flight sensor wrapper
 */

#include "tof.h"
#include "vl53l1_platform.h"
#include "main.h"
#include <stdio.h>

/*
 * Resets the VL53L1X by toggling XSHUT (active-low), loads the 135-byte
 * default configuration, selects long-range mode (up to ~4 m), sets a 50 ms
 * timing budget with a 55 ms inter-measurement period, then starts continuous
 * background ranging.
 * Input:  hi2c — I2C handle wired to the sensor
 * Output: 0 on success; -1 to -5 indicate which configuration step failed
 */
int8_t TOF_Init(I2C_HandleTypeDef *hi2c)
{
    vl53l1x_hi2c = hi2c;

    /* Hardware reset via XSHUT (active-low) */
    HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    if (VL53L1X_SensorInit(VL53L1X_DEV_ADDR) != VL53L1X_ERROR_NONE)
        return -1;

    /* Distance mode 2 = Long (up to ~4 m, better for obstacle detection) */
    if (VL53L1X_SetDistanceMode(VL53L1X_DEV_ADDR, 2) != VL53L1X_ERROR_NONE)
        return -2;

    /* 50 ms timing budget: good balance of speed vs. accuracy */
    if (VL53L1X_SetTimingBudgetInMs(VL53L1X_DEV_ADDR, 50) != VL53L1X_ERROR_NONE)
        return -3;

    /* Inter-measurement period must be ≥ timing budget */
    if (VL53L1X_SetInterMeasurementInMs(VL53L1X_DEV_ADDR, 55) != VL53L1X_ERROR_NONE)
        return -4;

    if (VL53L1X_StartRanging(VL53L1X_DEV_ADDR) != VL53L1X_ERROR_NONE)
        return -5;

    return 0;
}

/*
 * Polls the sensor for a completed measurement. If a new sample is ready,
 * copies the result into *out and clears the hardware interrupt flag.
 * Input:  out — destination for distance_mm and range_status
 * Output: 1 = fresh data written to *out, 0 = no new sample, -1 = read error
 */
int8_t TOF_Read(TOF_Data_t *out)
{
    uint8_t ready = 0;

    VL53L1X_CheckForDataReady(VL53L1X_DEV_ADDR, &ready);
    if (!ready) return 0;

    VL53L1X_Result_t result;
    if (VL53L1X_GetResult(VL53L1X_DEV_ADDR, &result) != VL53L1X_ERROR_NONE)
        return -1;

    VL53L1X_ClearInterrupt(VL53L1X_DEV_ADDR);

    out->distance_mm  = result.Distance;
    out->range_status = result.Status;
    return 1;
}
