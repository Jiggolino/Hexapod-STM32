/**
 * @file  imu.h
 * @brief Wrapper around LSM6DSO16IS driver with software low-pass filtering
 *
 * I2C address: 0x6A (7-bit, SA0=0) → 0xD4 for HAL (8-bit form)
 * ODR:  52 Hz accelerometer & gyroscope (high-performance mode)
 * Range: ±2 g  /  ±250 dps
 * Software filter: First-order IIR (α=0.1) on all axes
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* 7-bit I2C address: SA0 pulled low → 0x6A.  HAL needs (addr << 1) = 0xD4 — done in imu.c. */
#define LSM6DSO16IS_I2C_ADDR  0x6A

/** Scaled sensor output */
typedef struct {
    float accel_x_mg;   /* acceleration X in milli-g (filtered) */
    float accel_y_mg;
    float accel_z_mg;
    float gyro_x_mdps;  /* angular rate X in milli-dps (filtered) */
    float gyro_y_mdps;
    float gyro_z_mdps;
} IMU_Data_t;

/**
 * @brief  Initialize LSM6DSO16IS with hardware and software low-pass filtering.
 * @param  hi2c  HAL I2C handle (same bus as PCA9685 servo drivers)
 * @return 0 on success, non-zero on error
 */
int32_t IMU_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read filtered accelerometer and gyroscope data.
 * @param  out  Destination struct
 * @return 0 on success
 */
int32_t IMU_Read(IMU_Data_t *out);

/**
 * @brief  Compute calibrated roll and pitch from the last filtered sample.
 *         Bias constants (IMU_ROLL_BIAS_DEG / IMU_PITCH_BIAS_DEG) are
 *         subtracted here so callers always receive 0° at physical level.
 * @param  out_roll_deg   [out] roll  in degrees (positive = left side lower)
 * @param  out_pitch_deg  [out] pitch in degrees (positive = nose higher)
 */
void IMU_GetAngles(float *out_roll_deg, float *out_pitch_deg);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
