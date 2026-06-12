#include "hexapod/IMUSensor.hpp"

/*
 * Configures the LSM6DSO at 52 Hz, ±2 g / ±250 dps with hardware LP2 filter,
 * and initialises the software IIR low-pass filter (α = 0.1).
 * Input:  hi2c — I2C1 handle; sensor sits at 8-bit address 0xD4 (SA0 low)
 * Output: true on success, false if WHO_AM_I check or any register write fails
 */
bool IMUSensor::init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    return IMU_Init(hi2c) == 0;
}

/*
 * Reads one accel + gyro sample from the sensor and applies the software IIR LPF.
 * Filtered values are accessible through data() after a successful call.
 * Output: true on success, false on I2C error
 */
bool IMUSensor::read()
{
    return IMU_Read(&_data) == 0;
}
