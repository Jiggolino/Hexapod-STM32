#include "hexapod/ToFSensor.hpp"

/*
 * Resets the VL53L1X via XSHUT, loads its 90-register default configuration,
 * selects long-range mode (≤4 m), sets a 50 ms timing budget, and starts
 * continuous background ranging.
 * Input:  hi2c — I2C handle wired to the sensor
 * Output: true on success, false on any configuration step failure
 */
bool ToFSensor::init(I2C_HandleTypeDef *hi2c)
{
    return TOF_Init(hi2c) == 0;
}

/*
 * Polls the sensor for a completed measurement. If new data is ready, copies
 * the result into the internal buffer and clears the hardware interrupt.
 * Output: 1 = fresh distance stored in _data, 0 = no new sample, -1 = I2C error
 */
int8_t ToFSensor::read()
{
    return TOF_Read(&_data);
}
