#include "hexapod/ServoDriver.hpp"

/*
 * Stores hardware parameters in the PCA9685 device struct. Does not touch the I2C bus.
 * Input:  hi2c   — shared I2C1 handle
 *         addr   — 8-bit I2C address of this PCA9685 board
 *         min_us — minimum servo pulse width in µs (default 500)
 *         max_us — maximum servo pulse width in µs (default 2500)
 */
ServoDriver::ServoDriver(I2C_HandleTypeDef *hi2c, uint8_t addr,
                         uint16_t min_us, uint16_t max_us)
{
    _dev.hi2c    = hi2c;
    _dev.addr    = addr;
    _dev.freq_hz = 50.0f;
    _dev.min_us  = min_us;
    _dev.max_us  = max_us;
}

/*
 * Wakes the PCA9685, enables register auto-increment, sets totem-pole outputs,
 * and programs the prescaler for 50 Hz PWM.
 * Output: true if all I2C transactions return HAL_OK
 */
bool ServoDriver::init()
{
    return PCA9685_Init(&_dev) == HAL_OK;
}

/*
 * Converts an angle in degrees to a PWM pulse width, applies the per-joint
 * hardware trim from loko_config.h, and writes the value to the channel register.
 * Input:  ch  — channel index (0–8 for the three legs on this board)
 *         deg — target angle in degrees [0, 180]
 * Output: void
 */
void ServoDriver::setAngle(uint8_t ch, float deg)
{
    PCA9685_SetServoAngle(&_dev, ch, deg);
}
