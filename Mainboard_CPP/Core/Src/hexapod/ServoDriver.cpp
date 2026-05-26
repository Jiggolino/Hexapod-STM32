#include "hexapod/ServoDriver.hpp"

ServoDriver::ServoDriver(I2C_HandleTypeDef *hi2c, uint8_t addr,
                         uint16_t min_us, uint16_t max_us)
{
    _dev.hi2c    = hi2c;
    _dev.addr    = addr;
    _dev.freq_hz = 50.0f;
    _dev.min_us  = min_us;
    _dev.max_us  = max_us;
}

bool ServoDriver::init()
{
    return PCA9685_Init(&_dev) == HAL_OK;
}

void ServoDriver::setAngle(uint8_t ch, float deg)
{
    PCA9685_SetServoAngle(&_dev, ch, deg);
}

void ServoDriver::setPulse(uint8_t ch, uint16_t us)
{
    PCA9685_SetServoPulse(&_dev, ch, us);
}

void ServoDriver::sleep()
{
    PCA9685_Sleep(&_dev);
}

void ServoDriver::wake()
{
    PCA9685_Wake(&_dev);
}
