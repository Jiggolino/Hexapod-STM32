#pragma once
#include "pca9685.h"

class ServoDriver {
    PCA9685_t _dev;
public:
    ServoDriver(I2C_HandleTypeDef *hi2c, uint8_t addr,
                uint16_t min_us = PCA9685_SERVO_MIN_US,
                uint16_t max_us = PCA9685_SERVO_MAX_US);

    bool init();
    void setAngle(uint8_t ch, float deg);
    void setPulse(uint8_t ch, uint16_t us);
    void sleep();
    void wake();

    PCA9685_t* handle() { return &_dev; }
};
