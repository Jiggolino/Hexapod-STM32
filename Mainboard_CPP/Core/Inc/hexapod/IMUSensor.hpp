#pragma once
#include "imu.h"

class IMUSensor {
    IMU_Data_t _data;
    I2C_HandleTypeDef *_hi2c;
public:
    IMUSensor() : _hi2c(nullptr) {}

    bool init(I2C_HandleTypeDef *hi2c);
    bool read();

    const IMU_Data_t& data() const { return _data; }
};
