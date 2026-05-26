#include "hexapod/ToFSensor.hpp"

bool ToFSensor::init(I2C_HandleTypeDef *hi2c)
{
    return TOF_Init(hi2c) == 0;
}

int8_t ToFSensor::read()
{
    return TOF_Read(&_data);
}
