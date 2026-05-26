#include "hexapod/IMUSensor.hpp"

bool IMUSensor::init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    return IMU_Init(hi2c) == 0;
}

bool IMUSensor::read()
{
    return IMU_Read(&_data) == 0;
}

void IMUSensor::getAngles(float &roll_deg, float &pitch_deg) const
{
    IMU_GetAngles(&roll_deg, &pitch_deg);
}
