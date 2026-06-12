#pragma once
#include "tof.h"

class ToFSensor {
    TOF_Data_t _data;
public:
    ToFSensor() { _data.distance_mm = 0xFFFF; _data.range_status = 0xFF; }

    bool  init(I2C_HandleTypeDef *hi2c);
    int8_t read();

    uint16_t distanceMm()   const { return _data.distance_mm; }
};
