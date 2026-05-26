#pragma once
#include "battery.h"
#include <stdint.h>

class BatteryMonitor {
public:
    void    init(ADC_HandleTypeDef *hadc);
    float   voltage();
    uint8_t percent();
    uint32_t vddaMv();
};
