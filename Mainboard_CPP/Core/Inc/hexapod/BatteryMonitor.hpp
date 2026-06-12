#pragma once
#include "battery.h"
#include <stdint.h>

class BatteryMonitor {
public:
    void    init(ADC_HandleTypeDef *hadc);
    uint32_t vddaMv();
};
