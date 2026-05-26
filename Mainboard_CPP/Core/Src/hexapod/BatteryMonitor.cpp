#include "hexapod/BatteryMonitor.hpp"

void BatteryMonitor::init(ADC_HandleTypeDef *hadc)
{
    Battery_Init(hadc);
}

float BatteryMonitor::voltage()
{
    return Battery_GetVoltage();
}

uint8_t BatteryMonitor::percent()
{
    return Battery_GetPercentage8(Battery_GetVoltage());
}

uint32_t BatteryMonitor::vddaMv()
{
    return Battery_GetVddaMv();
}
