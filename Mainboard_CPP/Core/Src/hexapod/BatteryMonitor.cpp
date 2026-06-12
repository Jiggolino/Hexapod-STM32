#include "hexapod/BatteryMonitor.hpp"

/*
 * Calibrates the ADC Vdda reference so subsequent raw readings can be scaled
 * to actual pack voltage.
 * Input:  hadc — ADC handle (passed for API compatibility; ownership stays in adc.c)
 * Output: void
 */
void BatteryMonitor::init(ADC_HandleTypeDef *hadc)
{
    Battery_Init(hadc);
}

/*
 * Reads the internal Vdda rail via the Vrefint channel.
 * Output: Vdda in millivolts (typically ~3300 mV)
 */
uint32_t BatteryMonitor::vddaMv()
{
    return Battery_GetVddaMv();
}
