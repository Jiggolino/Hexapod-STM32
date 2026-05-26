/*
 * app_main.cpp — C++ application entry point
 */
#include "app_main.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"
#include "hexapod/HexapodRobot.hpp"

static HexapodRobot robot(&hi2c1, &htim1, &huart1, &hadc1, &hadc2, &hadc3);

extern "C" uint16_t tof_get_distance_mm(void)
{
    return robot.tof.distanceMm();
}

void app_main(void)
{
    robot.init();

    while (1) {
        robot.update(0.01f);
    }
}
