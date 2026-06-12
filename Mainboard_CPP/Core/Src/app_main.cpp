#include "app_main.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"
#include "hexapod/HexapodRobot.hpp"

/* Robot is constructed lazily inside app_main() AFTER all peripherals and
 * supply rails have stabilized. Constructing it as a global static ran the
 * heavy ctor (incl. ~3-4 KB LokoState) inside __libc_init_array, before the
 * 2.5 s POR delay in main() — caused random hard faults on cold boot. */
static HexapodRobot *robot_ptr = nullptr;

/*
 * C-linkage bridge so C translation units (loko_transitions.c) can query the
 * latest ToF distance without including C++ headers. Returns 0xFFFF if the
 * robot object has not been constructed yet.
 * Output: last measured distance in mm, or 0xFFFF if unavailable
 */
extern "C" uint16_t tof_get_distance_mm(void)
{
    return robot_ptr ? robot_ptr->tof.distanceMm() : 0xFFFFu;
}

/*
 * Application entry point called from main() after the 2.5 s POR delay.
 * Constructs HexapodRobot as a function-local static (ensuring construction
 * happens after all HAL peripherals are ready), calls init() once, then
 * spins calling update() in a tight loop (~100 Hz locomotion rate).
 * Output: never returns
 */
void app_main(void)
{
    static HexapodRobot robot(&hi2c1, &htim1, &huart1, &hadc3);
    robot_ptr = &robot;

    robot.init();

    while (1) {
        robot.update(0.01f);
    }
}
