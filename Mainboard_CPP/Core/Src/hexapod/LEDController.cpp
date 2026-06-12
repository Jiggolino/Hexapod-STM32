#include "hexapod/LEDController.hpp"

/*
 * Starts WS2812B PWM/DMA output on the configured timer channel.
 * Must be called once after the HAL timer is initialised.
 * Input:  htim — TIM1 handle used for the WS2812B bit-bang DMA stream
 * Output: void
 */
void LEDController::init(TIM_HandleTypeDef *htim) { ws2812_init(htim); }

/*
 * Advances the active LED animation by one frame. Call at ~100 Hz (locomotion tick rate).
 * Output: void
 */
void LEDController::tick() { ws2812_tick(); }

/*
 * Sets LEDs to a rotating boot/loading animation used while waiting for hardware init.
 * Output: void
 */
void LEDController::loading() { ws2812_mode_loading(); }
