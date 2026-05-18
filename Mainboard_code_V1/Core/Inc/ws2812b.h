#ifndef WS2812B_H
#define WS2812B_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * WS2812B driver for 2 strips on TIM1_CH1 (PA8) and TIM1_CH2 (PA9).
 *
 * Timing assumes TIM1 kernel clock = 240 MHz, PSC = 0, ARR = 299
 *   -> bit period 1.25 us (800 kHz).
 *   logical 0 compare = 84   (T0H ~= 0.35 us)
 *   logical 1 compare = 168  (T1H ~= 0.70 us)
 *
 * Usage (fire-and-forget):
 *     ws2812_init(&htim1);
 *     ws2812_mode_morph_blue();     // set once, forget
 *     while (1) {
 *         ws2812_tick();            // call as often as you like; self-throttled
 *         ...other work...
 *     }
 *
 * Or control pixels directly with ws2812_set_pixel / ws2812_show.
 */

#define WS2812_LEDS_PER_STRIP   3
#define WS2812_STRIP_COUNT      2

typedef enum {
    WS2812_STRIP_0 = 0,   /* PA8 / TIM1_CH1 */
    WS2812_STRIP_1 = 1,   /* PA9 / TIM1_CH2 */
} ws2812_strip_t;

/* ---- setup ---- */
void ws2812_init(TIM_HandleTypeDef *htim);

/* ---- low-level (manual control, bypasses mode engine) ---- */
void ws2812_set_pixel(ws2812_strip_t strip, uint8_t index,
                      uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_pixel_hsv(ws2812_strip_t strip, uint8_t index,
                          uint16_t hue, uint8_t value);
void ws2812_clear(ws2812_strip_t strip);
bool ws2812_show(ws2812_strip_t strip);
bool ws2812_is_busy(ws2812_strip_t strip);

/* ---- mode engine (fire-and-forget) ----
 * Set a mode, then call ws2812_tick() periodically (anywhere, any rate —
 * it throttles itself internally). The mode keeps running until you switch. */
void ws2812_tick(void);

void ws2812_mode_off(void);
void ws2812_mode_solid(uint8_t r, uint8_t g, uint8_t b);

/* Smooth sinusoidal pulse between dim and bright of the given hue family.
 * Convenience wrappers for red/green/blue; use ws2812_mode_morph(hue) for custom. */
void ws2812_mode_morph(uint16_t hue);
void ws2812_mode_morph_red(void);
void ws2812_mode_morph_green(void);
void ws2812_mode_morph_blue(void);

/* Bright->dim->bright pulse on the given RGB color (defaults to red). */
void ws2812_mode_pulse(uint8_t r, uint8_t g, uint8_t b);
void ws2812_mode_pulse_red(void);

/* Loading comet: bright blue head travels strip0 0->1->2 then strip1 2->1->0,
 * then loops. Trails one LED behind at 25% brightness, two behind at 1%. */
void ws2812_mode_loading(void);

/* Smoothly cycle hue through a list of waypoints (wraps around).
 * Shortest-path interpolation around the wheel. */
void ws2812_mode_hue_cycle(const uint16_t *hues, uint8_t count);

/* Convenience palettes that use ws2812_mode_hue_cycle. */
void ws2812_mode_cycle_blues(void);    /* blue  -> lila  -> purple */
void ws2812_mode_cycle_warms(void);    /* red   -> orange -> violet */
void ws2812_mode_cycle_greens(void);   /* green -> lime   -> yellow */

#endif /* WS2812B_H */
