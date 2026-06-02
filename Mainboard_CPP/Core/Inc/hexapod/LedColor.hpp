#pragma once
#include <stdint.h>

/* Single WS2812B LED color + brightness — mirrors UART_LED_t. */
struct LedColor {
    uint8_t r          = 0;
    uint8_t g          = 0;
    uint8_t b          = 0;
    uint8_t brightness = 0;

    LedColor() = default;
    LedColor(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 255)
        : r(r), g(g), b(b), brightness(brightness) {}
};
