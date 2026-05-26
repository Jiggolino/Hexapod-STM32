#pragma once
#include "ws2812b.h"

class LEDController {
public:
    void init(TIM_HandleTypeDef *htim);
    void tick();

    void off();
    void solid(uint8_t r, uint8_t g, uint8_t b);
    void pulse(uint8_t r, uint8_t g, uint8_t b);
    void morph(uint16_t hue);
    void morphRed();
    void morphGreen();
    void morphBlue();
    void loading();
    void hueCycle(const uint16_t *hues, uint8_t count);
    void cycleBlues();
    void cycleWarms();
    void cycleGreens();

    void setPixel(ws2812_strip_t strip, uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
    bool show(ws2812_strip_t strip);
};
