#include "hexapod/LEDController.hpp"

void LEDController::init(TIM_HandleTypeDef *htim)     { ws2812_init(htim); }
void LEDController::tick()                             { ws2812_tick(); }
void LEDController::off()                              { ws2812_mode_off(); }
void LEDController::solid(uint8_t r, uint8_t g, uint8_t b) { ws2812_mode_solid(r, g, b); }
void LEDController::pulse(uint8_t r, uint8_t g, uint8_t b) { ws2812_mode_pulse(r, g, b); }
void LEDController::morph(uint16_t hue)                { ws2812_mode_morph(hue); }
void LEDController::morphRed()                         { ws2812_mode_morph_red(); }
void LEDController::morphGreen()                       { ws2812_mode_morph_green(); }
void LEDController::morphBlue()                        { ws2812_mode_morph_blue(); }
void LEDController::loading()                          { ws2812_mode_loading(); }
void LEDController::cycleBlues()                       { ws2812_mode_cycle_blues(); }
void LEDController::cycleWarms()                       { ws2812_mode_cycle_warms(); }
void LEDController::cycleGreens()                      { ws2812_mode_cycle_greens(); }

void LEDController::hueCycle(const uint16_t *hues, uint8_t count)
{
    ws2812_mode_hue_cycle(hues, count);
}

void LEDController::setPixel(ws2812_strip_t strip, uint8_t idx, uint8_t r, uint8_t g, uint8_t b)
{
    ws2812_set_pixel(strip, idx, r, g, b);
}

bool LEDController::show(ws2812_strip_t strip)
{
    return ws2812_show(strip);
}
