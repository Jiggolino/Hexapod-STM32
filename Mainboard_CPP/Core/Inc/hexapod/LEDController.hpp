#pragma once
#include "ws2812b.h"

class LEDController {
public:
    void init(TIM_HandleTypeDef *htim);
    void tick();
    void loading();
};
