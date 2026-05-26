#pragma once
#include "loko_input.h"

class Controller {
    LokoInputPad &_pad;
public:
    explicit Controller(LokoInputPad &pad) : _pad(pad) {}

    void  update  (const float *values)   { loko_input_update(&_pad, values); }

    /* Positive flank: button went from not-pressed to pressed this tick */
    bool  pressed (LokoInputID id) const  { return loko_pressed (&_pad, id) != 0; }
    /* Negative flank: button was released this tick */
    bool  released(LokoInputID id) const  { return loko_released(&_pad, id) != 0; }
    /* True while button is held down */
    bool  held    (LokoInputID id) const  { return loko_held    (&_pad, id) != 0; }
    /* Raw float value for axes and analogue triggers */
    float axis    (LokoInputID id) const  { return loko_axis    (&_pad, id); }
};
