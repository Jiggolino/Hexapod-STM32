/*
 * loko_input.c  --  Centralised PS5 controller input state
 */

#include "loko_input.h"
#include <string.h>

void loko_input_update(LokoInputPad *pad, const float *new_values)
{
    memcpy(pad->prev, pad->cur,  sizeof(pad->cur));
    memcpy(pad->cur,  new_values, sizeof(pad->cur));
}
