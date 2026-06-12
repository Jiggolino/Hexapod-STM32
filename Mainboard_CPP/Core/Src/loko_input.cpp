/*
 * loko_input.c  --  Centralised PS5 controller input state
 */

#include "loko_input.h"
#include <string.h>

/*
 * Shifts the current input frame into the previous frame, then copies the
 * new values into the current frame. This enables edge-detection (pressed /
 * released) by comparing cur[] against prev[].
 * Input:  pad        — input state struct holding cur[] and prev[] arrays
 *         new_values — array of LOKO_INPUT_COUNT float values in [-1, 1]
 * Output: void
 */
void loko_input_update(LokoInputPad *pad, const float *new_values)
{
    memcpy(pad->prev, pad->cur,  sizeof(pad->cur));
    memcpy(pad->cur,  new_values, sizeof(pad->cur));
}
