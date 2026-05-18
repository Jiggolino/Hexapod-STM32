/*
 * loko_input.h  --  Centralised PS5 controller input state
 *
 * Every button and axis has an ID in LokoInputID.
 * LokoInputPad stores the current and previous tick value for each one,
 * so any loko_* file can check pressed / held / released / axis without
 * doing its own edge detection.
 *
 * Usage
 * ─────
 *   // once per tick, before transitions and update:
 *   float values[LOKO_INPUT_COUNT] = {0};
 *   values[BTN_R1]   = ps5_btn_R1;
 *   values[AXIS_LX]  = ps5_lx;
 *   // ... fill the rest ...
 *   loko_input_update(&st.pad, values);
 */

#ifndef LOKO_INPUT_H
#define LOKO_INPUT_H

#include <stdint.h>

/* ── Input IDs ───────────────────────────────────────────────────────────── */

typedef enum {
    /* face buttons */
    BTN_CROSS     = 0,
    BTN_CIRCLE,
    BTN_SQUARE,
    BTN_TRIANGLE,

    /* shoulder / trigger buttons (digital) */
    BTN_L1,
    BTN_R1,
    BTN_L2,        /* fully pressed threshold     */
    BTN_R2,        /* fully pressed threshold     */

    /* stick clicks */
    BTN_L3,
    BTN_R3,

    /* misc */
    BTN_OPTIONS,
    BTN_SHARE,
    BTN_PS,
    BTN_TOUCHPAD,

    /* d-pad */
    BTN_DPAD_UP,
    BTN_DPAD_DOWN,
    BTN_DPAD_LEFT,
    BTN_DPAD_RIGHT,

    /* analogue axes [-1, 1] */
    AXIS_LX,       /* left  stick X: left −1, right +1  */
    AXIS_LY,       /* left  stick Y: up   −1, down  +1  */
    AXIS_RX,       /* right stick X                     */
    AXIS_RY,       /* right stick Y                     */

    /* analogue triggers [0, 1] */
    AXIS_L2,
    AXIS_R2,

    LOKO_INPUT_COUNT
} LokoInputID;

/* ── Input state ─────────────────────────────────────────────────────────── */

typedef struct {
    float cur [LOKO_INPUT_COUNT];   /* values this tick  */
    float prev[LOKO_INPUT_COUNT];   /* values last tick  */
} LokoInputPad;

/* ── Update ──────────────────────────────────────────────────────────────── */

/* Shift cur → prev, then copy new_values[LOKO_INPUT_COUNT] into cur.
 * Call once per tick before loko_update_transitions() and loko_update(). */
void loko_input_update(LokoInputPad *pad, const float *new_values);

/* ── Helpers (inline, usable anywhere that includes this header) ─────────── */

/* Button went from not-pressed to pressed this tick. */
static inline uint8_t loko_pressed(const LokoInputPad *pad, LokoInputID id)
{
    return (pad->cur[id] > 0.5f) && (pad->prev[id] <= 0.5f);
}

/* Button is currently held down. */
static inline uint8_t loko_held(const LokoInputPad *pad, LokoInputID id)
{
    return pad->cur[id] > 0.5f;
}

/* Button was released this tick. */
static inline uint8_t loko_released(const LokoInputPad *pad, LokoInputID id)
{
    return (pad->cur[id] <= 0.5f) && (pad->prev[id] > 0.5f);
}

/* Raw float value — use for axes and analogue triggers. */
static inline float loko_axis(const LokoInputPad *pad, LokoInputID id)
{
    return pad->cur[id];
}

#endif /* LOKO_INPUT_H */
