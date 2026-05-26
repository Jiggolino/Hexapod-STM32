/*
 * loko_states.h  --  FSM state handlers and jump-table dispatch
 */
#ifndef LOKO_STATES_H
#define LOKO_STATES_H

#include "lokomotion.h"

/* Dispatch one tick to the handler for st->state.
 * Called by loko_update(); do not call directly. */
void loko_dispatch(LokoState *st, const LokoInput *in, float dt);

/* Reset 4-leg mode state to defaults */
void loko_reset_4leg_mode(void);

/* Get current IMU stabilizer roll/pitch adjustments (STAB_LEVEL) */
void loko_get_stabilizer_angles(float *out_roll, float *out_pitch);

/* Get current IMU stabilizer body XY shift (STAB_STABLE, mm) */
void loko_get_stabilizer_shift(float *out_shift_x_mm, float *out_shift_y_mm);

#endif /* LOKO_STATES_H */
