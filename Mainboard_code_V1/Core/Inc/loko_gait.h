/*
 * loko_gait.h  --  Gait phase advance and ICR arc foot-target pipeline
 */
#ifndef LOKO_GAIT_H
#define LOKO_GAIT_H

#include "lokomotion.h"

/* Advance the gait phase of every active leg.
 * Phase speed scales with the magnitude of (vx, vy, wz), clamped to [0,1]. */
void loko_advance_phases(LokoState *st, float vx, float vy, float wz, float dt);

/* Compute body-frame foot targets for all legs using the ICR arc pipeline.
 * Active legs follow the trajectory; inactive legs hold neutral position.
 * Re-applies gait timing weights (from st->duty_factor) after any geometry change. */
void loko_compute_foot_targets(LokoState *st, float vx, float vy, float wz);

/* Apply time weights to leg->traj so that foot spends beta of the cycle in
 * stance and (1-beta) in swing.  Call after any hexleg_set_params / hexleg_init.
 * Updates leg->last_duty so callers can detect whether a re-apply is needed. */
void loko_apply_gait_timing(LokoLeg *leg, float beta);

#endif /* LOKO_GAIT_H */
