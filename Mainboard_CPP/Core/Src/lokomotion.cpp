/*
 * lokomotion.c  --  Public lifecycle API
 *
 * This file owns only the six public functions that application code calls.
 * Everything else lives in dedicated sub-files:
 *
 *   loko_legs.c    loko_build_default_legs()
 *   loko_gait.c    loko_advance_phases(), loko_compute_foot_targets()
 *   loko_servo.c   loko_solve_and_write()
 *   loko_states.c  state handlers, jump table, loko_dispatch()
 */

#include "lokomotion.h"
#include "loko_legs.h"
#include "loko_gait.h"
#include "loko_states.h"
#include "loko_config.h"
#include <string.h>

void loko_init(LokoState *st, PCA9685_t *pca_right, PCA9685_t *pca_left)
{
    memset(st, 0, sizeof(*st));
    st->pca_right   = pca_right;
    st->pca_left    = pca_left;
    st->state       = LOKO_UN_ARMED;
    st->duty_factor = LOKO_DUTY_TRIPOD;
    st->gait_mode   = GAIT_TRIPOD;

    /* Default body height: shoulder to ground distance */
    st->body_height = CHASSIS_TO_SHOULDER + DESIRED_BELLY_CLEARANCE;

    loko_build_default_legs(st);

    /* Seed tripod timing on all legs so hexleg_point_at() is correct from
     * the very first tick, before any walk state sets duty_factor again. */
    for (int i = 0; i < LOKO_NUM_LEGS; ++i)
        loko_apply_gait_timing(&st->legs[i], st->duty_factor);
}

void loko_default_input(LokoInput *in)
{
    in->vx = 0.0f;
    in->vy = 0.0f;
    in->wz = 0.0f;
}

void loko_update(LokoState *st, const LokoInput *in, float dt)
{
    st->tick_count++;
    loko_dispatch(st, in, dt);
}

void loko_set_foot_local(LokoState *st, int leg_idx, float x, float y, float z)
{
    if (leg_idx < 0 || leg_idx >= LOKO_NUM_LEGS) return;
    st->legs[leg_idx].target.x = x;
    st->legs[leg_idx].target.y = y;
    st->legs[leg_idx].target.z = z;
    st->legs[leg_idx].target_is_body_frame = 0;
}

void loko_set_foot_body(LokoState *st, int leg_idx, float x, float y, float z)
{
    if (leg_idx < 0 || leg_idx >= LOKO_NUM_LEGS) return;
    st->legs[leg_idx].target.x = x;
    st->legs[leg_idx].target.y = y;
    st->legs[leg_idx].target.z = z;
    st->legs[leg_idx].target_is_body_frame = 1;
}

void loko_set_state(LokoState *st, LokoFSMState new_state)
{
    if (new_state < LOKO_STATE_COUNT)
        st->state = new_state;
}

void loko_enable(LokoState *st, uint8_t en)
{
    st->enabled = en ? 1u : 0u;
}

uint32_t loko_get_errors  (const LokoState *st) { return st->error_flags; }
void     loko_clear_errors(LokoState *st)        { st->error_flags = 0; }

void loko_stabiliser_configure(LokoState *st, float kp, float kd, float lpf_alpha)
{
    /* Stub — tilt constants live in loko_config.h as LOKO_STABLE_ROLL/PITCH */
    (void)st; (void)kp; (void)kd; (void)lpf_alpha;
}
