/*
 * lokomotion.c  --  Public lifecycle API
 *
 * This file owns only the public functions that application code calls.
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

/*
 * Zeros the entire LokoState, stores the PCA9685 handles, sets initial state
 * to UN_ARMED with tripod gait, calculates default body height, builds all six
 * leg structs, and seeds the tripod duty-factor timing on each leg so
 * hexleg_point_at() is correct from the very first tick.
 * Input:  st        — LokoState to initialise (memory zeroed here)
 *         pca_right — right-side PCA9685 device handle
 *         pca_left  — left-side PCA9685 device handle
 * Output: void
 */
void loko_init(LokoState *st, PCA9685_t *pca_right, PCA9685_t *pca_left)
{
    memset(st, 0, sizeof(*st));
    st->pca_right   = pca_right;
    st->pca_left    = pca_left;
    st->state       = LOKO_UN_ARMED;
    st->duty_factor = LOKO_DUTY_TRIPOD;
    st->gait_mode   = GAIT_TRIPOD;

    st->body_height = CHASSIS_TO_SHOULDER + DESIRED_BELLY_CLEARANCE;

    loko_build_default_legs(st);

    for (int i = 0; i < LOKO_NUM_LEGS; ++i)
        loko_apply_gait_timing(&st->legs[i], st->duty_factor);
}

/*
 * Writes zero velocity into all fields of a LokoInput struct (stopped, no rotation).
 * Input:  in — LokoInput struct to zero
 * Output: void
 */
void loko_default_input(LokoInput *in)
{
    in->vx = 0.0f;
    in->vy = 0.0f;
    in->wz = 0.0f;
}

/*
 * Increments the tick counter and calls loko_dispatch() to run the active FSM
 * state handler for one control cycle.
 * Input:  st — locomotion state
 *         in — velocity command {vx, vy, wz} normalised to [-1, 1]
 *         dt — elapsed time since previous call in seconds
 * Output: void
 */
void loko_update(LokoState *st, const LokoInput *in, float dt)
{
    st->tick_count++;
    loko_dispatch(st, in, dt);
}

/*
 * Sets a leg's foot target in leg-local frame. The coordinate origin is the
 * leg's pivot point; +x is forward along the leg axis.
 * Input:  st      — locomotion state
 *         leg_idx — leg index 0–5; out-of-range indices are silently ignored
 *         x,y,z   — target position in mm
 * Output: void
 */
void loko_set_foot_local(LokoState *st, int leg_idx, float x, float y, float z)
{
    if (leg_idx < 0 || leg_idx >= LOKO_NUM_LEGS) return;
    st->legs[leg_idx].target.x = x;
    st->legs[leg_idx].target.y = y;
    st->legs[leg_idx].target.z = z;
    st->legs[leg_idx].target_is_body_frame = 0;
}

/*
 * Sets a leg's foot target in body frame. The coordinate origin is the body
 * centre; +X forward, +Y left, +Z up.
 * Input:  st      — locomotion state
 *         leg_idx — leg index 0–5; out-of-range indices are silently ignored
 *         x,y,z   — target position in mm in body frame
 * Output: void
 */
void loko_set_foot_body(LokoState *st, int leg_idx, float x, float y, float z)
{
    if (leg_idx < 0 || leg_idx >= LOKO_NUM_LEGS) return;
    st->legs[leg_idx].target.x = x;
    st->legs[leg_idx].target.y = y;
    st->legs[leg_idx].target.z = z;
    st->legs[leg_idx].target_is_body_frame = 1;
}

/*
 * Overrides the FSM state directly, bypassing all transition guards.
 * Input:  st        — locomotion state
 *         new_state — target state; silently ignored if out of range
 * Output: void
 */
void loko_set_state(LokoState *st, LokoFSMState new_state)
{
    if (new_state < LOKO_STATE_COUNT)
        st->state = new_state;
}

/*
 * Enables or disables IK solving and servo writes.
 * Input:  st — locomotion state
 *         en — 1 to enable, 0 to disable
 * Output: void
 */
void loko_enable(LokoState *st, uint8_t en)
{
    st->enabled = en ? 1u : 0u;
}

/*
 * Returns the accumulated IK error bitmask since the last loko_clear_errors() call.
 * Output: bitmask of LOKO_ERR_* flags
 */
uint32_t loko_get_errors(const LokoState *st) { return st->error_flags; }

/*
 * Clears the IK error bitmask.
 * Output: void
 */
void loko_clear_errors(LokoState *st)         { st->error_flags = 0; }

/*
 * Stub for runtime stabiliser gain tuning. Gains are fixed in loko_config.h.
 * Input:  st, kp, kd, lpf_alpha — all ignored
 * Output: void
 */
void loko_stabiliser_configure(LokoState *st, float kp, float kd, float lpf_alpha)
{
    (void)st; (void)kp; (void)kd; (void)lpf_alpha;
}
