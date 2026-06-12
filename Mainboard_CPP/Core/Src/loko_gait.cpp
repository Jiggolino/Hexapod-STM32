/*
 * loko_gait.c  --  Gait phase advance and ICR arc foot-target pipeline
 */

#include "loko_gait.h"
#include "loko_config.h"
#include <math.h>

/*
 * Advances each leg's phase by (speed * dt / stride_period), where speed is
 * the normalised magnitude of the velocity command clamped to [0, 1].
 * The stride period is selected per gait mode from loko_config.h.
 * Phases wrap within [0, 1) so each leg cycles continuously.
 * Input:  st        — locomotion state containing per-leg phase values
 *         vx,vy,wz  — normalised velocity components (magnitude drives speed)
 *         dt        — elapsed time in seconds
 * Output: void (st->legs[i].phase updated in place)
 */
void loko_advance_phases(LokoState *st, float vx, float vy, float wz, float dt)
{
    float mag = sqrtf(vx*vx + vy*vy + wz*wz);
    if (mag > 1.0f) mag = 1.0f;

    static float dphase = 0;

    switch(st->gait_mode){
    case GAIT_TRIPOD:
    case GAIT_OBSTACLE:
        dphase = (mag * dt) / LOKO_STRIDE_PERIOD_S_TRIPOD;
    	break;

    case GAIT_WAVE:
        dphase = (mag * dt) / LOKO_STRIDE_PERIOD_S_WAVE;
    	break;

    case GAIT_RIPPLE:
        dphase = (mag * dt) / LOKO_STRIDE_PERIOD_S_RIPPLE;
    	break;

    default:
        dphase = (mag * dt) / LOKO_STRIDE_PERIOD_S_WAVE;
    	break;
    }

    for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
        st->legs[i].phase += dphase;
        if (st->legs[i].phase >= 1.0f)
            st->legs[i].phase -= floorf(st->legs[i].phase);
    }
}

/*
 * Applies a duty-factor β to a leg's time-weight table so that the stance
 * segment occupies β of the total cycle time and the three swing segments share
 * (1−β), preserving the physical path shape while stretching or compressing
 * the timing.
 *   w_stance = β / L_stance
 *   w_air    = (1−β) / (L_arc_left + L_swing + L_arc_right)
 * Input:  leg  — leg whose traj time-weight table is updated
 *         beta — desired stance duty factor in (0, 1)
 * Output: void (leg->traj time weights and leg->last_duty updated)
 */
void loko_apply_gait_timing(LokoLeg *leg, float beta)
{
    float L_stance = leg->traj.arc_len[HEXLEG_SEG_STANCE];
    float L_swing  = leg->traj.arc_len[HEXLEG_SEG_ARC_LEFT]
                   + leg->traj.arc_len[HEXLEG_SEG_SWING]
                   + leg->traj.arc_len[HEXLEG_SEG_ARC_RIGHT];

    if (L_stance < 1e-6f || L_swing < 1e-6f) return;

    float w_stance = beta / L_stance;
    float w_air    = (1.0f - beta) / L_swing;

    hexleg_set_time_weights(&leg->traj, w_stance, w_air, w_air, w_air);
    leg->last_duty = beta;
}

/*
 * Three-pass pipeline that computes body-frame foot targets for every active leg:
 *   Pass 1 — per-leg effective velocity in leg-local frame → heading angle
 *   Pass 2 — if |wz| > deadband: ICR arc parameters (C, L) per leg via
 *             hexleg_icr_compute(); else straight-line restore if currently curved
 *   Pass 3 — hexleg_point_at() evaluates the trajectory at each leg's current
 *             phase, producing body-frame x/y/z offsets from the leg's neutral
 * On state entry (duty_factor changed) any active leg whose cached duty differs
 * from the current value gets its time weights re-seeded here.
 * Input:  st        — locomotion state with per-leg phase and geometry
 *         vx,vy,wz  — normalised velocity command
 *         traj_h    — swing height override in mm (state-dependent)
 * Output: void (st->legs[i].target updated in place)
 */
void loko_compute_foot_targets(LokoState *st, float vx, float vy, float wz, float traj_h)
{
    float heading[LOKO_NUM_LEGS];
    for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
        const LokoLeg *L = &st->legs[i];
        const float veff_x = LOKO_ICR_BODY_RADIUS_MM * vx - wz * L->pivot_y;
        const float veff_y = LOKO_ICR_BODY_RADIUS_MM * vy + wz * L->pivot_x;

        float vleg_x, vleg_y;
        if (i < 3) {
            /* Right side: standard rotation matrix */
            vleg_x =  veff_x * L->mount_cos + veff_y * L->mount_sin;
            vleg_y = -veff_x * L->mount_sin + veff_y * L->mount_cos;
        } else {
            /* Left side: reflection matrix — physically mirrored legs */
            vleg_x =  veff_x * L->mount_cos + veff_y * L->mount_sin;
            vleg_y =  veff_x * L->mount_sin - veff_y * L->mount_cos;
        }
        heading[i] = atan2f(vleg_y, vleg_x);
    }

    if (fabsf(wz) > LOKO_ICR_WZ_DEADBAND) {
        const float v_mag = sqrtf(vx*vx + vy*vy);
        float icr_x, icr_y;
        if (v_mag > 1e-3f) {
            const float R_icr = LOKO_ICR_BODY_RADIUS_MM * v_mag / fabsf(wz);
            const float sw    = (wz >= 0.0f) ? 1.0f : -1.0f;
            icr_x = sw * R_icr * (-vy) / v_mag;
            icr_y = sw * R_icr * ( vx) / v_mag;
        } else {
            icr_x = icr_y = 0.0f;
        }

        float px[LOKO_NUM_LEGS], py[LOKO_NUM_LEGS];
        for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
            px[i] = st->legs[i].pivot_x;
            py[i] = st->legs[i].pivot_y;
        }

        float out_C[LOKO_NUM_LEGS], out_L[LOKO_NUM_LEGS];
        hexleg_icr_compute(px, py, LOKO_NUM_LEGS, icr_x, icr_y,
                           LOKO_MAX_REACH_MM, heading, LOKO_TRAJ_R,
                           out_C, out_L);

        for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
            if (!st->legs[i].active) continue;
            hexleg_set_params(&st->legs[i].traj, out_L[i], traj_h, LOKO_TRAJ_R, LOKO_TRAJ_S);
            loko_apply_gait_timing(&st->legs[i], st->duty_factor);
            hexleg_set_icr(&st->legs[i].traj, out_C[i]);
        }
    } else {
        /* Straight motion — revert from ICR arcs only if currently curved */
        for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
            if (!st->legs[i].active) continue;
            if (st->legs[i].traj.C != HEXLEG_C_STRAIGHT) {
                hexleg_set_params(&st->legs[i].traj, LOKO_TRAJ_L, traj_h, LOKO_TRAJ_R, LOKO_TRAJ_S);
                loko_apply_gait_timing(&st->legs[i], st->duty_factor);
                hexleg_set_icr(&st->legs[i].traj, HEXLEG_C_STRAIGHT);
            }
        }
    }

    /* Re-seed timing for any active leg whose cached duty_factor is stale
     * (covers straight-line walking where hexleg_set_params is not called
     * per tick). */
    for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
        LokoLeg *L = &st->legs[i];
        if (L->active && (L->last_duty != st->duty_factor))
            loko_apply_gait_timing(L, st->duty_factor);
    }

    for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
        LokoLeg *L = &st->legs[i];
        if (!L->active) {
            L->target.x = L->neutral_x;
            L->target.y = L->neutral_y;
            L->target.z = 0.0f;
            L->target_is_body_frame = 0;
            continue;
        }

        float p = L->phase + L->phase_offset;
        p -= floorf(p);

        float x, y, z;
        hexleg_point_at(&L->traj, p, heading[i], &x, &y, &z);

        L->target.x = L->neutral_x + x;
        L->target.y = L->neutral_y + y;
        L->target.z = z;
        L->target_is_body_frame = 0;
    }
}
