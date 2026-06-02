/*
 * loko_gait.c  --  Gait phase advance and ICR arc foot-target pipeline
 *
 * loko_advance_phases()      increments per-leg phase each tick.
 * loko_compute_foot_targets() runs the three-pass ICR arc pipeline:
 *   Pass 1 — per-leg heading from effective velocity
 *   Pass 2 — ICR arc parameters (C, L) or straight-line restore
 *   Pass 3 — hexleg_point_at() → body-frame foot position
 * loko_apply_gait_timing()   converts a duty factor β into time weights so
 *                             hexleg_point_at() stretches stance and compresses
 *                             swing without changing the physical path shape.
 */

#include "loko_gait.h"
#include "loko_config.h"
#include <math.h>

/* ── Phase advance ───────────────────────────────────────────────────────── */

void loko_advance_phases(LokoState *st, float vx, float vy, float wz, float dt)
{
    float mag = sqrtf(vx*vx + vy*vy + wz*wz);
    if (mag > 1.0f) mag = 1.0f;

    static float dphase = 0;

    switch(st->gait_mode){
    case GAIT_TRIPOD:
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

/* ── Gait timing ─────────────────────────────────────────────────────────── */

void loko_apply_gait_timing(LokoLeg *leg, float beta)
{
    float L_stance = leg->traj.arc_len[HEXLEG_SEG_STANCE];
    float L_swing  = leg->traj.arc_len[HEXLEG_SEG_ARC_LEFT]
                   + leg->traj.arc_len[HEXLEG_SEG_SWING]
                   + leg->traj.arc_len[HEXLEG_SEG_ARC_RIGHT];

    if (L_stance < 1e-6f || L_swing < 1e-6f) return;

    /* Time weights that make stance occupy β of total cycle time and
     * swing (including transition arcs) occupy (1−β).
     *
     *   time_len[seg] = arc_len[seg] * w[seg]    (definition in HexLeg)
     *   sum over all segs of time_len == β + (1−β) == 1.0  (normalised)
     *
     * Solving:  w_stance = β / L_stance
     *           w_air    = (1−β) / L_swing_total
     *
     * The three air segments (arc_left, swing, arc_right) share w_air so
     * each gets its arc-length fraction of the total air time. */
    float w_stance = beta / L_stance;
    float w_air    = (1.0f - beta) / L_swing;

    hexleg_set_time_weights(&leg->traj, w_stance, w_air, w_air, w_air);
    leg->last_duty = beta;
}

/* ── Foot target computation ─────────────────────────────────────────────── */

void loko_compute_foot_targets(LokoState *st, float vx, float vy, float wz)
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

    /* ICR arc management – only when rotating */
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
            hexleg_set_params(&st->legs[i].traj, out_L[i], LOKO_TRAJ_H, LOKO_TRAJ_R, LOKO_TRAJ_S);
            /* arc_len tables are fresh after set_params — recompute time weights */
            loko_apply_gait_timing(&st->legs[i], st->duty_factor);
            hexleg_set_icr(&st->legs[i].traj, out_C[i]);
        }
    } else {
        /* Straight motion – revert from ICR arcs only if currently curved */
        for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
            if (!st->legs[i].active) continue;
            if (st->legs[i].traj.C != HEXLEG_C_STRAIGHT) {
                hexleg_set_params(&st->legs[i].traj, LOKO_TRAJ_L, LOKO_TRAJ_H, LOKO_TRAJ_R, LOKO_TRAJ_S);
                loko_apply_gait_timing(&st->legs[i], st->duty_factor);
                hexleg_set_icr(&st->legs[i].traj, HEXLEG_C_STRAIGHT);
            }
        }
    }

    /* On state entry (duty_factor changed) or first tick, apply timing to any
     * active leg whose cached duty doesn't match the current gait config.
     * This covers straight-line walking where hexleg_set_params is not called
     * per tick and timing must be seeded once. */
    for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
        LokoLeg *L = &st->legs[i];
        if (L->active && (L->last_duty != st->duty_factor))
            loko_apply_gait_timing(L, st->duty_factor);
    }

    /* Evaluate foot position for each leg using its own phase + offset */
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
    }}
