/*
 * hexapod_leg.c -- implementation
 */
#include "trajectory_calculator.h"
#include <math.h>
#include <stddef.h>

#ifndef HEXLEG_PI
#define HEXLEG_PI 3.14159265358979323846f
#endif

/* ------------------------------------------------------------------ */
/* segment evaluators                                                  */
/*   Each returns the 2D sagittal position (x forward, z up) at the    */
/*   normalized local parameter u in [0, 1].  Same curves as Python.   */
/* ------------------------------------------------------------------ */
static void eval_segment(const HexLeg *leg, int seg, float u, float *x, float *z)
{
    const float L = leg->L;
    const float H = leg->H;
    const float R = leg->R;
    const float S = leg->S;

    switch (seg) {
    case HEXLEG_SEG_STANCE: {
        /* line (L-R, 0) -> (-(L-R), 0) */
        *x = (L - R) * (1.0f - 2.0f * u);
        *z = 0.0f;
        return;
    }
    case HEXLEG_SEG_ARC_LEFT: {
        /* ArcLeft traversed in reverse: t from 3*pi/2 down to 2-S+pi */
        const float t_a = 1.5f * HEXLEG_PI;
        const float t_b = 2.0f - S + HEXLEG_PI;
        const float t   = t_a + (t_b - t_a) * u;
        *x = -L + R + R * cosf(t);
        *z =  R + R * sinf(t);
        return;
    }
    case HEXLEG_SEG_SWING: {
        /* main Trajectory curve, t: 0 -> pi                         */
        const float cs = cosf(S - 2.0f);
        const float ss = sinf(S - 2.0f);
        const float A  = 2.0f * (L - R + R * cs);
        const float x0 = -L + R - R * cs;
        const float z0 =  R + R * ss;
        const float t  = HEXLEG_PI * u;
        *x = A * (t / HEXLEG_PI - sinf(2.0f * t) / (S * HEXLEG_PI)) + x0;
        *z = z0 + H * sinf(t);
        return;
    }
    case HEXLEG_SEG_ARC_RIGHT: {
        /* ArcRight traversed in reverse: t from S-2 down to -pi/2   */
        const float t_c = S - 2.0f;
        const float t_d = -0.5f * HEXLEG_PI;
        const float t   = t_c + (t_d - t_c) * u;
        *x = L - R + R * cosf(t);
        *z = R + R * sinf(t);
        return;
    }
    default:
        *x = 0.0f; *z = 0.0f;
        return;
    }
}

/* ------------------------------------------------------------------ */
/* table rebuild                                                       */
/* ------------------------------------------------------------------ */
static void rebuild(HexLeg *leg)
{
    float cum_time   = 0.0f;
    float cum_length = 0.0f;

    for (int seg = 0; seg < HEXLEG_NUM_SEGMENTS; ++seg) {
        float prev_x, prev_z;
        eval_segment(leg, seg, 0.0f, &prev_x, &prev_z);
        leg->s[seg][0] = 0.0f;

        for (int i = 1; i < HEXLEG_SAMPLES; ++i) {
            const float u = (float)i / (float)(HEXLEG_SAMPLES - 1);
            float x, z;
            eval_segment(leg, seg, u, &x, &z);
            const float dx = x - prev_x;
            const float dz = z - prev_z;
            leg->s[seg][i] = leg->s[seg][i - 1] + sqrtf(dx * dx + dz * dz);
            prev_x = x;
            prev_z = z;
        }

        leg->arc_len [seg] = leg->s[seg][HEXLEG_SAMPLES - 1];
        leg->time_len[seg] = leg->arc_len[seg] * leg->w[seg];
        leg->time_cum[seg] = cum_time;
        cum_time   += leg->time_len[seg];
        cum_length += leg->arc_len [seg];
    }
    leg->total_time   = cum_time;
    leg->total_length = cum_length;
}

/* ------------------------------------------------------------------ */
/* public API                                                          */
/* ------------------------------------------------------------------ */
void hexleg_init(HexLeg *leg, float L, float H, float R, float S)
{
    leg->L = L;
    leg->H = H;
    leg->R = R;
    leg->S = S;
    leg->C = HEXLEG_C_STRAIGHT;
    for (int i = 0; i < HEXLEG_NUM_SEGMENTS; ++i) leg->w[i] = 1.0f;
    rebuild(leg);
}

void hexleg_set_params(HexLeg *leg, float L, float H, float R, float S)
{
    leg->L = L;
    leg->H = H;
    leg->R = R;
    leg->S = S;
    rebuild(leg);
}

void hexleg_set_icr(HexLeg *leg, float C)
{
    leg->C = C;
    /* The 3D arc transform d → (C·sin(d/C), C·(1−cos(d/C)), z) is an
     * isometry: ds_3D == |dd|, so the arc-length tables remain valid. */
}

void hexleg_set_time_weights(HexLeg *leg, float w_stance, float w_arc_left, float w_swing, float w_arc_right)
{
    leg->w[HEXLEG_SEG_STANCE   ] = w_stance;
    leg->w[HEXLEG_SEG_ARC_LEFT ] = w_arc_left;
    leg->w[HEXLEG_SEG_SWING    ] = w_swing;
    leg->w[HEXLEG_SEG_ARC_RIGHT] = w_arc_right;
    rebuild(leg);
}

/* Invert the s[seg] table: given a local arc length, return the
 * corresponding u in [0,1] using linear interpolation.  Linear scan
 * is ~5% of a typical control tick on H7 at N=128, so not worth the
 * complication of a binary search. */
static float invert_arc_length(const float *s, float local)
{
    if (local <= 0.0f) return 0.0f;
    const float end = s[HEXLEG_SAMPLES - 1];
    if (local >= end) return 1.0f;

    int idx = HEXLEG_SAMPLES - 1;
    for (int i = 1; i < HEXLEG_SAMPLES; ++i) {
        if (s[i] >= local) { idx = i; break; }
    }
    const float s0 = s[idx - 1];
    const float s1 = s[idx];
    const float u0 = (float)(idx - 1) / (float)(HEXLEG_SAMPLES - 1);
    const float u1 = (float)(idx    ) / (float)(HEXLEG_SAMPLES - 1);
    const float ds = s1 - s0;
    if (ds <= 0.0f) return u0;
    return u0 + (local - s0) * (u1 - u0) / ds;
}

void hexleg_point_at(const HexLeg *leg, float phase, float angle_rad, float *out_x, float *out_y, float *out_z)
{
    /* wrap phase into [0,1) */
    phase = phase - floorf(phase);

    const float t_target = phase * leg->total_time;

    /* find segment */
    int seg = HEXLEG_NUM_SEGMENTS - 1;
    for (int i = 0; i < HEXLEG_NUM_SEGMENTS; ++i) {
        if (t_target <= leg->time_cum[i] + leg->time_len[i]) {
            seg = i;
            break;
        }
    }

    /* fraction of the way through the segment IN TIME */
    float frac = 0.0f;
    if (leg->time_len[seg] > 0.0f) {
        frac = (t_target - leg->time_cum[seg]) / leg->time_len[seg];
    }
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    /* map that to equal-arc-length u inside the segment */
    const float local = frac * leg->arc_len[seg];
    const float u     = invert_arc_length(leg->s[seg], local);

    /* evaluate the 2D sagittal curve */
    float x2d, z2d;
    eval_segment(leg, seg, u, &x2d, &z2d);

    /* Apply ICR arc transform.
     *
     * The sagittal displacement d = x2d is the arc-length parameter on
     * a circle of radius C centred at (C, 0) in leg-local frame:
     *   y_fwd = C · sin(d/C)          (forward component)
     *   x_lat = C · (1 − cos(d/C))    (lateral component, +left of heading)
     *
     * When C == HEXLEG_C_STRAIGHT the Taylor expansion recovers the
     * original straight-line behaviour: y_fwd = d, x_lat = 0.
     *
     * Then rotate both components into the body frame:
     *   out_x = y_fwd·cos(θ) − x_lat·sin(θ)
     *   out_y = y_fwd·sin(θ) + x_lat·cos(θ)
     */
    float y_fwd, x_lat;
    if (leg->C != HEXLEG_C_STRAIGHT) {
        const float inv_C = 1.0f / leg->C;
        y_fwd = leg->C * sinf(x2d * inv_C);
        x_lat = leg->C * (1.0f - cosf(x2d * inv_C));
    } else {
        y_fwd = x2d;
        x_lat = 0.0f;
    }

    const float ca = cosf(angle_rad);
    const float sa = sinf(angle_rad);
    if (out_x) *out_x = y_fwd * ca - x_lat * sa;
    if (out_y) *out_y = y_fwd * sa + x_lat * ca;
    if (out_z) *out_z = z2d;
}

void hexleg_path(const HexLeg *leg, float angle_rad, float *out_xyz, int n)
{
    if (n <= 0 || out_xyz == NULL) return;
    for (int i = 0; i < n; ++i) {
        const float phase = (n == 1) ? 0.0f : (float)i / (float)(n - 1);
        hexleg_point_at(leg, phase, angle_rad,
                        &out_xyz[3 * i + 0],
                        &out_xyz[3 * i + 1],
                        &out_xyz[3 * i + 2]);
    }
}

/* ------------------------------------------------------------------ */
/* ICR arc orchestration                                               */
/* ------------------------------------------------------------------ */

float hexleg_icr_signed_radius(float pivot_x, float pivot_y, float icr_x,   float icr_y, float heading_rad)
{
    const float dx = icr_x - pivot_x;
    const float dy = icr_y - pivot_y;
    const float r  = sqrtf(dx * dx + dy * dy);
    if (r < 1e-3f) return 0.0f;

    /* Project pivot→ICR onto the left-perpendicular of heading.
     * Left-perp of (cos θ, sin θ) is (−sin θ, cos θ). */
    const float sign = -sinf(heading_rad) * dx + cosf(heading_rad) * dy;
    return (sign >= 0.0f) ? r : -r;
}

float hexleg_icr_compute(const float *pivot_x,
                         const float *pivot_y,
                         int          num_legs,
                         float        icr_x,
                         float        icr_y,
                         float        reach_limit_mm,
                         const float *heading_rad,
                         float        traj_R,
                         float       *out_C,
                         float       *out_L)
{
    /* Pass 1: signed radii + find the largest (outer leg). */
    float max_r = 0.0f;
    for (int i = 0; i < num_legs; ++i) {
        out_C[i] = hexleg_icr_signed_radius(pivot_x[i], pivot_y[i],
                                             icr_x, icr_y,
                                             heading_rad[i]);
        const float r = out_C[i] < 0.0f ? -out_C[i] : out_C[i];
        if (r > max_r) max_r = r;
    }

    /* Rotation budget: outermost leg gets exactly reach_limit_mm of arc. */
    const float theta = (max_r > 1e-3f) ? reach_limit_mm / max_r : 0.0f;

    /* Pass 2: per-leg stride half-length scaled by their radius. */
    for (int i = 0; i < num_legs; ++i) {
        const float r = out_C[i] < 0.0f ? -out_C[i] : out_C[i];
        out_L[i] = traj_R + r * theta;
    }

    return theta;
}

void hexleg_segment_path(const HexLeg *leg, HexLegSegment seg, float angle_rad, float *out_xyz, int n)
{
    if (n <= 0 || out_xyz == NULL) return;
    const float ca = cosf(angle_rad);
    const float sa = sinf(angle_rad);
    for (int i = 0; i < n; ++i) {
        const float u = (n == 1) ? 0.0f : (float)i / (float)(n - 1);
        float x2d, z;
        eval_segment(leg, (int)seg, u, &x2d, &z);
        float y_fwd, x_lat;
        if (leg->C != HEXLEG_C_STRAIGHT) {
            const float inv_C = 1.0f / leg->C;
            y_fwd = leg->C * sinf(x2d * inv_C);
            x_lat = leg->C * (1.0f - cosf(x2d * inv_C));
        } else {
            y_fwd = x2d;
            x_lat = 0.0f;
        }
        out_xyz[3 * i + 0] = y_fwd * ca - x_lat * sa;
        out_xyz[3 * i + 1] = y_fwd * sa + x_lat * ca;
        out_xyz[3 * i + 2] = z;
    }
}
