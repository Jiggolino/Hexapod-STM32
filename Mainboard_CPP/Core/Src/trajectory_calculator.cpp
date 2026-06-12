#include "trajectory_calculator.h"
#include <math.h>
#include <stddef.h>

#ifndef HEXLEG_PI
#define HEXLEG_PI 3.14159265358979323846f
#endif

/*
 * Evaluates one of the four trajectory segments at normalised parameter u ∈ [0,1].
 * Returns the 2D sagittal position (x forward, z up) in leg-local coordinates.
 *
 * Segments:
 *   STANCE     — straight line from (L-R, 0) to (-(L-R), 0)
 *   ARC_LEFT   — circular arc blending swing exit into stance entry (traversed in reverse)
 *   SWING      — raised arc from stance-end to stance-start, peak at z = H
 *   ARC_RIGHT  — circular arc blending stance exit into swing entry (traversed in reverse)
 *
 * Input:  leg — trajectory parameters {L, H, R, S}
 *         seg — segment index (HEXLEG_SEG_*)
 *         u   — normalised position along segment [0, 1]
 * Output: *x, *z — position in mm
 */
static void eval_segment(const HexLeg *leg, int seg, float u, float *x, float *z)
{
    const float L = leg->L;
    const float H = leg->H;
    const float R = leg->R;
    const float S = leg->S;

    switch (seg) {
    case HEXLEG_SEG_STANCE: {
        *x = (L - R) * (1.0f - 2.0f * u);
        *z = 0.0f;
        return;
    }
    case HEXLEG_SEG_ARC_LEFT: {
        const float t_a = 1.5f * HEXLEG_PI;
        const float t_b = 2.0f - S + HEXLEG_PI;
        const float t   = t_a + (t_b - t_a) * u;
        *x = -L + R + R * cosf(t);
        *z =  R + R * sinf(t);
        return;
    }
    case HEXLEG_SEG_SWING: {
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

/*
 * Rebuilds the arc-length LUT (s[seg][]) and derived timing tables for all four
 * segments. Samples each segment at HEXLEG_SAMPLES points, integrates arc length
 * by summing chord distances, then computes time_len[seg] = arc_len[seg] * w[seg].
 * Must be called whenever L, H, R, S, or the time weights change.
 * Input:  leg — trajectory struct to update in place
 * Output: void (leg->s, arc_len, time_len, time_cum, total_time, total_length updated)
 */
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

/* ── Public API ──────────────────────────────────────────────────────────── */

/*
 * Initialises a HexLeg trajectory with the given shape parameters and uniform
 * time weights (w = 1 for all segments), then builds the arc-length tables.
 * Input:  leg  — HexLeg struct to initialise
 *         L    — stride half-length in mm
 *         H    — swing peak height in mm
 *         R    — arc blend radius in mm
 *         S    — swing shape parameter (controls horizontal velocity profile)
 * Output: void
 */
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

/*
 * Updates shape parameters and rebuilds the arc-length tables.
 * Used by the gait engine when ICR arc geometry or swing height changes.
 * Input:  leg, L, H, R, S — same as hexleg_init()
 * Output: void
 */
void hexleg_set_params(HexLeg *leg, float L, float H, float R, float S)
{
    leg->L = L;
    leg->H = H;
    leg->R = R;
    leg->S = S;
    rebuild(leg);
}

/*
 * Sets the ICR curve radius C for the 3D arc transform applied in
 * hexleg_point_at(). The arc transform is an isometry (ds_3D == |dd|) so
 * the arc-length tables computed by rebuild() remain valid — no rebuild needed.
 * Input:  leg — trajectory struct
 *         C   — signed ICR radius in mm; HEXLEG_C_STRAIGHT = straight-line motion
 * Output: void
 */
void hexleg_set_icr(HexLeg *leg, float C)
{
    leg->C = C;
}

/*
 * Sets per-segment time weights, then rebuilds the arc-length tables.
 * The weights control how much of the total cycle time each segment occupies,
 * stretching stance relative to swing to implement a duty factor β.
 * Input:  leg                        — trajectory struct
 *         w_stance, w_arc_left,
 *         w_swing, w_arc_right       — non-negative time weights
 * Output: void
 */
void hexleg_set_time_weights(HexLeg *leg, float w_stance, float w_arc_left, float w_swing, float w_arc_right)
{
    leg->w[HEXLEG_SEG_STANCE   ] = w_stance;
    leg->w[HEXLEG_SEG_ARC_LEFT ] = w_arc_left;
    leg->w[HEXLEG_SEG_SWING    ] = w_swing;
    leg->w[HEXLEG_SEG_ARC_RIGHT] = w_arc_right;
    rebuild(leg);
}

/*
 * Inverts the arc-length LUT for one segment: given a local arc-length value,
 * returns the corresponding normalised parameter u ∈ [0, 1] by linear scan
 * and interpolation. Linear scan is ~5% of a control tick on H7 at N=128,
 * so a binary search is not worth the code complexity.
 * Input:  s     — arc-length LUT for the segment (HEXLEG_SAMPLES entries)
 *         local — target arc length in mm
 * Output: u in [0, 1]
 */
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

/*
 * Evaluates the 3D foot position for a given gait phase and heading angle.
 * Steps:
 *   1. Map phase → time target within the cycle.
 *   2. Find the segment and fractional position in time.
 *   3. Invert arc-length LUT to get equal-arc-length u.
 *   4. Evaluate 2D sagittal curve → (x2d, z2d).
 *   5. Apply ICR arc transform to curve the stance track:
 *        y_fwd = C·sin(x2d/C),  x_lat = C·(1−cos(x2d/C))
 *      When C == HEXLEG_C_STRAIGHT this reduces to y_fwd = x2d, x_lat = 0.
 *   6. Rotate (y_fwd, x_lat) into body frame by heading_rad.
 *
 * Input:  leg       — trajectory struct with pre-built arc-length tables
 *         phase     — gait phase in [0, 1) (wrapped internally)
 *         angle_rad — leg heading in body frame (radians)
 * Output: *out_x, *out_y, *out_z — 3D foot position offset from neutral (mm)
 */
void hexleg_point_at(const HexLeg *leg, float phase, float angle_rad, float *out_x, float *out_y, float *out_z)
{
    phase = phase - floorf(phase);

    const float t_target = phase * leg->total_time;

    int seg = HEXLEG_NUM_SEGMENTS - 1;
    for (int i = 0; i < HEXLEG_NUM_SEGMENTS; ++i) {
        if (t_target <= leg->time_cum[i] + leg->time_len[i]) {
            seg = i;
            break;
        }
    }

    float frac = 0.0f;
    if (leg->time_len[seg] > 0.0f)
        frac = (t_target - leg->time_cum[seg]) / leg->time_len[seg];
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    const float local = frac * leg->arc_len[seg];
    const float u     = invert_arc_length(leg->s[seg], local);

    float x2d, z2d;
    eval_segment(leg, seg, u, &x2d, &z2d);

    /* ICR arc transform: maps the sagittal displacement x2d onto a circular arc
     * of radius C in leg-local frame, then rotates into body frame.
     * When C == HEXLEG_C_STRAIGHT the Taylor expansion recovers the straight-line
     * behaviour: y_fwd = x2d, x_lat = 0. */
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

/*
 * Samples the full trajectory at n equally-spaced phase values and writes
 * the 3D positions into out_xyz as a flat array [x0,y0,z0, x1,y1,z1, ...].
 * Input:  leg       — trajectory struct
 *         angle_rad — heading in radians
 *         out_xyz   — output buffer of at least 3*n floats
 *         n         — number of sample points
 * Output: void
 */
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

/* ── ICR arc orchestration ───────────────────────────────────────────────── */

/*
 * Computes the signed ICR radius for one leg pivot.
 * The sign convention: positive = ICR is to the left of the heading direction.
 * Returns 0 if the pivot-to-ICR distance is below 1 mm (effectively straight).
 * Input:  pivot_x, pivot_y — pivot position in body frame (mm)
 *         icr_x,   icr_y   — ICR position in body frame (mm)
 *         heading_rad       — leg heading direction (radians)
 * Output: signed radius in mm
 */
float hexleg_icr_signed_radius(float pivot_x, float pivot_y, float icr_x, float icr_y, float heading_rad)
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

/*
 * Computes per-leg ICR arc parameters for all legs simultaneously.
 *
 * Pass 1: compute signed radius out_C[i] for each leg and find the outermost leg.
 * Pass 2: scale stride half-length out_L[i] proportionally: the outermost leg
 *         gets exactly reach_limit_mm of arc; all others scale by their radius ratio.
 *
 * Input:  pivot_x, pivot_y — per-leg pivot positions (mm)
 *         num_legs          — number of legs
 *         icr_x, icr_y      — Instantaneous Centre of Rotation (mm)
 *         reach_limit_mm    — maximum stride arc for the outermost leg
 *         heading_rad       — per-leg heading array
 *         traj_R            — trajectory blend radius added to all L values
 * Output: out_C[i] — signed radius per leg (mm)
 *         out_L[i] — stride half-length per leg (mm)
 *         return    — total rotation angle (radians) for the outermost leg
 */
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
    float max_r = 0.0f;
    for (int i = 0; i < num_legs; ++i) {
        out_C[i] = hexleg_icr_signed_radius(pivot_x[i], pivot_y[i],
                                             icr_x, icr_y,
                                             heading_rad[i]);
        const float r = out_C[i] < 0.0f ? -out_C[i] : out_C[i];
        if (r > max_r) max_r = r;
    }

    const float theta = (max_r > 1e-3f) ? reach_limit_mm / max_r : 0.0f;

    for (int i = 0; i < num_legs; ++i) {
        const float r = out_C[i] < 0.0f ? -out_C[i] : out_C[i];
        out_L[i] = traj_R + r * theta;
    }

    return theta;
}

/*
 * Samples one segment of the trajectory at n points and writes the 3D positions
 * into out_xyz, applying the ICR arc transform and heading rotation.
 * Useful for debug visualisation of individual trajectory segments.
 * Input:  leg       — trajectory struct
 *         seg       — segment to sample (HEXLEG_SEG_*)
 *         angle_rad — heading in radians
 *         out_xyz   — output buffer of at least 3*n floats
 *         n         — number of sample points
 * Output: void
 */
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
