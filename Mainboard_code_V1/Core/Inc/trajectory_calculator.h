/*
 * trajectory_calculator.h -- single-leg foot-trajectory generator
 *
 * Target:   STM32H7 (Cortex-M7 @ 480 MHz, single/double FPU)
 *           but pure C99, no RTOS, no malloc, no HAL dependency.
 * Depends only on <math.h> (cosf, sinf, sqrtf, floorf).
 *
 * This is the C port of the Python HexapodLeg library.  Every
 * feature is preserved:
 *   - 4 geometry parameters:  L, H, R, S
 *   - 4 per-segment time weights (bias how long the foot spends on
 *     each piece of the cycle without changing its shape)
 *   - arc-length parameterized phase -> constant speed by default
 *   - heading angle that rotates the 2D sagittal curve around Z
 *   - query the full stitched path or a single named segment
 *   - query total arc length, total cycle time, per-segment lengths
 *
 * Typical use
 * -----------
 *   HexLeg leg;
 *   hexleg_init(&leg, 20.0f, 4.2f, 1.8f, 2.6f);
 *   hexleg_set_time_weights(&leg, 1.0f, 2.5f, 1.0f, 2.5f);  // optional
 *
 *   // inside your 1 kHz control loop:
 *   float phase = fmodf(t_seconds / stride_period, 1.0f);
 *   float x, y, z;
 *   hexleg_point_at(&leg, phase, heading_rad, &x, &y, &z);
 *   // -> feed (x, y, z) into your inverse kinematics
 *
 * Memory
 * ------
 * sizeof(HexLeg) is roughly 4 * HEXLEG_SAMPLES * sizeof(float) plus
 * a handful of scalars.  At the default HEXLEG_SAMPLES = 400 that is
 * about 6.5 kB -- nothing on an H743 with 1 MB of RAM.  Reduce it at
 * compile time (e.g. -DHEXLEG_SAMPLES=128) if you ever need to. */
#ifndef TRAJECTORY_CALCULATOR_H
#define TRAJECTORY_CALCULATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HEXLEG_SAMPLES
#define HEXLEG_SAMPLES 400
#endif

typedef enum {
    HEXLEG_SEG_STANCE    = 0,
    HEXLEG_SEG_ARC_LEFT  = 1,
    HEXLEG_SEG_SWING     = 2,
    HEXLEG_SEG_ARC_RIGHT = 3,
    HEXLEG_NUM_SEGMENTS  = 4
} HexLegSegment;

typedef struct {
    /* --- geometry (the four GeoGebra variables) --------------------- */
    float L;   /* stride half-length parameter                         */
    float H;   /* swing height above the lift-off point                */
    float R;   /* radius of the rounded lift-off / landing arcs        */
    float S;   /* shape parameter controlling where arcs meet swing    */

    /* --- ICR arc radius -------------------------------------------- */
    /* Signed ICR radius (mm).  Use HEXLEG_C_STRAIGHT (0) for straight
     * walking; set via hexleg_set_icr().  Positive = ICR to the left
     * of the leg's heading direction, negative = ICR to the right.
     *
     * The arc transform is an isometry: 3D arc length == sagittal
     * displacement, so the arc-length tables below never need
     * rebuilding when only C changes. */
    float C;

    /* --- per-segment time weights (all 1.0 = constant speed) -------- */
    float w[HEXLEG_NUM_SEGMENTS];

    /* --- precomputed tables (rebuilt by init / set_params / weights) */
    float s       [HEXLEG_NUM_SEGMENTS][HEXLEG_SAMPLES]; /* cum arc len */
    float arc_len [HEXLEG_NUM_SEGMENTS];
    float time_len[HEXLEG_NUM_SEGMENTS];
    float time_cum[HEXLEG_NUM_SEGMENTS]; /* start time of the segment  */
    float total_time;
    float total_length;
} HexLeg;

/* Sentinel: pass as C to hexleg_set_icr() for straight-line motion.
 * 0 is used here as "infinite radius" — not mathematically zero curvature. */
#define HEXLEG_C_STRAIGHT 0.0f

/* ------------------------------------------------------------------ */
/* lifecycle                                                           */
/* ------------------------------------------------------------------ */

/* Initialize a leg with the given geometry and all time weights = 1. */
void hexleg_init(HexLeg *leg, float L, float H, float R, float S);

/* Update geometry.  Rebuilds internal tables. */
void hexleg_set_params(HexLeg *leg, float L, float H, float R, float S);

/* Update per-segment time weights.  Rebuilds internal tables.
 * 1.0 = constant speed.  Raise arc_left / arc_right to slow the
 * foot through the corners.  Order matches HexLegSegment. */
void hexleg_set_time_weights(HexLeg *leg,
                             float w_stance,
                             float w_arc_left,
                             float w_swing,
                             float w_arc_right);

/* Set the signed ICR radius.  Does NOT rebuild tables — the 3D arc
 * transform is an isometry so existing arc-length data stays valid.
 * Pass HEXLEG_C_STRAIGHT (0) to revert to straight-line motion. */
void hexleg_set_icr(HexLeg *leg, float C);

/* ------------------------------------------------------------------ */
/* ICR arc orchestration                                               */
/* ------------------------------------------------------------------ */

/* Compute the signed ICR radius for one leg.
 *
 *   pivot_x/y    Body-frame coxa pivot position (mm).
 *   icr_x/y      Body-frame ICR position (mm).
 *   heading_rad  Per-leg heading angle, i.e. atan2(vleg_y, vleg_x).
 *
 * Returns positive if the ICR is to the left of the heading direction,
 * negative if to the right.  Returns 0 if ICR is at the pivot. */
float hexleg_icr_signed_radius(float pivot_x, float pivot_y,
                               float icr_x,   float icr_y,
                               float heading_rad);

/* Compute per-leg arc parameters for all legs in one call.
 *
 *   pivot_x/y[num_legs]  Body-frame coxa pivot XY per leg (mm).
 *   num_legs             Number of legs (typically 6).
 *   icr_x/y             Body-frame ICR position (mm).  For pure yaw
 *                        v=0 pass (0,0); for combined motion compute
 *                        from v/|wz| perpendicular to the velocity.
 *   reach_limit_mm       Maximum foot arc half-length (mm).  Determines
 *                        the rotation budget: theta = reach / max_radius.
 *   heading_rad[num_legs] Per-leg headings (from velocity direction).
 *   traj_R               Arc-corner radius R from the active gait params.
 *
 * Outputs:
 *   out_C[num_legs]  Signed ICR radius per leg (pass to hexleg_set_icr).
 *   out_L[num_legs]  Per-leg stride half-length L = R + |C_i|*theta_budget
 *                    (pass to hexleg_set_params).
 *
 * Returns the rotation budget (radians). */
float hexleg_icr_compute(const float *pivot_x,
                         const float *pivot_y,
                         int          num_legs,
                         float        icr_x,
                         float        icr_y,
                         float        reach_limit_mm,
                         const float *heading_rad,
                         float        traj_R,
                         float       *out_C,
                         float       *out_L);

/* ------------------------------------------------------------------ */
/* queries                                                             */
/* ------------------------------------------------------------------ */

/* Foot position at `phase` in [0,1) (wraps automatically) with
 * heading `angle_rad` (rotation around the vertical Z axis).
 * Out parameters may be NULL individually if you don't need them. */
void hexleg_point_at(const HexLeg *leg,
                     float phase, float angle_rad,
                     float *out_x, float *out_y, float *out_z);

/* Fill `out_xyz` (length n*3) with n points uniformly sampled in
 * phase around the whole cycle.  == Python leg.path(n, angle). */
void hexleg_path(const HexLeg *leg, float angle_rad,
                 float *out_xyz, int n);

/* Fill `out_xyz` (length n*3) with n points along a single segment.
 * == Python leg.segment_path(name, n, angle). */
void hexleg_segment_path(const HexLeg *leg, HexLegSegment seg,
                         float angle_rad, float *out_xyz, int n);

/* Convenience accessors. */
static inline float hexleg_total_length (const HexLeg *l) { return l->total_length;   }
static inline float hexleg_total_time   (const HexLeg *l) { return l->total_time;     }
static inline float hexleg_segment_length(const HexLeg *l, HexLegSegment s) { return l->arc_len[s]; }

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_CALCULATOR_H */
