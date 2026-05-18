/*
 * loko_config.h  --  Physical build constants for the hexapod locomotion layer
 *
 * Edit these values to match your specific robot's mechanical measurements.
 * All length values are in millimetres; angles are in radians unless noted.
 * This file is included by lokomotion.c and all loko_* sub-modules.
 */

#ifndef LOKO_CONFIG_H
#define LOKO_CONFIG_H

/* ── Leg link lengths (mm) ──────────────────────────────────────────────── */
#define L1_COXA_LENGTH    66.93f
#define L2_FEMUR_LENGTH   89.80f
#define L3_TIBIA_LENGTH  165.56f

/* ── Coxa pivot positions relative to body centre (mm) ─────────────────── */
#define FRONT_ROW_X   125.722f   /* +X = forward  */
#define FRONT_ROW_Y   113.425f   /* +Y = left     */
#define MID_ROW_X       0.0f
#define MID_ROW_Y     156.834f
#define BACK_ROW_X   -125.722f
#define BACK_ROW_Y    113.425f

/* ── Vertical geometry (mm) ─────────────────────────────────────────────── */
#define CHASSIS_TO_SHOULDER       40.0f   /* body bottom → coxa pivot      */
#define DESIRED_BELLY_CLEARANCE  100.0f   /* coxa pivot → ground at stand  */

/* ── Foot reach at rest (mm) ────────────────────────────────────────────── */
/* Horizontal distance from coxa pivot to foot in the neutral stance.
 * Increase for a wider stance; decrease if legs look over-extended. */
#define NEUTRAL_REACH_MM  150.0f

/* ── Tibia joint limits (radians) ──────────────────────────────────────── */
#define TIBIA_MIN_RAD  -2.1f     /* −120° — neutral stance requires ~−1.88 rad */
#define TIBIA_MAX_RAD   2.268f   /* +130° */

/* ── Servo output polarity (+1 or −1) ──────────────────────────────────── */
/* Flip the sign if a joint moves backwards relative to what is expected.  */
#define COXA_DIR   1.0f
#define FEMUR_DIR  1.0f
#define TIBIA_DIR -1.0f

/* ── IMU mounting offset (mm) ───────────────────────────────────────────── */
/* Forward distance of the accelerometer from the body centre.
 * Used by the stabiliser to subtract centripetal acceleration caused by yaw. */
#define IMU_OFFSET_X_MM  53.7f

/* ── IMU angle bias (degrees) ───────────────────────────────────────────── */
/* Measured angles when robot stands on flat ground. Subtracted inside
 * IMU_GetAngles() so callers see 0° at physical level. */
#define IMU_ROLL_BIAS_DEG   (-0.47f)
#define IMU_PITCH_BIAS_DEG  ( 0.87f)

/* ── Sit pose height (mm) ───────────────────────────────────────────────── */
/* foot_z when sitting: 60 mm below coxa pivot → belly clearance ≈ 20 mm.  */
#define SIT_Z_MM  -60.0f

/* ── Default trajectory shape (GeoGebra parameters) ────────────────────── */
/* These drive hexleg_init() in loko_build_default_legs().
 * L  stride half-length (mm)   H  swing height (mm)
 * R  arc-corner radius (mm)    S  shape factor (controls arc–swing join) */
#define LOKO_TRAJ_L   50.0f
#define LOKO_TRAJ_H   50.0f
#define LOKO_TRAJ_R   7.0f
#define LOKO_TRAJ_S   2.0f

/* Gait cycle period (seconds).  Phase advances at speed / period per tick. */
#define LOKO_STRIDE_PERIOD_S  0.4f

/* How much to raise front legs (mm above neutral_z) in STAND_4_LEGS. */
#define LOKO_FRONT_RAISE_MM  20.0f

/* ── ICR arc turning ────────────────────────────────────────────────────── */
/* |wz| threshold below which straight-line motion is used (no per-tick
 * rebuild of the arc-length tables). */
#define LOKO_ICR_WZ_DEADBAND  0.05f

/* Characteristic body radius (mm) used to convert the dimensionless wz
 * and v inputs into a physical ICR distance.  At full forward + full turn
 * (vx=wz=1) the ICR sits this far from the body centre.
 * ≈ sqrt(FRONT_ROW_X² + FRONT_ROW_Y²) — retune if arcs feel too tight/wide. */
#define LOKO_ICR_BODY_RADIUS_MM  170.0f

/* Maximum allowed foot arc half-length (mm).  Determines the rotation
 * budget: theta_budget = LOKO_MAX_REACH_MM / max_icr_radius_across_legs.
 * The outermost leg's stride arc is capped here; all inner legs scale
 * proportionally so the body rotates as a rigid unit. */
#define LOKO_MAX_REACH_MM  50.0f

/* ── IMU stabiliser PID gains ───────────────────────────────────────────── */
/* Deadzone: correction is suppressed while tilt is within this band.
 * Prevents reacting to IMU noise on flat ground.  0.2° in radians.          */
#define STAB_DEADZONE_RAD  (0.2f * 3.14159f / 180.0f)

/* STABLE mode — COM body shift; output units: mm
 * Ideal Kp = body_height (140 mm/rad) — shifts COM exactly over polygon at
 * steady state.  No D: the deadzone-reset creates artificial step inputs that
 * the D-term amplifies into violent overshoots (effective D gain = Kd*alpha/dt).
 * No I: horizontal body shift cannot close the IMU tilt loop.                */
#define STAB_STABLE_KP   80.0f
#define STAB_STABLE_KI    0.0f
#define STAB_STABLE_KD    0.0f
#define STAB_STABLE_I_CLAMP_MM   0.0f

/* LEVEL mode — body tilt correction; output units: rad
 * Pure P+I only.  D removed: at 100 Hz with alpha=0.3, each deadzone exit
 * creates Δlpf = alpha*err on tick 1, giving D = Kd*alpha/dt ≈ 30*Kd*err —
 * with Kd=0.06 that was 0.9× the error in one shot → oscillation.
 * Small I closes steady-state error on real slopes; clamp keeps it tight.    */
#define STAB_LEVEL_KP  0.4f
#define STAB_LEVEL_KI  0.01f
#define STAB_LEVEL_KD  0.0f
#define STAB_LEVEL_I_CLAMP_RAD  (3.0f * 3.14159f / 180.0f)  /* 3° clamp   */

/* Input low-pass alpha (0..1).
 * 0.30 → time constant ≈ 33 ms at 100 Hz.  Safe to raise now that Kd=0
 * (D-term no longer amplifies the filter's step response).                   */
#define STAB_LPF_ALPHA 0.3f

/* ── Gait duty factors ───────────────────────────────────────────────────── */
/* β = fraction of the stride cycle a foot spends on the ground.
 * Higher β → more legs planted simultaneously → higher static stability.
 *
 *  TRIPOD  β = 0.50  — 3 of 6 legs airborne at once (classic tripod)
 *  WAVE    β = 0.833 — 1 of 6 legs airborne at once (wave gait, max stability)
 *  4LEG    β = 0.75  — 1 of 4 active legs airborne at once
 *
 * These values feed loko_apply_gait_timing() which converts them into
 * per-segment time weights so hexleg_point_at() stretches stance and
 * compresses swing automatically, regardless of stride length or ICR radius. */
#define LOKO_DUTY_TRIPOD  0.5000f
#define LOKO_DUTY_WAVE    0.8333f
#define LOKO_DUTY_RIPPLE  0.6667f   /* 4/6: 2 legs airborne simultaneously  */
#define LOKO_DUTY_4LEG    0.7500f   /* 3/4: 1 of 4 active legs airborne     */

/* ── Servo debug print throttle ─────────────────────────────────────────── */
/* Print one servo-angle line every N calls to loko_solve_and_write.
 * At 100 Hz (10 ms loop) LOKO_SERVO_PRINT_EVERY = 10 → one line per 100 ms. */
#define LOKO_SERVO_PRINT_EVERY  1

#endif /* LOKO_CONFIG_H */
