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

/* ── IMU angle bias (degrees) ───────────────────────────────────────────── */
/* Measured angles when robot stands on flat ground. Subtracted inside
 * IMU_GetAngles() so callers see 0° at physical level. */
#define IMU_ROLL_BIAS_DEG   (-0.47f)
#define IMU_PITCH_BIAS_DEG  ( 0.87f)

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
#define STAB_DEADZONE_RAD  (0.8f * 3.14159f / 180.0f)

/* STABLE mode — COM body shift; output units: mm
 * Ideal shift = body_height * tan(tilt_angle).
 * STAB_STABLE_GAIN is the body height in mm (~140 mm).  The stabiliser
 * computes gain * tanf(angle) directly for accuracy at all angles.
 * Output is low-pass filtered to avoid jerky servo motion.                   */
#define STAB_STABLE_GAIN  140.0f   /* mm — actual body height; gain * tan(angle) = correct COM shift */
#define STAB_STABLE_LPF   0.08f   /* output LPF alpha; 0.08 → ~115 ms at 100 Hz — tiny step each tick */

/* LEVEL mode — body tilt correction; output units: rad
 *
 * Why KP < 1.0:
 *   KP=1.0 corrects the full error in one tick, but I2C + servo mechanics
 *   add one or more ticks of delay before that correction is felt by the IMU.
 *   By then another full correction is already issued → overshoot → oscillation.
 *   KP=0.55 applies ~55% per tick; the integral closes the remaining gap
 *   within ~0.5 s without ringing.
 *
 * Why stronger KI (0.08) with a larger clamp (10°):
 *   Previous KI=0.01 with 3° clamp could only ever add 3° of integral output,
 *   which was too small to close the steady-state error.  0.08 / 10° winds up
 *   fast enough to reach full correction within a few hundred ms.             */
#define STAB_LEVEL_KP  0.6f
#define STAB_LEVEL_KI  0.010f
#define STAB_LEVEL_KD  0.01f
#define STAB_LEVEL_I_CLAMP_RAD  (30.0f * 3.14159f / 180.0f)  /* 30° clamp  */

/* Input low-pass alpha for LEVEL mode (0..1).
 * 0.5 → time constant ≈ 10 ms at 100 Hz.
 * Faster than original 0.3 (less phase lag) but not so fast it passes through
 * vibration noise at KP=0.55.                                                 */
#define STAB_LPF_ALPHA 0.12f   /* 0.08 → ~115 ms time constant — same as STABLE, tiny step each tick */

/* ── Look-around angle limits (degrees) ─────────────────────────────────── */
#define LOOK_MAX_YAW_DEG    30.0f
#define LOOK_MAX_PITCH_DEG  30.0f

/* ── Gait duty factors ───────────────────────────────────────────────────── */
/* β = fraction of the stride cycle a foot spends on the ground.
 * Higher β → more legs planted simultaneously → higher static stability.
 *
 *  TRIPOD  β = 0.50  — 3 of 6 legs airborne at once (classic tripod)
 *  WAVE    β = 0.833 — 1 of 6 legs airborne at once (wave gait, max stability)
 *  4LEG    β = 0.75  — 1 of 4 active legs airborne at
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

/* ── Servo output angle limits (degrees) ────────────────────────────────── */
/* Physical travel of all servos driven through the PCA9685 boards. */
#define SERVO_ANGLE_MIN_DEG   0.0f
#define SERVO_ANGLE_MAX_DEG 180.0f

/* ── 4-leg mode coxa rotation offset (degrees) ──────────────────────────── */
/* In STAND_4_LEGS / WALK_4_LEGS the two front legs are rotated outward by
 * this offset so they clear the body at the wider 4-leg stance. */
#define LOKO_COXA_4LEG_OFFSET_DEG  45.0f

/* ── Stabiliser input clamp limits ─────────────────────────────────────── */
/* IMU tilt readings are clamped to ±STAB_MAX_TILT_DEG before entering the
 * PID so a single sensor spike cannot produce a violent body motion. */
#define STAB_MAX_TILT_DEG       30.0f
/* Body COM shift is clamped to ±STAB_MAX_BODY_SHIFT_MM (STAB_STABLE mode). */
#define STAB_MAX_BODY_SHIFT_MM  60.0f

/* ── Servo base offsets (degrees) ───────────────────────────────────────── */
/* Mechanical zero for each joint type.  Added to the IK angle before
 * sending to the servo so that 0° IK → neutral servo position.
 * The tibia on the left side gets the inverted-scale offset (180°) because
 * the servo is mirrored and the scale is already negated. */
#define SERVO_BASE_COXA_DEG         90.0f
#define SERVO_BASE_FEMUR_DEG        90.0f
#define SERVO_BASE_TIBIA_RIGHT_DEG   0.0f
#define SERVO_BASE_TIBIA_LEFT_DEG  180.0f

/* ── Per-joint hardware calibration trims (degrees) ────────────────────── */
/* Physical servo trim measured per leg/joint on the assembled robot.
 * Positive = nudge the servo clockwise; negative = counter-clockwise.
 * Adjust these when a joint sits off-neutral after running loko_init(). */

/* RIGHT board — FR(ch 0-2), MR(ch 3-5), BR(ch 6-8) */
#define SERVO_TRIM_FR_COXA_DEG      0.0f
#define SERVO_TRIM_FR_FEMUR_DEG     3.9f
#define SERVO_TRIM_FR_TIBIA_DEG    39.1f

#define SERVO_TRIM_MR_COXA_DEG      0.0f
#define SERVO_TRIM_MR_FEMUR_DEG     7.8f
#define SERVO_TRIM_MR_TIBIA_DEG    41.1f

#define SERVO_TRIM_BR_COXA_DEG      0.0f
#define SERVO_TRIM_BR_FEMUR_DEG     3.0f
#define SERVO_TRIM_BR_TIBIA_DEG    42.6f

/* LEFT board — FL(ch 0-2), ML(ch 3-5), BL(ch 6-8) */
#define SERVO_TRIM_FL_COXA_DEG      0.0f
#define SERVO_TRIM_FL_FEMUR_DEG    -9.3f
#define SERVO_TRIM_FL_TIBIA_DEG   -32.3f

#define SERVO_TRIM_ML_COXA_DEG      0.0f
#define SERVO_TRIM_ML_FEMUR_DEG    -6.4f
#define SERVO_TRIM_ML_TIBIA_DEG   -36.2f

#define SERVO_TRIM_BL_COXA_DEG      0.0f
#define SERVO_TRIM_BL_FEMUR_DEG    -2.9f
#define SERVO_TRIM_BL_TIBIA_DEG   -34.7f

#endif /* LOKO_CONFIG_H */
