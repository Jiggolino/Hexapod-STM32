/*
 * lokomotion.h  --  Hexapod locomotion controller
 *
 * Coordinate frame (body-centred, right-hand):
 *   +X forward    +Y left    +Z up
 *
 * State machine
 * -------------
 * Transitions are the caller's responsibility — call loko_set_state().
 * loko_update() dispatches via a jump table to the active state handler.
 *
 *   UN_ARMED        Servos not driven.
 *   STAND           All six legs at neutral stance, servos held.
 *   WALK            Six-leg tripod walk (vx, vy, wz from LokoInput).
 *   WALK_4_LEGS     Four-leg walk; front pair (FR, FL) held at neutral.
 *   STAND_4_LEGS    Stand with front pair raised.
 *   ROTATE_IN_PLACE Spin in place; only wz is used from LokoInput.
 *   DANCING         User-defined motion sequence (stub).
 */

#ifndef LOKOMOTION_H
#define LOKOMOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pca9685.h"
#include "hexapod_ik.h"
#include "trajectory_calculator.h"
#include "loko_input.h"
#include "loko_tilt.h"
#include "imu.h"

/* ── Constants ───────────────────────────────────────────────────────────── */
// ... (rest of constants)
#define LOKO_NUM_LEGS  6

#define LOKO_ERR_IK_UNREACHABLE   (1u << 0)
#define LOKO_ERR_IK_OUT_OF_LIMITS (1u << 1)

/* ── Gait mode enum ──────────────────────────────────────────────────────── */

typedef enum {
    GAIT_TRIPOD = 0,   /* 6 legs, 3 on ground (default)   β = 0.50  */
    GAIT_WAVE   = 1,   /* 6 legs, 5 on ground             β = 0.833 */
    GAIT_RIPPLE = 2,   /* 6 legs, 4 on ground (2 pairs)   β = 0.667 */
    GAIT_MODE_COUNT
} LokoGaitMode;

/* ── Stabilisation mode enum ─────────────────────────────────────────────── */

typedef enum {
    STAB_OFF    = 0,   /* No IMU correction                                    */
    STAB_STABLE = 1,   /* Shift COM horizontally to stay over support polygon  */
    STAB_LEVEL  = 2,   /* Tilt body to keep platform surface level             */
    STAB_MODE_COUNT
} LokoStabMode;

/* ── FSM state enum ──────────────────────────────────────────────────────── */
// ... (rest of enum)
typedef enum {
    LOKO_UN_ARMED        = 0,
    LOKO_STAND           = 1,
    LOKO_WALK            = 2,
    LOKO_WALK_4_LEGS     = 3,
    LOKO_STAND_4_LEGS    = 4,
    LOKO_ROTATE_IN_PLACE = 5,
    LOKO_DANCING         = 6,
    LOKO_LOOK_AROUND     = 7,
    LOKO_STATE_COUNT
} LokoFSMState;

/* ── Input vector (filled by caller every tick) ──────────────────────────── */

typedef struct {
    float vx;   /* forward  (+) / backward (−)  [-1, 1] normalised */
    float vy;   /* strafe left (+) / right (−)  [-1, 1] normalised */
    float wz;   /* yaw CCW  (+) / CW      (−)  [-1, 1] normalised */
} LokoInput;

/* ── Per-leg state ───────────────────────────────────────────────────────── */

typedef struct {
    HexLeg       traj;       /* foot trajectory (trajectory_calculator.h)   */
    HexLegConfig ik_cfg;     /* IK link lengths and joint limits             */

    float phase;             /* current gait phase [0, 1)                   */
    float phase_offset;      /* fixed tripod offset (0.0 or 0.5)            */

    /* Universal target (input to the tilt+IK pipeline) */
    Vector3f target;
    uint8_t  target_is_body_frame; /* 1 = body frame, 0 = leg-local frame */

    float foot_x;            /* solved foot position in leg frame (mm)      */
    float foot_y;
    float foot_z;

    float neutral_x;         /* neutral stance foot position (mm)           */
    float neutral_y;
    float neutral_z;

    float theta1;            /* solved joint angles (rad), coxa/femur/tibia */
    float theta2;
    float theta3;

    float pivot_x;           /* body-frame coxa pivot position (mm)         */
    float pivot_y;
    float mount_cos;         /* precomputed trig of mount angle              */
    float mount_sin;

    uint8_t servo_ch_coxa;
    uint8_t servo_ch_femur;
    uint8_t servo_ch_tibia;
    uint8_t on_right_board;  /* 1 = pca_right, 0 = pca_left                */

    float coxa_scale;        /* deg = scale * theta_rad + offset            */
    float femur_scale;
    float tibia_scale;
    float coxa_offset_deg;
    float femur_offset_deg;
    float tibia_offset_deg;

    uint8_t active;          /* 0 = held at neutral (4-leg modes)           */

    float last_duty;         /* duty factor β last applied to traj weights  */
} LokoLeg;

/* ── Top-level locomotion state ──────────────────────────────────────────── */

typedef struct {
    LokoLeg      legs[LOKO_NUM_LEGS];
    PCA9685_t   *pca_right;
    PCA9685_t   *pca_left;
    LokoInputPad pad;          /* controller input — current and previous tick */
    LokoFSMState state;
    uint32_t     tick_count;
    uint8_t      enabled;
    uint32_t     error_flags;

    /* Body orientation and positioning */
    float body_roll;     /* radians */
    float body_pitch;    /* radians */
    float body_yaw;      /* radians — used by LOOK_AROUND state */
    float body_height;   /* mm, coxa pivot to ground */

    /* Stabiliser mode saved on entering LOOK_AROUND; restored on exit */
    LokoStabMode stab_mode_saved;

    /* Gait timing — duty factor β in [0,1); set by the active state handler.
     * loko_compute_foot_targets() reads this to maintain correct stance/swing ratio. */
    float duty_factor;

    /* Active gait mode — cycled with D-pad left/right in STAND/WALK. */
    LokoGaitMode gait_mode;

    /* Active stabilisation mode — cycled with D-pad up/down in STAND/WALK. */
    LokoStabMode stab_mode;

    /* IMU sensor data (filtered) */
    IMU_Data_t imu_data;
} LokoState;

/* ── Public API ──────────────────────────────────────────────────────────── */

/* Initialise, zero state, build default leg geometry. */
void loko_init(LokoState *st, PCA9685_t *pca_right, PCA9685_t *pca_left);

/* Build safe defaults for the locomotion input vector. */
void loko_default_input(LokoInput *in);

/* Advance one control tick.  Dispatches to the active state handler. */
void loko_update(LokoState *st, const LokoInput *in, float dt);

/* Universal coordinate setters */
void loko_set_foot_local(LokoState *st, int leg_idx, float x, float y, float z);
void loko_set_foot_body (LokoState *st, int leg_idx, float x, float y, float z);

/* Request a state transition.  Ignored if new_state >= LOKO_STATE_COUNT. */
void loko_set_state(LokoState *st, LokoFSMState new_state);

/* Arm (1) or disarm (0) servo output. */
void loko_enable(LokoState *st, uint8_t en);

/* Configure body stabiliser gains. */
void loko_stabiliser_configure(LokoState *st, float kp, float kd, float lpf_alpha);

/* Error flags — set by the update pipeline, cleared manually. */
uint32_t loko_get_errors  (const LokoState *st);
void     loko_clear_errors(LokoState *st);

#ifdef __cplusplus
}
#endif

#endif /* LOKOMOTION_H */
