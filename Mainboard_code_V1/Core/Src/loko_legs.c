/*
 * loko_legs.c  --  Default leg geometry, IK config, and servo mapping
 *
 * All physical constants come from loko_config.h.
 * Leg numbering: 0=FR 1=MR 2=BR 3=BL 4=ML 5=FL
 * Body frame:    +X forward, +Y left, +Z up
 *
  ┌───────┬──────────────┬──────────────┐
  │ Index │ Leg Name     │ Abbreviation │
  ├───────┼──────────────┼──────────────┤
  │ 0     │ Front Right  │ FR           │
  │ 1     │ Middle Right │ MR           │
  │ 2     │ Back Right   │ BR           │
  │ 3     │ Back Left    │ BL           │
  │ 4     │ Middle Left  │ ML           │
  │ 5     │ Front Left   │ FL           │
  └───────┴──────────────┴──────────────┘
 *
 */

#include "loko_legs.h"
#include "loko_config.h"
#include <math.h>

void loko_build_default_legs(LokoState *st)
{
    /* Right-side legs have negative pivot_y (body +Y = left). */
    static const float pivot_x[6] = {
         FRONT_ROW_X,  MID_ROW_X,  BACK_ROW_X,
         BACK_ROW_X,   MID_ROW_X,  FRONT_ROW_X
    };
    static const float pivot_y[6] = {
        -FRONT_ROW_Y, -MID_ROW_Y, -BACK_ROW_Y,
         BACK_ROW_Y,   MID_ROW_Y,  FRONT_ROW_Y
    };

    static const uint8_t on_right [6] = { 1, 1, 1, 0, 0, 0 };
    /* Servo channels organized front→middle→back on both boards.
     * RIGHT board: FR(0,1,2), MR(3,4,5), BR(6,7,8)
     * LEFT board:  FL(0,1,2), ML(3,4,5), BL(6,7,8)  ← reversed leg order */
    static const uint8_t ch_coxa  [6] = { 0, 3, 6, 6, 3, 0 };
    static const uint8_t ch_femur [6] = { 1, 4, 7, 7, 4, 1 };
    static const uint8_t ch_tibia [6] = { 2, 5, 8, 8, 5, 2 };

    /* Alternating pairs within rows: FR/BL phase 0.0, FL/BR phase 0.5, MR/ML opposite.
     * Each row's left and right legs alternate for stable pair-wise stepping. */
    static const float phase_offset[6] = { 0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f };

    const float neutral_z = -(CHASSIS_TO_SHOULDER + DESIRED_BELLY_CLEARANCE);
    const float RAD2DEG   = 57.29578f;

    for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
        LokoLeg *L = &st->legs[i];

        hexleg_init(&L->traj, LOKO_TRAJ_L, LOKO_TRAJ_H, LOKO_TRAJ_R, LOKO_TRAJ_S);

        L->ik_cfg.L1     = L1_COXA_LENGTH;
        L->ik_cfg.L2     = L2_FEMUR_LENGTH;
        L->ik_cfg.L3     = L3_TIBIA_LENGTH;
        L->ik_cfg.t1_min = -3.14159f;
        L->ik_cfg.t1_max =  3.14159f;
        L->ik_cfg.t2_min = -1.5708f;
        L->ik_cfg.t2_max =  1.5708f;
        L->ik_cfg.t3_min = TIBIA_MIN_RAD;
        L->ik_cfg.t3_max = TIBIA_MAX_RAD;

        L->neutral_x = NEUTRAL_REACH_MM;

        L->neutral_y = 0.0f;
        L->neutral_z = neutral_z;

        /* Initialize target to neutral stance */
        L->target.x = L->neutral_x;
        L->target.y = L->neutral_y;
        L->target.z = 0.0f; /* 0 = on ground */
        L->target_is_body_frame = 0;

        L->pivot_x   = pivot_x[i];
        L->pivot_y   = pivot_y[i];
        const float mount = atan2f(pivot_y[i], pivot_x[i]);
        L->mount_cos = cosf(mount);
        L->mount_sin = sinf(mount);

        L->phase        = 0.0f;
        L->phase_offset = phase_offset[i];

        L->on_right_board = on_right[i];
        L->servo_ch_coxa  = ch_coxa[i];
        L->servo_ch_femur = ch_femur[i];
        L->servo_ch_tibia = ch_tibia[i];

        /* Mirror the servos on the left side (legs 3, 4, 5).
         * All joint directions are flipped for proper left-right symmetry. */
        const float side_dir    = (i < 3) ? 1.0f : -1.0f;
        L->coxa_scale           = side_dir * COXA_DIR  * RAD2DEG;
        L->femur_scale          = side_dir * FEMUR_DIR * RAD2DEG;
        L->tibia_scale          = side_dir * TIBIA_DIR * RAD2DEG;
        
        /* Base offsets are 90/90/0. Calibration is now handled in pca9685.c */
        L->coxa_offset_deg      = 90.0f;
        L->femur_offset_deg     = 90.0f;
        L->tibia_offset_deg     = (i < 3) ? 0.0f : 180.0f;  /* left side offset for inverted scale */

        L->active = 1;
    }
}
