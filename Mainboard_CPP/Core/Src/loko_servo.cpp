/*
 * loko_servo.c  --  IK solve and servo output
 *
 * No debug prints – otherwise printf blocks UART RX and kills command response.
 */

#include "loko_servo.h"
#include "loko_config.h"
#include "loko_tilt.h"
#include "loko_states.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}


void loko_solve_and_write(LokoState *st)
{
    if (!st->enabled) return;

    /* Get IMU stabilizer outputs (only one pair is non-zero per mode) */
    float stable_roll = 0.0f, stable_pitch = 0.0f;
    float shift_x = 0.0f, shift_y = 0.0f;
    loko_get_stabilizer_angles(&stable_roll, &stable_pitch);
    loko_get_stabilizer_shift(&shift_x, &shift_y);

    float coxa_deg [LOKO_NUM_LEGS];
    float femur_deg[LOKO_NUM_LEGS];
    float tibia_deg[LOKO_NUM_LEGS];
    char  status   [LOKO_NUM_LEGS];

    for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
        LokoLeg *L = &st->legs[i];

        /* Resolve target into body-frame offset from pivot */
        Vector3f leg_point;
        if (L->target_is_body_frame) {
            leg_point.x = L->target.x - L->pivot_x;
            leg_point.y = L->target.y - L->pivot_y;
            leg_point.z = L->target.z;
        } else {
            leg_point.x = L->target.x * L->mount_cos - L->target.y * L->mount_sin;
            leg_point.y = L->target.x * L->mount_sin + L->target.y * L->mount_cos;
            leg_point.z = L->target.z;
        }

        /* STAB_STABLE: differential shift — right side gets the opposite
         * correction to the left side, creating a body tilt rather than
         * a pure translation.                                                */
        leg_point.x += (L->on_right_board ? -shift_x : shift_x);
        leg_point.y += -shift_y;

        /* Apply body tilt + height (STAB_LEVEL roll/pitch, or zero in STAB_STABLE) */
        Vector3f mount_pos = { L->pivot_x, L->pivot_y, 0.0f };
        Vector3f out_ik;
        prepare_for_ik(leg_point, mount_pos,
                       L->mount_cos, L->mount_sin,
                       st->body_height,
                       stable_roll, stable_pitch, st->body_yaw,
                       &out_ik);

        L->foot_x = out_ik.x;
        L->foot_y = out_ik.y;
        L->foot_z = out_ik.z;

        /* IK solve */
        IKResult r = hex_leg_ik(&L->ik_cfg,
                                L->foot_x, L->foot_y, L->foot_z,
                                &L->theta1, &L->theta2, &L->theta3);
        if (r == IK_UNREACHABLE)      st->error_flags |= LOKO_ERR_IK_UNREACHABLE;
        else if (r == IK_OUT_OF_LIMITS) st->error_flags |= LOKO_ERR_IK_OUT_OF_LIMITS;

        /* status character for prints */
        if (r == IK_OK) status[i] = '.';
        else if (r == IK_UNREACHABLE) status[i] = 'R';
        else                         status[i] = 'L';

        /* Convert to servo degrees and clamp to physical travel */
        coxa_deg[i]  = clampf(L->coxa_scale  * L->theta1 + L->coxa_offset_deg,  SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
        femur_deg[i] = clampf(L->femur_scale * L->theta2 + L->femur_offset_deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
        tibia_deg[i] = clampf(L->tibia_scale * L->theta3 + L->tibia_offset_deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
    }

    /* Front leg horizontal rotation in 4-leg modes */
    if (st->state == LOKO_STAND_4_LEGS || st->state == LOKO_WALK_4_LEGS) {
        coxa_deg[0] = clampf(coxa_deg[0] + LOKO_COXA_4LEG_OFFSET_DEG, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);  /* FR */
        coxa_deg[5] = clampf(coxa_deg[5] - LOKO_COXA_4LEG_OFFSET_DEG, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);  /* FL */
    }

    /* Build and send the debug line (throttled by LOKO_SERVO_PRINT_EVERY) */
    static uint32_t s_print_count = 0;
    if (++s_print_count >= LOKO_SERVO_PRINT_EVERY) {
        s_print_count = 0;
        static const char * const LEG_NAME[LOKO_NUM_LEGS] = {"FR", "MR", "BR", "BL", "ML", "FL"};
        char line[256];
        int pos = 0;
        pos += snprintf(line + pos, sizeof(line) - pos, "SERVO");
        for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
            pos += snprintf(line + pos, sizeof(line) - pos,
                            " %s%c[C:%3d F:%3d T:%3d]",
                            LEG_NAME[i], status[i],
                            (int)coxa_deg[i], (int)femur_deg[i], (int)tibia_deg[i]);
            if (pos >= (int)sizeof(line)) break;
        }
        pos += snprintf(line + pos, sizeof(line) - pos, "\r\n");
        printf("%s", line);   // now goes through DMA ring buffer
    }

    /* Drive the servos */
    for (int i = 0; i < LOKO_NUM_LEGS; ++i) {
        LokoLeg *L = &st->legs[i];
        PCA9685_t *board = L->on_right_board ? st->pca_right : st->pca_left;
        PCA9685_SetServoAngle(board, L->servo_ch_coxa,  coxa_deg[i]);
        PCA9685_SetServoAngle(board, L->servo_ch_femur, femur_deg[i]);
        PCA9685_SetServoAngle(board, L->servo_ch_tibia, tibia_deg[i]);
    }
}
