/*
 * hexapod_ik.h  --  Simple 3-DOF inverse kinematics for one hexapod leg.
 *
 * Convention (right-handed, origin at the coxa/shoulder joint):
 *   +x : forward out of the shoulder (in the leg's neutral direction)
 *   +y : sideways
 *   +z : up
 *
 * Joint angles (radians):
 *   theta1 : coxa   -- rotation about z (horizontal swing)
 *   theta2 : femur  -- pitch, 0 = femur horizontal, +ve = femur up
 *   theta3 : tibia  -- knee angle relative to femur,
 *                     0 = tibia in line with femur (fully extended),
 *                     negative = knee bent so foot folds toward body
 *
 * With the knee-up choice made inside hex_leg_ik(), theta3 is returned
 * negative, i.e. the knee sits above the foot.
 */

#ifndef HEXAPOD_IK_H
#define HEXAPOD_IK_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* link lengths, mm */
    float L1;   /* coxa  */
    float L2;   /* femur */
    float L3;   /* tibia */

    /* joint limits in radians */
    float t1_min, t1_max;
    float t2_min, t2_max;
    float t3_min, t3_max;
} HexLegConfig;

typedef enum {
    IK_OK             = 0,
    IK_UNREACHABLE    = 1,   /* target outside the leg's workspace     */
    IK_OUT_OF_LIMITS  = 2    /* solution exists but violates joint cap */
} IKResult;

/*
 * Solve inverse kinematics for one leg.
 * Returns IK_OK on success. On IK_OUT_OF_LIMITS the angles are still
 * written so you can clamp or log them.
 */
IKResult hex_leg_ik(const HexLegConfig *cfg,
                    float x, float y, float z,
                    float *theta1, float *theta2, float *theta3);

#ifdef __cplusplus
}
#endif

#endif /* HEXAPOD_IK_H */
