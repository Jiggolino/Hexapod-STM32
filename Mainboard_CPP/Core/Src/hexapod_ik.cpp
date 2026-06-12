/*
 * hexapod_ik.c  --  Closed-form IK for a 3-DOF hexapod leg.
 *
 * Derivation (in the leg's vertical plane after removing coxa rotation):
 *
 *   r = sqrt(x^2 + y^2) - L1         (horizontal distance from femur joint)
 *   d = sqrt(r^2 + z^2)               (straight-line shoulder→foot)
 *
 *   Law of cosines on the triangle (L2, L3, d):
 *     cos(theta3) = (d^2 - L2^2 - L3^2) / (2 * L2 * L3)
 *
 *   theta3 = -acos(...)   ← knee-up solution (elbow-up, knee above foot for
 *                           normal downward-pointing walking geometry)
 *
 *   theta2 closes the triangle:
 *     theta2 = atan2(z, r) - atan2(L3*sin(theta3), L2 + L3*cos(theta3))
 */

#include "hexapod_ik.h"
#include <math.h>

/*
 * Solves the 3-DOF inverse kinematics for one hexapod leg and checks joint limits.
 *
 * Steps:
 *   1. Coxa angle theta1 = atan2(y, x)
 *   2. Project into the 2D femur-tibia plane: r = sqrt(x²+y²) − L1
 *   3. Reachability check on d = sqrt(r²+z²) against [|L2−L3|, L2+L3] + 5 mm tolerance
 *   4. Knee angle theta3 = −acos((d²−L2²−L3²)/(2·L2·L3))  (knee-up branch)
 *   5. Femur angle theta2 from atan2
 *   6. Write outputs and test against configured joint limits
 *
 * All arithmetic uses single-precision FP which maps to hardware instructions
 * on the STM32H7 FPU (-mfpu=fpv5-sp-d16 -mfloat-abi=hard).
 *
 * Input:  cfg            — link lengths L1/L2/L3 and per-joint angle limits
 *         x, y, z        — foot target in leg-local frame (mm)
 *         theta1/2/3     — output joint angles in radians
 * Output: IK_OK, IK_UNREACHABLE (target out of workspace), or IK_OUT_OF_LIMITS
 */
IKResult hex_leg_ik(const HexLegConfig *cfg,
                    float x, float y, float z,
                    float *theta1, float *theta2, float *theta3)
{
    /* 1. Coxa: rotate the leg plane to face the target */
    float t1 = atan2f(y, x);

    /* 2. Reduce to the 2-link planar problem */
    float r = sqrtf(x * x + y * y) - cfg->L1;   /* can be negative if
                                                   target is behind coxa  */
    float d2 = r * r + z * z;
    float d  = sqrtf(d2);

    /* 3. Reachability check */
    float reach_max = cfg->L2 + cfg->L3;
    float reach_min = fabsf(cfg->L2 - cfg->L3);
    float tolerance = 5.0f;  /* 5 mm tolerance for FP errors and stabiliser adjustments */
    if (d > reach_max + tolerance || d < reach_min - tolerance) {
        return IK_UNREACHABLE;
    }

    /* 4. Knee angle (two-solution step: pick knee-up = negative branch) */
    float cos_t3 = (d2 - cfg->L2 * cfg->L2 - cfg->L3 * cfg->L3)
                 / (2.0f * cfg->L2 * cfg->L3);

    /* Clamp for numerical safety: acosf() is undefined outside [-1, 1] */
    if (cos_t3 >  1.0f) cos_t3 =  1.0f;
    if (cos_t3 < -1.0f) cos_t3 = -1.0f;

    float t3 = -acosf(cos_t3);

    /* 5. Femur angle */
    float s3 = sinf(t3);
    float c3 = cosf(t3);
    float t2 = atan2f(z, r) - atan2f(cfg->L3 * s3, cfg->L2 + cfg->L3 * c3);

    /* 6. Write outputs and check joint limits */
    *theta1 = t1;
    *theta2 = t2;
    *theta3 = t3;

    if (t1 < cfg->t1_min || t1 > cfg->t1_max ||
        t2 < cfg->t2_min || t2 > cfg->t2_max ||
        t3 < cfg->t3_min || t3 > cfg->t3_max) {
        return IK_OUT_OF_LIMITS;
    }
    return IK_OK;
}
