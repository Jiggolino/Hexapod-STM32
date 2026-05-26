/*
 * hexapod_ik.c  --  Closed-form IK for a 3-DOF hexapod leg.
 *
 * Derivation (in the leg's vertical plane after removing coxa rotation):
 *
 *   r = sqrt(x^2 + y^2) - L1         (horizontal distance from femur joint)
 *   d = sqrt(r^2 + z^2)               (straight-line shoulder->foot)
 *
 *   Law of cosines on the triangle (L2, L3, d):
 *     d^2 = L2^2 + L3^2 + 2*L2*L3*cos(theta3)
 *     =>  cos(theta3) = (d^2 - L2^2 - L3^2) / (2 * L2 * L3)
 *
 *   That's the quadratic-equation-with-two-solutions step you mentioned:
 *     theta3 = +acos(...)   --> knee-down (elbow down)
 *     theta3 = -acos(...)   --> knee-up   (elbow up)  <-- we pick this one
 *
 *   Then theta2 closes the triangle:
 *     theta2 = atan2(z, r) - atan2(L3*sin(theta3), L2 + L3*cos(theta3))
 */

#include "hexapod_ik.h"
#include <math.h>

/* Use single-precision math everywhere -- STM32 H7 has a single-precision
 * FPU (VFPv5), so sqrtf/atan2f/acosf/sinf/cosf map to fast hardware paths
 * when you compile with -mfpu=fpv5-sp-d16 -mfloat-abi=hard. */

IKResult hex_leg_ik(const HexLegConfig *cfg,
                    float x, float y, float z,
                    float *theta1, float *theta2, float *theta3)
{
    /* ---- 1. Coxa: rotate the leg plane to face the target -------------- */
    float t1 = atan2f(y, x);

    /* ---- 2. Reduce to the 2-link planar problem ------------------------ */
    float r = sqrtf(x * x + y * y) - cfg->L1;   /* can be negative if
                                                   target is behind coxa  */
    float d2 = r * r + z * z;
    float d  = sqrtf(d2);

    /* ---- 3. Reachability check ---------------------------------------- */
    float reach_max = cfg->L2 + cfg->L3;
    float reach_min = fabsf(cfg->L2 - cfg->L3);
    float tolerance = 5.0f;  /* 5mm tolerance for FP errors and stabilizer adjustments */
    if (d > reach_max + tolerance || d < reach_min - tolerance) {
        return IK_UNREACHABLE;
    }

    /* ---- 4. Knee angle (the two-solution step) ------------------------ */
    float cos_t3 = (d2 - cfg->L2 * cfg->L2 - cfg->L3 * cfg->L3)
                 / (2.0f * cfg->L2 * cfg->L3);

    /* numerical safety: acosf() is undefined outside [-1, 1] */
    if (cos_t3 >  1.0f) cos_t3 =  1.0f;
    if (cos_t3 < -1.0f) cos_t3 = -1.0f;

    /* Pick knee-up: negative branch puts the knee above the foot when
     * the target is below-and-out-from the shoulder (normal walking). */
    float t3 = -acosf(cos_t3);

    /* ---- 5. Femur angle ---------------------------------------------- */
    float s3 = sinf(t3);
    float c3 = cosf(t3);
    float t2 = atan2f(z, r) - atan2f(cfg->L3 * s3, cfg->L2 + cfg->L3 * c3);

    /* ---- 6. Write outputs and check joint limits --------------------- */
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
