#include "loko_tilt.h"
#include <math.h>
#include <stddef.h>

/*
 * Transforms a leg's foot target into IK-ready coordinates by applying the
 * body's height offset and a composite rotation (yaw → pitch → roll) to all
 * six legs simultaneously. The rotation acts on the world (legs) relative to
 * the body, so rotating the body left is equivalent to rotating the legs right.
 *
 * Steps:
 *   1. Place the target in ground-aligned body frame: add pivot offset, subtract height.
 *   2. Rotate around Z (yaw).
 *   3. Rotate around Y (pitch).
 *   4. Rotate around X (roll).
 *   5. Translate back to leg-local frame by removing pivot offset, then project
 *      onto the leg's mount axis (mount_cos / mount_sin).
 *
 * Input:  leg_point  — foot target offset from pivot in body frame (mm)
 *         mount_pos  — pivot position in body frame (mm)
 *         mount_cos  — cosine of the leg's mounting angle
 *         mount_sin  — sine of the leg's mounting angle
 *         height     — body height above ground (mm)
 *         roll       — desired body roll in radians (STAB_LEVEL output)
 *         pitch      — desired body pitch in radians (STAB_LEVEL output)
 *         yaw        — desired body yaw in radians (look-around / 4-leg mode)
 *         out_ik     — resulting position in leg-local IK frame (mm)
 * Output: void (out_ik written)
 */
void prepare_for_ik(Vector3f leg_point, Vector3f mount_pos, float mount_cos, float mount_sin, float height, float roll, float pitch, float yaw, Vector3f *out_ik)
{
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    float rx = mount_pos.x + leg_point.x;
    float ry = mount_pos.y + leg_point.y;
    float rz = leg_point.z - height;

    /* Rotation around Z (Yaw) */
    float rx_y = rx * cy - ry * sy;
    float ry_y = rx * sy + ry * cy;
    rx = rx_y;
    ry = ry_y;

    /* Rotation around Y (Pitch) */
    float x1 = rx * cp + rz * sp;
    float y1 = ry;
    float z1 = -rx * sp + rz * cp;

    /* Rotation around X (Roll) */
    float x2 = x1;
    float y2 = y1 * cr - z1 * sr;
    float z2 = y1 * sr + z1 * cr;

    float rel_x = x2 - mount_pos.x;
    float rel_y = y2 - mount_pos.y;

    out_ik->x =  rel_x * mount_cos + rel_y * mount_sin;
    out_ik->y = -rel_x * mount_sin + rel_y * mount_cos;
    out_ik->z = z2;
}
