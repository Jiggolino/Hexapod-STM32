#include "loko_tilt.h"
#include <math.h>
#include <stddef.h>

void prepare_for_ik(Vector3f leg_point, Vector3f mount_pos, float mount_cos, float mount_sin, float height, float roll, float pitch, Vector3f *out_ik)
{
    /* 1. Calculate Rotation Matrices */
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);

    /* 2. Target point relative to body center in ground-aligned frame */
    float rx = mount_pos.x + leg_point.x;
    float ry = mount_pos.y + leg_point.y;
    float rz = leg_point.z - height;

    /* 3. Apply Body Rotation (Pitch then Roll)
     * This rotates the 'world' (legs) relative to the body. 
     * To tilt the body FORWARD, we rotate the legs BACKWARD. */
    
    /* Rotation around Y (Pitch) */
    float x1 = rx * cp + rz * sp;
    float y1 = ry;
    float z1 = -rx * sp + rz * cp;

    /* Rotation around X (Roll) */
    float x2 = x1;
    float y2 = y1 * cr - z1 * sr;
    float z2 = y1 * sr + z1 * cr;

    /* 4. Translate back to Leg-Local Space */
    float rel_x = x2 - mount_pos.x;
    float rel_y = y2 - mount_pos.y;

    out_ik->x =  rel_x * mount_cos + rel_y * mount_sin;
    out_ik->y = -rel_x * mount_sin + rel_y * mount_cos;
    out_ik->z = z2;
}
