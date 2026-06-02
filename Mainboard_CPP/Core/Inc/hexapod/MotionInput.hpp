#pragma once

/* Locomotion velocity input for one control tick.
 * All values are normalized [-1, 1].
 *   vx: forward (+) / backward (-)
 *   vy: strafe left (+) / right (-)
 *   wz: yaw CCW (+) / CW (-)
 */
struct MotionInput {
    float vx = 0.f;
    float vy = 0.f;
    float wz = 0.f;

    MotionInput() = default;
    MotionInput(float vx, float vy, float wz) : vx(vx), vy(vy), wz(wz) {}
};
