#include "hexapod/Locomotion.hpp"

/*
 * Zeros LokoState, builds the default six-leg geometry from loko_config.h,
 * seeds tripod timing on all legs, binds the PCA9685 handles, and enables
 * servo output.
 * Input:  right, left — fully initialised ServoDriver instances for each board
 * Output: void
 */
void Locomotion::init(ServoDriver &right, ServoDriver &left)
{
    loko_init(&_st, right.handle(), left.handle());
    loko_enable(&_st, 1u);
}

/*
 * Runs one FSM dispatch tick: advances gait phases, computes body-frame foot
 * targets via the ICR arc pipeline, solves 3-DOF IK for each leg, applies IMU
 * stabiliser correction, and writes angles to servos.
 * Input:  in — velocity command {vx, vy, wz} in normalised [-1, 1]
 *         dt — elapsed time since previous call in seconds
 * Output: void (IK error flags set internally on failure)
 */
void Locomotion::update(const LokoInput &in, float dt)
{
    loko_update(&_st, &in, dt);
}

/*
 * Reads the current controller pad state and applies the FSM transition rules
 * for the active state. Must be called once per tick, before update().
 * Output: void
 */
void Locomotion::updateTransitions()
{
    loko_update_transitions(&_st);
}

/*
 * Enables or disables IK solving and servo writes.
 * Input:  en — true to enable outputs
 * Output: void
 */
void Locomotion::enable(bool en)
{
    loko_enable(&_st, en ? 1u : 0u);
}

/*
 * Overrides the FSM state directly, bypassing all transition guards.
 * Input:  s — target LokoFSMState
 * Output: void
 */
void Locomotion::setState(LokoFSMState s)
{
    loko_set_state(&_st, s);
}

/*
 * Sets a leg's foot target in leg-local frame (x forward along leg axis,
 * y lateral, z vertical; all in mm relative to the leg's pivot point).
 * Input:  leg   — index 0–5 (FR=0 MR=1 BR=2 BL=3 ML=4 FL=5)
 *         x,y,z — target position in mm
 * Output: void
 */
void Locomotion::setFootLocal(int leg, float x, float y, float z)
{
    loko_set_foot_local(&_st, leg, x, y, z);
}

/*
 * Sets a leg's foot target in body frame (origin at body centre, +X forward).
 * Input:  leg   — index 0–5
 *         x,y,z — target position in mm in body frame
 * Output: void
 */
void Locomotion::setFootBody(int leg, float x, float y, float z)
{
    loko_set_foot_body(&_st, leg, x, y, z);
}

/*
 * Stub for runtime stabiliser gain tuning. Gains are currently fixed in loko_config.h.
 * Input:  kp, kd, lpf_alpha — ignored
 * Output: void
 */
void Locomotion::configureStabilizer(float kp, float kd, float lpf_alpha)
{
    loko_stabiliser_configure(&_st, kp, kd, lpf_alpha);
}

/*
 * Returns accumulated IK error flags since the last clearErrors() call.
 * Output: bitmask of LOKO_ERR_* flags (IK_UNREACHABLE, IK_OUT_OF_LIMITS)
 */
uint32_t Locomotion::errors() const
{
    return loko_get_errors(&_st);
}

/*
 * Clears the accumulated IK error bitmask.
 * Output: void
 */
void Locomotion::clearErrors()
{
    loko_clear_errors(&_st);
}
