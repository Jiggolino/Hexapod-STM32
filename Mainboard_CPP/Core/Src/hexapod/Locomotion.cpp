#include "hexapod/Locomotion.hpp"

void Locomotion::init(ServoDriver &right, ServoDriver &left)
{
    loko_init(&_st, right.handle(), left.handle());
    loko_enable(&_st, 1u);
}

void Locomotion::update(const LokoInput &in, float dt)
{
    loko_update(&_st, &in, dt);
}

void Locomotion::updateTransitions()
{
    loko_update_transitions(&_st);
}

void Locomotion::enable(bool en)
{
    loko_enable(&_st, en ? 1u : 0u);
}

void Locomotion::setState(LokoFSMState s)
{
    loko_set_state(&_st, s);
}

void Locomotion::setFootLocal(int leg, float x, float y, float z)
{
    loko_set_foot_local(&_st, leg, x, y, z);
}

void Locomotion::setFootBody(int leg, float x, float y, float z)
{
    loko_set_foot_body(&_st, leg, x, y, z);
}

void Locomotion::configureStabilizer(float kp, float kd, float lpf_alpha)
{
    loko_stabiliser_configure(&_st, kp, kd, lpf_alpha);
}

uint32_t Locomotion::errors() const
{
    return loko_get_errors(&_st);
}

void Locomotion::clearErrors()
{
    loko_clear_errors(&_st);
}

