#pragma once
#include "lokomotion.h"
#include "loko_transitions.h"
#include "ServoDriver.hpp"
#include "Controller.hpp"

class Locomotion {
    LokoState  _st;
    Controller _controller;
public:
    Locomotion() : _controller(_st.pad) {}

    void init(ServoDriver &right, ServoDriver &left);
    void update(const LokoInput &in, float dt);
    void updateTransitions();
    void enable(bool en);
    void setState(LokoFSMState s);

    void setFootLocal(int leg, float x, float y, float z);
    void setFootBody(int leg, float x, float y, float z);

    void configureStabilizer(float kp, float kd, float lpf_alpha);

    uint32_t errors() const;
    void clearErrors();

    Controller&       controller()       { return _controller; }
    const Controller& controller() const { return _controller; }

    LokoState&       raw()       { return _st; }
    const LokoState& raw() const { return _st; }
};
