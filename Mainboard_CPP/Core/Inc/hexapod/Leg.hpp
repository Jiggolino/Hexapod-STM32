#pragma once
#include "lokomotion.h"
#include "Servo.hpp"

/*
 * Leg — C++ view over a LokoLeg + three Servo objects for its joints.
 *
 * The underlying LokoLeg data is owned by LokoState; this class holds a
 * pointer into it so no state is duplicated.  Servo calibration is copied
 * from LokoLeg once via initServos() after loko_init() runs.
 *
 * Leg numbering (matches loko_legs.cpp):
 *   0=FR  1=MR  2=BR  3=BL  4=ML  5=FL
 */
class Leg {
    LokoLeg *_leg = nullptr;
public:
    Servo coxa;
    Servo femur;
    Servo tibia;

    Leg() = default;
    explicit Leg(LokoLeg *leg) : _leg(leg) {}

    /* Call once after loko_init() so Servo objects know their board + calibration. */
    void initServos(PCA9685_t *pca_right, PCA9685_t *pca_left) {
        if (!_leg) return;
        PCA9685_t *board = _leg->on_right_board ? pca_right : pca_left;
        coxa .configure(board, _leg->servo_ch_coxa,  _leg->coxa_scale,  _leg->coxa_offset_deg);
        femur.configure(board, _leg->servo_ch_femur, _leg->femur_scale, _leg->femur_offset_deg);
        tibia.configure(board, _leg->servo_ch_tibia, _leg->tibia_scale, _leg->tibia_offset_deg);
    }

    /* ── Target control ──────────────────────────────────────────────────── */

    /* Set foot target in leg-local frame (standard) */
    void setTarget(float x, float y, float z) {
        if (!_leg) return;
        _leg->target.x = x;
        _leg->target.y = y;
        _leg->target.z = z;
        _leg->target_is_body_frame = 0;
    }

    /* Set foot target in body frame */
    void setTargetBodyFrame(float x, float y, float z) {
        if (!_leg) return;
        _leg->target.x = x;
        _leg->target.y = y;
        _leg->target.z = z;
        _leg->target_is_body_frame = 1;
    }

    /* Return leg to neutral stance position at ground level */
    void resetToNeutral() {
        if (!_leg) return;
        _leg->target.x = _leg->neutral_x;
        _leg->target.y = _leg->neutral_y;
        _leg->target.z = 0.0f;
        _leg->target_is_body_frame = 0;
    }

    /* ── State ───────────────────────────────────────────────────────────── */

    bool active()           const { return _leg && _leg->active; }
    void setActive(bool a)        { if (_leg) _leg->active = a ? 1 : 0; }

    float phase()           const { return _leg ? _leg->phase    : 0.f; }

    /* ── Neutral position ────────────────────────────────────────────────── */

    float neutralX()        const { return _leg ? _leg->neutral_x : 0.f; }
    float neutralY()        const { return _leg ? _leg->neutral_y : 0.f; }
    float neutralZ()        const { return _leg ? _leg->neutral_z : 0.f; }

    /* ── Current target ──────────────────────────────────────────────────── */

    float targetX()         const { return _leg ? _leg->target.x : 0.f; }
    float targetY()         const { return _leg ? _leg->target.y : 0.f; }
    float targetZ()         const { return _leg ? _leg->target.z : 0.f; }

    /* ── Solved IK angles (after loko_solve_and_write) ───────────────────── */

    float theta1()          const { return _leg ? _leg->theta1 : 0.f; }
    float theta2()          const { return _leg ? _leg->theta2 : 0.f; }
    float theta3()          const { return _leg ? _leg->theta3 : 0.f; }

    /* ── Raw access ──────────────────────────────────────────────────────── */

    LokoLeg*       raw()       { return _leg; }
    const LokoLeg* raw() const { return _leg; }
};
