#pragma once
#include "lokomotion.h"

/*
 * State — C++ view over the LokoState FSM fields.
 *
 * Provides named predicates and string labels so callers never need to
 * switch on raw LokoFSMState values.
 */
class State {
    LokoState *_st = nullptr;
public:
    State() = default;
    explicit State(LokoState *st) : _st(st) {}

    /* ── Current FSM value ───────────────────────────────────────────────── */

    LokoFSMState current()  const { return _st ? _st->state     : LOKO_UN_ARMED; }
    LokoGaitMode gaitMode() const { return _st ? _st->gait_mode : GAIT_TRIPOD;   }
    LokoStabMode stabMode() const { return _st ? _st->stab_mode : STAB_OFF;      }

    /* Request a transition (ignored if out of range) */
    void set(LokoFSMState s) { if (_st) loko_set_state(_st, s); }

    /* ── Named predicates ────────────────────────────────────────────────── */

    bool isUnArmed()        const { return current() == LOKO_UN_ARMED; }
    bool isArmed()          const { return current() != LOKO_UN_ARMED; }
    bool isStanding()       const { return current() == LOKO_STAND; }
    bool isWalking()        const { auto s = current(); return s == LOKO_WALK || s == LOKO_WALK_4_LEGS; }
    bool isMoving()         const { auto s = current(); return s == LOKO_WALK || s == LOKO_WALK_4_LEGS || s == LOKO_ROTATE_IN_PLACE; }
    bool isRotating()       const { return current() == LOKO_ROTATE_IN_PLACE; }
    bool isLookingAround()  const { return current() == LOKO_LOOK_AROUND; }
    bool isDancing()        const { return current() == LOKO_DANCING; }
    bool is4LegMode()       const { auto s = current(); return s == LOKO_STAND_4_LEGS || s == LOKO_WALK_4_LEGS; }

    /* ── String labels ───────────────────────────────────────────────────── */

    const char* name() const {
        if (!_st) return "UNKNOWN";
        switch (_st->state) {
        case LOKO_UN_ARMED:        return "UN_ARMED";
        case LOKO_STAND:           return "STAND";
        case LOKO_WALK:            return "WALK";
        case LOKO_WALK_4_LEGS:     return "WALK_4_LEGS";
        case LOKO_STAND_4_LEGS:    return "STAND_4_LEGS";
        case LOKO_ROTATE_IN_PLACE: return "ROTATE_IN_PLACE";
        case LOKO_DANCING:         return "DANCING";
        case LOKO_LOOK_AROUND:     return "LOOK_AROUND";
        default:                   return "UNKNOWN";
        }
    }

    const char* gaitName() const {
        if (!_st) return "TRIPOD";
        switch (_st->gait_mode) {
        case GAIT_WAVE:   return "WAVE";
        case GAIT_RIPPLE: return "RIPPLE";
        default:          return "TRIPOD";
        }
    }

    const char* stabName() const {
        if (!_st) return "OFF";
        switch (_st->stab_mode) {
        case STAB_STABLE: return "STABLE";
        case STAB_LEVEL:  return "LEVEL";
        default:          return "OFF";
        }
    }

    /* ── Raw access ──────────────────────────────────────────────────────── */

    LokoState*       raw()       { return _st; }
    const LokoState* raw() const { return _st; }
};
