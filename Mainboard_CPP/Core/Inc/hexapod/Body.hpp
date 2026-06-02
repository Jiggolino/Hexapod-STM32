#pragma once
#include "lokomotion.h"

/*
 * Body — C++ view over LokoState's body orientation and height fields.
 *
 * Holds a pointer into the shared LokoState — no data duplication.
 */
class Body {
    LokoState *_st = nullptr;
public:
    Body() = default;
    explicit Body(LokoState *st) : _st(st) {}

    /* ── Getters ─────────────────────────────────────────────────────────── */

    float roll()    const { return _st ? _st->body_roll    : 0.f; }
    float pitch()   const { return _st ? _st->body_pitch   : 0.f; }
    float yaw()     const { return _st ? _st->body_yaw     : 0.f; }
    float height()  const { return _st ? _st->body_height  : 0.f; }
    float shiftX()  const { return _st ? _st->body_shift_x : 0.f; }
    float shiftY()  const { return _st ? _st->body_shift_y : 0.f; }

    /* ── Setters ─────────────────────────────────────────────────────────── */

    void setRoll  (float r) { if (_st) _st->body_roll    = r; }
    void setPitch (float p) { if (_st) _st->body_pitch   = p; }
    void setYaw   (float y) { if (_st) _st->body_yaw     = y; }
    void setHeight(float h) { if (_st) _st->body_height  = h; }
    void setShiftX(float x) { if (_st) _st->body_shift_x = x; }
    void setShiftY(float y) { if (_st) _st->body_shift_y = y; }

    /* Zero all body pose overrides (used when leaving LOOK_AROUND) */
    void resetOrientation() {
        if (!_st) return;
        _st->body_roll    = 0.f;
        _st->body_pitch   = 0.f;
        _st->body_yaw     = 0.f;
        _st->body_shift_x = 0.f;
        _st->body_shift_y = 0.f;
    }

    /* ── Raw access ──────────────────────────────────────────────────────── */

    LokoState*       raw()       { return _st; }
    const LokoState* raw() const { return _st; }
};
