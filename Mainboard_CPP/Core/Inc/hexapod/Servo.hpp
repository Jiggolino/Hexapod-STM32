#pragma once
#include "pca9685.h"
#include "loko_config.h"

/* Single servo channel — holds calibration and writes via PCA9685. */
class Servo {
    PCA9685_t *_board   = nullptr;
    uint8_t    _channel = 0;
    float      _scale   = 1.0f;   /* deg = scale * angle_rad + offset */
    float      _offset  = 90.0f;
    float      _lastDeg = 90.0f;

    static float _clamp(float v, float lo, float hi) {
        return v < lo ? lo : v > hi ? hi : v;
    }
public:
    Servo() = default;

    void configure(PCA9685_t *board, uint8_t ch, float scale, float offset) {
        _board = board; _channel = ch; _scale = scale; _offset = offset;
    }

    /* Write using the leg's rad→deg calibration */
    void writeRad(float rad) {
        writeDeg(_scale * rad + _offset);
    }

    /* Write a pre-computed degree value (still clamped) */
    void writeDeg(float deg) {
        _lastDeg = _clamp(deg, SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG);
        if (_board) PCA9685_SetServoAngle(_board, _channel, _lastDeg);
    }

    float   lastDeg() const  { return _lastDeg;  }
    float   scale()   const  { return _scale;    }
    float   offset()  const  { return _offset;   }
    uint8_t channel() const  { return _channel;  }
};
