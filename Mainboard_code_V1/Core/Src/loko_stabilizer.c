/**
 * @file  loko_stabilizer.c
 * @brief IMU-based stabiliser — STABLE (COM shift) and LEVEL (platform tilt)
 *
 * STAB_STABLE
 *   Translates the body horizontally so gravity's projection stays centred
 *   over the support polygon.  When tilted by θ the ideal shift is
 *   body_height·tan(θ) ≈ body_height·θ — that is the Kp baseline.
 *   Output: shift_x_mm, shift_y_mm (mm).  roll/pitch outputs = 0.
 *
 * STAB_LEVEL
 *   Tilts the body equal-and-opposite to the measured slope so the top
 *   platform stays level.
 *   Output: roll_rad, pitch_rad (rad).  shift outputs = 0.
 *
 * Both modes run independent PID state to avoid integral pollution on
 * mode switches.  D-term derivative is taken on the low-pass filtered
 * error to reject high-frequency IMU noise.
 *
 * Gains are in loko_config.h.
 */

#include "loko_stabilizer.h"
#include "loko_config.h"
#include <math.h>

#define DEG2RAD  (3.14159f / 180.0f)

typedef struct {
    float integral;
    float prev_lpf;
    float lpf;
} PIDState;

static PIDState s_level_roll  = {0};
static PIDState s_level_pitch = {0};
static PIDState s_stable_roll  = {0};
static PIDState s_stable_pitch = {0};

static void pid_reset(PIDState *s)
{
    s->integral = s->prev_lpf = s->lpf = 0.0f;
}

/* Returns P+I+D output.  i_clamp limits integral windup. */
static float pid_tick(PIDState *s,
                      float err,
                      float kp, float ki, float kd,
                      float i_clamp,
                      float dt)
{
    /* Low-pass filter on error — D acts on smoothed signal only */
    s->lpf = STAB_LPF_ALPHA * err + (1.0f - STAB_LPF_ALPHA) * s->lpf;

    float derivative = (dt > 0.0f) ? (s->lpf - s->prev_lpf) / dt : 0.0f;
    s->prev_lpf = s->lpf;

    s->integral += s->lpf * dt;
    if (s->integral >  i_clamp) s->integral =  i_clamp;
    if (s->integral < -i_clamp) s->integral = -i_clamp;

    return kp * s->lpf + ki * s->integral + kd * derivative;
}

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* ─────────────────────────────────────────────────────────────────────────── */

void loko_stabilizer_init(LokoStabilizerConfig *cfg)
{
    cfg->max_roll_rad      = 10.0f * DEG2RAD;
    cfg->max_pitch_rad     = 10.0f * DEG2RAD;
    cfg->max_body_shift_mm = 50.0f;
    cfg->wave_disable      = 1;
}

void loko_stabilizer_update(const LokoStabilizerConfig *cfg,
                            const IMU_Data_t           *imu_data,
                            LokoGaitMode                gait_mode,
                            LokoStabMode                stab_mode,
                            float                       dt,
                            float                      *out_roll,
                            float                      *out_pitch,
                            float                      *out_shift_x_mm,
                            float                      *out_shift_y_mm)
{
    *out_roll       = 0.0f;
    *out_pitch      = 0.0f;
    *out_shift_x_mm = 0.0f;
    *out_shift_y_mm = 0.0f;

    if (stab_mode == STAB_OFF) {
        pid_reset(&s_level_roll);  pid_reset(&s_level_pitch);
        pid_reset(&s_stable_roll); pid_reset(&s_stable_pitch);
        return;
    }

    if (cfg->wave_disable && gait_mode == GAIT_WAVE) {
        pid_reset(&s_level_roll);  pid_reset(&s_level_pitch);
        pid_reset(&s_stable_roll); pid_reset(&s_stable_pitch);
        return;
    }

    float roll_deg, pitch_deg;
    IMU_GetAngles(&roll_deg, &pitch_deg);

    float roll_rad  = clampf(roll_deg  * DEG2RAD, -cfg->max_roll_rad,  cfg->max_roll_rad);
    float pitch_rad = clampf(pitch_deg * DEG2RAD, -cfg->max_pitch_rad, cfg->max_pitch_rad);

    /* Deadzone: zero the error (and drain integral) while inside the band.
     * This stops IMU noise from driving the servos at rest. */
    if (roll_rad  >  STAB_DEADZONE_RAD) roll_rad  -= STAB_DEADZONE_RAD;
    else if (roll_rad  < -STAB_DEADZONE_RAD) roll_rad  += STAB_DEADZONE_RAD;
    else { roll_rad  = 0.0f; pid_reset(&s_level_roll);  pid_reset(&s_stable_roll); }

    if (pitch_rad >  STAB_DEADZONE_RAD) pitch_rad -= STAB_DEADZONE_RAD;
    else if (pitch_rad < -STAB_DEADZONE_RAD) pitch_rad += STAB_DEADZONE_RAD;
    else { pitch_rad = 0.0f; pid_reset(&s_level_pitch); pid_reset(&s_stable_pitch); }

    if (stab_mode == STAB_LEVEL) {
        /* ── LEVEL: negate tilt so platform surface stays horizontal ────── */
        pid_reset(&s_stable_roll);
        pid_reset(&s_stable_pitch);

        float r = pid_tick(&s_level_roll,  roll_rad,  STAB_LEVEL_KP, STAB_LEVEL_KI, STAB_LEVEL_KD, STAB_LEVEL_I_CLAMP_RAD, dt);
        float p = pid_tick(&s_level_pitch, pitch_rad, STAB_LEVEL_KP, STAB_LEVEL_KI, STAB_LEVEL_KD, STAB_LEVEL_I_CLAMP_RAD, dt);

        /* Correction is opposite to measured tilt */
        *out_roll  = clampf(-r, -cfg->max_roll_rad,  cfg->max_roll_rad);
        *out_pitch = clampf(-p, -cfg->max_pitch_rad, cfg->max_pitch_rad);

    } else { /* STAB_STABLE */
        /* ── STABLE: shift body so COM projects over polygon centre ─────── */
        /* IMU convention (imu.c:197):
         *   +pitch = nose UP   → COM shifts backward → body shifts forward (+X)
         *   +roll  = left DOWN → COM shifts left     → body shifts right   (-Y)
         * Negate roll input so the PID drives body in the correct direction. */
        pid_reset(&s_level_roll);
        pid_reset(&s_level_pitch);

        float sx = pid_tick(&s_stable_pitch,  pitch_rad, STAB_STABLE_KP, STAB_STABLE_KI, STAB_STABLE_KD, STAB_STABLE_I_CLAMP_MM, dt);
        float sy = pid_tick(&s_stable_roll,  -roll_rad,  STAB_STABLE_KP, STAB_STABLE_KI, STAB_STABLE_KD, STAB_STABLE_I_CLAMP_MM, dt);

        *out_shift_x_mm = clampf(sx, -cfg->max_body_shift_mm, cfg->max_body_shift_mm);
        *out_shift_y_mm = clampf(sy, -cfg->max_body_shift_mm, cfg->max_body_shift_mm);
    }
}
