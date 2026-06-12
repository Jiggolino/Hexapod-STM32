/**
 * @file  loko_stabilizer.c
 * @brief IMU-based stabiliser — STABLE (COM shift) and LEVEL (platform tilt)
 *
 * STAB_STABLE
 *   Shifts the body horizontally so the projected COM stays over the support
 *   polygon when the robot is tilted.  Ideal shift = body_height * tan(θ).
 *
 *   Sign conventions (physical, verified against hardware):
 *     pitch_rad > 0  →  nose UP    →  shift body FORWARD  (+shift_x)
 *     roll_rad  > 0  →  right DOWN →  shift body RIGHT    (-shift_y; IMU roll axis inverted vs. convention)
 *
 *   In loko_servo.c the shift is applied as:
 *     leg_point.x -= shift_x;   (feet move back  → body moves forward)
 *     leg_point.y -= shift_y;   (feet move right → body moves left)
 *
 * STAB_LEVEL
 *   Tilts the body equal-and-opposite to the measured slope so the top
 *   platform stays level.  Uses P+I control with a first-order input LPF.
 *
 * Gains are in loko_config.h.
 */

#include "loko_stabilizer.h"
#include "loko_config.h"
#include <math.h>

#define DEG2RAD  (3.14159f / 180.0f)

/* ── PID state (shared by both LEVEL and STABLE) ────────────────────────── */
typedef struct {
    float integral;
    float prev_lpf;
    float lpf;
} PIDState;

static PIDState s_level_roll  = {0};
static PIDState s_level_pitch = {0};

/* ── STABLE mode output LPF state ───────────────────────────────────────── */
static float s_stable_lpf_x = 0.0f;
static float s_stable_lpf_y = 0.0f;

/*
 * Zeros all PID integrator and LPF state for one axis.
 * Input:  s — PIDState to reset
 * Output: void
 */
static void pid_reset(PIDState *s)
{
    s->integral = s->prev_lpf = s->lpf = 0.0f;
}

/*
 * Runs one PID tick with a non-linear adaptive LPF on the error signal.
 * The LPF alpha scales with |error|² relative to max_err, making the filter
 * more responsive at small errors and less aggressive at large sudden changes.
 *   alpha = base + (1−base) * (|err| / max_err)²
 * Input:  s        — PID integrator and LPF state
 *         err      — error signal (desired − measured)
 *         kp,ki,kd — PID gains
 *         i_clamp  — integrator anti-windup clamp
 *         dt       — time step in seconds
 * Output: PID output value
 */
static float pid_tick(PIDState *s,
                      float err,
                      float kp, float ki, float kd,
                      float i_clamp,
                      float dt)
{
    float max_err = STAB_MAX_TILT_DEG * (3.14159f / 180.0f);
    float t       = err / max_err;
    if (t < 0.0f) t = -t;
    if (t > 1.0f) t = 1.0f;
    float alpha = STAB_LPF_ALPHA + (1.0f - STAB_LPF_ALPHA) * t * t;
    s->lpf = alpha * err + (1.0f - alpha) * s->lpf;

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

/*
 * Populates a LokoStabilizerConfig with the maximum tilt, shift, and wave-mode
 * disable flag from loko_config.h constants.
 * Input:  cfg — config struct to initialise
 * Output: void
 */
void loko_stabilizer_init(LokoStabilizerConfig *cfg)
{
    cfg->max_roll_rad      = STAB_MAX_TILT_DEG * DEG2RAD;
    cfg->max_pitch_rad     = STAB_MAX_TILT_DEG * DEG2RAD;
    cfg->max_body_shift_mm = STAB_MAX_BODY_SHIFT_MM;
    cfg->wave_disable      = 1;
}

/*
 * Computes stabiliser correction for one control tick.
 *
 * Converts raw accelerometer data to roll/pitch angles via atan2, applies a
 * dead-zone to suppress noise, then skips PID updates when IMU data is stale
 * (repeated sample guard). In STAB_LEVEL mode the correction is a tilt angle
 * (roll/pitch in radians) written to out_roll/out_pitch. In STAB_STABLE mode
 * the correction is a horizontal shift in mm written to out_shift_x/y_mm.
 * All outputs are zero-initialised before processing and cleared entirely if
 * stab_mode is STAB_OFF or if wave gait is active with wave_disable set.
 *
 * Input:  cfg          — max limits and wave_disable flag
 *         imu_data     — latest accel/gyro sample in mg and mdps
 *         gait_mode    — current gait (suppresses correction in WAVE if flagged)
 *         stab_mode    — STAB_OFF / STAB_STABLE / STAB_LEVEL
 *         is_walking   — 1 if legs are cycling (scales gains down in LEVEL mode)
 *         dt           — time step in seconds
 * Output: out_roll, out_pitch      — body tilt angles in rad (STAB_LEVEL only)
 *         out_shift_x/y_mm        — body shift in mm (STAB_STABLE only)
 */
void loko_stabilizer_update(const LokoStabilizerConfig *cfg,
                            const IMU_Data_t           *imu_data,
                            LokoGaitMode                gait_mode,
                            LokoStabMode                stab_mode,
                            uint8_t                     is_walking,
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
        pid_reset(&s_level_roll);
        pid_reset(&s_level_pitch);
        s_stable_lpf_x = s_stable_lpf_y = 0.0f;
        return;
    }

    if (cfg->wave_disable && gait_mode == GAIT_WAVE) {
        pid_reset(&s_level_roll);
        pid_reset(&s_level_pitch);
        s_stable_lpf_x = s_stable_lpf_y = 0.0f;
        return;
    }

    /* Raw accelerometer → tilt angles.
     * Convention (inverted relative to IMU_GetAngles, verified on hardware):
     *   pitch_rad > 0  when nose UP
     *   roll_rad  > 0  when right side DOWN                                  */
    float ax_g = imu_data->accel_x_mg / 1000.0f;
    float ay_g = imu_data->accel_y_mg / 1000.0f;
    float az_g = imu_data->accel_z_mg / 1000.0f;

    float roll_rad  = clampf((atan2f(ax_g, az_g) - IMU_ROLL_BIAS_DEG  * DEG2RAD),
                             -cfg->max_roll_rad,  cfg->max_roll_rad);
    float pitch_rad = clampf((atan2f(ay_g, az_g) - IMU_PITCH_BIAS_DEG * DEG2RAD),
                             -cfg->max_pitch_rad, cfg->max_pitch_rad);

    /* Dead-zone: suppress correction within the sensor noise band */
    if      (pitch_rad >  STAB_DEADZONE_RAD) pitch_rad -= STAB_DEADZONE_RAD;
    else if (pitch_rad < -STAB_DEADZONE_RAD) pitch_rad += STAB_DEADZONE_RAD;
    else    pitch_rad = 0.0f;

    if      (roll_rad >  STAB_DEADZONE_RAD) roll_rad -= STAB_DEADZONE_RAD;
    else if (roll_rad < -STAB_DEADZONE_RAD) roll_rad += STAB_DEADZONE_RAD;
    else    roll_rad = 0.0f;

    /* Stale-sample guard: don't advance integrals on repeated IMU data */
    static float prev_ax = 0.0f, prev_ay = 0.0f;
    int fresh = (imu_data->accel_x_mg != prev_ax || imu_data->accel_y_mg != prev_ay);
    prev_ax = imu_data->accel_x_mg;
    prev_ay = imu_data->accel_y_mg;
    float pid_dt = fresh ? dt : 0.0f;

    if (stab_mode == STAB_LEVEL) {
        s_stable_lpf_x = s_stable_lpf_y = 0.0f;

        /* While walking, slow the loop ~10× so per-step bobbing is averaged
         * out but slow terrain tilt is still corrected. */
        float gscale = is_walking ? STAB_LEVEL_WALK_GAIN_SCALE : 1.0f;
        float kp = STAB_LEVEL_KP * gscale;
        float ki = STAB_LEVEL_KI * gscale;
        float kd = STAB_LEVEL_KD * gscale;

        float r = pid_tick(&s_level_roll,  roll_rad,  kp, ki, kd, STAB_LEVEL_I_CLAMP_RAD, pid_dt);
        float p = pid_tick(&s_level_pitch, pitch_rad, kp, ki, kd, STAB_LEVEL_I_CLAMP_RAD, pid_dt);

        *out_roll  = clampf(-r, -cfg->max_roll_rad,  cfg->max_roll_rad);
        *out_pitch = clampf(p, -cfg->max_pitch_rad, cfg->max_pitch_rad);

    } else { /* STAB_STABLE */
        float r = pid_tick(&s_level_roll,  roll_rad,  STAB_STABLE_KP, STAB_STABLE_KI, STAB_STABLE_KD, STAB_STABLE_I_CLAMP_RAD, pid_dt);
        float p = pid_tick(&s_level_pitch, pitch_rad, STAB_STABLE_KP, STAB_STABLE_KI, STAB_STABLE_KD, STAB_STABLE_I_CLAMP_RAD, pid_dt);

        *out_shift_x_mm = clampf(p * 100.0f, -cfg->max_body_shift_mm, cfg->max_body_shift_mm);
        *out_shift_y_mm = clampf(-r * 100.0f, -cfg->max_body_shift_mm, cfg->max_body_shift_mm);
    }
}
