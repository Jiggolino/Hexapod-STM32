/**
 * @file  loko_stabilizer.h
 * @brief IMU-based stabiliser — two distinct modes
 *
 * STAB_STABLE  Shifts the body horizontally so the COM stays centred over
 *              the support polygon.  Outputs shift_x/y in mm; roll/pitch = 0.
 *              Use when walking on slopes where tipping is the risk.
 *
 * STAB_LEVEL   Tilts the body to keep the top platform level.
 *              Outputs roll/pitch in rad; shift_x/y = 0.
 *              Use when a level sensor / camera platform is required.
 *
 * Both modes use independent PID state so switching is clean.
 * Gains live in loko_config.h (STAB_STABLE_K* / STAB_LEVEL_K*).
 */

#ifndef LOKO_STABILIZER_H
#define LOKO_STABILIZER_H

#include "lokomotion.h"
#include "imu.h"

typedef struct {
    float max_roll_rad;       /* Max roll correction output (default 10°) */
    float max_pitch_rad;      /* Max pitch correction output (default 10°) */
    float max_body_shift_mm;  /* Max COM shift output (default 50 mm) */
    uint8_t wave_disable;     /* Disable stabilisation during wave gait */
} LokoStabilizerConfig;

void loko_stabilizer_init(LokoStabilizerConfig *cfg);

/**
 * Run one stabiliser tick.
 *
 * STAB_LEVEL  → out_roll/out_pitch carry body tilt correction (rad);
 *               out_shift_x/y = 0.
 * STAB_STABLE → out_shift_x/y carry body XY translation (mm);
 *               out_roll/out_pitch = 0.
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
                            float                      *out_shift_y_mm);

#endif /* LOKO_STABILIZER_H */
