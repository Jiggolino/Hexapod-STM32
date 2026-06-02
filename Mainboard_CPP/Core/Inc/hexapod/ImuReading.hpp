#pragma once

/* Filtered IMU sensor output — mirrors IMU_Data_t but as a C++ struct. */
struct ImuReading {
    float accel_x_mg  = 0.f;   /* acceleration X in milli-g (filtered) */
    float accel_y_mg  = 0.f;
    float accel_z_mg  = 0.f;
    float gyro_x_mdps = 0.f;   /* angular rate X in milli-dps (filtered) */
    float gyro_y_mdps = 0.f;
    float gyro_z_mdps = 0.f;
};
