/**
 * @file  imu.c
 * @brief LSM6DSO (LSM6DSOTR) wrapper with low-pass filtering
 *
 * Chip: LSM6DSOTR  WHO_AM_I=0x6C  SA0 pulled low → 7-bit addr 0x6A → HAL 8-bit 0xD4
 * ODR: 52 Hz, ±2 g / ±250 dps, hardware LP2 + software IIR α=0.1
 */

#include "imu.h"
#include "loko_config.h"
#include "lsm6dso_reg.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static void imu_i2c_recover(I2C_HandleTypeDef *hi2c)
{
    hi2c->State     = HAL_I2C_STATE_READY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->Instance->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
}

/* HAL expects 8-bit address (7-bit << 1).  SA0 low → 0x6A → 0xD4 */
#define IMU_I2C_ADDR_8BIT  (LSM6DSO16IS_I2C_ADDR << 1)   /* 0x6A << 1 = 0xD4 */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    uint8_t msg[256];
    if (len > 255) return -1;
    msg[0] = reg;
    memcpy(&msg[1], buf, len);

    HAL_StatusTypeDef s = HAL_ERROR;
    for (int attempt = 0; attempt < 3; attempt++) {
        s = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDR_8BIT, msg, len + 1, 20);
        if (s == HAL_OK) break;
        imu_i2c_recover(hi2c);
        HAL_Delay(5);
    }
    return (s == HAL_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    HAL_StatusTypeDef s = HAL_ERROR;

    for (int attempt = 0; attempt < 3; attempt++) {
        s = HAL_I2C_Master_Transmit(hi2c, IMU_I2C_ADDR_8BIT, &reg, 1, 20);
        if (s == HAL_OK) break;
        imu_i2c_recover(hi2c);
        HAL_Delay(5);
    }
    if (s != HAL_OK) return -1;

    for (int attempt = 0; attempt < 3; attempt++) {
        s = HAL_I2C_Master_Receive(hi2c, IMU_I2C_ADDR_8BIT, buf, len, 20);
        if (s == HAL_OK) break;
        imu_i2c_recover(hi2c);
        HAL_Delay(5);
    }
    return (s == HAL_OK) ? 0 : -1;
}

static stmdev_ctx_t imu_dev_ctx = {0};
static uint8_t imu_initialized = 0;

typedef struct {
    float alpha;
    float x_filt, y_filt, z_filt;
} LPF1_t;

static LPF1_t accel_lpf = {0};
static LPF1_t gyro_lpf  = {0};

static void lpf1_reset(LPF1_t *f, float alpha)
{
    f->alpha = alpha;
    f->x_filt = f->y_filt = f->z_filt = 0.0f;
}

static void lpf1_update3(LPF1_t *f, float rx, float ry, float rz,
                         float *ox, float *oy, float *oz)
{
    f->x_filt = f->alpha * rx + (1.0f - f->alpha) * f->x_filt;
    f->y_filt = f->alpha * ry + (1.0f - f->alpha) * f->y_filt;
    f->z_filt = f->alpha * rz + (1.0f - f->alpha) * f->z_filt;
    *ox = f->x_filt;
    *oy = f->y_filt;
    *oz = f->z_filt;
}

int32_t IMU_Init(I2C_HandleTypeDef *hi2c)
{
    int32_t ret = 0;

    if (hi2c == NULL) {
        printf("IMU: NULL I2C handle\r\n");
        return -1;
    }

    int retry = 10;
    while (hi2c->State != HAL_I2C_STATE_READY && retry-- > 0)
        HAL_Delay(10);
    if (hi2c->State != HAL_I2C_STATE_READY) {
        printf("IMU: I2C not ready (State=%d)\r\n", hi2c->State);
        return -1;
    }

    imu_dev_ctx.write_reg = platform_write;
    imu_dev_ctx.read_reg  = platform_read;
    imu_dev_ctx.handle    = (void *)hi2c;
    imu_dev_ctx.mdelay    = HAL_Delay;

    /* WHO_AM_I: LSM6DSO = 0x6C */
    uint8_t whoami = 0;
    ret = lsm6dso_read_reg(&imu_dev_ctx, LSM6DSO_WHO_AM_I, &whoami, 1);
    if (ret != 0) {
        printf("IMU: WHO_AM_I read failed (ret=%ld)\r\n", ret);
        return -1;
    }
    if (whoami != LSM6DSO_ID) {
        printf("IMU: WHO_AM_I mismatch (got 0x%02X, expected 0x%02X)\r\n", whoami, LSM6DSO_ID);
        return -1;
    }
    printf("IMU: LSM6DSO detected (WHO_AM_I=0x%02X)\r\n", whoami);

    /* Soft reset */
    ret = lsm6dso_reset_set(&imu_dev_ctx, PROPERTY_ENABLE);
    if (ret != 0) { printf("IMU: reset failed\r\n"); return ret; }
    HAL_Delay(50);
    ret = lsm6dso_reset_set(&imu_dev_ctx, PROPERTY_DISABLE);
    if (ret != 0) { printf("IMU: reset-clear failed\r\n"); return ret; }
    HAL_Delay(50);

    /* Accelerometer: ±2 g, 52 Hz */
    ret = lsm6dso_xl_full_scale_set(&imu_dev_ctx, LSM6DSO_2g);
    if (ret != 0) { printf("IMU: accel FS failed\r\n"); return ret; }
    ret = lsm6dso_xl_data_rate_set(&imu_dev_ctx, LSM6DSO_XL_ODR_52Hz);
    if (ret != 0) { printf("IMU: accel ODR failed\r\n"); return ret; }

    /* Gyroscope: ±250 dps, 52 Hz */
    ret = lsm6dso_gy_full_scale_set(&imu_dev_ctx, LSM6DSO_250dps);
    if (ret != 0) { printf("IMU: gyro FS failed\r\n"); return ret; }
    ret = lsm6dso_gy_data_rate_set(&imu_dev_ctx, LSM6DSO_GY_ODR_52Hz);
    if (ret != 0) { printf("IMU: gyro ODR failed\r\n"); return ret; }

    /* Hardware LP2 filter on accelerometer (ODR/4 = ~13 Hz cutoff) */
    ret = lsm6dso_xl_filter_lp2_set(&imu_dev_ctx, PROPERTY_ENABLE);
    if (ret != 0) { printf("IMU: LP2 filter failed\r\n"); return ret; }

    /* Block data update */
    ret = lsm6dso_block_data_update_set(&imu_dev_ctx, PROPERTY_ENABLE);
    if (ret != 0) { printf("IMU: BDU failed\r\n"); return ret; }

    lpf1_reset(&accel_lpf, 0.1f);
    lpf1_reset(&gyro_lpf,  0.1f);

    printf("IMU: init OK\r\n");
    imu_initialized = 1;
    return 0;
}

int32_t IMU_Read(IMU_Data_t *out)
{
    if (!out) return -1;
    if (!imu_initialized) {
        memset(out, 0, sizeof(IMU_Data_t));
        return 0;
    }

    int16_t raw_accel[3] = {0};
    int16_t raw_gyro[3]  = {0};

    int32_t ret = lsm6dso_acceleration_raw_get(&imu_dev_ctx, raw_accel);
    if (ret != 0) {
        memset(out, 0, sizeof(IMU_Data_t));
        return ret;
    }
    ret = lsm6dso_angular_rate_raw_get(&imu_dev_ctx, raw_gyro);
    if (ret != 0) {
        memset(out, 0, sizeof(IMU_Data_t));
        return ret;
    }

    float ax = lsm6dso_from_fs2_to_mg(raw_accel[0]);
    float ay = lsm6dso_from_fs2_to_mg(raw_accel[1]);
    float az = lsm6dso_from_fs2_to_mg(raw_accel[2]);
    float gx = lsm6dso_from_fs250_to_mdps(raw_gyro[0]);
    float gy = lsm6dso_from_fs250_to_mdps(raw_gyro[1]);
    float gz = lsm6dso_from_fs250_to_mdps(raw_gyro[2]);

    lpf1_update3(&accel_lpf, ax, ay, az,
                 &out->accel_x_mg, &out->accel_y_mg, &out->accel_z_mg);
    lpf1_update3(&gyro_lpf, gx, gy, gz,
                 &out->gyro_x_mdps, &out->gyro_y_mdps, &out->gyro_z_mdps);
    return 0;
}

void IMU_GetAngles(float *out_roll_deg, float *out_pitch_deg)
{
    IMU_Data_t d;
    IMU_Read(&d);
    float ax = d.accel_x_mg / 1000.0f;
    float ay = d.accel_y_mg / 1000.0f;
    float az = d.accel_z_mg / 1000.0f;
    /* Subtract mounting bias, then negate so conventions match robot frame:
     * roll: right side lower → negative   pitch: nose lower → negative */
    if (out_roll_deg)  *out_roll_deg  = -(atan2f(ax, az) * (180.0f / 3.14159f) - IMU_ROLL_BIAS_DEG);
    if (out_pitch_deg) *out_pitch_deg = -(atan2f(ay, az) * (180.0f / 3.14159f) - IMU_PITCH_BIAS_DEG);
}

void IMU_Print(const IMU_Data_t *data)
{
    if (data)
        printf("IMU | A: %+6.1f %+6.1f %+6.1f mg | G: %+7.1f %+7.1f %+7.1f mdps\r\n",
               data->accel_x_mg, data->accel_y_mg, data->accel_z_mg,
               data->gyro_x_mdps, data->gyro_y_mdps, data->gyro_z_mdps);
}
