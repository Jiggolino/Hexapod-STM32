/**
 * @file  pca9685.c
 * @brief PCA9685 16-channel PWM driver implementation
 *
 * Internal oscillator: 25 MHz (typical)
 * PWM resolution:      12 bit → 4096 ticks per period
 *
 * Prescale formula:  round(25 000 000 / (4096 × freq_hz)) − 1
 *
 * Pulse-to-tick:     tick = pulse_us × 4096 × freq_hz / 1 000 000
 */

#include "pca9685.h"
#include "loko_config.h"
#include <math.h>

/* ── Helpers ────────────────────────────────────────────────────────────── */

/* After a HAL I2C error the peripheral can be left with State != READY,
 * causing every subsequent call to return HAL_BUSY immediately.
 * Manually clearing State and ErrorCode lets the retry actually attempt
 * a new transaction instead of failing instantly. */
static void i2c_recover(I2C_HandleTypeDef *hi2c)
{
    hi2c->State     = HAL_I2C_STATE_READY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    /* Clear any NACK / STOP / BERR flags in the hardware status register */
    hi2c->Instance->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
}

static HAL_StatusTypeDef write_reg(PCA9685_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    HAL_StatusTypeDef s;
    for (int i = 0; i < 3; i++) {
        s = HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, buf, 2, 10);
        if (s == HAL_OK) break;
        i2c_recover(dev->hi2c);
        HAL_Delay(1);
    }
    return s;
}

static HAL_StatusTypeDef read_reg(PCA9685_t *dev, uint8_t reg, uint8_t *val)
{
    HAL_StatusTypeDef s;
    for (int i = 0; i < 3; i++) {
        s = HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, &reg, 1, 10);
        if (s == HAL_OK) {
            s = HAL_I2C_Master_Receive(dev->hi2c, dev->addr, val, 1, 10);
            if (s == HAL_OK) break;
        }
        i2c_recover(dev->hi2c);
        HAL_Delay(1);
    }
    return s;
}

/* ── Public functions ───────────────────────────────────────────────────── */

HAL_StatusTypeDef PCA9685_SoftwareReset(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd = 0x06;
    return HAL_I2C_Master_Transmit(hi2c, PCA9685_SW_RESET_ADDR, &cmd, 1, 10);
}

HAL_StatusTypeDef PCA9685_Init(PCA9685_t *dev)
{
    HAL_StatusTypeDef s;

    /* Default pulse limits if not set by caller */
    if (dev->min_us == 0) dev->min_us = PCA9685_SERVO_MIN_US;
    if (dev->max_us == 0) dev->max_us = PCA9685_SERVO_MAX_US;

    /* Ensure auto-increment is on and device is awake */
    s = write_reg(dev, PCA9685_MODE1, PCA9685_MODE1_AI);
    if (s != HAL_OK) return s;

    /* MODE2: output change on STOP command, totem-pole outputs */
    s = write_reg(dev, PCA9685_MODE2, 0x04);
    if (s != HAL_OK) return s;

    /* Set frequency (defaults to 50 Hz for servos) */
    float freq = (dev->freq_hz > 0.0f) ? dev->freq_hz : 50.0f;
    return PCA9685_SetFrequency(dev, freq);
}

HAL_StatusTypeDef PCA9685_SetFrequency(PCA9685_t *dev, float freq_hz)
{
    HAL_StatusTypeDef s;
    uint8_t old_mode, prescale;

    /* prescale = round(25 MHz / (4096 × freq)) − 1 */
    prescale = (uint8_t)(roundf(25000000.0f / (4096.0f * freq_hz)) - 1.0f);

    /* Read current MODE1 so we can restore non-sleep bits.
     * Always force AI on: on cold boot the read can return the power-on value
     * (0x11) before the earlier MODE1 write in PCA9685_Init has settled,
     * which would strip the AI bit from every subsequent write and silently
     * break all burst PWM writes. */
    s = read_reg(dev, PCA9685_MODE1, &old_mode);
    if (s != HAL_OK) return s;
    old_mode |= PCA9685_MODE1_AI;

    /* 1. Enter sleep (oscillator off) to allow prescale write */
    s = write_reg(dev, PCA9685_MODE1, (old_mode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP);
    if (s != HAL_OK) return s;

    /* 2. Write prescale */
    s = write_reg(dev, PCA9685_PRESCALE, prescale);
    if (s != HAL_OK) return s;

    /* 3. Wake up: clear sleep bit */
    s = write_reg(dev, PCA9685_MODE1, old_mode & ~PCA9685_MODE1_SLEEP);
    if (s != HAL_OK) return s;

    /* 4. Wait for oscillator to stabilize (datasheet says 500us min) */
    HAL_Delay(5);

    /* 5. Set RESTART bit to resume PWM (if it was previously set) */
    s = write_reg(dev, PCA9685_MODE1, (old_mode & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART);

    dev->freq_hz = freq_hz;
    return s;
}

HAL_StatusTypeDef PCA9685_SetPWM(PCA9685_t *dev, uint8_t ch,
                                   uint16_t on, uint16_t off)
{
    uint8_t buf[5];
    buf[0] = PCA9685_LED0_ON_L + (4u * ch);
    buf[1] = (uint8_t)(on  & 0xFF);
    buf[2] = (uint8_t)(on  >> 8);
    buf[3] = (uint8_t)(off & 0xFF);
    buf[4] = (uint8_t)(off >> 8);
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr, buf, 5, 10);
}

HAL_StatusTypeDef PCA9685_SetServoPulse(PCA9685_t *dev, uint8_t ch,
                                          uint16_t pulse_us)
{
    /* Clamp to configured limits */
    if (pulse_us < dev->min_us) pulse_us = dev->min_us;
    if (pulse_us > dev->max_us) pulse_us = dev->max_us;

    /* Convert µs → 12-bit tick count */
    uint16_t off = (uint16_t)((pulse_us * 4096.0f * dev->freq_hz) / 1000000.0f);

    return PCA9685_SetPWM(dev, ch, 0, off);
}

HAL_StatusTypeDef PCA9685_SetServoAngle(PCA9685_t *dev, uint8_t ch,
                                          float angle_deg)
{
    float offset = 0.0f;
    uint8_t physical_ch = ch;

    /* Apply per-joint hardware calibration trims (defined in loko_config.h) */
    if (dev->addr == PCA9685_ADDR_RIGHT) {
        /* FR, MR, BR (Legs 0, 1, 2) */
        if      (ch == 0) offset = SERVO_TRIM_FR_COXA_DEG;
        else if (ch == 1) offset = SERVO_TRIM_FR_FEMUR_DEG;
        else if (ch == 2) offset = SERVO_TRIM_FR_TIBIA_DEG;
        else if (ch == 3) offset = SERVO_TRIM_MR_COXA_DEG;
        else if (ch == 4) offset = SERVO_TRIM_MR_FEMUR_DEG;
        else if (ch == 5) offset = SERVO_TRIM_MR_TIBIA_DEG;
        else if (ch == 6) offset = SERVO_TRIM_BR_COXA_DEG;
        else if (ch == 7) offset = SERVO_TRIM_BR_FEMUR_DEG;
        else if (ch == 8) offset = SERVO_TRIM_BR_TIBIA_DEG;
    }
    else if (dev->addr == PCA9685_ADDR_LEFT) {
        /* LEFT board channels: FL(0,1,2), ML(3,4,5), BL(6,7,8) */
        if      (ch == 0) offset = SERVO_TRIM_FL_COXA_DEG;
        else if (ch == 1) offset = SERVO_TRIM_FL_FEMUR_DEG;
        else if (ch == 2) offset = SERVO_TRIM_FL_TIBIA_DEG;
        else if (ch == 3) offset = SERVO_TRIM_ML_COXA_DEG;
        else if (ch == 4) offset = SERVO_TRIM_ML_FEMUR_DEG;
        else if (ch == 5) offset = SERVO_TRIM_ML_TIBIA_DEG;
        else if (ch == 6) offset = SERVO_TRIM_BL_COXA_DEG;
        else if (ch == 7) offset = SERVO_TRIM_BL_FEMUR_DEG;
        else if (ch == 8) offset = SERVO_TRIM_BL_TIBIA_DEG;
    }

    angle_deg += offset;

	if (angle_deg < 0.0f)   angle_deg = 0.0f;
	if (angle_deg > 180.0f) angle_deg = 180.0f;

    uint16_t pulse_us = (uint16_t)(dev->min_us + (angle_deg / 180.0f) * (dev->max_us - dev->min_us));

    return PCA9685_SetServoPulse(dev, physical_ch, pulse_us);
}

void PCA9685_Sleep(PCA9685_t *dev)
{
    uint8_t mode;
    if (read_reg(dev, PCA9685_MODE1, &mode) == HAL_OK)
        write_reg(dev, PCA9685_MODE1, mode | PCA9685_MODE1_SLEEP);
}

void PCA9685_Wake(PCA9685_t *dev)
{
    uint8_t mode;
    if (read_reg(dev, PCA9685_MODE1, &mode) == HAL_OK) {
        write_reg(dev, PCA9685_MODE1, (mode | PCA9685_MODE1_AI) & ~PCA9685_MODE1_SLEEP);
        HAL_Delay(5);
    }
}
