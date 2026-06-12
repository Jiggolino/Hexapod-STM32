/**
 * @file  pca9685.c
 * @brief PCA9685 16-channel PWM driver
 *
 * Internal oscillator: 25 MHz (typical)
 * PWM resolution:      12 bit → 4096 ticks per period
 * Prescale formula:    round(25 000 000 / (4096 × freq_hz)) − 1
 * Pulse-to-tick:       tick = pulse_us × 4096 × freq_hz / 1 000 000
 */

#include "pca9685.h"
#include "loko_config.h"
#include <math.h>

/* ── Helpers ────────────────────────────────────────────────────────────── */

/*
 * After a HAL I2C error the peripheral can be left with State != READY,
 * causing every subsequent call to return HAL_BUSY instantly without attempting
 * a real transaction. Manually clearing State and ErrorCode lets the retry
 * actually attempt a new transaction.
 * Input:  hi2c — I2C handle to recover
 * Output: void
 */
static void i2c_recover(I2C_HandleTypeDef *hi2c)
{
    hi2c->State     = HAL_I2C_STATE_READY;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c->Instance->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
}

/*
 * Writes one byte to a PCA9685 register. Retries up to 3 times with I2C
 * recovery between attempts.
 * Input:  dev — PCA9685 device with hi2c and addr
 *         reg — register address
 *         val — byte to write
 * Output: HAL_OK or last HAL error code
 */
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

/*
 * Reads one byte from a PCA9685 register. Retries up to 3 times with I2C
 * recovery between attempts.
 * Input:  dev — PCA9685 device with hi2c and addr
 *         reg — register address to read
 *         val — output byte
 * Output: HAL_OK or last HAL error code
 */
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

/*
 * Sends the I2C general-call software reset command (0x06) to address 0x00,
 * resetting all PCA9685 chips on the bus simultaneously.
 * Input:  hi2c — I2C handle
 * Output: HAL_OK or HAL error code
 */
HAL_StatusTypeDef PCA9685_SoftwareReset(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd = 0x06;
    return HAL_I2C_Master_Transmit(hi2c, PCA9685_SW_RESET_ADDR, &cmd, 1, 10);
}

/*
 * Wakes the PCA9685, enables register auto-increment (required for burst writes),
 * sets MODE2 for totem-pole outputs with output-change-on-STOP, fills in default
 * pulse limits if the caller left them at zero, then calls PCA9685_SetFrequency()
 * to program the prescaler.
 * Input:  dev — device struct with hi2c, addr, freq_hz, min_us, max_us
 * Output: HAL_OK or the error code from the first failing register write
 */
HAL_StatusTypeDef PCA9685_Init(PCA9685_t *dev)
{
    HAL_StatusTypeDef s;

    if (dev->min_us == 0) dev->min_us = PCA9685_SERVO_MIN_US;
    if (dev->max_us == 0) dev->max_us = PCA9685_SERVO_MAX_US;

    s = write_reg(dev, PCA9685_MODE1, PCA9685_MODE1_AI);
    if (s != HAL_OK) return s;

    /* MODE2: output change on STOP command, totem-pole outputs */
    s = write_reg(dev, PCA9685_MODE2, 0x04);
    if (s != HAL_OK) return s;

    float freq = (dev->freq_hz > 0.0f) ? dev->freq_hz : 50.0f;
    return PCA9685_SetFrequency(dev, freq);
}

/*
 * Programs the prescaler register for the requested PWM frequency.
 * Sequence: read MODE1 → enter sleep (oscillator off) → write prescale →
 * clear sleep → wait 5 ms for oscillator → set RESTART bit.
 * The AI bit is always forced on in the restored MODE1 value to prevent
 * silent breakage if the read returns the power-on default before the earlier
 * MODE1 write has settled on cold boot.
 * Input:  dev     — device struct
 *         freq_hz — desired PWM frequency (typically 50 Hz for servos)
 * Output: HAL_OK or error code
 */
HAL_StatusTypeDef PCA9685_SetFrequency(PCA9685_t *dev, float freq_hz)
{
    HAL_StatusTypeDef s;
    uint8_t old_mode, prescale;

    /* prescale = round(25 MHz / (4096 × freq)) − 1 */
    prescale = (uint8_t)(roundf(25000000.0f / (4096.0f * freq_hz)) - 1.0f);

    s = read_reg(dev, PCA9685_MODE1, &old_mode);
    if (s != HAL_OK) return s;
    old_mode |= PCA9685_MODE1_AI;

    s = write_reg(dev, PCA9685_MODE1, (old_mode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP);
    if (s != HAL_OK) return s;

    s = write_reg(dev, PCA9685_PRESCALE, prescale);
    if (s != HAL_OK) return s;

    s = write_reg(dev, PCA9685_MODE1, old_mode & ~PCA9685_MODE1_SLEEP);
    if (s != HAL_OK) return s;

    /* Datasheet requires ≥500 µs for the oscillator to stabilise */
    HAL_Delay(5);

    s = write_reg(dev, PCA9685_MODE1, (old_mode & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART);

    dev->freq_hz = freq_hz;
    return s;
}

/*
 * Writes raw ON/OFF 12-bit tick values to one channel's four registers using
 * a 5-byte burst (register address + 4 data bytes, auto-increment).
 * Input:  dev — device struct
 *         ch  — channel 0–15
 *         on  — tick at which the output goes high (0–4095)
 *         off — tick at which the output goes low  (0–4095)
 * Output: HAL_OK or I2C error
 */
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

/*
 * Clamps the pulse to [min_us, max_us], converts µs to a 12-bit tick count
 * (tick = pulse_us × 4096 × freq_hz / 1 000 000), and calls PCA9685_SetPWM().
 * Input:  dev      — device struct
 *         ch       — channel 0–15
 *         pulse_us — pulse width in microseconds
 * Output: HAL_OK or I2C error
 */
HAL_StatusTypeDef PCA9685_SetServoPulse(PCA9685_t *dev, uint8_t ch,
                                          uint16_t pulse_us)
{
    if (pulse_us < dev->min_us) pulse_us = dev->min_us;
    if (pulse_us > dev->max_us) pulse_us = dev->max_us;

    uint16_t off = (uint16_t)((pulse_us * 4096.0f * dev->freq_hz) / 1000000.0f);

    return PCA9685_SetPWM(dev, ch, 0, off);
}

/*
 * Looks up the per-joint hardware trim for the given board address and channel
 * from loko_config.h, adds it to angle_deg, clamps to [0°, 180°], converts to
 * a pulse width via linear interpolation between min_us and max_us, and calls
 * PCA9685_SetServoPulse().
 * Input:  dev       — device struct (addr selects LEFT or RIGHT trim table)
 *         ch        — channel 0–8 (maps to a specific leg joint)
 *         angle_deg — commanded angle in degrees [0, 180] before trim
 * Output: HAL_OK or I2C error
 */
HAL_StatusTypeDef PCA9685_SetServoAngle(PCA9685_t *dev, uint8_t ch,
                                          float angle_deg)
{
    float offset = 0.0f;
    uint8_t physical_ch = ch;

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
