#include "hexapod/HexapodRobot.hpp"
#include "main.h"
#include "i2c.h"
#include "pca9685.h"
#include "uart_dma_tx.h"
#include "loko_config.h"
#include <stdio.h>

#define SYS_ERR_PCA_RIGHT  (1u << 0)
#define SYS_ERR_PCA_LEFT   (1u << 1)
#define SYS_ERR_IMU        (1u << 2)
#define SYS_ERR_TOF        (1u << 3)
#define SYS_ERR_LOKO       (1u << 4)

/*
 * Stores peripheral handles and constructs both ServoDriver instances with
 * their board addresses. Does not touch any hardware.
 * Input:  hi2c  — shared I2C1 handle for all peripherals
 *         htim  — TIM1 handle for WS2812B DMA output
 *         huart — USART1 handle for UART protocol
 *         hadc3 — ADC3 handle for battery voltage measurement
 */
HexapodRobot::HexapodRobot(I2C_HandleTypeDef *hi2c,
                            TIM_HandleTypeDef *htim,
                            UART_HandleTypeDef *huart,
                            ADC_HandleTypeDef *hadc3)
    : servoRight(hi2c, PCA9685_ADDR_RIGHT),
      servoLeft (hi2c, PCA9685_ADDR_LEFT),
      _hi2c(hi2c), _htim(htim), _huart(huart),
      _hadc3(hadc3),
      _errFlags(0), _tofDistMm(0xFFFF),
      _imuLastMs(0), _tofLastMs(0), _lokoLastMs(0),
      controller(loko.controller())
{
}

/*
 * Sequentially initialises every subsystem with retry loops on I2C failures.
 * Order: UART (so printf works) → battery → PCA9685 ×2 → servo center → IMU → ToF → locomotion → LEDs.
 * Each peripheral retries up to 3–5 times with I2C bus-clear between attempts.
 * Failures set bits in _errFlags; the robot is considered healthy only when _errFlags == 0.
 * Output: true if all hardware initialised without errors
 */
bool HexapodRobot::init()
{
    UART_Protocol_Config_t cfg;
    cfg.huart             = _huart;
    cfg.pca_right         = servoRight.handle();
    cfg.pca_left          = servoLeft.handle();
    cfg.loko              = &loko.raw();
    uart.init(cfg);

    battery.init(_hadc3);
    printf("/BATTERY/VDDA/%lu mV\r\n", (unsigned long)battery.vddaMv());
    printf("\r\n=== Hexapod Mainboard (C++) ===\r\n");

    bool pca_r = false, pca_l = false;
    for (int attempt = 0; attempt < 5; attempt++) {
        pca_r = servoRight.init();
        pca_l = servoLeft.init();
        if (pca_r && pca_l) break;
        HAL_I2C_DeInit(_hi2c);
        I2C_BusClear();
        I2C1_Init();
        PCA9685_SoftwareReset(_hi2c);
        HAL_Delay(50);
    }

    if (pca_r) printf("PCA9685 Right: OK\r\n");
    else { printf("PCA9685 Right: FAIL\r\n"); _errFlags |= SYS_ERR_PCA_RIGHT; }
    if (pca_l) printf("PCA9685 Left:  OK\r\n");
    else { printf("PCA9685 Left:  FAIL\r\n"); _errFlags |= SYS_ERR_PCA_LEFT; }

    for (uint8_t ch = 0; ch < 9; ch++) {
        servoRight.setAngle(ch, 90.0f);
        servoLeft.setAngle(ch, 90.0f);
    }

    /* The LSM6DSO can NACK on cold boot if its internal regulator hasn't fully
     * settled, or if a prior NACK left the I2C peripheral in an error state. */
    bool imu_ok = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        imu_ok = imu.init(_hi2c);
        if (imu_ok) break;
        printf("IMU: retry %d\r\n", attempt + 1);
        HAL_I2C_DeInit(_hi2c);
        I2C_BusClear();
        I2C1_Init();
        HAL_Delay(50);
    }
    if (imu_ok) printf("LSM6DSO16IS IMU: OK\r\n");
    else { printf("LSM6DSO16IS IMU: FAIL\r\n"); _errFlags |= SYS_ERR_IMU; }

    /* VL53L1X SensorInit writes 90 registers and polls boot status — any single
     * NACK aborts the chain, so a clean I2C state per attempt is critical. */
    bool tof_ok = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        tof_ok = tof.init(_hi2c);
        if (tof_ok) break;
        printf("TOF: retry %d\r\n", attempt + 1);
        HAL_I2C_DeInit(_hi2c);
        I2C_BusClear();
        I2C1_Init();
        HAL_Delay(50);
    }
    if (tof_ok) printf("VL53L1X ToF:     OK\r\n");
    else { printf("VL53L1X ToF:     FAIL\r\n"); _errFlags |= SYS_ERR_TOF; }

    loko.init(servoRight, servoLeft);
    printf("Locomotion:      INIT (UN_ARMED)\r\n");

    if (_errFlags)
        printf("/ERR/INIT/0x%08lx\r\n", (unsigned long)_errFlags);
    printf("Ready.\r\n");

    leds.init(_htim);
    leds.loading();

    return _errFlags == 0;
}

/*
 * Main loop tick. Runs three independent rate-limited tasks:
 *   20 Hz  — ToF distance poll
 *   100 Hz — IMU read, controller input mapping, locomotion FSM update
 * Additionally drains the UART RX ring buffer and ticks the LED animation
 * every call. Blinks the red GPIO LED if any init error is latched.
 *
 * Button/axis mapping (bitmasks from /CONTROLL packet):
 *   face_buttons:    X=bit0  A=bit1  B=bit2  Y=bit3
 *   stick_buttons:   L3=bit0 R3=bit1
 *   trigger_buttons: LT=bit0 View=bit1 LB=bit2 RT=bit3 RB=bit4 Start=bit5
 *   dpad_x/y:        negative = left/up, positive = right/down
 *
 * Input:  dt_hint — unused; actual dt is computed from HAL_GetTick()
 * Output: void
 */
void HexapodRobot::update(float /*dt_hint*/)
{
    uart.update();
    leds.tick();

    uint32_t now = HAL_GetTick();
    if (now - _tofLastMs >= 50u) {
        _tofLastMs = now;
        if (tof.read() == 1)
            _tofDistMm = tof.distanceMm();
    }

    now = HAL_GetTick();
    if (now - _lokoLastMs >= 10u) {
        float dt = (float)(now - _lokoLastMs) * 0.001f;
        _lokoLastMs = now;

        float input_values[LOKO_INPUT_COUNT] = {0};
        const UART_ControllerState_t &ctrl = uart.controller();

        input_values[AXIS_LX] = ctrl.left_stick_x;

        /* Block forward movement when an obstacle is within wall-detection range */
        if ((float)tof_get_distance_mm() <= WALL_DETECTION_DISTANCE_FAST && ctrl.left_stick_y < -0.5f)
            input_values[AXIS_LY] = 0.0f;
        else if ((float)tof_get_distance_mm() <= WALL_DETECTION_DISTANCE_SLOW && ctrl.left_stick_y < 0.0f)
            input_values[AXIS_LY] = 0.0f;
        else
            input_values[AXIS_LY] = ctrl.left_stick_y;

        /* Right stick X is negated to match V1 hardware stick orientation */
        input_values[AXIS_RX] = -ctrl.right_stick_x;
        input_values[AXIS_RY] =  ctrl.right_stick_y;

        input_values[BTN_SQUARE]   = (ctrl.face_buttons & (1 << 0)) ? 1.0f : 0.0f;
        input_values[BTN_CROSS]    = (ctrl.face_buttons & (1 << 1)) ? 1.0f : 0.0f;
        input_values[BTN_CIRCLE]   = (ctrl.face_buttons & (1 << 2)) ? 1.0f : 0.0f;
        input_values[BTN_TRIANGLE] = (ctrl.face_buttons & (1 << 3)) ? 1.0f : 0.0f;

        input_values[BTN_L3] = (ctrl.stick_buttons & (1 << 0)) ? 1.0f : 0.0f;
        input_values[BTN_R3] = (ctrl.stick_buttons & (1 << 1)) ? 1.0f : 0.0f;

        input_values[BTN_L2]      = (ctrl.trigger_buttons & (1 << 0)) ? 1.0f : 0.0f;
        input_values[BTN_SHARE]   = (ctrl.trigger_buttons & (1 << 1)) ? 1.0f : 0.0f;
        input_values[BTN_L1]      = (ctrl.trigger_buttons & (1 << 2)) ? 1.0f : 0.0f;
        input_values[BTN_R2]      = (ctrl.trigger_buttons & (1 << 3)) ? 1.0f : 0.0f;
        input_values[BTN_R1]      = (ctrl.trigger_buttons & (1 << 4)) ? 1.0f : 0.0f;
        input_values[BTN_OPTIONS] = (ctrl.trigger_buttons & (1 << 5)) ? 1.0f : 0.0f;

        input_values[BTN_DPAD_LEFT]  = (ctrl.dpad_x < 0) ? 1.0f : 0.0f;
        input_values[BTN_DPAD_RIGHT] = (ctrl.dpad_x > 0) ? 1.0f : 0.0f;
        input_values[BTN_DPAD_UP]    = (ctrl.dpad_y < 0) ? 1.0f : 0.0f;
        input_values[BTN_DPAD_DOWN]  = (ctrl.dpad_y > 0) ? 1.0f : 0.0f;

        imu.read();
        loko.raw().imu_data = imu.data();

        controller.update(input_values);
        loko.updateTransitions();

        LokoInput loko_in;
        loko_default_input(&loko_in);

        /* Negations here match V1 stick convention:
         *   vx: left-stick Y negated → forward (+) / backward (−)
         *   vy: left-stick X negated → right (+) / left (−), ±50 mm lateral
         *   wz: right-stick X already negated above → CCW (+) / CW (−), ±30° */
        loko_in.vx = -controller.axis(AXIS_LY);
        loko_in.vy = -controller.axis(AXIS_LX);
        loko_in.wz =  controller.axis(AXIS_RX);

        loko.update(loko_in, dt);

        /* IK errors are printed for visibility but NOT latched into _errFlags.
         * A transient unreachable target during a state transition would
         * permanently light the red error LED even though the robot is fine. */
        uint32_t lerr = loko.errors();
        if (lerr) {
            loko.clearErrors();
            printf("/ERR/LOKO/0x%08lx\r\n", (unsigned long)lerr);
        }
    }

    if (_errFlags) {
        static uint32_t err_blink_ms    = 0;
        static uint8_t  err_blink_state = 0;
        if (HAL_GetTick() - err_blink_ms >= 250u) {
            err_blink_ms    = HAL_GetTick();
            err_blink_state ^= 1u;
            HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
                              err_blink_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }
}
