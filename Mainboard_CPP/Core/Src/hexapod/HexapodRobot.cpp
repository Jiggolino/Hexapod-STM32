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

HexapodRobot::HexapodRobot(I2C_HandleTypeDef *hi2c,
                            TIM_HandleTypeDef *htim,
                            UART_HandleTypeDef *huart,
                            ADC_HandleTypeDef *hadc1,
                            ADC_HandleTypeDef *hadc2,
                            ADC_HandleTypeDef *hadc3)
    : servoRight(hi2c, PCA9685_ADDR_RIGHT),
      servoLeft (hi2c, PCA9685_ADDR_LEFT),
      _hi2c(hi2c), _htim(htim), _huart(huart),
      _hadc1(hadc1), _hadc2(hadc2), _hadc3(hadc3),
      _errFlags(0), _tofDistMm(0xFFFF),
      _imuLastMs(0), _tofLastMs(0), _lokoLastMs(0),
      controller(loko.controller())
{
}

bool HexapodRobot::init()
{
    /* Bring up UART protocol first so printf works */
    UART_Protocol_Config_t cfg;
    cfg.huart             = _huart;
    cfg.pca_right         = servoRight.handle();
    cfg.pca_left          = servoLeft.handle();
    cfg.adc_current_right = _hadc1;
    cfg.adc_current_left  = _hadc2;
    cfg.loko              = &loko.raw();
    uart.init(cfg);

    battery.init(_hadc3);
    printf("/BATTERY/VDDA/%lu mV\r\n", (unsigned long)battery.vddaMv());
    printf("\r\n=== Hexapod Mainboard (C++) ===\r\n");

    /* Retry PCA9685 init up to 5 times */
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

    /* Pre-load servo registers at center before arming */
    for (uint8_t ch = 0; ch < 9; ch++) {
        servoRight.setAngle(ch, 90.0f);
        servoLeft.setAngle(ch, 90.0f);
    }

    if (imu.init(_hi2c)) printf("LSM6DSO16IS IMU: OK\r\n");
    else { printf("LSM6DSO16IS IMU: FAIL\r\n"); _errFlags |= SYS_ERR_IMU; }

    if (tof.init(_hi2c)) printf("VL53L1X ToF:     OK\r\n");
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

void HexapodRobot::update(float /*dt_hint*/)
{
    uart.update();
    leds.tick();

    /* 20 Hz ToF tick */
    uint32_t now = HAL_GetTick();
    if (now - _tofLastMs >= 50u) {
        _tofLastMs = now;
        if (tof.read() == 1)
            _tofDistMm = tof.distanceMm();
    }

    /* 100 Hz locomotion tick */
    now = HAL_GetTick();
    if (now - _lokoLastMs >= 10u) {
        float dt = (float)(now - _lokoLastMs) * 0.001f;
        _lokoLastMs = now;

        float input_values[LOKO_INPUT_COUNT] = {0};
        const UART_ControllerState_t &ctrl = uart.controller();

        /* LEFT STICK: Position/Movement Control
         * X-axis → lateral strafe (±50mm max)
         * Y-axis → forward/backward (with wall collision at 250mm)
         * Store raw values; negations applied at LokoInput mapping
         */
        input_values[AXIS_LX] = ctrl.left_stick_x;      /* Left X: raw strafe value */

        /* Forward/Backward with wall collision prevention */
        if ((float)tof_get_distance_mm() <= WALL_DETECTION_DISTANCE_FAST && ctrl.left_stick_y < -0.5f)
            input_values[AXIS_LY] = 0.0f;
        else
            if ((float)tof_get_distance_mm() <= WALL_DETECTION_DISTANCE_SLOW && ctrl.left_stick_y < 0.0f)
                input_values[AXIS_LY] = 0.0f;
            else
            	input_values[AXIS_LY] = ctrl.left_stick_y;   /* Left Y: raw forward/backward */

        /* RIGHT STICK: Rotation and Look Control
         * X-axis → body rotation (yaw, ±30 degrees max)
         * Y-axis → pitch/look angle (±30 degrees max)
         * RX: negate for V1 compatibility; RY: raw value
         */
        input_values[AXIS_RX] = -ctrl.right_stick_x;    /* Right X (negated): yaw rotation */
        input_values[AXIS_RY] = ctrl.right_stick_y;     /* Right Y: raw pitch/look value */

        /* Face buttons: X(1) A(2) B(4) Y(8) */
        input_values[BTN_SQUARE]   = (ctrl.face_buttons & (1 << 0)) ? 1.0f : 0.0f;
        input_values[BTN_CROSS]    = (ctrl.face_buttons & (1 << 1)) ? 1.0f : 0.0f;
        input_values[BTN_CIRCLE]   = (ctrl.face_buttons & (1 << 2)) ? 1.0f : 0.0f;
        input_values[BTN_TRIANGLE] = (ctrl.face_buttons & (1 << 3)) ? 1.0f : 0.0f;

        /* Stick clicks: L3(1), R3(2) */
        input_values[BTN_L3] = (ctrl.stick_buttons & (1 << 0)) ? 1.0f : 0.0f;
        input_values[BTN_R3] = (ctrl.stick_buttons & (1 << 1)) ? 1.0f : 0.0f;

        /* Trigger/shoulder: LT(1) View(2) LB(4) RT(8) RB(16) Start(32) */
        input_values[BTN_L2]      = (ctrl.trigger_buttons & (1 << 0)) ? 1.0f : 0.0f;
        input_values[BTN_SHARE]   = (ctrl.trigger_buttons & (1 << 1)) ? 1.0f : 0.0f;
        input_values[BTN_L1]      = (ctrl.trigger_buttons & (1 << 2)) ? 1.0f : 0.0f;
        input_values[BTN_R2]      = (ctrl.trigger_buttons & (1 << 3)) ? 1.0f : 0.0f;
        input_values[BTN_R1]      = (ctrl.trigger_buttons & (1 << 4)) ? 1.0f : 0.0f;
        input_values[BTN_OPTIONS] = (ctrl.trigger_buttons & (1 << 5)) ? 1.0f : 0.0f;

        /* D-pad */
        input_values[BTN_DPAD_LEFT]  = (ctrl.dpad_x < 0) ? 1.0f : 0.0f;
        input_values[BTN_DPAD_RIGHT] = (ctrl.dpad_x > 0) ? 1.0f : 0.0f;
        input_values[BTN_DPAD_UP]    = (ctrl.dpad_y < 0) ? 1.0f : 0.0f;
        input_values[BTN_DPAD_DOWN]  = (ctrl.dpad_y > 0) ? 1.0f : 0.0f;

        /* Read IMU every locomotion tick so the stabilizer has fresh data */
        imu.read();
        loko.raw().imu_data = imu.data();

        controller.update(input_values);
        loko.updateTransitions();

        LokoInput loko_in;
        loko_default_input(&loko_in);

        /* Map locomotion inputs with negations applied here (matching V1)
         * LokoInput expects normalized [-1, 1] values:
         *   vx: forward (+) / backward (−), max ≈ forward velocity
         *   vy: left (+) / right (−), max ±50mm lateral movement
         *   wz: CCW (+) / CW (−), max ±30 degrees rotation
         * Note: Right stick Y (pitch) is handled by LOOK_AROUND state handler
         */
        loko_in.vx = -controller.axis(AXIS_LY);    /* Forward/backward from left stick Y (negated) */
        loko_in.vy = -controller.axis(AXIS_LX);    /* Left/right strafe from left stick X (negated, ±50mm) */
        loko_in.wz =  controller.axis(AXIS_RX);    /* Yaw rotation from right stick X (±30°) */

        loko.update(loko_in, dt);

        uint32_t lerr = loko.errors();
        if (lerr) {
            _errFlags |= SYS_ERR_LOKO;
            loko.clearErrors();
            printf("/ERR/LOKO/0x%08lx\r\n", (unsigned long)lerr);
        }
    }

    /* Error indication: blink red LED */
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
