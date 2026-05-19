/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Hexapod mainboard — IMU + ToF + dual PCA9685 servo control
  *
  * Host ↔ STM32 communication lives entirely in uart_protocol.{c,h}.
  * See that header for the wire format.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include "pca9685.h"
#include "imu.h"
#include "tof.h"
#include "uart_protocol.h"
#include "ws2812b.h"
#include "battery.h"
#include "adc.h"
#include "lokomotion.h"
#include "loko_input.h"
#include "loko_transitions.h"
#include "uart_dma_tx.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

I2C_HandleTypeDef hi2c1;

/* Servo Drivers */
static PCA9685_t pca_right;
static PCA9685_t pca_left;

/* IMU */
// static IMU_Data_t   imu_data;

/* Locomotion */
static LokoState loko;

/* System-level error flags — set during init and runtime */
#define SYS_ERR_PCA_RIGHT  (1u << 0)
#define SYS_ERR_PCA_LEFT   (1u << 1)
#define SYS_ERR_IMU        (1u << 2)
#define SYS_ERR_TOF        (1u << 3)
#define SYS_ERR_LOKO       (1u << 4)
static uint32_t sys_err_flags = 0;
static uint16_t s_tof_distance_mm = 0xFFFFu;

uint16_t tof_get_distance_mm(void) { return s_tof_distance_mm; }

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void center_all_servos(void);
static void I2C_BusClear(void);
static void I2C1_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Release any slave that held the I2C bus when the MCU reset.
 *
 * Root cause of the "8-resets" problem: a slave mid-transaction holds SDA low.
 * The old code broke out of the clock loop when SDA appeared HIGH, which happens
 * naturally between byte boundaries — so 0 clocks were sent and the slave stayed
 * stuck.  One 9-clock pass only recovers one stuck byte; a multi-byte PCA9685
 * write can leave 8+ bytes outstanding, matching the 8-reset symptom exactly.
 *
 * Fix: always send 9 clocks unconditionally, then issue a STOP.  Repeat three
 * rounds so even a slave deep inside a long transaction reaches a byte boundary
 * and recognises the STOP.  SDA is kept as INPUT during clocking so the slave
 * can drive its data bits; we switch to output only for the STOP pulse. */
static void I2C_BusClear(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* SCL = PB8 open-drain output */
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SDA = PB9 input (let slave drive) */
    GPIO_InitStruct.Pin  = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* 18 rounds of 9 SCL pulses + STOP.  Each round clocks one byte out of
     * the slave, bringing it to a byte boundary where the STOP is recognised.
     * 18 rounds covers slaves stuck up to 18 bytes deep in a transaction
     * (PCA9685 burst writes, VL53L1X 135-byte config writes, IMU multi-byte reads).
     * Do NOT break early on SDA high — the slave can have SDA=1 mid-byte. */
    for (int round = 0; round < 18; round++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);

        for (int clk = 0; clk < 9; clk++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_Delay(1);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_Delay(1);
        }

        /* Switch SDA to open-drain output to drive the STOP condition */
        GPIO_InitStruct.Pin   = GPIO_PIN_9;
        GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* STOP: SDA low → high while SCL is high */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_Delay(1);

        /* Switch SDA back to input for next round */
        GPIO_InitStruct.Pin  = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Bus free — no point sending more rounds */
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) break;
    }

    /* Final STOP with SDA as output (leaves pins in a clean state for I2C1_Init) */
    GPIO_InitStruct.Pin   = GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);
}

static void I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x00B03FDB;   /* 400 kHz @ 64 MHz PCLK1 */
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

/* Move all nine channels of both PCA9685 boards to their mechanical midpoint. */
static void center_all_servos(void)
{
    for (uint8_t ch = 0; ch < 9; ch++) {
        PCA9685_SetServoAngle(&pca_right, ch, 90.0f);
        PCA9685_SetServoAngle(&pca_left,  ch, 90.0f);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Drive servo OE pins HIGH before anything else runs.
   * After MCU reset all GPIOs are inputs (floating). The PCA9685 OE pin
   * has an internal pull-down, so it immediately enables the outputs and
   * drives whatever stale angles are in its registers (PCA9685 is powered
   * independently and does NOT reset when the MCU resets).
   * In debug mode the debugger halts here for seconds — without this the
   * servos are live the entire time.
   *   Right_Enable = PB0   Left_Enable = PG3
   */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  GPIOB->BSRR = GPIO_PIN_0;   /* PB0 HIGH  (Right_Enable, OE disabled) */
  GPIOG->BSRR = GPIO_PIN_3;   /* PG3 HIGH  (Left_Enable,  OE disabled) */
  /* Set pins as push-pull output so they actively drive HIGH, not just
   * pre-load ODR. MODER bits [1:0] per pin: 01 = output. */
  GPIOB->MODER = (GPIOB->MODER & ~(3UL << (0 * 2))) | (1UL << (0 * 2));
  GPIOG->MODER = (GPIOG->MODER & ~(3UL << (3 * 2))) | (1UL << (3 * 2));
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  uart_dma_tx_init(&huart1, &hdma_usart1_tx);

  /* On cold boot, PCA9685 is powered independently and may have stuck the I2C bus.
   * DeInit resets state to RESET so the next init re-runs MspInit (GPIO/clock).
   * Uses BSP_I2C1_Init (custom_bus.c) — survives CubeMX regeneration when I2C1
   * is disabled from Connectivity and removed from main.c. */
  /* Release XSHUT so the VL53L1X is not holding the I2C bus during bus clear.
   * MX_GPIO_Init() drives it LOW by default; bring it HIGH here before we
   * touch SCL/SDA, then TOF_Init() will do its own reset sequence later. */
  HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(5);   /* VL53L1X boot time after XSHUT release */

  /* Give all I2C devices time to finish powering up before we touch the bus.
   * 200 ms covers PCA9685 cold-boot oscillator start and VL53L1X firmware load. */
  HAL_Delay(200);

  HAL_I2C_DeInit(&hi2c1);
  I2C_BusClear();
  I2C1_Init();

  /* Global software reset for all PCA9685 boards on the bus */
  PCA9685_SoftwareReset(&hi2c1);
  HAL_Delay(50);  /* datasheet min 500us; give extra margin after bus clear on cold boot */

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);
  ADC_Init(&hadc1, &hadc2, &hadc3);

    /* Fill the PCA9685 structs before handing pointers to the protocol
     * module; the module only dereferences them when commands arrive. */
    pca_right.hi2c    = &hi2c1;
    pca_right.addr    = PCA9685_ADDR_RIGHT;
    pca_right.freq_hz = 50.0f;
    pca_right.min_us  = PCA9685_SERVO_MIN_US;
    pca_right.max_us  = PCA9685_SERVO_MAX_US;

    pca_left.hi2c    = &hi2c1;
    pca_left.addr    = PCA9685_ADDR_LEFT;
    pca_left.freq_hz = 50.0f;
    pca_left.min_us  = PCA9685_SERVO_MIN_US;
    pca_left.max_us  = PCA9685_SERVO_MAX_US;

    /* Bring up the protocol first so the banner printfs below go out. */
    const UART_Protocol_Config_t cfg = {
        .huart             = &huart1,
        .pca_right         = &pca_right,
        .pca_left          = &pca_left,
        .adc_current_right = &hadc1,
        .adc_current_left  = &hadc2,
        .loko              = &loko,
    };
    UART_Protocol_Init(&cfg);
    Battery_Init(&hadc3);
    printf("/BATTERY/VDDA/%lu mV\r\n", (unsigned long)Battery_GetVddaMv());

    printf("\r\n=== Hexapod Mainboard ===\r\n");

    /* Retry PCA9685 init up to 5 times.  On a cold boot the I2C HAL can be
     * left in a BUSY state after a failed transaction; force-resetting the
     * peripheral clock clears that flag and lets the next attempt succeed.
     * Retry only if either board fails (not just both), so one stuck device
     * doesn't mask the other. */
    HAL_StatusTypeDef pca_r = HAL_ERROR, pca_l = HAL_ERROR;
    for (int attempt = 0; attempt < 5; attempt++) {
        pca_r = PCA9685_Init(&pca_right);
        pca_l = PCA9685_Init(&pca_left);
        if (pca_r == HAL_OK && pca_l == HAL_OK) break;

        /* Reset the I2C peripheral and re-run the bus clear before retrying.
         * DeInit sets State=RESET so MX_I2C1_Init calls MspInit and
         * restores PB8/PB9 to AF mode after the bus-clear GPIO toggling. */
        HAL_I2C_DeInit(&hi2c1);
        I2C_BusClear();
        I2C1_Init();
        PCA9685_SoftwareReset(&hi2c1);
        HAL_Delay(50);  /* match the increased delay above */
    }

    if (pca_r == HAL_OK) printf("PCA9685 Right: OK\r\n");
    else { printf("PCA9685 Right: FAIL\r\n"); sys_err_flags |= SYS_ERR_PCA_RIGHT; }
    if (pca_l == HAL_OK) printf("PCA9685 Left:  OK\r\n");
    else { printf("PCA9685 Left:  FAIL\r\n"); sys_err_flags |= SYS_ERR_PCA_LEFT;  }

    /* OE pins stay HIGH (outputs disabled) until an explicit arm command.
     * center_all_servos() still pre-loads the PWM registers so the first
     * move after arming goes to a known position without a jerk. */
    center_all_servos();

    if (IMU_Init(&hi2c1) == 0) printf("LSM6DSO16IS IMU (filtered): OK\r\n");
    else { printf("LSM6DSO16IS IMU (filtered): FAIL\r\n"); sys_err_flags |= SYS_ERR_IMU; }

    if (TOF_Init(&hi2c1) == 0) printf("VL53L1X ToF:   OK\r\n");
    else { printf("VL53L1X ToF:   FAIL\r\n"); sys_err_flags |= SYS_ERR_TOF; }

    /* Locomotion controller — starts in UN_ARMED; servo output is printf only.
     * loko_enable(1) is always set because we are not driving hardware yet —
     * switch back to PCA9685 writes in loko_servo.c when hardware is ready. */
    loko_init(&loko, &pca_right, &pca_left);
    loko_enable(&loko, 1u);
    printf("Locomotion:    INIT (UN_ARMED, printf mode)\r\n");
    printf("  Press L1 to arm, R1 to disarm at any time.\r\n");

    if (sys_err_flags) {
        printf("/ERR/INIT/0x%08lx\r\n", (unsigned long)sys_err_flags);
    }
    printf("Ready.\r\n");

    ws2812_init(&htim1);
    ws2812_mode_loading();   /* UN_ARMED startup — state machine takes over from here */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

      UART_Update();

      ws2812_tick();
      /* ── 50 Hz IMU read tick (20 ms) ────────────────────────────────────── */
      static uint32_t imu_last_ms = 0;
      {
          uint32_t imu_now = HAL_GetTick();
          if (imu_now - imu_last_ms >= 20u) {
              imu_last_ms = imu_now;
              IMU_Read(&loko.imu_data);
          }
      }

      /* ── 20 Hz TOF read tick (50 ms) ─────────────────────────────────────── */
      static uint32_t tof_last_ms = 0;
      {
          uint32_t tof_now = HAL_GetTick();
          if (tof_now - tof_last_ms >= 50u) {
              tof_last_ms = tof_now;
              TOF_Data_t tof = {0};
              if (TOF_Read(&tof) == 1)
                  s_tof_distance_mm = tof.distance_mm;
          }
      }

      /* ── 100 Hz locomotion control tick (10 ms) ──────────────────────────── */
      static uint32_t loko_last_ms = 0;
      {
          uint32_t loko_now = HAL_GetTick();
          if (loko_now - loko_last_ms >= 10u) {
              float dt = (float)(loko_now - loko_last_ms) * 0.001f;
              loko_last_ms = loko_now;

              /* Build locomotion input from PS5 controller */
              float input_values[LOKO_INPUT_COUNT] = {0};
              const UART_ControllerState_t *ctrl = UART_GetController();

              input_values[AXIS_RX] = -ctrl->right_stick_x;
              input_values[AXIS_RY] = ctrl->right_stick_y;
              input_values[AXIS_LX] = ctrl->left_stick_x;

              //prevention so it cant walk in to a wall
              if((250.0f >= (float)tof_get_distance_mm()) && (ctrl->left_stick_y < 0.0f)){
            	  input_values[AXIS_LY] = 0.0f;
              } else {
            	  input_values[AXIS_LY] = ctrl->left_stick_y;
              }


              /* Face buttons bitmask: X(1), A(2), B(4), Y(8) */
              input_values[BTN_SQUARE]   = (ctrl->face_buttons & (1 << 0)) ? 1.0f : 0.0f;
              input_values[BTN_CROSS]    = (ctrl->face_buttons & (1 << 1)) ? 1.0f : 0.0f;
              input_values[BTN_CIRCLE]   = (ctrl->face_buttons & (1 << 2)) ? 1.0f : 0.0f;
              input_values[BTN_TRIANGLE] = (ctrl->face_buttons & (1 << 3)) ? 1.0f : 0.0f;

              /* Stick click buttons: L3(1), R3(2) */
              input_values[BTN_L3]       = (ctrl->stick_buttons & (1 << 0)) ? 1.0f : 0.0f;
              input_values[BTN_R3]       = (ctrl->stick_buttons & (1 << 1)) ? 1.0f : 0.0f;

              /* Trigger/shoulder buttons bitmask: LT(1), View(2), LB(4), RT(8), RB(16), Start(32) */
              input_values[BTN_L2]      = (ctrl->trigger_buttons & (1 << 0)) ? 1.0f : 0.0f;
              input_values[BTN_SHARE]   = (ctrl->trigger_buttons & (1 << 1)) ? 1.0f : 0.0f;
              input_values[BTN_L1]      = (ctrl->trigger_buttons & (1 << 2)) ? 1.0f : 0.0f;
              input_values[BTN_R2]      = (ctrl->trigger_buttons & (1 << 3)) ? 1.0f : 0.0f;
              input_values[BTN_R1]      = (ctrl->trigger_buttons & (1 << 4)) ? 1.0f : 0.0f;
              input_values[BTN_OPTIONS] = (ctrl->trigger_buttons & (1 << 5)) ? 1.0f : 0.0f;

              /* D-pad: dpad_x < 0 = left, > 0 = right; dpad_y < 0 = up, > 0 = down */
              input_values[BTN_DPAD_LEFT]  = (ctrl->dpad_x < 0) ? 1.0f : 0.0f;
              input_values[BTN_DPAD_RIGHT] = (ctrl->dpad_x > 0) ? 1.0f : 0.0f;
              input_values[BTN_DPAD_UP]    = (ctrl->dpad_y < 0) ? 1.0f : 0.0f;
              input_values[BTN_DPAD_DOWN]  = (ctrl->dpad_y > 0) ? 1.0f : 0.0f;

              loko_input_update(&loko.pad, input_values);
              loko_update_transitions(&loko);

              LokoInput loko_in;
              loko_default_input(&loko_in);
              /* Map stick inputs to translation and rotation. */
              loko_in.vx = -loko_axis(&loko.pad, AXIS_LY);
              loko_in.vy = -loko_axis(&loko.pad, AXIS_LX);
              loko_in.wz =  loko_axis(&loko.pad, AXIS_RX);

              loko_update(&loko, &loko_in, dt);

              /* Check and report locomotion errors */
              uint32_t lerr = loko_get_errors(&loko);
              if (lerr) {
                  sys_err_flags |= SYS_ERR_LOKO;
                  loko_clear_errors(&loko);
                  printf("/ERR/LOKO/0x%08lx\r\n", (unsigned long)lerr);
              }
          }
      }

      /* ── Error state indication ───────────────────────────────────────────
       * Any error in sys_err_flags → red LED blinks at 2 Hz and ws2812
       * switches to pulsing red.  Clears automatically once flags are cleared
       * (no automatic clearing — requires a reset or explicit flag clear). */
      if (sys_err_flags) {
          static uint32_t err_blink_ms    = 0;
          static uint8_t  err_blink_state = 0;
          if (HAL_GetTick() - err_blink_ms >= 250u) {
              err_blink_ms    = HAL_GetTick();
              err_blink_state ^= 1u;
              HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,
                                err_blink_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
              if (err_blink_state); //ws2812_mode_pulse_red();
          }
      }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 32;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 32;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_14B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 32;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 299;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_GREEN_Pin|LED_RED_Pin|Right_A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Left_A2_Pin|Left_A1_Pin|Left_A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Right_Enable_GPIO_Port, Right_Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Right_A1_Pin|Right_A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, XSHUT_Pin|LED_CAM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Left_Enable_GPIO_Port, Left_Enable_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin Right_A2_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|Right_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_A2_Pin Left_A1_Pin Left_A0_Pin */
  GPIO_InitStruct.Pin = Left_A2_Pin|Left_A1_Pin|Left_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_Enable_Pin Right_A1_Pin Right_A0_Pin */
  GPIO_InitStruct.Pin = Right_Enable_Pin|Right_A1_Pin|Right_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XSHUT_Pin LED_CAM_Pin */
  GPIO_InitStruct.Pin = XSHUT_Pin|LED_CAM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_1_TOF_Pin */
  GPIO_InitStruct.Pin = GPIO_1_TOF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO_1_TOF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Left_Enable_Pin */
  GPIO_InitStruct.Pin = Left_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Left_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
