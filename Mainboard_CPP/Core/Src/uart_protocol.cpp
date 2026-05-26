/**
 * @file  uart_protocol.c
 * @brief See uart_protocol.h for the wire format.
 */

#include "uart_protocol.h"
#include "battery.h"
#include "main.h"
#include "current.h"
#include "uart_dma_tx.h"
#include "loko_states.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* ─── tunables ───────────────────────────────────────────────────── */
#define UART_RX_BUF_LEN     256u     /* longest line is /SERVO/18x"-180.00," ≈ 160 bytes */
#define STREAM_PERIOD_MS    20u      /* 50 Hz */


/* ─── module state ───────────────────────────────────────────────── */
static UART_Protocol_Config_t s_cfg;

#define RX_RING_SIZE 1024u
static struct {
    uint8_t  buf[RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} s_rx_ring;

static char     s_rx_buf[UART_RX_BUF_LEN];
static uint16_t s_rx_idx;
static uint8_t  s_rx_overflow;   /* drop current line if it exceeded the buffer */

static uint8_t  s_stream_on;
static uint32_t s_stream_next_ms;

static UART_RobotState_t      s_state;
static UART_ControllerState_t s_controller;
static UART_LED_t             s_led_right[3];
static UART_LED_t             s_led_left[3];
static UART_LED_t             s_led_ctrl;

/* ─── printf retarget ────────────────────────────────────────────── */
/* Full Newlib calls _write; Newlib Nano walks character-by-character
 * through __io_putchar. Both are provided so either libc works. */

extern "C" int _write(int file, char *ptr, int len) {
    (void)file;
    uart_dma_tx_send((uint8_t*)ptr, len);
    return len;
}

extern "C" int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    uart_dma_tx_send(&c, 1);
    return ch;
}

/* ─── tokenisers (destructive: they modify 'str' in place) ───────── */
static int parse_floats(char *str, float *out, int n)
{
    if (!str) return 0;
    int   count = 0;
    char *save  = NULL;
    char *tok   = strtok_r(str, ",", &save);
    while (tok) {
        if (count >= n) return 0;
        char *end;
        float v = strtof(tok, &end);
        if (end == tok) return 0;                 /* no digits */
        while (*end == ' ' || *end == '\t') end++;
        if (*end != '\0') return 0;               /* trailing junk */
        out[count++] = v;
        tok = strtok_r(NULL, ",", &save);
    }
    return (count == n);
}

static int parse_ints(char *str, int *out, int n)
{
    if (!str) return 0;
    int   count = 0;
    char *save  = NULL;
    char *tok   = strtok_r(str, ",", &save);
    while (tok) {
        if (count >= n) return 0;
        char *end;
        long v = strtol(tok, &end, 10);
        if (end == tok) return 0;
        while (*end == ' ' || *end == '\t') end++;
        if (*end != '\0') return 0;
        out[count++] = (int)v;
        tok = strtok_r(NULL, ",", &save);
    }
    return (count == n);
}

/* ─── command handlers ───────────────────────────────────────────── */
static void handle_servo(char *arg)
{
    if (!arg) return;
    if (strcmp(arg, "EN") == 0) {
        HAL_GPIO_WritePin(Right_Enable_GPIO_Port, Right_Enable_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Left_Enable_GPIO_Port,  Left_Enable_Pin,  GPIO_PIN_RESET);
        return;
    }
    if (strcmp(arg, "DIS") == 0) {
        HAL_GPIO_WritePin(Right_Enable_GPIO_Port, Right_Enable_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Left_Enable_GPIO_Port,  Left_Enable_Pin,  GPIO_PIN_SET);
        return;
    }

    float a[18];
    if (!parse_floats(arg, a, 18)) return;
    if (!s_cfg.pca_right || !s_cfg.pca_left) return;
    for (uint8_t i = 0; i < 9; i++) {
        PCA9685_SetServoAngle(s_cfg.pca_right, i, a[i]);
        PCA9685_SetServoAngle(s_cfg.pca_left,  i, a[i + 9]);
    }
}

static void handle_stream(char *arg)
{
    if (!arg) return;
    if      (strcmp(arg, "EN")  == 0) { s_stream_on = 1; s_stream_next_ms = HAL_GetTick(); }
    else if (strcmp(arg, "DIS") == 0) { s_stream_on = 0; }
}

void handle_current(char *arg)
{
    (void)arg;

    float current_matrix[8][2];

    get_current_servos(current_matrix);

    // Print each row separated by commas
    for (uint8_t i = 0; i < 8; i++) {
        // Format: Index, RightValue, LeftValue
        printf("%.3f", current_matrix[i][0]);
    }
    printf("\r\n");

    // Print each row separated by commas
    for (uint8_t i = 0; i < 8; i++) {
        // Format: Index, RightValue, LeftValue
        printf("%.3f", current_matrix[i][1]);
    }
    printf("\r\n");
}

static void handle_battery(char *arg)
{
    if (!arg) return;
    if (strcmp(arg, "V") == 0) {
        printf("/BATTERY/V/%.2f\n", Battery_GetVoltage());
    } else if (strcmp(arg, "P") == 0) {
        printf("/BATTERY/P/%.1f\n", Battery_GetPercentageF(Battery_GetVoltage()));
    }
}

static void handle_led_r(char *arg)
{
    int v[12];
    if (!parse_ints(arg, v, 12)) return;
    for (int i = 0; i < 3; i++) {
        s_led_right[i].r          = (uint8_t)v[i*4 + 0];
        s_led_right[i].g          = (uint8_t)v[i*4 + 1];
        s_led_right[i].b          = (uint8_t)v[i*4 + 2];
        s_led_right[i].brightness = (uint8_t)v[i*4 + 3];
    }
    /* TODO: push s_led_right[] to the right WS2812B strip on TIM1 / PA8. */
}

static void handle_led_l(char *arg)
{
    int v[12];
    if (!parse_ints(arg, v, 12)) return;
    for (int i = 0; i < 3; i++) {
        s_led_left[i].r          = (uint8_t)v[i*4 + 0];
        s_led_left[i].g          = (uint8_t)v[i*4 + 1];
        s_led_left[i].b          = (uint8_t)v[i*4 + 2];
        s_led_left[i].brightness = (uint8_t)v[i*4 + 3];
    }
    /* TODO: push s_led_left[] to the left WS2812B strip on TIM1 / PA9. */
}

static void handle_tof(char *arg)
{
    (void)arg;
    printf("/TOF/%u\n", (unsigned)tof_get_distance_mm());
}

static void handle_imu(char *arg)
{
    (void)arg;
    IMU_Data_t d = {0};
    if (IMU_Read(&d) != 0) return;
    printf("/IMU/%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
           d.accel_x_mg, d.accel_y_mg, d.accel_z_mg,
           d.gyro_x_mdps, d.gyro_y_mdps, d.gyro_z_mdps);
}

static void handle_mode(char *arg)
{
    (void)arg;
    if (!s_cfg.loko) return;

    const char *gait_name = "Unknown";
    switch (s_cfg.loko->gait_mode) {
        case GAIT_TRIPOD: gait_name = "Tripod"; break;
        case GAIT_WAVE:   gait_name = "Wave";   break;
        case GAIT_RIPPLE: gait_name = "Ripple"; break;
        default: break;
    }
    printf("/MODE/%s\n", gait_name);
}

static void handle_stab(char *arg)
{
    (void)arg;
    if (!s_cfg.loko) return;

    const char *stab_name = "Unknown";
    switch (s_cfg.loko->stab_mode) {
        case STAB_OFF:    stab_name = "Off";    break;
        case STAB_STABLE: stab_name = "Stable"; break;
        case STAB_LEVEL:  stab_name = "Level";  break;
        default: break;
    }
    printf("/STAB/%s\n", stab_name);
}

static void handle_tilt(char *arg)
{
    (void)arg;
    if (!s_cfg.loko) return;

    float roll_deg, pitch_deg;
    IMU_GetAngles(&roll_deg, &pitch_deg);
    printf("/TILT/%.2f,%.2f\n", roll_deg, pitch_deg);
}

static void handle_state(char *arg)
{
    if (!arg) return;
    if (strcmp(arg, "Disarmed") == 0 || strcmp(arg, "DISARMED") == 0) {
        s_state = UART_STATE_DISARMED;
        /* TODO: arming side-effects — probably force SERVO DIS and stop
         * the locomotion pipeline so nothing moves while disarmed. */
    } else if (strcmp(arg, "ARMED") == 0 || strcmp(arg, "AMRED") == 0) {
        /* accept the diagram's "AMRED" spelling and the correct one */
        s_state = UART_STATE_ARMED;
        /* TODO: matching side-effects on entering ARMED. */
    }
    /* FUTURE EXPANSION slot: silently accepted */
}

/* /CONTROLL payload: 10 ints per CONTROL_PROTOCOL.txt
 *   rsx, rsy, lsx, lsy, hat_x, hat_y, face, sticks, trig, back */
static void handle_controll(char *arg)
{
    int v[10];
    if (!parse_ints(arg, v, 10)) return;
    /* Sticks are sent as integers in [-100, 100] by the host.
     * Divide by 100 to get the [-1, 1] float range the locomotion expects. */
    s_controller.right_stick_x   = (float)v[0] / 100.0f;
    s_controller.right_stick_y   = (float)v[1] / 100.0f;
    s_controller.left_stick_x    = (float)v[2] / 100.0f;
    s_controller.left_stick_y    = (float)v[3] / 100.0f;
    s_controller.dpad_x          = (int8_t) v[4];
    s_controller.dpad_y          = (int8_t) v[5];
    s_controller.face_buttons    = (uint8_t)v[6];
    s_controller.stick_buttons   = (uint8_t)v[7];
    s_controller.trigger_buttons = (uint8_t)v[8];
    s_controller.back_buttons    = (uint8_t)v[9];
}

static void handle_ledc(char *arg)
{
    int v[4];
    if (!parse_ints(arg, v, 4)) return;
    s_led_ctrl.r          = (uint8_t)v[0];
    s_led_ctrl.g          = (uint8_t)v[1];
    s_led_ctrl.b          = (uint8_t)v[2];
    s_led_ctrl.brightness = (uint8_t)v[3];
    /* The controller lightbar lives on the Pi side — the STM32 just stores
     * the latest value so the Pi can read it back if it needs to. Nothing
     * else to do here unless a dedicated channel gets added. */
}

/* ─── line dispatch ──────────────────────────────────────────────── */
static void dispatch_line(char *line)
{
    if (line[0] != '/') return;                   /* must start with '/' */

    char *cat = line + 1;
    char *arg = strchr(cat, '/');
    if (arg) { *arg = '\0'; arg++; }              /* split category / arg */

    /* trim trailing \r and whitespace on the argument */
    if (arg) {
        size_t n = strlen(arg);
        while (n && (arg[n-1] == '\r' || arg[n-1] == ' ' || arg[n-1] == '\t')) {
            arg[--n] = '\0';
        }
    }

    if      (strcmp(cat, "SERVO")    == 0) handle_servo   (arg);
    else if (strcmp(cat, "STREAM")   == 0) handle_stream  (arg);
    else if (strcmp(cat, "CURRENT")  == 0) handle_current (arg);
    else if (strcmp(cat, "BATTERY")  == 0) handle_battery (arg);
    else if (strcmp(cat, "LED_R")    == 0) handle_led_r   (arg);
    else if (strcmp(cat, "LED_L")    == 0) handle_led_l   (arg);
    else if (strcmp(cat, "TOF")      == 0) handle_tof     (arg);
    else if (strcmp(cat, "IMU")      == 0) handle_imu     (arg);
    else if (strcmp(cat, "MODE")     == 0) handle_mode    (arg);
    else if (strcmp(cat, "STAB")     == 0) handle_stab    (arg);
    else if (strcmp(cat, "TILT")     == 0) handle_tilt    (arg);
    else if (strcmp(cat, "STATE")    == 0) handle_state   (arg);
    else if (strcmp(cat, "CONTROLL") == 0) handle_controll(arg);
    else if (strcmp(cat, "LEDC")     == 0) handle_ledc    (arg);
    /* unknown category → silent drop */
}

/* ─── RX polling ─────────────────────────────────────────────────── */
static void rx_poll(void)
{
    /* Drain the ring buffer populated by the ISR */
    while (s_rx_ring.head != s_rx_ring.tail)
    {
        uint8_t c = s_rx_ring.buf[s_rx_ring.tail];
        s_rx_ring.tail = (uint16_t)((s_rx_ring.tail + 1) % RX_RING_SIZE);

        // Check for EOL: Handle \n (Linux) or \r (Mac/Terminal)
        if (c == '\n' || c == '\r') {
            if (s_rx_idx > 0) { // Only dispatch if we have data
                if (!s_rx_overflow) {
                    s_rx_buf[s_rx_idx] = '\0';
                    dispatch_line(s_rx_buf);
                }
                s_rx_idx      = 0;
                s_rx_overflow = 0;
            }
        }
        // Ignore null bytes or other control chars if necessary
        else if (c >= 32 && c <= 126) {
            if (s_rx_idx < UART_RX_BUF_LEN - 1) {
                s_rx_buf[s_rx_idx++] = (char)c;
            } else {
                s_rx_overflow = 1;
            }
        }
    }

    /* Check for hardware errors on the UART itself */
    if (__HAL_UART_GET_FLAG(s_cfg.huart, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(s_cfg.huart, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(s_cfg.huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FLAG(s_cfg.huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    }
}

/** Called from USART1_IRQHandler to push bytes into the ring buffer. */
void UART_Protocol_RX_Callback(void)
{
    UART_HandleTypeDef *huart = s_cfg.huart;
    if (!huart) return;

    /* Check RXNE flag */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        uint8_t c = (uint8_t)(huart->Instance->RDR & 0xFFu);
        uint16_t next = (uint16_t)((s_rx_ring.head + 1) % RX_RING_SIZE);
        if (next != s_rx_ring.tail) {
            s_rx_ring.buf[s_rx_ring.head] = c;
            s_rx_ring.head = next;
        }
    }

    /* Clean up errors that block reception */
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(huart, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    }
}

/* ─── streaming telemetry (50 Hz when enabled) ───────────────────── */
static void stream_tick(void)
{
    if (!s_stream_on) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_stream_next_ms) < 0) return;
    s_stream_next_ms = now + STREAM_PERIOD_MS;

    /* Format: 18 currents, battery V, battery %, 1 TOF distance, 6 IMU. */
    printf("/STREAM/");

    for (int i = 0; i < 9; i++) printf("%.3f,", 0.0f);   /* r1..r9 currents */
    for (int i = 0; i < 9; i++) printf("%.3f,", 0.0f);   /* l1..l9 currents */

    float v = Battery_GetVoltage();
    printf("%.2f,%.1f,", v, Battery_GetPercentageF(v));

    printf("%u,", (unsigned)tof_get_distance_mm());

    IMU_Data_t d = {0};
    (void)IMU_Read(&d);
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
           d.accel_x_mg, d.accel_y_mg, d.accel_z_mg,
           d.gyro_x_mdps, d.gyro_y_mdps, d.gyro_z_mdps);
}

/* ─── public API ─────────────────────────────────────────────────── */
void UART_Protocol_Init(const UART_Protocol_Config_t *cfg)
{
    s_cfg = *cfg;
    s_rx_idx         = 0;
    s_rx_overflow    = 0;
    s_rx_ring.head   = 0;
    s_rx_ring.tail   = 0;
    s_stream_on      = 0;
    s_stream_next_ms = 0;
    s_state          = UART_STATE_DISARMED;
    memset(&s_controller, 0, sizeof(s_controller));
    memset(s_led_right,   0, sizeof(s_led_right));
    memset(s_led_left,    0, sizeof(s_led_left));
    memset(&s_led_ctrl,   0, sizeof(s_led_ctrl));

    /* Enable RXNE interrupt */
    __HAL_UART_ENABLE_IT(s_cfg.huart, UART_IT_RXNE);

    /* unbuffered so replies go out immediately */
    setvbuf(stdout, NULL, _IONBF, 0);
}

void UART_Update(void)
{
    rx_poll();
    stream_tick();
}

const UART_ControllerState_t *UART_GetController(void)    { return &s_controller; }
UART_RobotState_t             UART_GetState(void)         { return s_state; }
const UART_LED_t             *UART_GetRightLEDs(void)     { return s_led_right; }
const UART_LED_t             *UART_GetLeftLEDs(void)      { return s_led_left; }
const UART_LED_t             *UART_GetControllerLED(void) { return &s_led_ctrl; }
