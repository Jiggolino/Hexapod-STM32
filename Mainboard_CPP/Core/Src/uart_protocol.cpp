/**
 * @file  uart_protocol.c
 * @brief See uart_protocol.h for the wire format.
 *
 * All messages use the format /CATEGORY/ARG\n
 * Incoming lines are parsed from an ISR-filled ring buffer.
 * Outgoing data is written via the DMA TX ring buffer (uart_dma_tx.c).
 */

#include "uart_protocol.h"
#include "battery.h"
#include "main.h"
#include "uart_dma_tx.h"
#include "loko_states.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define UART_RX_BUF_LEN     256u
#define STREAM_PERIOD_MS    20u      /* 50 Hz telemetry rate */

static UART_Protocol_Config_t s_cfg;

#define RX_RING_SIZE 1024u
static struct {
    uint8_t  buf[RX_RING_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} s_rx_ring;

static char     s_rx_buf[UART_RX_BUF_LEN];
static uint16_t s_rx_idx;
static uint8_t  s_rx_overflow;

static uint8_t  s_stream_on;
static uint32_t s_stream_next_ms;

static UART_RobotState_t      s_state;
static UART_ControllerState_t s_controller;
static UART_LED_t             s_led_ctrl;

/* ─── printf retarget ────────────────────────────────────────────────────── */
/* Full Newlib calls _write; Newlib Nano walks character-by-character through
 * __io_putchar.  Both are provided so either libc variant works. */

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

/* ─── tokenisers (destructive: they modify 'str' in place) ──────────────── */

/*
 * Splits str on commas, converts each token to float via strtof, and stores
 * exactly n values in out[]. Rejects tokens with trailing non-whitespace chars.
 * Input:  str — null-terminated comma-separated string (modified in place)
 *         out — output array of at least n floats
 *         n   — expected number of fields
 * Output: 1 if exactly n valid floats were parsed, 0 otherwise
 */
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
        if (end == tok) return 0;
        while (*end == ' ' || *end == '\t') end++;
        if (*end != '\0') return 0;
        out[count++] = v;
        tok = strtok_r(NULL, ",", &save);
    }
    return (count == n);
}

/*
 * Same as parse_floats() but uses strtol and stores integers.
 * Input:  str — null-terminated comma-separated string (modified in place)
 *         out — output array of at least n ints
 *         n   — expected number of fields
 * Output: 1 if exactly n valid integers were parsed, 0 otherwise
 */
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

/* ─── command handlers ───────────────────────────────────────────────────── */

/*
 * /SERVO/EN   — enables servo power (OE low on both boards)
 * /SERVO/DIS  — disables servo power (OE high)
 * /SERVO/a,b,…,r — sets 9 right + 9 left channels from 18 float angles in degrees
 * Input:  arg — string after the second '/'
 * Output: void
 */
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

/*
 * /STREAM/EN — starts 50 Hz telemetry streaming
 * /STREAM/DIS — stops streaming
 * Input:  arg — "EN" or "DIS"
 * Output: void
 */
static void handle_stream(char *arg)
{
    if (!arg) return;
    if      (strcmp(arg, "EN")  == 0) { s_stream_on = 1; s_stream_next_ms = HAL_GetTick(); }
    else if (strcmp(arg, "DIS") == 0) { s_stream_on = 0; }
}

/*
 * /BATTERY/V — replies with pack voltage: /BATTERY/V/<float>
 * /BATTERY/P — replies with SOC percent:  /BATTERY/P/<float>
 * Input:  arg — "V" or "P"
 * Output: void (prints to UART)
 */
static void handle_battery(char *arg)
{
    if (!arg) return;
    if (strcmp(arg, "V") == 0) {
        printf("/BATTERY/V/%.2f\n", Battery_GetVoltage());
    } else if (strcmp(arg, "P") == 0) {
        printf("/BATTERY/P/%.1f\n", Battery_GetPercentageF(Battery_GetVoltage()));
    }
}

/*
 * /TOF/ — replies with the latest distance: /TOF/<mm>
 * Output: void (prints to UART)
 */
static void handle_tof(char *arg)
{
    (void)arg;
    printf("/TOF/%u\n", (unsigned)tof_get_distance_mm());
}

/*
 * /IMU/ — replies with 6 values: accel_x/y/z (mg), gyro_x/y/z (mdps)
 * Format: /IMU/<ax>,<ay>,<az>,<gx>,<gy>,<gz>
 * Output: void (prints to UART)
 */
static void handle_imu(char *arg)
{
    (void)arg;
    IMU_Data_t d = {0};
    if (IMU_Read(&d) != 0) return;
    printf("/IMU/%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
           d.accel_x_mg, d.accel_y_mg, d.accel_z_mg,
           d.gyro_x_mdps, d.gyro_y_mdps, d.gyro_z_mdps);
}

/*
 * /MODE/ — replies with the current gait mode name: /MODE/<name>
 * Output: void (prints to UART)
 */
static void handle_mode(char *arg)
{
    (void)arg;
    if (!s_cfg.loko) return;

    const char *gait_name = "Unknown";
    switch (s_cfg.loko->gait_mode) {
        case GAIT_TRIPOD:   gait_name = "Tripod";   break;
        case GAIT_WAVE:     gait_name = "Wave";     break;
        case GAIT_RIPPLE:   gait_name = "Ripple";   break;
        case GAIT_OBSTACLE: gait_name = "Obstacle"; break;
        default: break;
    }
    printf("/MODE/%s\n", gait_name);
}

/*
 * /STAB/ — replies with the current stabiliser mode name: /STAB/<name>
 * Output: void (prints to UART)
 */
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

/*
 * /TILT/ — replies with IMU roll and pitch in degrees: /TILT/<roll>,<pitch>
 * Output: void (prints to UART)
 */
static void handle_tilt(char *arg)
{
    (void)arg;
    if (!s_cfg.loko) return;

    float roll_deg, pitch_deg;
    IMU_GetAngles(&roll_deg, &pitch_deg);
    printf("/TILT/%.2f,%.2f\n", roll_deg, pitch_deg);
}

/*
 * /STATE/<name> — updates the internal robot state flag.
 * Accepted values: "Disarmed", "DISARMED", "ARMED", "AMRED" (typo accepted).
 * Input:  arg — state name string
 * Output: void
 */
static void handle_state(char *arg)
{
    if (!arg) return;
    if (strcmp(arg, "Disarmed") == 0 || strcmp(arg, "DISARMED") == 0) {
        s_state = UART_STATE_DISARMED;
    } else if (strcmp(arg, "ARMED") == 0 || strcmp(arg, "AMRED") == 0) {
        s_state = UART_STATE_ARMED;
    }
}

/*
 * /CONTROLL/<rsx>,<rsy>,<lsx>,<lsy>,<hat_x>,<hat_y>,<face>,<sticks>,<trig>,<back>
 * Decodes 10 integers per CONTROL_PROTOCOL.txt. Stick axes are integers in
 * [-100, 100] and are divided by 100 to produce the [-1, 1] floats that the
 * locomotion engine expects.
 * Input:  arg — comma-separated 10-integer string
 * Output: void (updates s_controller)
 */
static void handle_controll(char *arg)
{
    int v[10];
    if (!parse_ints(arg, v, 10)) return;
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

/*
 * /LEDC/<r>,<g>,<b>,<brightness> — stores the controller lightbar colour.
 * The STM32 stores the value so the Pi can read it back; actual lightbar
 * control lives on the Pi side.
 * Input:  arg — 4-integer string
 * Output: void (updates s_led_ctrl)
 */
static void handle_ledc(char *arg)
{
    int v[4];
    if (!parse_ints(arg, v, 4)) return;
    s_led_ctrl.r          = (uint8_t)v[0];
    s_led_ctrl.g          = (uint8_t)v[1];
    s_led_ctrl.b          = (uint8_t)v[2];
    s_led_ctrl.brightness = (uint8_t)v[3];
}

/* ─── line dispatch ──────────────────────────────────────────────────────── */

/*
 * Splits a complete received line at the second '/', looks up the category
 * string, and calls the matching handler. Lines not starting with '/' are
 * silently dropped. Unknown categories are also silently dropped.
 * Input:  line — null-terminated string (modified in place)
 * Output: void
 */
static void dispatch_line(char *line)
{
    if (line[0] != '/') return;

    char *cat = line + 1;
    char *arg = strchr(cat, '/');
    if (arg) { *arg = '\0'; arg++; }

    if (arg) {
        size_t n = strlen(arg);
        while (n && (arg[n-1] == '\r' || arg[n-1] == ' ' || arg[n-1] == '\t'))
            arg[--n] = '\0';
    }

    if      (strcmp(cat, "SERVO")    == 0) handle_servo   (arg);
    else if (strcmp(cat, "STREAM")   == 0) handle_stream  (arg);
    else if (strcmp(cat, "BATTERY")  == 0) handle_battery (arg);
    else if (strcmp(cat, "TOF")      == 0) handle_tof     (arg);
    else if (strcmp(cat, "IMU")      == 0) handle_imu     (arg);
    else if (strcmp(cat, "MODE")     == 0) handle_mode    (arg);
    else if (strcmp(cat, "STAB")     == 0) handle_stab    (arg);
    else if (strcmp(cat, "TILT")     == 0) handle_tilt    (arg);
    else if (strcmp(cat, "STATE")    == 0) handle_state   (arg);
    else if (strcmp(cat, "CONTROLL") == 0) handle_controll(arg);
    else if (strcmp(cat, "LEDC")     == 0) handle_ledc    (arg);
}

/* ─── RX polling ─────────────────────────────────────────────────────────── */

/*
 * Drains bytes from the ISR ring buffer into a line accumulation buffer.
 * On '\n' or '\r' the accumulated line is dispatched if not overflowed.
 * Printable ASCII bytes (32–126) are accepted; all other control codes are
 * silently ignored. Hardware ORE/NE/FE error flags are cleared on each call.
 * Output: void
 */
static void rx_poll(void)
{
    while (s_rx_ring.head != s_rx_ring.tail)
    {
        uint8_t c = s_rx_ring.buf[s_rx_ring.tail];
        s_rx_ring.tail = (uint16_t)((s_rx_ring.tail + 1) % RX_RING_SIZE);

        if (c == '\n' || c == '\r') {
            if (s_rx_idx > 0) {
                if (!s_rx_overflow) {
                    s_rx_buf[s_rx_idx] = '\0';
                    dispatch_line(s_rx_buf);
                }
                s_rx_idx      = 0;
                s_rx_overflow = 0;
            }
        } else if (c >= 32 && c <= 126) {
            if (s_rx_idx < UART_RX_BUF_LEN - 1) {
                s_rx_buf[s_rx_idx++] = (char)c;
            } else {
                s_rx_overflow = 1;
            }
        }
    }

    if (__HAL_UART_GET_FLAG(s_cfg.huart, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(s_cfg.huart, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(s_cfg.huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FLAG(s_cfg.huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    }
}

/*
 * Called from USART1_IRQHandler on every RXNE interrupt. Reads one byte from
 * the UART data register and pushes it into the ring buffer if space is
 * available; otherwise the byte is dropped. Clears framing/noise/overrun errors.
 * Output: void (s_rx_ring.head advanced)
 */
void UART_Protocol_RX_Callback(void)
{
    UART_HandleTypeDef *huart = s_cfg.huart;
    if (!huart) return;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        uint8_t c = (uint8_t)(huart->Instance->RDR & 0xFFu);
        uint16_t next = (uint16_t)((s_rx_ring.head + 1) % RX_RING_SIZE);
        if (next != s_rx_ring.tail) {
            s_rx_ring.buf[s_rx_ring.head] = c;
            s_rx_ring.head = next;
        }
    }

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(huart, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(huart, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF);
    }
}

/* ─── streaming telemetry ────────────────────────────────────────────────── */

/*
 * Sends one 50 Hz telemetry frame if streaming is enabled and the next
 * deadline has passed. Frame format (all on one line):
 *   /STREAM/<r1..r9 currents>,<l1..l9 currents>,<Vbatt>,<SOC%>,<ToF mm>,
 *           <ax>,<ay>,<az mg>,<gx>,<gy>,<gz mdps>
 * Current fields are always 0.0 (no current sensing on this board revision).
 * Output: void (prints to UART via DMA ring buffer)
 */
static void stream_tick(void)
{
    if (!s_stream_on) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - s_stream_next_ms) < 0) return;
    s_stream_next_ms = now + STREAM_PERIOD_MS;

    printf("/STREAM/");

    for (int i = 0; i < 9; i++) printf("%.3f,", 0.0f);
    for (int i = 0; i < 9; i++) printf("%.3f,", 0.0f);

    float v = Battery_GetVoltage();
    printf("%.2f,%.1f,", v, Battery_GetPercentageF(v));

    printf("%u,", (unsigned)tof_get_distance_mm());

    IMU_Data_t d = {0};
    (void)IMU_Read(&d);
    printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
           d.accel_x_mg, d.accel_y_mg, d.accel_z_mg,
           d.gyro_x_mdps, d.gyro_y_mdps, d.gyro_z_mdps);
}

/* ─── public API ─────────────────────────────────────────────────────────── */

/*
 * Copies the config, zeroes all RX/TX state, enables the RXNE interrupt, and
 * sets stdout unbuffered so printf replies arrive at the host immediately.
 * Input:  cfg — {huart, pca_right, pca_left, loko} used by command handlers
 * Output: void
 */
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
    memset(&s_led_ctrl,   0, sizeof(s_led_ctrl));

    __HAL_UART_ENABLE_IT(s_cfg.huart, UART_IT_RXNE);

    setvbuf(stdout, NULL, _IONBF, 0);
}

/*
 * Main protocol service routine. Drains the RX ring buffer and optionally
 * emits one telemetry frame. Call every control cycle (~100 Hz).
 * Output: void
 */
void UART_Update(void)
{
    rx_poll();
    stream_tick();
}

/* Simple accessors for module state */
const UART_ControllerState_t *UART_GetController(void)    { return &s_controller; }
UART_RobotState_t             UART_GetState(void)         { return s_state; }
const UART_LED_t             *UART_GetControllerLED(void) { return &s_led_ctrl; }
