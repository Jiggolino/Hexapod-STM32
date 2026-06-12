/**
 * @file  uart_protocol.h
 * @brief Line-based UART protocol between Raspberry Pi host and STM32H7.
 *
 * Wire format  :  /<CATEGORY>/<ARG>\n
 *                 - every command starts with '/' and ends with '\n'
 *                 - malformed lines are silently dropped
 *                 - the RX buffer is drained every UART_Update() so multiple
 *                   commands in one read are all processed
 *
 * Commands (PI → STM32)
 * ---------------------
 *   /SERVO/EN                           enable both PCA9685 outputs (OE low)
 *   /SERVO/DIS                          disable both PCA9685 outputs (OE high)
 *   /SERVO/a1,a2,...,a18                18 angles: r1..r9, l1..l9 (deg)
 *   /STREAM/EN                          start 50 Hz telemetry stream
 *   /STREAM/DIS                         stop telemetry stream
 *   /CURRENT                            one-shot current snapshot (TODO)
 *   /BATTERY/V                          reply with /BATTERY/V/<volts>
 *   /BATTERY/P                          reply with /BATTERY/P/<percent>
 *   /LED_R/R,G,B,Br,R,G,B,Br,R,G,B,Br   3 right-side WS2812B LEDs
 *   /LED_L/R,G,B,Br,R,G,B,Br,R,G,B,Br   3 left-side WS2812B LEDs
 *   /TOF/NF                             reply with /TOF/NF/<near>,<far>
 *   /TOF/FULL                           reply with /TOF/FULL/<64 values>
 *   /IMU                                reply with /IMU/ax,ay,az,gx,gy,gz
 *   /STATE/Disarmed | /STATE/ARMED      robot state machine
 *   /CONTROLL/<9 ints>                  PS5 controller snapshot (see struct)
 *   /LEDC/R,G,B,Br                      controller lightbar colour
 *
 * All non-matching or malformed lines are ignored without a reply.
 */

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "pca9685.h"
#include "imu.h"
#include "tof.h"
#include "lokomotion.h"

/** PS5 controller snapshot set by /CONTROLL
 *  Maps to: /CONTROLL/{rsx},{rsy},{lsx},{lsy},{hat_x},{hat_y},{face},{sticks},{trig},{back}\n
 *  Sticks: -100 to 100 (scaled to -1.0..+1.0 by handler)
 *  Dpad:   -1, 0, 1
 *  Buttons: bitmasks (see protocol spec) */
typedef struct {
    float   right_stick_x;    /* -1.0 .. +1.0 */
    float   right_stick_y;
    float   left_stick_x;
    float   left_stick_y;
    int8_t  dpad_x;           /* hat_x: -1 (left), 0, 1 (right) */
    int8_t  dpad_y;           /* hat_y: -1 (down), 0, 1 (up) */
    uint8_t face_buttons;     /* bitmask: X(1) A(2) B(4) Y(8) */
    uint8_t stick_buttons;    /* bitmask: L3(1) R3(2) */
    uint8_t trigger_buttons;  /* bitmask: LT(1) View(2) LB(4) RT(8) RB(16) Start(32) */
    uint8_t back_buttons;     /* bitmask: M1(1) M2(2) M3(4) Y1(8) Y2(16) Y3(32) (reserved, always 0) */
} UART_ControllerState_t;

typedef enum {
    UART_STATE_DISARMED = 0,
    UART_STATE_ARMED    = 1,
} UART_RobotState_t;

typedef struct {
    uint8_t r, g, b, brightness;
} UART_LED_t;

/** Handles the protocol needs. Pointers are captured and must outlive the module. */
typedef struct {
    UART_HandleTypeDef *huart;
    PCA9685_t          *pca_right;
    PCA9685_t          *pca_left;
    LokoState          *loko;               /* Locomotion state for mode queries */
} UART_Protocol_Config_t;

/** Must be called once after all peripherals are initialised.
 *  Also retargets printf onto the given UART (via _write / __io_putchar). */
void UART_Protocol_Init(const UART_Protocol_Config_t *cfg);

/** Drains the UART RX, dispatches any complete commands, and sends the
 *  telemetry packet when STREAM is enabled. Call every main-loop iteration. */
void UART_Update(void);

/** Internal: Called from ISR to push bytes into ring buffer. */
void UART_Protocol_RX_Callback(void);

/** Latest PS5 controller snapshot. Never NULL after Init. */
const UART_ControllerState_t *UART_GetController(void);

/** Latest armed/disarmed state. */
UART_RobotState_t UART_GetState(void);

/** Latest colour+brightness sent by the host for the controller lightbar. */
const UART_LED_t *UART_GetControllerLED(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_PROTOCOL_H */
