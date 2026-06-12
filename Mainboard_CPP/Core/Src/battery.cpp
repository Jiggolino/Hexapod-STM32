/*
 * battery.cpp — Pack-voltage measurement with Vrefint/Vdda calibration,
 * plus lookup-table state-of-charge.
 */

#include "battery.h"
#include "adc.h"
#include "ws2812b.h"
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Battery divider: R1 = 1 k, R2 = 620 R → V_adc = V_bat · 620/1620 */
#define ADC16_MAX           65535.0f
#define BATT_RATIO          (620.0f / (1000.0f + 620.0f))

/* Two-point linear calibration: actual = raw · GAIN + OFFSET.
 * Measured against a precision PSU at 5.998 / 7.00 / 7.50 / 8.00 V with
 * Vrefint-calibrated Vdda applied. The dominant error is a DC offset (not a
 * gain error), probably from extra series R or ADC input leakage. */
#define BATT_CAL_GAIN       1.0f
#define BATT_CAL_OFFSET     0.0f

/* LiPo safety bias: reported voltage is deliberately pulled down so any
 * protection threshold trips a touch early rather than late. */
#define BATT_SAFETY_MARGIN  0.050f

/*
 * Calibrates the ADC Vdda reference using the internal Vrefint channel.
 * The hadc parameter is accepted for API compatibility; ownership stays in adc.c.
 * Input:  hadc — ADC handle (unused)
 * Output: void
 */
void Battery_Init(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    ADC_CalibrateVdda();
}

/*
 * Reads the raw ADC count for the battery divider output, converts it to a
 * pack voltage using the calibrated Vdda, applies the two-point calibration
 * and safety margin. If voltage drops below 6.6 V (≈ 3.3 V/cell) the function
 * enters an infinite safety loop: disables servos, pulses red LEDs, and streams
 * the low-battery message every 10 ms.
 * Output: battery pack voltage in volts (returns only when voltage ≥ 6.6 V)
 */
float Battery_GetVoltage(void)
{
    uint32_t raw   = ADC_ReadBatteryRaw();
    float    vdda  = ADC_GetVddaMv() * 0.001f;
    float    v_raw = (raw / ADC16_MAX) * vdda / BATT_RATIO;
    float v = v_raw * BATT_CAL_GAIN + BATT_CAL_OFFSET - BATT_SAFETY_MARGIN;
    if (v < 6.6f) {
        while (1) {
            HAL_GPIO_WritePin(Right_Enable_GPIO_Port, Right_Enable_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Left_Enable_GPIO_Port,  Left_Enable_Pin,  GPIO_PIN_SET);
            ws2812_mode_pulse_red();
            ws2812_tick();
            printf("/BATTERY/V/6.6\n");
            HAL_Delay(10);
        }
    }
    return v;
}

/*
 * Returns the internally calibrated Vdda in millivolts.
 * Output: Vdda in mV (typically ~3300)
 */
uint32_t Battery_GetVddaMv(void)
{
    return ADC_GetVddaMv();
}

/* ── State-of-charge lookup table ────────────────────────────────────────── */

#define CLAMP_F(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/*
 * Per-cell OCV breakpoints [V] — strictly monotonically increasing for binary search.
 *
 *  Cell voltage   Pack voltage (×2)   SOC
 *  ──────────────────────────────────────
 *     3.00 V          6.00 V           0 %   ← hard cut-off
 *     3.70 V          7.40 V          32 %   ← plateau begins
 *     4.00 V          8.00 V          84 %   ← plateau ends
 *     4.20 V          8.40 V         100 %   ← fully charged
 */
static const float LUT_CELL_V[] = {
    3.00f, 3.10f, 3.20f, 3.30f, 3.40f,
    3.50f, 3.60f, 3.70f, 3.75f, 3.80f,
    3.85f, 3.90f, 3.95f, 4.00f, 4.05f,
    4.10f, 4.15f, 4.20f
};

static const float LUT_SOC[] = {
     0.0f,  1.0f,  3.0f,  6.0f, 10.0f,
    15.0f, 22.0f, 32.0f, 40.0f, 52.0f,
    62.0f, 70.0f, 78.0f, 84.0f, 89.0f,
    93.0f, 97.0f, 100.0f
};

#define LUT_SIZE  (sizeof(LUT_CELL_V) / sizeof(LUT_CELL_V[0]))

/*
 * Binary-searches the OCV table for the bracketing interval, then linearly
 * interpolates to produce state-of-charge in percent.
 * Input:  v_cell — per-cell voltage in volts
 * Output: SOC in percent [0, 100]
 */
static float soc_from_cell_voltage(float v_cell)
{
    if (v_cell <= LUT_CELL_V[0])            return 0.0f;
    if (v_cell >= LUT_CELL_V[LUT_SIZE - 1]) return 100.0f;

    uint32_t lo = 0U;
    uint32_t hi = (uint32_t)(LUT_SIZE - 1U);

    while ((hi - lo) > 1U) {
        uint32_t mid = (lo + hi) >> 1U;
        if (v_cell >= LUT_CELL_V[mid])
            lo = mid;
        else
            hi = mid;
    }

    float t = (v_cell        - LUT_CELL_V[lo]) /
              (LUT_CELL_V[hi] - LUT_CELL_V[lo]);

    return LUT_SOC[lo] + t * (LUT_SOC[hi] - LUT_SOC[lo]);
}

/*
 * Divides the pack voltage by the number of cells, looks up SOC via the OCV
 * table, and clamps to [0, 100] %.
 * Input:  voltage — pack voltage in volts
 * Output: state-of-charge as a float in [0.0, 100.0]
 */
float Battery_GetPercentageF(float voltage)
{
    float v_cell = voltage / (float)BATTERY_CELLS;
    float soc    = soc_from_cell_voltage(v_cell);
    return CLAMP_F(soc, 0.0f, 100.0f);
}

/*
 * Same as Battery_GetPercentageF() but rounds to the nearest integer and
 * returns as uint8_t in [0, 100].
 * Input:  voltage — pack voltage in volts
 * Output: state-of-charge as uint8_t
 */
uint8_t Battery_GetPercentage8(float voltage)
{
    float    soc    = Battery_GetPercentageF(voltage);
    uint32_t result = (uint32_t)(soc + 0.5f);
    return (result > 100U) ? 100U : (uint8_t)result;
}
