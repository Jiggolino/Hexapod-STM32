/*
 * Battery: pack-voltage measurement with Vrefint/Vdda calibration,
 * plus lookup-table state-of-charge.
 */

#include "battery.h"
#include "adc.h"
#include "ws2812b.h"
#include <stdint.h>
#include <stddef.h>

/* ─── voltage-measurement tunables ──────────────────────────────────── */

/* Battery divider: R1 = 1 k, R2 = 620 R → V_adc = V_bat · 620/1620 */
#define ADC16_MAX           65535.0f
#define BATT_RATIO          (620.0f / (1000.0f + 620.0f))

/* Two-point linear calibration: actual = raw · GAIN + OFFSET.
 * Measured against a precision PSU at 5.998 / 7.00 / 7.50 / 8.00 V with
 * the Vrefint-calibrated Vdda applied. Least-squares fit gave gain≈1.028
 * and offset≈+0.41 V with <20 mV residuals across the whole range. The
 * dominant error is a DC offset (not a gain error), probably from extra
 * series R in the high-side of the divider or ADC input leakage. */
#define BATT_CAL_GAIN       1.0f
#define BATT_CAL_OFFSET     0.0f

/* LiPo safety bias: reported voltage is deliberately pulled down by this
 * much so any protection threshold (cutoff, warning) trips a touch early
 * rather than late. Prefer to stop a bit above min-cell than let a pack
 * sag below 3.0 V/cell under load. */
#define BATT_SAFETY_MARGIN  0.050f

/* ─── voltage API ───────────────────────────────────────────────────── */

void Battery_Init(ADC_HandleTypeDef *hadc)
{
    (void)hadc;  /* handle now owned by adc.c; kept for API compatibility */
    ADC_CalibrateVdda();
}

float Battery_GetVoltage(void)
{
    uint32_t raw   = ADC_ReadBatteryRaw();
    float    vdda  = ADC_GetVddaMv() * 0.001f;
    float    v_raw = (raw / ADC16_MAX) * vdda / BATT_RATIO;
    float v = v_raw * BATT_CAL_GAIN + BATT_CAL_OFFSET - BATT_SAFETY_MARGIN;
    if(v < 6.6f){
    	while(1){
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

uint32_t Battery_GetVddaMv(void)
{
    return ADC_GetVddaMv();
}

/* ─── percentage lookup (unchanged) ─────────────────────────────────── */

#define CLAMP_F(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

/**
 * Per-cell OCV breakpoints [V].
 * Must be strictly monotonically increasing — the binary search relies on it.
 *
 *  Cell voltage   Pack voltage (×2)   SOC
 *  ──────────────────────────────────────
 *     3.00 V          6.00 V           0 %   ← hard cut-off
 *     3.10 V          6.20 V           1 %
 *     3.20 V          6.40 V           3 %
 *     3.30 V          6.60 V           6 %
 *     3.40 V          6.80 V          10 %
 *     3.50 V          7.00 V          15 %
 *     3.60 V          7.20 V          22 %
 *     3.70 V          7.40 V          32 %   ← plateau begins
 *     3.75 V          7.50 V          40 %
 *     3.80 V          7.60 V          52 %
 *     3.85 V          7.70 V          62 %
 *     3.90 V          7.80 V          70 %
 *     3.95 V          7.90 V          78 %
 *     4.00 V          8.00 V          84 %   ← plateau ends
 *     4.05 V          8.10 V          89 %
 *     4.10 V          8.20 V          93 %
 *     4.15 V          8.30 V          97 %
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

static float soc_from_cell_voltage(float v_cell)
{
    if (v_cell <= LUT_CELL_V[0])            return 0.0f;
    if (v_cell >= LUT_CELL_V[LUT_SIZE - 1]) return 100.0f;

    uint32_t lo = 0U;
    uint32_t hi = (uint32_t)(LUT_SIZE - 1U);

    while ((hi - lo) > 1U)
    {
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

float Battery_GetPercentageF(float voltage)
{
    float v_cell = voltage / (float)BATTERY_CELLS;
    float soc    = soc_from_cell_voltage(v_cell);
    return CLAMP_F(soc, 0.0f, 100.0f);
}

uint8_t Battery_GetPercentage8(float voltage)
{
    float    soc    = Battery_GetPercentageF(voltage);
    uint32_t result = (uint32_t)(soc + 0.5f);
    return (result > 100U) ? 100U : (uint8_t)result;
}
