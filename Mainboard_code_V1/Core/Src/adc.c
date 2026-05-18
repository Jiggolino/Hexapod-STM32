#include "adc.h"
#include <stddef.h>

/* ── module state ────────────────────────────────────────────────────── */

static ADC_HandleTypeDef *s_hadc_right   = NULL;
static ADC_HandleTypeDef *s_hadc_left    = NULL;
static ADC_HandleTypeDef *s_hadc_battery = NULL;
static uint32_t           s_vdda_mv      = 3300u;

/* ── public init ─────────────────────────────────────────────────────── */

void ADC_Init(ADC_HandleTypeDef *hadc1_current_right,
              ADC_HandleTypeDef *hadc2_current_left,
              ADC_HandleTypeDef *hadc3_battery)
{
    s_hadc_right   = hadc1_current_right;
    s_hadc_left    = hadc2_current_left;
    s_hadc_battery = hadc3_battery;
}

/* ── low-level primitives ────────────────────────────────────────────── */

uint32_t ADC_ReadRaw(ADC_HandleTypeDef *hadc, uint32_t channel)
{
    if (!hadc) return 0u;

    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = channel;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
    cfg.SingleDiff   = ADC_SINGLE_ENDED;
    cfg.OffsetNumber = ADC_OFFSET_NONE;

    HAL_ADC_ConfigChannel(hadc, &cfg);
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return val;
}

uint32_t ADC_ReadRawAvg(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t n)
{
    if (!hadc || n == 0u) return 0u;

    uint64_t acc = 0u;
    for (uint32_t i = 0u; i < n; ++i)
        acc += ADC_ReadRaw(hadc, channel);

    return (uint32_t)(acc / n);
}

/* ── named helpers ───────────────────────────────────────────────────── */

uint32_t ADC_ReadCurrentRight(void)
{
    return ADC_ReadRawAvg(s_hadc_right, ADC_CH_CURRENT_RIGHT, ADC_SW_AVG_N);
}

uint32_t ADC_ReadCurrentLeft(void)
{
    return ADC_ReadRawAvg(s_hadc_left, ADC_CH_CURRENT_LEFT, ADC_SW_AVG_N);
}

uint32_t ADC_ReadBatteryRaw(void)
{
    return ADC_ReadRawAvg(s_hadc_battery, ADC_CH_BATTERY, ADC_SW_AVG_N);
}

uint32_t ADC_ReadVrefintRaw(void)
{
    return ADC_ReadRawAvg(s_hadc_battery, ADC_CHANNEL_VREFINT, ADC_SW_AVG_N);
}

/* ── Vdda calibration ────────────────────────────────────────────────── */

uint32_t ADC_CalibrateVdda(void)
{
    uint32_t raw = ADC_ReadVrefintRaw();
    if (raw == 0u) return s_vdda_mv;
    s_vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw, ADC_RESOLUTION_16B);
    return s_vdda_mv;
}

uint32_t ADC_GetVddaMv(void)
{
    return s_vdda_mv;
}
