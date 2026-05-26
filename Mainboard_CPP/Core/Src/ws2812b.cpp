#include "ws2812b.h"
#include <string.h>

#define WS_BITS_PER_LED     24
#define WS_DATA_SLOTS       (WS_BITS_PER_LED * WS2812_LEDS_PER_STRIP)
#define WS_LEAD_SLOTS       48          /* 60 us low before data: guarantees latch & hides first-CCR garbage */
#define WS_RESET_SLOTS      48          /* 60 us low after data: latch into LEDs */
#define WS_BUFFER_LEN       (WS_LEAD_SLOTS + WS_DATA_SLOTS + WS_RESET_SLOTS)

#define WS_CMP_0            84          /* T0H ~ 0.35 us @ ARR=299 */
#define WS_CMP_1            168         /* T1H ~ 0.70 us @ ARR=299 */

/* DMA buffer must sit in a region DMA1/DMA2 can reach (AXI SRAM, not DTCM).
 * With the stock H7 linker script, plain .bss globals land in AXI SRAM,
 * so no explicit section attribute is needed here. */
static uint16_t ws_buf[WS2812_STRIP_COUNT][WS_BUFFER_LEN] __attribute__((aligned(32)));
static uint8_t  ws_rgb[WS2812_STRIP_COUNT][WS2812_LEDS_PER_STRIP][3]; /* R,G,B */
static volatile bool ws_busy[WS2812_STRIP_COUNT];

static TIM_HandleTypeDef *ws_htim;

static const uint32_t ws_channel[WS2812_STRIP_COUNT] = {
    TIM_CHANNEL_1,
    TIM_CHANNEL_2,
};

void ws2812_init(TIM_HandleTypeDef *htim)
{
    ws_htim = htim;
    memset(ws_buf, 0, sizeof(ws_buf));
    for (int s = 0; s < WS2812_STRIP_COUNT; ++s) {
        ws2812_clear((ws2812_strip_t)s);
        ws_busy[s] = false;
    }
    /* TIM1 is an advanced-control timer: main output must be enabled. */
    __HAL_TIM_MOE_ENABLE(htim);
    /* Make sure both compare registers are low before any channel goes live,
     * so we don't latch a stale CCR as a spurious first bit. */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
    /* Force an update event so PSC/ARR and CCR preloads are all latched before
     * the very first DMA transfer starts — avoids a ragged first bit. */
    HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

    /* Wait for the LEDs' 5V rail to settle after power-on.
     * Without this, a cold boot sends data while WS2812Bs are still powering up
     * and they latch into a bad state until the next NRST. */
    HAL_Delay(50);

    /* Send a priming all-zero frame on every strip. This drives the line low
     * for the full buffer duration (~210 us total: 60us lead + 90us data-as-zeros
     * + 60us tail) which guarantees a clean >50us reset before any real data,
     * and also absorbs any first-transfer glitch on the advanced-timer output. */
    for (int s = 0; s < WS2812_STRIP_COUNT; ++s) {
        ws2812_clear((ws2812_strip_t)s);
        ws2812_show((ws2812_strip_t)s);
    }
    for (int s = 0; s < WS2812_STRIP_COUNT; ++s) {
        while (ws2812_is_busy((ws2812_strip_t)s)) { }
    }
}

void ws2812_set_pixel(ws2812_strip_t strip, uint8_t index,
                      uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LEDS_PER_STRIP) return;
    ws_rgb[strip][index][0] = r;
    ws_rgb[strip][index][1] = g;
    ws_rgb[strip][index][2] = b;
}

void ws2812_clear(ws2812_strip_t strip)
{
    memset(ws_rgb[strip], 0, sizeof(ws_rgb[strip]));
}

static void ws_encode(ws2812_strip_t strip)
{
    uint16_t *p = ws_buf[strip];
    for (int i = 0; i < WS_LEAD_SLOTS; ++i) {
        *p++ = 0;
    }
    for (int led = 0; led < WS2812_LEDS_PER_STRIP; ++led) {
        /* WS2812B wire order is GRB, MSB first. */
        uint8_t g = ws_rgb[strip][led][1];
        uint8_t r = ws_rgb[strip][led][0];
        uint8_t b = ws_rgb[strip][led][2];
        uint32_t grb = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
        for (int bit = 23; bit >= 0; --bit) {
            *p++ = (grb & (1u << bit)) ? WS_CMP_1 : WS_CMP_0;
        }
    }
    for (int i = 0; i < WS_RESET_SLOTS; ++i) {
        *p++ = 0;
    }
}

bool ws2812_show(ws2812_strip_t strip)
{
    if (ws_busy[strip]) return false;
    ws_encode(strip);
    ws_busy[strip] = true;

    /* If you ever enable D-cache (SCB_EnableDCache), uncomment this so the
     * encoded buffer is visible to DMA. Stock H743 CubeMX leaves D-cache off,
     * and calling this on some setups has been implicated in imprecise bus faults.
     *   SCB_CleanDCache_by_Addr((uint32_t *)ws_buf[strip], sizeof(ws_buf[strip]));
     */

    /* Zero this channel's compare before enabling it, so the line stays low
     * until DMA loads the first real sample. Without this the first frame
     * picks up whatever value was in CCR and shifts all bits -> random colors. */
    __HAL_TIM_SET_COMPARE(ws_htim, ws_channel[strip], 0);

    /* HAL marks the per-channel state BUSY at Start_DMA and only clears it in
     * HAL_TIM_PWM_Stop_DMA. Since we deliberately don't call Stop_DMA, force
     * the state back to READY here so Start_DMA isn't rejected as HAL_BUSY. */
    if (strip == WS2812_STRIP_0) {
        ws_htim->State = HAL_TIM_STATE_READY;
        ws_htim->ChannelState[TIM_CHANNEL_1 >> 2] = HAL_TIM_CHANNEL_STATE_READY;
    } else {
        ws_htim->State = HAL_TIM_STATE_READY;
        ws_htim->ChannelState[TIM_CHANNEL_2 >> 2] = HAL_TIM_CHANNEL_STATE_READY;
    }

    if (HAL_TIM_PWM_Start_DMA(ws_htim, ws_channel[strip],
                              (uint32_t *)ws_buf[strip],
                              WS_BUFFER_LEN) != HAL_OK) {
        ws_busy[strip] = false;
        return false;
    }
    return true;
}

bool ws2812_is_busy(ws2812_strip_t strip)
{
    return ws_busy[strip];
}

/* Integer HSV -> RGB. hue in [0,360), s=255 (full), v=0..255.
 * Classic 6-sector formulation; kept in u8 arithmetic so it's cheap. */
static void hsv_to_rgb(uint16_t hue, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    hue %= 360;
    uint16_t sector = hue / 60;           /* 0..5 */
    uint16_t frac   = (hue - sector * 60) * 255 / 60; /* 0..255 */
    uint8_t  p = 0;
    uint8_t  q = (uint8_t)((uint16_t)v * (255 - frac) / 255);
    uint8_t  t = (uint8_t)((uint16_t)v * frac / 255);

    switch (sector) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default:*r = v; *g = p; *b = q; break;
    }
}

void ws2812_set_pixel_hsv(ws2812_strip_t strip, uint8_t index,
                          uint16_t hue, uint8_t value)
{
    uint8_t r, g, b;
    hsv_to_rgb(hue, value, &r, &g, &b);
    ws2812_set_pixel(strip, index, r, g, b);
}

/* 16-entry quarter-sine table (0..32767). We reflect/invert it to cover a full
 * period, so effective resolution is 64 steps of a smooth sine — plenty for
 * eye-smooth color morph on 3 LEDs. */
static const uint16_t sine_q[16] = {
        0,  3212,  6393,  9512, 12539, 15446, 18204, 20787,
    23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609
};

/* sine(x) where x wraps every 65536 -> returns 0..65535. */
static uint16_t sine16(uint16_t x)
{
    uint8_t  quadrant = (x >> 14) & 0x3;     /* 0..3 */
    uint16_t idx_frac = x & 0x3FFF;          /* 0..16383 within quadrant */
    uint8_t  idx      = idx_frac >> 10;      /* 0..15 */
    uint16_t frac     = idx_frac & 0x3FF;    /* 0..1023 for linear interp */

    uint16_t a, b;
    switch (quadrant) {
        case 0: { /* 0..pi/2 : rising 0 -> 32767 */
            a = sine_q[idx];
            b = (idx == 15) ? 32767 : sine_q[idx + 1];
            uint16_t v = a + (uint16_t)((uint32_t)(b - a) * frac / 1024);
            return (uint16_t)(32768 + v);          /* shift into 0..65535 (centered) */
        }
        case 1: { /* pi/2..pi : falling 32767 -> 0 */
            uint8_t j = 15 - idx;
            a = (idx == 0) ? 32767 : sine_q[j + 1];
            b = sine_q[j];
            uint16_t v = a + (uint16_t)(((int32_t)b - (int32_t)a) * (int32_t)frac / 1024);
            return (uint16_t)(32768 + v);
        }
        case 2: { /* pi..3pi/2 : falling 0 -> -32767 */
            a = sine_q[idx];
            b = (idx == 15) ? 32767 : sine_q[idx + 1];
            uint16_t v = a + (uint16_t)((uint32_t)(b - a) * frac / 1024);
            return (uint16_t)(32767 - v);
        }
        default: { /* 3pi/2..2pi : rising -32767 -> 0 */
            uint8_t j = 15 - idx;
            a = (idx == 0) ? 32767 : sine_q[j + 1];
            b = sine_q[j];
            uint16_t v = a + (uint16_t)(((int32_t)b - (int32_t)a) * (int32_t)frac / 1024);
            return (uint16_t)(32767 - v);
        }
    }
}

/* ============================================================
 *  Mode engine — set a mode, then just call ws2812_tick() from
 *  your main loop. tick() throttles itself to 10 ms so you can
 *  call it as often as you like.
 * ============================================================ */

typedef enum {
    MODE_OFF = 0,
    MODE_SOLID,
    MODE_MORPH,
    MODE_PULSE,
    MODE_LOADING,
    MODE_HUE_CYCLE,
} ws_mode_t;

static volatile ws_mode_t ws_mode = MODE_OFF;

/* Per-mode parameters, packed in a small union-ish struct. */
static struct {
    uint8_t         r, g, b;       /* SOLID / PULSE */
    uint16_t        hue;           /* MORPH         */
    const uint16_t *hues;          /* HUE_CYCLE     */
    uint8_t         n_hues;        /* HUE_CYCLE     */
} ws_param;

static uint32_t ws_last_tick = 0;
static uint32_t ws_mode_start = 0;

/* Blend: snapshot of pixel values at the moment of a mode switch.
 * For WS_BLEND_MS after a transition, rendered output is crossfaded
 * from this snapshot to the new mode using a sine ease-in curve. */
#define WS_BLEND_MS  900
static uint8_t ws_blend_from[WS2812_STRIP_COUNT][WS2812_LEDS_PER_STRIP][3];


/* ---- mode setters ---- */

static void ws_switch_mode(ws_mode_t m)
{
    /* Snapshot current pixel state so we can crossfade from it. */
    for (int s = 0; s < WS2812_STRIP_COUNT; ++s)
        for (int i = 0; i < WS2812_LEDS_PER_STRIP; ++i)
            for (int c = 0; c < 3; ++c)
                ws_blend_from[s][i][c] = ws_rgb[s][i][c];

    ws_mode = m;
    ws_mode_start = HAL_GetTick();
    ws_last_tick = 0;               /* force immediate redraw on next tick */
}

void ws2812_mode_off(void)
{
    ws_switch_mode(MODE_OFF);
}

void ws2812_mode_solid(uint8_t r, uint8_t g, uint8_t b)
{
    ws_param.r = r; ws_param.g = g; ws_param.b = b;
    ws_switch_mode(MODE_SOLID);
}

void ws2812_mode_morph(uint16_t hue)
{
    ws_param.hue = hue % 360;
    ws_switch_mode(MODE_MORPH);
}
void ws2812_mode_morph_red(void)   { ws2812_mode_morph(0);   }
void ws2812_mode_morph_green(void) { ws2812_mode_morph(120); }
void ws2812_mode_morph_blue(void)  { ws2812_mode_morph(240); }

void ws2812_mode_pulse(uint8_t r, uint8_t g, uint8_t b)
{
    ws_param.r = r; ws_param.g = g; ws_param.b = b;
    ws_switch_mode(MODE_PULSE);
}
void ws2812_mode_pulse_red(void) { ws2812_mode_pulse(255, 0, 0); }

void ws2812_mode_loading(void)
{
    ws_switch_mode(MODE_LOADING);
}

/* ---- hue-cycle palettes ----
 * Each entry is a hue in degrees (0..359). We interpolate between consecutive
 * entries along the shortest path around the color wheel and wrap at the end. */
static const uint16_t palette_blues[]  = { 240, 275, 300 };   /* blue, lila, purple */
static const uint16_t palette_warms[]  = {   0,  25, 280 };   /* red, orange, violet */
static const uint16_t palette_greens[] = { 120,  90,  60 };   /* green, lime, yellow */

void ws2812_mode_hue_cycle(const uint16_t *hues, uint8_t count)
{
    if (hues == NULL || count == 0) return;
    ws_param.hues   = hues;
    ws_param.n_hues = count;
    ws_switch_mode(MODE_HUE_CYCLE);
}

void ws2812_mode_cycle_blues(void)
{
    ws2812_mode_hue_cycle(palette_blues,
                          sizeof(palette_blues) / sizeof(palette_blues[0]));
}
void ws2812_mode_cycle_warms(void)
{
    ws2812_mode_hue_cycle(palette_warms,
                          sizeof(palette_warms) / sizeof(palette_warms[0]));
}
void ws2812_mode_cycle_greens(void)
{
    ws2812_mode_hue_cycle(palette_greens,
                          sizeof(palette_greens) / sizeof(palette_greens[0]));
}

/* ---- mode renderers ---- */

static void render_solid(void)
{
    for (int s = 0; s < WS2812_STRIP_COUNT; ++s) {
        for (int i = 0; i < WS2812_LEDS_PER_STRIP; ++i) {
            ws2812_set_pixel((ws2812_strip_t)s, i,
                             ws_param.r, ws_param.g, ws_param.b);
        }
    }
}

/* Morph: hue drifts between the base color and a deeper adjacent shade
 * (~35 deg further around the wheel toward the "dark" side), staying at
 * full brightness so the shift reads as a color change, not dimming.
 * 4 s cycle, sine-eased so it lingers at each end and glides through the middle. */
#define WS_MORPH_HUE_SHIFT  12      /* degrees toward the adjacent dark shade */
#define WS_MORPH_PERIOD_MS  4000UL

static uint16_t hue_lerp(uint16_t a, uint16_t b, uint8_t mix);

static void render_morph(uint32_t ms)
{
    uint16_t phase16 = (uint16_t)((ms * 65536UL / WS_MORPH_PERIOD_MS) & 0xFFFF);
    uint16_t s16 = sine16(phase16);   /* 0..65535 */
    /* mix 0..255: how far we've drifted toward the shifted hue. */
    uint8_t mix = (uint8_t)((uint32_t)s16 * 255 / 65535);

    uint16_t hue = hue_lerp(ws_param.hue,
                            (uint16_t)((ws_param.hue + WS_MORPH_HUE_SHIFT) % 360),
                            mix);

    for (int s = 0; s < WS2812_STRIP_COUNT; ++s) {
        for (int i = 0; i < WS2812_LEDS_PER_STRIP; ++i) {
            ws2812_set_pixel_hsv((ws2812_strip_t)s, i, hue, 210);
        }
    }
}

/* Pulse: sharper breathing on a fixed RGB color, floor = 0 (fully off at trough). */
static void render_pulse(uint32_t ms)
{
    uint16_t phase16 = (uint16_t)((ms * 65536UL / 1500UL) & 0xFFFF); /* 1.5 s */
    uint16_t s16 = sine16(phase16);
    uint32_t mix = s16;                      /* 0..65535 */

    uint8_t r = (uint8_t)((uint32_t)ws_param.r * mix / 65535);
    uint8_t g = (uint8_t)((uint32_t)ws_param.g * mix / 65535);
    uint8_t b = (uint8_t)((uint32_t)ws_param.b * mix / 65535);

    for (int s = 0; s < WS2812_STRIP_COUNT; ++s) {
        for (int i = 0; i < WS2812_LEDS_PER_STRIP; ++i) {
            ws2812_set_pixel((ws2812_strip_t)s, i, r, g, b);
        }
    }
}

/* Loading comet path: 6 positions, looping.
 *   0..2 -> strip 0 LED 0..2
 *   3..5 -> strip 1 LED 2..0   (comes back down the second strip)
 * Trail: head = 100 %, head-1 = 25 %, head-2 = 1 %. */
#define LOAD_PATH_LEN   (2 * WS2812_LEDS_PER_STRIP)   /* = 6 */
#define LOAD_STEP_MS    150                            /* comet speed */

static void load_pos_to_sl(uint8_t pos, ws2812_strip_t *strip, uint8_t *led)
{
    if (pos < WS2812_LEDS_PER_STRIP) {
        *strip = WS2812_STRIP_0;
        *led   = pos;                               /* 0,1,2 */
    } else {
        *strip = WS2812_STRIP_1;
        *led   = (uint8_t)(LOAD_PATH_LEN - 1 - pos); /* 2,1,0 */
    }
}

static void render_loading(uint32_t ms)
{
    /* Clear by writing directly so we can additive-blend below. */
    for (int s = 0; s < WS2812_STRIP_COUNT; ++s)
        for (int i = 0; i < WS2812_LEDS_PER_STRIP; ++i)
            ws_rgb[s][i][0] = ws_rgb[s][i][1] = ws_rgb[s][i][2] = 0;

    uint32_t step = ms / LOAD_STEP_MS;
    /* frac 0..255: how far between the current step and the next. */
    uint8_t  frac = (uint8_t)((ms % LOAD_STEP_MS) * 255UL / LOAD_STEP_MS);

    /* Head=255, trail-1=64, trail-2=8. */
    static const uint8_t trail_scale[3] = { 255, 64, 8 };

    for (int k = 0; k < 3; ++k) {
        /* Outgoing position (fading out as frac rises). */
        uint8_t pos_a = (uint8_t)((step + LOAD_PATH_LEN - k) % LOAD_PATH_LEN);
        /* Incoming position (fading in). */
        uint8_t pos_b = (uint8_t)((step + 1 + LOAD_PATH_LEN - k) % LOAD_PATH_LEN);

        uint8_t ba = (uint8_t)((uint16_t)trail_scale[k] * (255 - frac) / 255);
        uint8_t bb = (uint8_t)((uint16_t)trail_scale[k] * frac         / 255);

        ws2812_strip_t sa, sb; uint8_t la, lb;
        load_pos_to_sl(pos_a, &sa, &la);
        load_pos_to_sl(pos_b, &sb, &lb);

        /* Additive blend with saturation — keeps overlapping trail pixels bright. */
        uint8_t *pa = &ws_rgb[sa][la][2];
        *pa = (uint8_t)((*pa > 255 - ba) ? 255 : *pa + ba);
        uint8_t *pb = &ws_rgb[sb][lb][2];
        *pb = (uint8_t)((*pb > 255 - bb) ? 255 : *pb + bb);
    }
}

/* Shortest-path hue interpolation: moves the short way around the 360 deg wheel.
 * a, b in [0,360). mix in [0,255]. Returns a hue in [0,360). */
static uint16_t hue_lerp(uint16_t a, uint16_t b, uint8_t mix)
{
    int32_t diff = (int32_t)b - (int32_t)a;
    if (diff >  180) diff -= 360;       /* go the short way */
    if (diff < -180) diff += 360;
    int32_t h = (int32_t)a + diff * mix / 255;
    if (h < 0)    h += 360;
    if (h >= 360) h -= 360;
    return (uint16_t)h;
}

static void render_hue_cycle(uint32_t ms)
{
    if (ws_param.hues == NULL || ws_param.n_hues == 0) return;

    const uint32_t seg_ms   = 2500;                         /* 2.5 s per hop */
    const uint32_t total_ms = seg_ms * ws_param.n_hues;
    uint32_t t = ms % total_ms;

    uint8_t  seg     = (uint8_t)(t / seg_ms);               /* which hop */
    uint32_t in_seg  = t - (uint32_t)seg * seg_ms;          /* 0..seg_ms-1 */

    uint16_t hue_a = ws_param.hues[seg];
    uint16_t hue_b = ws_param.hues[(seg + 1U) % ws_param.n_hues];

    /* Sine-eased blend factor 0..255 so color doesn't change at constant rate -
     * it lingers near each waypoint and accelerates through the middle. */
    uint16_t phase16 = (uint16_t)((in_seg * 16384UL) / seg_ms);  /* 0..16383 = 0..pi/2 */
    uint16_t s16 = sine16(phase16);                              /* 32768..65535 on 0..pi/2 */
    uint8_t  mix = (uint8_t)(((s16 - 32768U) * 255U) / 32767U);  /* 0..255 eased */

    uint16_t hue = hue_lerp(hue_a, hue_b, mix);

    for (int s = 0; s < WS2812_STRIP_COUNT; ++s) {
        for (int i = 0; i < WS2812_LEDS_PER_STRIP; ++i) {
            ws2812_set_pixel_hsv((ws2812_strip_t)s, i, hue, 230);
        }
    }
}

/* ---- tick ---- */

void ws2812_tick(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - ws_last_tick) < 10) return;   /* rate-limit to 100 Hz */
    ws_last_tick = now;

    uint32_t ms_in_mode = now - ws_mode_start;

    switch (ws_mode) {
        case MODE_OFF:
            ws2812_clear(WS2812_STRIP_0);
            ws2812_clear(WS2812_STRIP_1);
            break;
        case MODE_SOLID:     render_solid();               break;
        case MODE_MORPH:     render_morph(ms_in_mode);     break;
        case MODE_PULSE:     render_pulse(ms_in_mode);     break;
        case MODE_LOADING:   render_loading(ms_in_mode);   break;
        case MODE_HUE_CYCLE: render_hue_cycle(ms_in_mode); break;
    }

    /* Crossfade: for the first WS_BLEND_MS after any mode switch, linearly
     * interpolate (sine-eased) from the pre-switch snapshot to the new output.
     * This runs in-place on ws_rgb before DMA so no extra buffer is needed. */
    if (ms_in_mode < WS_BLEND_MS) {
        /* Quarter-sine easing: starts slow, accelerates into the new mode. */
        uint16_t phase16 = (uint16_t)((ms_in_mode * 16384UL) / WS_BLEND_MS); /* 0..16383 */
        uint16_t s16 = sine16(phase16);                    /* 32768..65535 over 0..pi/2 */
        uint8_t  t   = (uint8_t)(((s16 - 32768U) * 255U) / 32767U); /* 0..255 */

        for (int s = 0; s < WS2812_STRIP_COUNT; ++s) {
            for (int i = 0; i < WS2812_LEDS_PER_STRIP; ++i) {
                for (int c = 0; c < 3; ++c) {
                    int32_t from = ws_blend_from[s][i][c];
                    int32_t to   = ws_rgb[s][i][c];
                    ws_rgb[s][i][c] = (uint8_t)(from + (to - from) * t / 255);
                }
            }
        }
    }

    /* Push both frames. If the previous DMA hasn't finished yet (we got called
     * again too soon), ws2812_show returns false and we simply skip this frame;
     * next tick will try again. No blocking, no busy-wait. */
    ws2812_show(WS2812_STRIP_0);
    ws2812_show(WS2812_STRIP_1);
}

/* TC callback — one full frame (data + reset slots) sent.
 * DMA is in Normal mode, so the stream has already disabled itself and
 * the final CCR value is 0 (tail reset slots) -> line stays low.
 * We deliberately do NOT call HAL_TIM_PWM_Stop_DMA here: doing so from
 * inside the DMA ISR aborts DMA, toggles MOE and the counter, and leaves
 * htim->State in a racy mid-transition where the next Start_DMA can be
 * rejected as HAL_BUSY (manifests as LEDs randomly not lighting up). */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != ws_htim) return;
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        ws_busy[WS2812_STRIP_0] = false;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        ws_busy[WS2812_STRIP_1] = false;
    }
}
