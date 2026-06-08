/*
 * loko_states.c  --  FSM state handlers and jump-table dispatch
 *
 * Each state handler is responsible for:
 *   1. Setting the active flag on each leg (which legs participate).
 *   2. Calling loko_advance_phases() and/or loko_compute_foot_targets().
 *   3. Calling loko_solve_and_write() if st->enabled.
 *
 * Transition logic is NOT here — the caller owns that via loko_set_state().
 */

#include "loko_states.h"
#include "loko_gait.h"
#include "loko_servo.h"
#include "loko_config.h"
#include "loko_input.h"
#include "loko_stabilizer.h"
#include "main.h"
#include "loko_tilt.h"
#include "uart_protocol.h"
#include "imu.h"
#include "ws2812b.h"
#include "hexapod/Controller.hpp"

/* ── 4-leg mode state ──────────────────────────────────────────────────── */

typedef struct {
    int mode_hands;
    int old_R2;
    int recharge;
    float coordinate_RF[3];
    float coordinate_LF[3];
} Loko4LegState;

static Loko4LegState leg4_state = {
    0,                                             /* mode_hands */
    0,                                             /* old_R2 */
    0,                                             /* recharge */
    {NEUTRAL_REACH_MM, 0.0f, LOKO_FRONT_RAISE_MM}, /* coordinate_RF */
    {NEUTRAL_REACH_MM, 0.0f, LOKO_FRONT_RAISE_MM}  /* coordinate_LF */
};

/* IMU-based stabilizer */
static LokoStabilizerConfig stabilizer_cfg;
static float s_stabilizer_roll    = 0.0f;
static float s_stabilizer_pitch   = 0.0f;
static float s_stabilizer_shift_x = 0.0f;  /* mm, STAB_STABLE only */
static float s_stabilizer_shift_y = 0.0f;  /* mm, STAB_STABLE only */

void loko_reset_4leg_mode(void)
{
    leg4_state.mode_hands = 0;
    leg4_state.old_R2 = 0;
    leg4_state.recharge = 0;
    leg4_state.coordinate_RF[0] = NEUTRAL_REACH_MM;
    leg4_state.coordinate_RF[1] = 0.0f;
    leg4_state.coordinate_RF[2] = LOKO_FRONT_RAISE_MM;
    leg4_state.coordinate_LF[0] = NEUTRAL_REACH_MM;
    leg4_state.coordinate_LF[1] = 0.0f;
    leg4_state.coordinate_LF[2] = LOKO_FRONT_RAISE_MM;
}

/* ── State handlers ──────────────────────────────────────────────────────── */

/* UN_ARMED — servos not driven */
static void state_un_armed(LokoState *st, const LokoInput *in, float dt)
{
    (void)st; (void)in; (void)dt;
}

/* STAND — all six legs held at neutral stance */
static void state_stand(LokoState *st, const LokoInput *in, float dt)
{
    for (int i = 0; i < LOKO_NUM_LEGS; i++) {
        st->legs[i].active = 1;
        st->legs[i].target.x = st->legs[i].neutral_x;
        st->legs[i].target.y = st->legs[i].neutral_y;
        st->legs[i].target.z = 0.0f;
        st->legs[i].target_is_body_frame = 0;
    }
    loko_solve_and_write(st);
}

/* WALK — six-leg tripod, six-leg wave, or four-leg walk depending on gait_mode */
static void state_walk(LokoState *st, const LokoInput *in, float dt)
{
    /* Phase offsets indexed by leg (FR=0 MR=1 BR=2 BL=3 ML=4 FL=5).
     *
     * RIPPLE: 3 diagonal pairs swing in sequence, each pair offset by 1/3.
     *   Pair A FR(0)+BL(3): phase 0.000  → MR,BR,ML,FL on ground while they swing
     *   Pair B BR(2)+FL(5): phase 0.333  → FR,MR,BL,ML on ground while they swing
     *   Pair C MR(1)+ML(4): phase 0.667  → FR,BR,BL,FL on ground while they swing */
    static const float tripod_offsets[LOKO_NUM_LEGS] = {
        0.0f,        0.5f,        0.0f,        0.5f,        0.0f,        0.5f };
    static const float wave_offsets[LOKO_NUM_LEGS] = {
        0.0f,  1.0f/6, 2.0f/6, 3.0f/6, 4.0f/6, 5.0f/6 };
    static const float ripple_offsets[LOKO_NUM_LEGS] = {
        0.0f,  2.0f/3, 1.0f/3, 0.0f,   2.0f/3, 1.0f/3 };

    const float *offsets;
    float traj_h;
    switch (st->gait_mode) {
    case GAIT_WAVE:
        st->duty_factor = LOKO_DUTY_WAVE;
        offsets = wave_offsets;
        st->body_height = CHASSIS_TO_SHOULDER + DESIRED_BELLY_CLEARANCE;
        traj_h = LOKO_TRAJ_H;
        break;
    case GAIT_RIPPLE:
        st->duty_factor = LOKO_DUTY_RIPPLE;
        offsets = ripple_offsets;
        st->body_height = CHASSIS_TO_SHOULDER + DESIRED_BELLY_CLEARANCE;
        traj_h = LOKO_TRAJ_H;
        break;
    case GAIT_OBSTACLE: /* tripod motion, body raised, swing taller — for rough terrain */
        st->duty_factor = LOKO_DUTY_TRIPOD;
        offsets = tripod_offsets;
        st->body_height = CHASSIS_TO_SHOULDER + DESIRED_BELLY_CLEARANCE + LOKO_OBSTACLE_BODY_EXTRA_MM;
        traj_h = LOKO_TRAJ_H + LOKO_OBSTACLE_SWING_EXTRA_MM;
        break;
    default: /* GAIT_TRIPOD */
        st->duty_factor = LOKO_DUTY_TRIPOD;
        offsets = tripod_offsets;
        st->body_height = CHASSIS_TO_SHOULDER + DESIRED_BELLY_CLEARANCE;
        traj_h = LOKO_TRAJ_H;
        break;
    }

    for (int i = 0; i < LOKO_NUM_LEGS; i++) {
        st->legs[i].active = 1;
        st->legs[i].phase_offset = offsets[i];
    }

    loko_advance_phases(st, in->vx, in->vy, in->wz, dt);
    loko_compute_foot_targets(st, in->vx, in->vy, in->wz, traj_h);
    loko_solve_and_write(st);
}

/* WALK_4_LEGS — mid and rear legs walk; front pair can do "hit" on R2 with L3/R3 selection */
static void state_walk_4_legs(LokoState *st, const LokoInput *in, float dt)
{
    // const UART_ControllerState_t *ctrl = UART_GetController();

    /* 4 active legs → 1 airborne at any moment → β = 3/4 */
    st->duty_factor = LOKO_DUTY_4LEG;

    /* Wave phase offsets spread the 4 active legs evenly (0, 0.25, 0.5, 0.75) */
    static const float wave_phases[6] = { 0, 0.0f, 0.25f, 0.50f, 0.75f, 0 };

    /* Always enable walking for middle/rear legs */
    for (int i = 0; i < LOKO_NUM_LEGS; i++) {
        st->legs[i].active = (i == 0 || i == 5) ? 0 : 1;
        if (st->legs[i].active) {
            st->legs[i].phase_offset = wave_phases[i];
        }
    }

    /* Run walk logic for legs 1,2,3,4 */
    loko_advance_phases(st, in->vx, in->vy, in->wz, dt);
    loko_compute_foot_targets(st, in->vx, in->vy, in->wz, LOKO_TRAJ_H);

    /* Mid legs (1, 4) get forward offset for stability */
    st->legs[1].target.y += 100.0f;
    st->legs[4].target.y += 100.0f;

    /* Front legs (0, 5) stay at neutral by default */
    st->legs[0].target.x = st->legs[0].neutral_x;
    st->legs[0].target.y = st->legs[0].neutral_y;
    st->legs[0].target.z = LOKO_FRONT_RAISE_MM;
    st->legs[0].target_is_body_frame = 0;

    st->legs[5].target.x = st->legs[5].neutral_x;
    st->legs[5].target.y = st->legs[5].neutral_y;
    st->legs[5].target.z = LOKO_FRONT_RAISE_MM;
    st->legs[5].target_is_body_frame = 0;

    /* Mode switching logic: use 'A' button (BTN_CROSS) to cycle hand modes */
    {
        Controller ctl(st->pad);
        if (ctl.pressed(BTN_CROSS)) {
            if (leg4_state.recharge) {
                if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) leg4_state.coordinate_RF[0] -= 100;
                if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) leg4_state.coordinate_LF[0] -= 100;
                leg4_state.recharge = 0;
            }
            leg4_state.mode_hands = (leg4_state.mode_hands + 1) % 3;
        }

        /* Hit motion on selected leg(s) using LT (BTN_L2) or RT (BTN_R2) */
        int hit_active = ctl.held(BTN_L2) || ctl.held(BTN_R2);
        if (hit_active && !leg4_state.recharge) {
            if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) {
                leg4_state.coordinate_RF[0] += 100;
            }
            if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) {
                leg4_state.coordinate_LF[0] += 100;
            }
            leg4_state.recharge = 1;
        } else if (!hit_active && leg4_state.recharge) {
            if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) {
                leg4_state.coordinate_RF[0] -= 100;
            }
            if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) {
                leg4_state.coordinate_LF[0] -= 100;
            }
            leg4_state.recharge = 0;
        }

        /* Apply manually controlled leg positions (only selected ones) */
        if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) {
            loko_set_foot_local(st, 0, leg4_state.coordinate_RF[0], leg4_state.coordinate_RF[1], leg4_state.coordinate_RF[2]);
        }
        if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) {
            loko_set_foot_local(st, 5, leg4_state.coordinate_LF[0], leg4_state.coordinate_LF[1], leg4_state.coordinate_LF[2]);
        }

        leg4_state.old_R2 = hit_active;
    }
    loko_solve_and_write(st);
}


/* STAND_4_LEGS — stand with front pair (FR=0, FL=5) raised */
static void state_stand_4_legs(LokoState *st, const LokoInput *in, float dt)
{
    const UART_ControllerState_t *ctrl = UART_GetController();

    for (int i = 0; i < LOKO_NUM_LEGS; i++) {
        st->legs[i].active = 0; /* Nobody walks */
        st->legs[i].target.x = st->legs[i].neutral_x;
        st->legs[i].target.y = st->legs[i].neutral_y;
        if (i == 0 || i == 5) {
            st->legs[i].target.z = LOKO_FRONT_RAISE_MM;  /* raised above ground */
        } else {
            st->legs[i].target.z = 0.0f;  /* foot on ground */
        }
        st->legs[i].target_is_body_frame = 0;
    }

    /* Mid legs (1, 4) get forward offset for stability */
    st->legs[1].target.y += 100.0f;
    st->legs[4].target.y += 100.0f;

    Controller ctl(st->pad);

    /* Mode switching logic: use 'A' button (BTN_CROSS) to cycle hand modes */
    if (ctl.pressed(BTN_CROSS)) {
        if (leg4_state.recharge) {
            if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) leg4_state.coordinate_RF[0] -= 100;
            if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) leg4_state.coordinate_LF[0] -= 100;
            leg4_state.recharge = 0;
        }
        leg4_state.mode_hands = (leg4_state.mode_hands + 1) % 3;
    }

    /* Hit motion on selected leg(s) using LT (BTN_L2) or RT (BTN_R2) */
    int hit_active = ctl.held(BTN_L2) || ctl.held(BTN_R2);
    if (hit_active && !leg4_state.recharge) {
        if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) leg4_state.coordinate_RF[0] += 100;
        if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) leg4_state.coordinate_LF[0] += 100;
        leg4_state.recharge = 1;
    } else if (!hit_active && leg4_state.recharge) {
        if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) leg4_state.coordinate_RF[0] -= 100;
        if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) leg4_state.coordinate_LF[0] -= 100;
        leg4_state.recharge = 0;
    }

    /* D-pad and Stick adjustments for the selected leg(s) (only when not hitting) */
    if (!leg4_state.recharge) {
        if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) {
            if (ctrl->dpad_x < 0) {leg4_state.coordinate_RF[1] += -0.5f; }
            if (ctrl->dpad_x > 0) {leg4_state.coordinate_RF[1] +=  0.5f; }
            if (ctrl->dpad_y > 0) {leg4_state.coordinate_RF[2] +=  0.5f; }
            if (ctrl->dpad_y < 0) {leg4_state.coordinate_RF[2] += -0.5f; }
            leg4_state.coordinate_RF[0] += 0.5f * ctrl->right_stick_y;
        }
        if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) {
            if (ctrl->dpad_x < 0) {leg4_state.coordinate_LF[1] += -0.5f; }
            if (ctrl->dpad_x > 0) {leg4_state.coordinate_LF[1] +=  0.5f; }
            if (ctrl->dpad_y > 0) {leg4_state.coordinate_LF[2] +=  0.5f; }
            if (ctrl->dpad_y < 0) {leg4_state.coordinate_LF[2] += -0.5f; }
            leg4_state.coordinate_LF[0] += 0.5f * ctrl->right_stick_y;
        }
    }

    /* Apply positions to the leg IK pipeline */
    if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) {
        loko_set_foot_local(st, 0, leg4_state.coordinate_RF[0], leg4_state.coordinate_RF[1], leg4_state.coordinate_RF[2]);
    }
    if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) {
        loko_set_foot_local(st, 5, leg4_state.coordinate_LF[0], leg4_state.coordinate_LF[1], leg4_state.coordinate_LF[2]);
    }

    leg4_state.old_R2 = hit_active;

    loko_solve_and_write(st);
}

/* ROTATE_IN_PLACE — pure spin; vx and vy are ignored */
static void state_rotate_in_place(LokoState *st, const LokoInput *in, float dt)
{
    st->duty_factor = LOKO_DUTY_TRIPOD;
    for (int i = 0; i < LOKO_NUM_LEGS; i++) {
        st->legs[i].active = 1;
    }
    loko_advance_phases(st, 0.0f, 0.0f, in->wz, dt);
    loko_compute_foot_targets(st, 0.0f, 0.0f, in->wz, LOKO_TRAJ_H);
    loko_solve_and_write(st);
}

/* LOOK_AROUND — hold R2 to pan/tilt body with right stick; stabilizer suppressed */
static void state_look_around(LokoState *st, const LokoInput *in, float dt)
{
    (void)in; (void)dt;
    const UART_ControllerState_t *ctrl = UART_GetController();

    const float max_yaw_rad   = LOOK_MAX_YAW_DEG   * (3.14159f / 180.0f);
    const float max_pitch_rad = LOOK_MAX_PITCH_DEG * (3.14159f / 180.0f);

    st->body_yaw       =  ctrl->right_stick_x * max_yaw_rad;
    s_stabilizer_pitch = ctrl->right_stick_y * max_pitch_rad;

    for (int i = 0; i < LOKO_NUM_LEGS; i++) {
        st->legs[i].active = 1;
        st->legs[i].target.x = st->legs[i].neutral_x;
        st->legs[i].target.y = st->legs[i].neutral_y;
        st->legs[i].target.z = 0.0f;
        st->legs[i].target_is_body_frame = 0;
    }
    loko_solve_and_write(st);
}

/* DANCING — stub for a user-defined motion sequence */
static void state_dancing(LokoState *st, const LokoInput *in, float dt)
{
    /* Hold stand for now */
    state_stand(st, in, dt);
}

/* ── Jump table ──────────────────────────────────────────────────────────── */

typedef void (*StateHandler)(LokoState *, const LokoInput *, float);

static const StateHandler state_table[LOKO_STATE_COUNT] = {
    state_un_armed,        /* LOKO_UN_ARMED */
    state_stand,           /* LOKO_STAND */
    state_walk,            /* LOKO_WALK */
    state_walk_4_legs,     /* LOKO_WALK_4_LEGS */
    state_stand_4_legs,    /* LOKO_STAND_4_LEGS */
    state_rotate_in_place, /* LOKO_ROTATE_IN_PLACE */
    state_dancing,         /* LOKO_DANCING */
    state_look_around,     /* LOKO_LOOK_AROUND */
};

/* Update LED mode on FSM state or gait mode change — called once per transition */
static void loko_update_leds(LokoFSMState state, LokoGaitMode gait)
{
    switch (state) {
    case LOKO_UN_ARMED:
        ws2812_mode_loading();
        break;
    case LOKO_STAND:
        ws2812_mode_morph_blue();
        break;
    case LOKO_WALK:
        switch (gait) {
        case GAIT_WAVE:     ws2812_mode_cycle_greens();    break;
        case GAIT_RIPPLE:   ws2812_mode_cycle_warms();     break;
        case GAIT_OBSTACLE: ws2812_mode_solid(255, 80, 0); break; /* orange — obstacle mode */
        default:            ws2812_mode_cycle_blues();     break;
        }
        break;
    case LOKO_WALK_4_LEGS:
        ws2812_mode_morph_green();
        break;
    case LOKO_STAND_4_LEGS:
        ws2812_mode_morph_red();
        break;
    case LOKO_ROTATE_IN_PLACE:
        ws2812_mode_solid(0, 255, 255);
        break;
    case LOKO_DANCING:
        ws2812_mode_morph(300);   /* purple */
        break;
    case LOKO_LOOK_AROUND:
        ws2812_mode_solid(255, 128, 0);   /* orange */
        break;
    default:
        break;
    }
}

void loko_dispatch(LokoState *st, const LokoInput *in, float dt)
{
    /* Initialize stabilizer on first dispatch */
    static uint8_t stabilizer_initialized = 0;
    if (!stabilizer_initialized) {
        loko_stabilizer_init(&stabilizer_cfg);
        stabilizer_initialized = 1;
    }

    /* Detect state or gait change and update LEDs once */
    static LokoFSMState prev_state    = (LokoFSMState)-1;
    static LokoGaitMode prev_gait     = (LokoGaitMode)-1;
    if (st->state != prev_state || st->gait_mode != prev_gait) {
        loko_update_leds(st->state, st->gait_mode);

        /* Reset all leg phases on entry/exit of rotate-in-place and 4-leg modes */
        auto is_phase_reset_state = [](LokoFSMState s) {
            return s == LOKO_ROTATE_IN_PLACE ||
                   s == LOKO_WALK_4_LEGS     ||
                   s == LOKO_STAND_4_LEGS;
        };
        if (is_phase_reset_state(st->state) || is_phase_reset_state(prev_state)) {
            for (int i = 0; i < LOKO_NUM_LEGS; ++i)
                st->legs[i].phase = 0.0f;
        }

        prev_state = st->state;
        prev_gait  = st->gait_mode;
    }

    /* Read IMU and update stabilizer every tick */
    if (IMU_Read(&st->imu_data) == 0) {
        /* "Walking" = legs are cycling. While walking, the stabiliser slows
         * its LEVEL-mode correction so per-step bobbing is averaged out
         * instead of being chased every tick. */
        uint8_t is_walking = (st->state == LOKO_WALK ||
                              st->state == LOKO_WALK_4_LEGS ||
                              st->state == LOKO_ROTATE_IN_PLACE ||
                              st->state == LOKO_DANCING) ? 1u : 0u;

        loko_stabilizer_update(&stabilizer_cfg, &st->imu_data, st->gait_mode,
                               st->stab_mode, is_walking, dt,
                               &s_stabilizer_roll, &s_stabilizer_pitch,
                               &s_stabilizer_shift_x, &s_stabilizer_shift_y);
    }

    /* LOOK_AROUND: zero IMU-driven corrections; state handler writes pitch/yaw directly */
    if (st->state == LOKO_LOOK_AROUND) {
        s_stabilizer_roll    = 0.0f;
        s_stabilizer_shift_x = 0.0f;
        s_stabilizer_shift_y = 0.0f;
        /* s_stabilizer_pitch is set by state_look_around below */
    }

    state_table[st->state](st, in, dt);
}

void loko_get_stabilizer_angles(float *out_roll, float *out_pitch)
{
    if (out_roll)  *out_roll  = s_stabilizer_roll;
    if (out_pitch) *out_pitch = s_stabilizer_pitch;
}

void loko_get_stabilizer_shift(float *out_shift_x_mm, float *out_shift_y_mm)
{
    if (out_shift_x_mm) *out_shift_x_mm = s_stabilizer_shift_x;
    if (out_shift_y_mm) *out_shift_y_mm = s_stabilizer_shift_y;
}
