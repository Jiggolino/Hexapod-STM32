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
    0,
    0,
    0,
    {NEUTRAL_REACH_MM, 0.0f, LOKO_FRONT_RAISE_MM},
    {NEUTRAL_REACH_MM, 0.0f, LOKO_FRONT_RAISE_MM}
};

static LokoStabilizerConfig stabilizer_cfg;
static float s_stabilizer_roll    = 0.0f;
static float s_stabilizer_pitch   = 0.0f;
static float s_stabilizer_shift_x = 0.0f;
static float s_stabilizer_shift_y = 0.0f;

/*
 * Resets the 4-leg hand-mode state to initial position (both front legs at neutral reach).
 * Output: void
 */
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

/*
 * UN_ARMED — all servos disabled; does nothing while waiting for L1 press.
 */
static void state_un_armed(LokoState *st, const LokoInput *in, float dt)
{
    (void)st; (void)in; (void)dt;
}

/*
 * STAND — holds all six legs at their neutral stance position on the ground.
 * Drives servos every tick to maintain position against disturbances.
 * Input:  st — locomotion state
 * Output: void
 */
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

/*
 * WALK — six-leg gait (tripod, wave, ripple, or obstacle) depending on gait_mode.
 * Selects per-gait phase offsets, duty factor, body height, and swing height, then
 * advances phases, computes ICR arc foot targets, and writes servo angles.
 *
 * Ripple offsets: 3 diagonal pairs each offset by 1/3 of the cycle.
 *   FR+BL: 0.000, BR+FL: 0.333, MR+ML: 0.667
 *
 * Input:  in — velocity command {vx,vy,wz}; dt — time step
 * Output: void
 */
static void state_walk(LokoState *st, const LokoInput *in, float dt)
{
    static const float tripod_offsets[LOKO_NUM_LEGS] = {
        0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f };
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
    case GAIT_OBSTACLE:
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

/*
 * WALK_4_LEGS — middle and rear legs walk (wave gait, β = 3/4); front pair
 * (FR=0, FL=5) are raised and can be positioned manually. A button cycles
 * which front leg is in "hand" mode; L2 or R2 triggers a reach/retract motion.
 * Input:  in — velocity command; dt — time step
 * Output: void
 */
static void state_walk_4_legs(LokoState *st, const LokoInput *in, float dt)
{
    st->duty_factor = LOKO_DUTY_4LEG;

    /* Wave phases spread 4 active legs evenly (0, 0.25, 0.5, 0.75) */
    static const float wave_phases[6] = { 0, 0.0f, 0.25f, 0.50f, 0.75f, 0 };

    for (int i = 0; i < LOKO_NUM_LEGS; i++) {
        st->legs[i].active = (i == 0 || i == 5) ? 0 : 1;
        if (st->legs[i].active) {
            st->legs[i].phase_offset = wave_phases[i];
        }
    }

    loko_advance_phases(st, in->vx, in->vy, in->wz, dt);
    loko_compute_foot_targets(st, in->vx, in->vy, in->wz, LOKO_TRAJ_H);

    /* Mid legs pushed forward for stability under the shifted COM */
    st->legs[1].target.y += 100.0f;
    st->legs[4].target.y += 100.0f;

    st->legs[0].target.x = st->legs[0].neutral_x;
    st->legs[0].target.y = st->legs[0].neutral_y;
    st->legs[0].target.z = LOKO_FRONT_RAISE_MM;
    st->legs[0].target_is_body_frame = 0;

    st->legs[5].target.x = st->legs[5].neutral_x;
    st->legs[5].target.y = st->legs[5].neutral_y;
    st->legs[5].target.z = LOKO_FRONT_RAISE_MM;
    st->legs[5].target_is_body_frame = 0;

    /* BTN_CROSS cycles hand mode (0=RF only, 1=LF only, 2=both);
     * L2 or R2 triggers a +100 mm reach while held. */
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

        if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2)
            loko_set_foot_local(st, 0, leg4_state.coordinate_RF[0], leg4_state.coordinate_RF[1], leg4_state.coordinate_RF[2]);
        if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2)
            loko_set_foot_local(st, 5, leg4_state.coordinate_LF[0], leg4_state.coordinate_LF[1], leg4_state.coordinate_LF[2]);

        leg4_state.old_R2 = hit_active;
    }
    loko_solve_and_write(st);
}

/*
 * STAND_4_LEGS — stands on four legs (mid + rear) with front pair raised.
 * D-pad and right stick adjust selected front leg position; L2/R2 triggers reach.
 * Input:  in — unused; dt — unused
 * Output: void
 */
static void state_stand_4_legs(LokoState *st, const LokoInput *in, float dt)
{
    const UART_ControllerState_t *ctrl = UART_GetController();

    for (int i = 0; i < LOKO_NUM_LEGS; i++) {
        st->legs[i].active = 0;
        st->legs[i].target.x = st->legs[i].neutral_x;
        st->legs[i].target.y = st->legs[i].neutral_y;
        st->legs[i].target.z = (i == 0 || i == 5) ? LOKO_FRONT_RAISE_MM : 0.0f;
        st->legs[i].target_is_body_frame = 0;
    }

    /* Mid legs pushed forward for stability */
    st->legs[1].target.y += 100.0f;
    st->legs[4].target.y += 100.0f;

    Controller ctl(st->pad);

    if (ctl.pressed(BTN_CROSS)) {
        if (leg4_state.recharge) {
            if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) leg4_state.coordinate_RF[0] -= 100;
            if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) leg4_state.coordinate_LF[0] -= 100;
            leg4_state.recharge = 0;
        }
        leg4_state.mode_hands = (leg4_state.mode_hands + 1) % 3;
    }

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

    /* D-pad and right-stick fine control of the selected leg(s) */
    if (!leg4_state.recharge) {
        if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2) {
            if (ctrl->dpad_x < 0) leg4_state.coordinate_RF[1] += -0.5f;
            if (ctrl->dpad_x > 0) leg4_state.coordinate_RF[1] +=  0.5f;
            if (ctrl->dpad_y > 0) leg4_state.coordinate_RF[2] +=  0.5f;
            if (ctrl->dpad_y < 0) leg4_state.coordinate_RF[2] += -0.5f;
            leg4_state.coordinate_RF[0] += 0.5f * ctrl->right_stick_y;
        }
        if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2) {
            if (ctrl->dpad_x < 0) leg4_state.coordinate_LF[1] += -0.5f;
            if (ctrl->dpad_x > 0) leg4_state.coordinate_LF[1] +=  0.5f;
            if (ctrl->dpad_y > 0) leg4_state.coordinate_LF[2] +=  0.5f;
            if (ctrl->dpad_y < 0) leg4_state.coordinate_LF[2] += -0.5f;
            leg4_state.coordinate_LF[0] += 0.5f * ctrl->right_stick_y;
        }
    }

    if (leg4_state.mode_hands == 0 || leg4_state.mode_hands == 2)
        loko_set_foot_local(st, 0, leg4_state.coordinate_RF[0], leg4_state.coordinate_RF[1], leg4_state.coordinate_RF[2]);
    if (leg4_state.mode_hands == 1 || leg4_state.mode_hands == 2)
        loko_set_foot_local(st, 5, leg4_state.coordinate_LF[0], leg4_state.coordinate_LF[1], leg4_state.coordinate_LF[2]);

    leg4_state.old_R2 = hit_active;

    loko_solve_and_write(st);
}

/*
 * ROTATE_IN_PLACE — pure yaw spin using tripod gait; vx and vy are forced to zero.
 * Input:  in->wz — rotation rate; dt — time step
 * Output: void
 */
static void state_rotate_in_place(LokoState *st, const LokoInput *in, float dt)
{
    st->duty_factor = LOKO_DUTY_TRIPOD;
    for (int i = 0; i < LOKO_NUM_LEGS; i++)
        st->legs[i].active = 1;
    loko_advance_phases(st, 0.0f, 0.0f, in->wz, dt);
    loko_compute_foot_targets(st, 0.0f, 0.0f, in->wz, LOKO_TRAJ_H);
    loko_solve_and_write(st);
}

/*
 * LOOK_AROUND — holds all legs at neutral stance while mapping the right stick
 * directly to body yaw and pitch. IMU roll and shift corrections are suppressed
 * so the commanded pose is applied cleanly.
 * Input:  right_stick_x → body_yaw; right_stick_y → body pitch via s_stabilizer_pitch
 * Output: void
 */
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

/*
 * DANCING — placeholder state; currently delegates to state_stand().
 * Output: void
 */
static void state_dancing(LokoState *st, const LokoInput *in, float dt)
{
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

/*
 * Sets the WS2812B LED animation to match the current FSM state and gait mode.
 * Called once per state or gait transition by loko_dispatch().
 * Input:  state — new FSM state; gait — new gait mode
 * Output: void
 */
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
        case GAIT_OBSTACLE: ws2812_mode_solid(255, 80, 0); break;
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
        ws2812_mode_morph(300);
        break;
    case LOKO_LOOK_AROUND:
        ws2812_mode_solid(255, 128, 0);
        break;
    default:
        break;
    }
}

/*
 * Central per-tick FSM dispatcher. On first call, initialises the stabiliser.
 * Detects state or gait-mode changes, updates LEDs, and resets leg phases when
 * entering/exiting rotation or 4-leg modes. Reads a fresh IMU sample and runs
 * the stabiliser update before calling the active state handler. In LOOK_AROUND
 * mode, IMU-driven roll and shift are suppressed so the commanded pose is clean.
 * Input:  st — locomotion state; in — velocity command; dt — time step in seconds
 * Output: void
 */
void loko_dispatch(LokoState *st, const LokoInput *in, float dt)
{
    static uint8_t stabilizer_initialized = 0;
    if (!stabilizer_initialized) {
        loko_stabilizer_init(&stabilizer_cfg);
        stabilizer_initialized = 1;
    }

    static LokoFSMState prev_state = (LokoFSMState)-1;
    static LokoGaitMode prev_gait  = (LokoGaitMode)-1;
    if (st->state != prev_state || st->gait_mode != prev_gait) {
        loko_update_leds(st->state, st->gait_mode);

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

    if (IMU_Read(&st->imu_data) == 0) {
        /* Slow LEVEL-mode correction while walking so per-step bobbing is
         * averaged out rather than chased every tick. */
        uint8_t is_walking = (st->state == LOKO_WALK          ||
                              st->state == LOKO_WALK_4_LEGS   ||
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
    }

    state_table[st->state](st, in, dt);
}

/*
 * Returns the current IMU-based stabiliser roll and pitch angles.
 * In STAB_LEVEL mode these are non-zero; in STAB_STABLE they are zero.
 * Output: *out_roll and *out_pitch in radians (NULL pointers are safe to pass)
 */
void loko_get_stabilizer_angles(float *out_roll, float *out_pitch)
{
    if (out_roll)  *out_roll  = s_stabilizer_roll;
    if (out_pitch) *out_pitch = s_stabilizer_pitch;
}

/*
 * Returns the current IMU-based horizontal body shift.
 * Non-zero only in STAB_STABLE mode.
 * Output: *out_shift_x_mm and *out_shift_y_mm in mm (NULL pointers are safe)
 */
void loko_get_stabilizer_shift(float *out_shift_x_mm, float *out_shift_y_mm)
{
    if (out_shift_x_mm) *out_shift_x_mm = s_stabilizer_shift_x;
    if (out_shift_y_mm) *out_shift_y_mm = s_stabilizer_shift_y;
}
