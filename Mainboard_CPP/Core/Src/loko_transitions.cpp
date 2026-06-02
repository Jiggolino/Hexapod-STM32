/*
 * loko_transitions.c  --  State transition logic
 *
 * Reads st->pad (filled by loko_input_update() each tick) and sets
 * st->state when a condition is met.  Edge detection is provided by
 * loko_pressed() / loko_released() from loko_input.h — no manual
 * prev-state tracking needed here.
 *
 * Full transition map
 * ───────────────────
 *  R1 press  (any state)         → UN_ARMED
 *  L1 press  in UN_ARMED         → STAND
 *  left stick moved in STAND        → WALK
 *  left stick released in WALK      → STAND
 *  Square press in STAND/WALK       → STAND_4_LEGS
 *  left stick moved in STAND_4_LEGS → WALK_4_LEGS
 *  left stick released in WALK_4_LEGS → STAND_4_LEGS
 *  Circle press in WALK_4_LEGS/STAND_4_LEGS → STAND
 *  Triangle press in STAND          → DANCING
 *  Circle  press in DANCING         → STAND
 *  L2 held  in STAND                → ROTATE_IN_PLACE
 *  L2 released in ROTATE            → STAND
 *  R2 held  in STAND                → LOOK_AROUND  (saves stab_mode)
 *  R2 released in LOOK_AROUND       → STAND        (restores stab_mode, resets body_yaw/pitch)
 */

#include "loko_transitions.h"
#include "loko_states.h"
#include "main.h"
#include "pca9685.h"
#include "hexapod/Controller.hpp"
#include "loko_config.h"

/* Forward declarations */
static void Set_servo_default(LokoState *st);

/* ── Per-state transition functions ─────────────────────────────────────── */

static void transition_un_armed(LokoState *st)
{
    Controller ctl(st->pad);
    if (ctl.pressed(BTN_L1)) {
        Set_servo_default(st);

        /* Arm the servos: OE is Active Low, so RESET enables outputs. 
         * Staggered to reduce power spikes. */
        HAL_GPIO_WritePin(Right_Enable_GPIO_Port, Right_Enable_Pin, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(Left_Enable_GPIO_Port,  Left_Enable_Pin,  GPIO_PIN_RESET);

        st->state = LOKO_STAND;
    }
}

static void transition_stand(LokoState *st)
{
    Controller ctl(st->pad);
    const float lx = ctl.axis(AXIS_LX);
    const float ly = ctl.axis(AXIS_LY);
    const float stick_mag = lx*lx + ly*ly;

    if (ctl.pressed(BTN_SQUARE)) {
        st->state = LOKO_STAND_4_LEGS;
        loko_reset_4leg_mode();
    }
    else if (ctl.pressed(BTN_TRIANGLE))
        st->state = LOKO_DANCING;
    else if (ctl.pressed(BTN_DPAD_RIGHT))
        st->gait_mode = (LokoGaitMode)((st->gait_mode + 1) % GAIT_MODE_COUNT);
    else if (ctl.pressed(BTN_DPAD_LEFT))
        st->gait_mode = (LokoGaitMode)((st->gait_mode + GAIT_MODE_COUNT - 1) % GAIT_MODE_COUNT);
    else if (ctl.pressed(BTN_DPAD_UP))
        st->stab_mode = (LokoStabMode)((st->stab_mode + 1) % STAB_MODE_COUNT);
    else if (ctl.pressed(BTN_DPAD_DOWN))
        st->stab_mode = (LokoStabMode)((st->stab_mode + STAB_MODE_COUNT - 1) % STAB_MODE_COUNT);
    else if (ctl.held(BTN_L2))
        st->state = LOKO_ROTATE_IN_PLACE;
    else if (ctl.held(BTN_R2)) {
        st->stab_mode_saved = st->stab_mode;
        st->state = LOKO_LOOK_AROUND;
    }
    else if (stick_mag > STICK_DEADBAND * STICK_DEADBAND)
        st->state = LOKO_WALK;
}

static void transition_walk(LokoState *st)
{
    Controller ctl(st->pad);
    if (ctl.pressed(BTN_SQUARE)) {
        st->state = LOKO_STAND_4_LEGS;
        loko_reset_4leg_mode();
    }
    else if (ctl.pressed(BTN_TRIANGLE))
        st->state = LOKO_DANCING;
    else if (ctl.pressed(BTN_DPAD_RIGHT))
        st->gait_mode = (LokoGaitMode)((st->gait_mode + 1) % GAIT_MODE_COUNT);
    else if (ctl.pressed(BTN_DPAD_LEFT))
        st->gait_mode = (LokoGaitMode)((st->gait_mode + GAIT_MODE_COUNT - 1) % GAIT_MODE_COUNT);
    else if (ctl.pressed(BTN_DPAD_UP))
        st->stab_mode = (LokoStabMode)((st->stab_mode + 1) % STAB_MODE_COUNT);
    else if (ctl.pressed(BTN_DPAD_DOWN))
        st->stab_mode = (LokoStabMode)((st->stab_mode + STAB_MODE_COUNT - 1) % STAB_MODE_COUNT);
    else if (ctl.held(BTN_L2))
        st->state = LOKO_ROTATE_IN_PLACE;
    else {
        const float lx = ctl.axis(AXIS_LX);
        const float ly = ctl.axis(AXIS_LY);
        if (lx*lx + ly*ly <= STICK_DEADBAND * STICK_DEADBAND)
            st->state = LOKO_STAND;
    }
}

static void transition_walk_4_legs(LokoState *st)
{
    Controller ctl(st->pad);
    if (ctl.pressed(BTN_CIRCLE)) {
        st->state = LOKO_STAND;
        return;
    }
    const float lx = ctl.axis(AXIS_LX);
    const float ly = ctl.axis(AXIS_LY);
    if (lx*lx + ly*ly <= STICK_DEADBAND * STICK_DEADBAND)
        st->state = LOKO_STAND_4_LEGS;
}

static void transition_stand_4_legs(LokoState *st)
{
    Controller ctl(st->pad);
    if (ctl.pressed(BTN_CIRCLE)) {
        st->state = LOKO_STAND;
        return;
    }
    const float lx = ctl.axis(AXIS_LX);
    const float ly = ctl.axis(AXIS_LY);
    if (lx*lx + ly*ly > STICK_DEADBAND * STICK_DEADBAND)
        st->state = LOKO_WALK_4_LEGS;
}

static void transition_rotate_in_place(LokoState *st)
{
    Controller ctl(st->pad);
    if (ctl.released(BTN_L2))
        st->state = LOKO_STAND;
}

static void transition_dancing(LokoState *st)
{
    Controller ctl(st->pad);
    if (ctl.pressed(BTN_CIRCLE))
        st->state = LOKO_STAND;
}

static void transition_look_around(LokoState *st)
{
    Controller ctl(st->pad);
    if (ctl.released(BTN_R2)) {
        st->body_yaw   = 0.0f;
        st->body_pitch = 0.0f;
        st->stab_mode  = st->stab_mode_saved;
        st->state = LOKO_STAND;
    }
}

/* ── Switch dispatcher ───────────────────────────────────────────────────── */

void loko_update_transitions(LokoState *st)
{
    /* R1 or Hand infront of sensor always disarms from any state. */
    Controller ctl(st->pad);
    if (ctl.pressed(BTN_R1) || (DISARM_DISTANCE >= (float)tof_get_distance_mm())) {
        /* Disarm servos: OE HIGH = Disabled */
        HAL_GPIO_WritePin(Right_Enable_GPIO_Port, Right_Enable_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Left_Enable_GPIO_Port,  Left_Enable_Pin,  GPIO_PIN_SET);
        st->state = LOKO_UN_ARMED;
        return;
    }

    switch (st->state) {
    case LOKO_UN_ARMED:        transition_un_armed       (st); break;
    case LOKO_STAND:           transition_stand          (st); break;
    case LOKO_WALK:            transition_walk           (st); break;
    case LOKO_WALK_4_LEGS:     transition_walk_4_legs    (st); break;
    case LOKO_STAND_4_LEGS:    transition_stand_4_legs   (st); break;
    case LOKO_ROTATE_IN_PLACE: transition_rotate_in_place(st); break;
    case LOKO_DANCING:         transition_dancing        (st); break;
    case LOKO_LOOK_AROUND:     transition_look_around    (st); break;
    default: break;
    }
}

static void Set_servo_default(LokoState *st)
{
    /* Set all 9 used channels on both boards to 90 degrees (mid-point) */
    for (int i = 0; i < 9; i++) {
        PCA9685_SetServoAngle(st->pca_right, i, 90.0f);
        PCA9685_SetServoAngle(st->pca_left,  i, 90.0f);
    }
}
