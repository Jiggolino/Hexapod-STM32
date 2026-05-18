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
 */

#include "loko_transitions.h"
#include "loko_states.h"
#include "main.h"
#include "pca9685.h"

#define STICK_DEADBAND 0.1f

/* Forward declarations */
static void Set_servo_default(LokoState *st);

/* ── Per-state transition functions ─────────────────────────────────────── */

static void transition_un_armed(LokoState *st)
{
    if (loko_pressed(&st->pad, BTN_L1)) {
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
    const float lx = loko_axis(&st->pad, AXIS_LX);
    const float ly = loko_axis(&st->pad, AXIS_LY);
    const float stick_mag = lx*lx + ly*ly;

    if (loko_pressed(&st->pad, BTN_SQUARE)) {
        st->state = LOKO_STAND_4_LEGS;
        loko_reset_4leg_mode();
    }
    else if (loko_pressed(&st->pad, BTN_TRIANGLE))
        st->state = LOKO_DANCING;
    else if (loko_pressed(&st->pad, BTN_DPAD_RIGHT))
        st->gait_mode = (LokoGaitMode)((st->gait_mode + 1) % GAIT_MODE_COUNT);
    else if (loko_pressed(&st->pad, BTN_DPAD_LEFT))
        st->gait_mode = (LokoGaitMode)((st->gait_mode + GAIT_MODE_COUNT - 1) % GAIT_MODE_COUNT);
    else if (loko_pressed(&st->pad, BTN_DPAD_UP))
        st->stab_mode = (LokoStabMode)((st->stab_mode + 1) % STAB_MODE_COUNT);
    else if (loko_pressed(&st->pad, BTN_DPAD_DOWN))
        st->stab_mode = (LokoStabMode)((st->stab_mode + STAB_MODE_COUNT - 1) % STAB_MODE_COUNT);
    else if (loko_held(&st->pad, BTN_L2))
        st->state = LOKO_ROTATE_IN_PLACE;
    else if (stick_mag > STICK_DEADBAND * STICK_DEADBAND)
        st->state = LOKO_WALK;
}

static void transition_walk(LokoState *st)
{
    if (loko_pressed(&st->pad, BTN_SQUARE)) {
        st->state = LOKO_STAND_4_LEGS;
        loko_reset_4leg_mode();
    }
    else if (loko_pressed(&st->pad, BTN_TRIANGLE))
        st->state = LOKO_DANCING;
    else if (loko_pressed(&st->pad, BTN_DPAD_RIGHT))
        st->gait_mode = (LokoGaitMode)((st->gait_mode + 1) % GAIT_MODE_COUNT);
    else if (loko_pressed(&st->pad, BTN_DPAD_LEFT))
        st->gait_mode = (LokoGaitMode)((st->gait_mode + GAIT_MODE_COUNT - 1) % GAIT_MODE_COUNT);
    else if (loko_pressed(&st->pad, BTN_DPAD_UP))
        st->stab_mode = (LokoStabMode)((st->stab_mode + 1) % STAB_MODE_COUNT);
    else if (loko_pressed(&st->pad, BTN_DPAD_DOWN))
        st->stab_mode = (LokoStabMode)((st->stab_mode + STAB_MODE_COUNT - 1) % STAB_MODE_COUNT);
    else if (loko_held(&st->pad, BTN_L2))
        st->state = LOKO_ROTATE_IN_PLACE;
    else {
        const float lx = loko_axis(&st->pad, AXIS_LX);
        const float ly = loko_axis(&st->pad, AXIS_LY);
        if (lx*lx + ly*ly <= STICK_DEADBAND * STICK_DEADBAND)
            st->state = LOKO_STAND;
    }
}

static void transition_walk_4_legs(LokoState *st)
{
    if (loko_pressed(&st->pad, BTN_CIRCLE)) {
        st->state = LOKO_STAND;
        return;
    }
    const float lx = loko_axis(&st->pad, AXIS_LX);
    const float ly = loko_axis(&st->pad, AXIS_LY);
    if (lx*lx + ly*ly <= STICK_DEADBAND * STICK_DEADBAND)
        st->state = LOKO_STAND_4_LEGS;
}

static void transition_stand_4_legs(LokoState *st)
{
    if (loko_pressed(&st->pad, BTN_CIRCLE)) {
        st->state = LOKO_STAND;
        return;
    }
    const float lx = loko_axis(&st->pad, AXIS_LX);
    const float ly = loko_axis(&st->pad, AXIS_LY);
    if (lx*lx + ly*ly > STICK_DEADBAND * STICK_DEADBAND)
        st->state = LOKO_WALK_4_LEGS;
}

static void transition_rotate_in_place(LokoState *st)
{
    if (loko_released(&st->pad, BTN_L2))
        st->state = LOKO_STAND;
}

static void transition_dancing(LokoState *st)
{
    if (loko_pressed(&st->pad, BTN_CIRCLE))
        st->state = LOKO_STAND;
}

/* ── Switch dispatcher ───────────────────────────────────────────────────── */

void loko_update_transitions(LokoState *st)
{
    /* R1 or Hand infront of sensor always disarms from any state. */
    if (loko_pressed(&st->pad, BTN_R1 )||( 15.0f >= (float)tof_get_distance_mm())) {
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
