/*
 * loko_transitions.h  --  State transition logic
 *
 * Call loko_update_transitions() each tick AFTER loko_input_update()
 * and BEFORE loko_update().  It reads st->pad directly.
 */
#ifndef LOKO_TRANSITIONS_H
#define LOKO_TRANSITIONS_H

#include "lokomotion.h"

/* Check transition conditions for the current state and update st->state. */
void loko_update_transitions(LokoState *st);

#endif /* LOKO_TRANSITIONS_H */
