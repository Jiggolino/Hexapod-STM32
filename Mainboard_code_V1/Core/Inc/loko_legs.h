/*
 * loko_legs.h  --  Default leg geometry initialisation
 */
#ifndef LOKO_LEGS_H
#define LOKO_LEGS_H

#include "lokomotion.h"

/* Build the six-leg geometry, IK config, and servo mapping from loko_config.h.
 * Called once by loko_init(); may be called again to reset to default stance. */
void loko_build_default_legs(LokoState *st);

#endif /* LOKO_LEGS_H */
