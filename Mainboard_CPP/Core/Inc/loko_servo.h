/*
 * loko_servo.h  --  IK solve and PCA9685 servo write
 */
#ifndef LOKO_SERVO_H
#define LOKO_SERVO_H

#include "lokomotion.h"

/* Solve IK for every leg's current foot target and write the resulting
 * joint angles to the PCA9685 boards.  Sets LOKO_ERR_* flags on failure. */
void loko_solve_and_write(LokoState *st);

#endif /* LOKO_SERVO_H */
