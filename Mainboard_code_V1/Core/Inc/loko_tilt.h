/*
 * loko_tilt.h
 *
 *  Created on: Apr 27, 2026
 *      Author: linus
 */

#ifndef INC_LOKO_TILT_H_
#define INC_LOKO_TILT_H_

typedef struct {
    float x, y, z;
} Vector3f;

void prepare_for_ik(Vector3f leg_point, Vector3f mount_pos, float mount_cos, float mount_sin, float height, float roll, float pitch, Vector3f *out_ik);

#endif /* INC_LOKO_TILT_H_ */
