/*
 * loko_tilt.h
 *
 *  Created on: Apr 27, 2026
 *      Author: linus
 */

#ifndef INC_LOKO_TILT_H_
#define INC_LOKO_TILT_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x, y, z;
} Vector3f;

void prepare_for_ik(Vector3f leg_point, Vector3f mount_pos, float mount_cos, float mount_sin, float height, float roll, float pitch, float yaw, Vector3f *out_ik);

#ifdef __cplusplus
}
#endif

#endif /* INC_LOKO_TILT_H_ */
