/*
 * xiao_camera_processing.h
 *
 *  Created on: 2023Äê6ÔÂ28ÈÕ
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_CAMERA_PROCESSING_H_
#define CODE_XIAO_CAMERA_PROCESSING_H_

#include "xiao_common.h"
#include "../user/cpu0_main.h"
#include "xiao_image_processing.h"
struct Camera_DistortImage {
    uint8 x;
    uint8 y;
    uint8 gray;
};
extern uint8 map_x[120][188];
extern uint8 map_y[120][188];
//extern double H[3][3];
//extern uint8 inv_map_x[120][188];
//extern uint8 inv_map_y[120][188];
//void Camera_GetInverse(uint8 x, uint8 y, uint8 inv[2]);



#endif /* CODE_XIAO_CAMERA_PROCESSING_H_ */
