/*
 * xiao_basic_function.h
 *
 *  Created on: 2023年6月29日
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_BASIC_FUNCTION_H_
#define CODE_XIAO_BASIC_FUNCTION_H_

#include "xiao_common.h"

//------------------------------文件作用------------------------------
//提供基本函数供其它函数调用

//--------------------规定上下限--------------------
int16 bf_clip(int16 x, int16 low, int16 up);
float bf_fclip(float x, float low, float up);

//--------------------获取最值--------------------
int16 bf_min(int16 x, int16 y);
int16 bf_max(int16 x, int16 y);
float bf_fmin(float x, float y);
float bf_fmax(float x, float y);
int difference_sum( uint8 a,uint8 b );
#endif /* CODE_XIAO_BASIC_FUNCTION_H_ */
