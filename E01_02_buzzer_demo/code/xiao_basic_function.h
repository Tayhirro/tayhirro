/*
 * xiao_basic_function.h
 *
 *  Created on: 2023��6��29��
 *      Author: Jayden_NaN
 */

#ifndef CODE_XIAO_BASIC_FUNCTION_H_
#define CODE_XIAO_BASIC_FUNCTION_H_

#include "xiao_common.h"

//------------------------------�ļ�����------------------------------
//�ṩ����������������������

//--------------------�涨������--------------------
int16 bf_clip(int16 x, int16 low, int16 up);
float bf_fclip(float x, float low, float up);

//--------------------��ȡ��ֵ--------------------
int16 bf_min(int16 x, int16 y);
int16 bf_max(int16 x, int16 y);
float bf_fmin(float x, float y);
float bf_fmax(float x, float y);
int difference_sum( uint8 a,uint8 b );
#endif /* CODE_XIAO_BASIC_FUNCTION_H_ */
