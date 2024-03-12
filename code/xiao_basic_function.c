/*
 * xiao_basic_function.c
 *
 *  Created on: 2023Äê6ÔÂ29ÈÕ
 *      Author: Jayden_NaN
 */
#include "xiao_basic_function.h"
int16 bf_clip(int16 x, int16 low, int16 up) {
    return x > up ? up : x < low ? low : x;
}

float bf_fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}

int16 bf_min(int16 x, int16 y) {
    return x > y ? y : x;
}

int16 bf_max(int16 x, int16 y) {
    return x > y ? x : y;
}

float bf_fmin(float x, float y) {
    return x > y ? y : x;
}

float bf_fmax(float x, float y) {
    return x > y ? x : y;
}
int difference_sum( uint8 a,uint8 b )
{
 if(a+b==0)
 {
   return 0;
 }
  return (a-b)*300/(a+b);
}

float invSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;          // get bits for floating value
    i =  0x5f375a86 - (i>>1);    // gives initial guess
    x = *(float*)&i;            // convert bits back to float
    x = x * (1.5f - xhalf*x*x); // Newton step
    return x;
}
