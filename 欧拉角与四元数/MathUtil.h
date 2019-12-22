#pragma once
#ifndef _MATHUTIL_H_INCLUDED_
#define _MATHUTIL_H_INCLUDED_

#include <math.h>
const float kPi = 3.1415965f;
const float k2Pi = kPi * 2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;

//将角度限制在-pi pi区间内（添加周期2pi的适当倍数）
extern float wrapPi(float theta);


extern float safeAcos(float x);

//某些平台与，如果需要这两个值，同时计算比分开计算快
inline void sinCos(float *returnSin, float *returnCos, float theta) {
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}

#endif