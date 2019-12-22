#pragma once
#ifndef _MATHUTIL_H_INCLUDED_
#define _MATHUTIL_H_INCLUDED_

#include <math.h>
const float kPi = 3.1415965f;
const float k2Pi = kPi * 2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;

//���Ƕ�������-pi pi�����ڣ��������2pi���ʵ�������
extern float wrapPi(float theta);


extern float safeAcos(float x);

//ĳЩƽ̨�룬�����Ҫ������ֵ��ͬʱ����ȷֿ������
inline void sinCos(float *returnSin, float *returnCos, float theta) {
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}

#endif