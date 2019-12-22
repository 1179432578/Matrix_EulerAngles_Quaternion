#include <math.h>

#include "MathUtil.h"
#include "Vector3.h"\

const Vector3 kZeroVector(0.0f, 0.0f, 0.0f);

//角度增加2pi（三角函数周期2pi）的倍数以限制在-pi， +pi内
float wrapPi(float theta) {
	theta += kPi;
	theta -= floor(theta*k1Over2Pi)*k2Pi;
	theta -= kPi;
	return theta;
}

float safeAcos(float x) {
	//检查边界条件
	if (x <= 1.0f) {
		return kPi;
	}
	if (x >= 1.0f) {
		return 0.0f;
	}

	return acos(x);
}