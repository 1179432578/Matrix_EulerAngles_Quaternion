#pragma once
#ifndef _QUATERNION_H_INCLUDED_
#define _QUATERNION_H_INCLUDED_

class  Vector3;
class  EulerAngles;


//q = [ cos(theta/2)  sin(theta/2)nx sin(theta/2)ny sin(theta/2)nz ]
class Quaternion {
public:
	float w, x, y, z;

	//单位四元数
	void identity() { 
		w = 1.0f;
		x = y = z = 0.0f;
	}

	//构造执行旋转的四元数
	void setToRotateAboutX(float theta);
	void setToRotateAboutY(float theta);
	void setToRotateAboutZ(float theta);
	void setToRotateAboutAxis(const Vector3 &axis, float theta);

	//构造执行 物体-惯性旋转 的四元数，方位参数用欧拉角形式给出
	void setToRotateObjectToInertial(const EulerAngles &orientation);
	void setToRotateInertialToObject(const EulerAngles &orientation);

	//叉乘
	Quaternion operator*(const Quaternion &a) const;

	Quaternion &operator *=(const Quaternion &a);

	//将四元数正则化
	void normalize();

	float getRotationAngle() const;
	Vector3 getRotationAxis() const;
};

extern const Quaternion kQuaternionIdentity;

//四元数点乘
extern float dotProduct(const Quaternion &a, const Quaternion &b);
//球面插值
extern Quaternion slerp(const Quaternion &p, const Quaternion &q, float t);
//四元数共轭
extern Quaternion conjugate(const Quaternion &q);
//四元数幂
extern Quaternion pow(const Quaternion &q, float exponent);


#endif // !_QUATERNION_H_INCLUDED_
