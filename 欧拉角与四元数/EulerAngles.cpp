#include <math.h>

#include "EulerAngles.h"
#include "Quaternion.h"
#include "MathUtil.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"

const EulerAngles kEulerAnglesIdentity(0.0f, 0.0f, 0.0f);

//限制欧拉角，尽量避免万向死锁，插值的问题
void EulerAngles::canonize() {
	pitch = wrapPi(pitch);

	//pitch变换到-pi/2， pi/2之间
	if (pitch < -kPiOver2) {
		pitch = -kPi - pitch;
		heading += kPi;
		bank += kPi;
	}
	else if (pitch > kPiOver2) {
		pitch = kPi - pitch;
		heading += kPi;
		bank += kPi;
	}

	//检查万向死锁,允许一定的误差
	if (fabs(pitch) > kPiOver2 - 1e-4) {
		heading += bank;
		bank = 0.0f;
	}
	else {
		//非万向锁，将bank转换到限制集中
		heading = wrapPi(heading);
	}

}

void EulerAngles::fromObjectToInertialQuaternion(const Quaternion &q) {
	float sp = -2.0f*(q.y*q.z - q.w*q.x);//计算sin(pitch)

	if (fabs(sp) > 0.9999f) {
		//从正上方或正下方看
		pitch = kPiOver2 * sp;
		//bank置零，计算heading
		heading = atan2(-q.x*q.z + q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);// atan2(double y, double x) 返回以弧度表示的 y/x 的反正切
		bank = 0.0f;
	}
	else {
		//之前已经在检查万向锁问题时检查过范围错误了，因此不必要使用安全的asin函数
		pitch = asin(sp);
		heading = atan2(q.x*q.z + q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}


//从惯性-物体四元数到欧拉角
void EulerAngles::fromInertialQuaternionToObject(const Quaternion &q) {
	float sp = -2.0f*(q.y*q.z + q.w*q.x);//计算sin(pitch)
	if (fabs(sp) > 0.9999f) {
		//从正上方或正下方看
		pitch = kPiOver2 * sp;
		//bank置零，计算heading
		heading = atan2(-q.x*q.z -q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);// atan2(double y, double x) 返回以弧度表示的 y/x 的反正切
		bank = 0.0f;
	}
	else {
		//之前已经在检查万向锁问题时检查过范围错误了，因此不必要使用安全的asin函数
		pitch = asin(sp);
		heading = atan2(q.x*q.z - q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y - q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}
//物体-世界坐标系变换矩阵到欧拉角
//忽略平移，假设矩阵正交
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m) {
	float sp = -m.m32;

	if (fabs(sp) > 9.9999f) {
		//从正上方或正下方看
		pitch = kPiOver2 * sp;

		//bank置零，计算heading
		heading = atan2(-m.m23, m.m11);
		bank = 0.0f;
	}
	else {
		heading = atan2(m.m31, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m12, m.m22);
	}
}

void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3 &m) {
	float sp = -m.m32;

	if (fabs(sp) > 9.9999f) {
		//从正上方或正下方看
		pitch = kPiOver2 * sp;

		//bank置零，计算heading
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		heading = atan2(m.m31, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m12, m.m22);
	}
}

//根据旋转矩阵构造欧拉角
void EulerAngles::fromRotationMatrix(const RotationMatrix &m) {
	float sp = -m.m32;

	//检查万向锁
	if (fabs(sp) > 9.9999f) {
		pitch = kPiOver2 * sp;
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}

}

