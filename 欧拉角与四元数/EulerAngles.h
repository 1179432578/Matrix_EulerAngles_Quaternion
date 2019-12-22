#pragma once

#ifndef _EULERANGLES_H_INCLUDED_
#define _EULERANGLES_H_INCLUDED_

class Quaternion;
class Matrix4x3;
class RotationMatrix;

class EulerAngles {
public:
	//使用弧度保存三个角度
	float heading;//绕y轴
	float pitch;//绕x轴
	float bank;//绕z轴

	EulerAngles(){ }
	EulerAngles(float h, float p, float b):heading(h), pitch(p), bank(b){}

	void identity() { pitch = bank = heading = 0.0f; }

	//变换为"限制集"欧拉角
	void canonize();

	//从四元数转换为欧拉角

	//输入的四元数为物体-惯性四元数
	void fromObjectToInertialQuaternion(const Quaternion& q);
	//输入的四元数为惯性-物体四元数
	void fromInertialQuaternionToObject(const Quaternion& q);

	//从矩阵转欧拉角
	//平移部分被省略，并假设矩阵是正交的AAT = E

	//输入矩阵为物体-世界转换矩阵
	void fromObjectToWorldMatrix(const Matrix4x3 &m);
	//输入矩阵为世界-物体转换矩阵
	void fromWorldToObjectMatrix(const Matrix4x3 &m);

	//从旋转矩阵转换到欧拉角
	void fromRotationMatrix(const RotationMatrix &m);
};

extern const EulerAngles kEulerAnglesIdentity;

#endif // !_EULERANGLES_H_INCLUDED_
