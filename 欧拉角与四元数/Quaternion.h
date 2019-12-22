#pragma once
#ifndef _QUATERNION_H_INCLUDED_
#define _QUATERNION_H_INCLUDED_

class  Vector3;
class  EulerAngles;


//q = [ cos(theta/2)  sin(theta/2)nx sin(theta/2)ny sin(theta/2)nz ]
class Quaternion {
public:
	float w, x, y, z;

	//��λ��Ԫ��
	void identity() { 
		w = 1.0f;
		x = y = z = 0.0f;
	}

	//����ִ����ת����Ԫ��
	void setToRotateAboutX(float theta);
	void setToRotateAboutY(float theta);
	void setToRotateAboutZ(float theta);
	void setToRotateAboutAxis(const Vector3 &axis, float theta);

	//����ִ�� ����-������ת ����Ԫ������λ������ŷ������ʽ����
	void setToRotateObjectToInertial(const EulerAngles &orientation);
	void setToRotateInertialToObject(const EulerAngles &orientation);

	//���
	Quaternion operator*(const Quaternion &a) const;

	Quaternion &operator *=(const Quaternion &a);

	//����Ԫ������
	void normalize();

	float getRotationAngle() const;
	Vector3 getRotationAxis() const;
};

extern const Quaternion kQuaternionIdentity;

//��Ԫ�����
extern float dotProduct(const Quaternion &a, const Quaternion &b);
//�����ֵ
extern Quaternion slerp(const Quaternion &p, const Quaternion &q, float t);
//��Ԫ������
extern Quaternion conjugate(const Quaternion &q);
//��Ԫ����
extern Quaternion pow(const Quaternion &q, float exponent);


#endif // !_QUATERNION_H_INCLUDED_
