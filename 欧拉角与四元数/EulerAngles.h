#pragma once

#ifndef _EULERANGLES_H_INCLUDED_
#define _EULERANGLES_H_INCLUDED_

class Quaternion;
class Matrix4x3;
class RotationMatrix;

class EulerAngles {
public:
	//ʹ�û��ȱ��������Ƕ�
	float heading;//��y��
	float pitch;//��x��
	float bank;//��z��

	EulerAngles(){ }
	EulerAngles(float h, float p, float b):heading(h), pitch(p), bank(b){}

	void identity() { pitch = bank = heading = 0.0f; }

	//�任Ϊ"���Ƽ�"ŷ����
	void canonize();

	//����Ԫ��ת��Ϊŷ����

	//�������Ԫ��Ϊ����-������Ԫ��
	void fromObjectToInertialQuaternion(const Quaternion& q);
	//�������Ԫ��Ϊ����-������Ԫ��
	void fromInertialQuaternionToObject(const Quaternion& q);

	//�Ӿ���תŷ����
	//ƽ�Ʋ��ֱ�ʡ�ԣ������������������AAT = E

	//�������Ϊ����-����ת������
	void fromObjectToWorldMatrix(const Matrix4x3 &m);
	//�������Ϊ����-����ת������
	void fromWorldToObjectMatrix(const Matrix4x3 &m);

	//����ת����ת����ŷ����
	void fromRotationMatrix(const RotationMatrix &m);
};

extern const EulerAngles kEulerAnglesIdentity;

#endif // !_EULERANGLES_H_INCLUDED_
