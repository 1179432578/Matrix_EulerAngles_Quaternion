#pragma once

#ifndef _MATRIX4x3_H_INCLUDED_
#define _MATRIX4x3_H_INCLUDED_

class Matrix4x3 {
public:
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
	float tx, ty, tz;

	void identity();

	//ƽ��
	void zeroTranslation();
	void setTranslation(const Vector3 &d);
	void setupTranslation(const Vector3 &d);

	//���ռ�<->�ֲ��ռ�ı任���󣬼ٶ��ֲ��ռ���ָ����λ�úͷ�λ����λ������ʹ��ŷ���ǻ���ת�����ʾ��
	void setupLocalToParent(const Vector3 &pos, const EulerAngles &orient);
	void setupLocalToParent(const Vector3 &pos, const RotationMatrix &orient);
	void setupParentToLocal(const Vector3 &pos, const EulerAngles &orient);
	void setupParentToLocal(const Vector3 &pos, const RotationMatrix &orient);

	//��������������ת�ľ���
	void setupRotate(int axis, float theta);
	//��������������ת�ľ���
	void setupRotate(const Vector3 &axis, float theta);
	//����Ԫ��������ת����
	void fromQuaterion(const Quaternion &q);
	//���������������ŵľ���
	void setupScale(const Vector3 &s);
	//���������������ŵľ���
	void setupScaleAlongAxis(const Vector3 &axis, float k);
	//�����б����
	void setupShear(int axis, float s, float t);
	//����ͶӰ����ͶӰƽ���ԭ��
	void setupProject(const Vector3 &n);
	//���췴�����
	void setupReflect(int axis, float k = 0.0f);
	//����������ƽ�淴��ľ���
	void setupReflect(const Vector3 &n);
};

//�����* �����任������Ӿ���
//�˷���˳�����������ر任��˳�����
//�������������ض�Ԫ���������Ҫ������������
Vector3 operator*(const Vector3 &p, const Matrix4x3 &m);
Matrix4x3 operator*(const Matrix4x3 &a, const Matrix4x3 &b);

//�����*=
Vector3 &operator*=(Vector3 &p, const Matrix4x3 &m);
Matrix4x3 &operator*=(const Matrix4x3 &a, const Matrix4x3 &m);

//����3X3���ֵ�����ʽֵ
float determinant(const Matrix4x3&m);

//����������
Matrix4x3 inverse(const Matrix4x3 &m);

//��ȡ�����ƽ�Ʋ���
Vector3 getTranslation(const Matrix4x3 &m);

//�� �ֲ�����->������ ��ȡλ��/��λ
Vector3 getPositionFromParentToLocalMatrix(const Matrix4x3 &m);
//�� ������->������ ��ȡλ��/��λ
Vector3 getPositionFromLocalToParentMatrix(const Matrix4x3 &m);

#endif // !_MATRIX4x3_H_INCLUDED_
