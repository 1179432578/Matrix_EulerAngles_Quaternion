#include <math.h>

#include "EulerAngles.h"
#include "Quaternion.h"
#include "MathUtil.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"

const EulerAngles kEulerAnglesIdentity(0.0f, 0.0f, 0.0f);

//����ŷ���ǣ���������������������ֵ������
void EulerAngles::canonize() {
	pitch = wrapPi(pitch);

	//pitch�任��-pi/2�� pi/2֮��
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

	//�����������,����һ�������
	if (fabs(pitch) > kPiOver2 - 1e-4) {
		heading += bank;
		bank = 0.0f;
	}
	else {
		//������������bankת�������Ƽ���
		heading = wrapPi(heading);
	}

}

void EulerAngles::fromObjectToInertialQuaternion(const Quaternion &q) {
	float sp = -2.0f*(q.y*q.z - q.w*q.x);//����sin(pitch)

	if (fabs(sp) > 0.9999f) {
		//�����Ϸ������·���
		pitch = kPiOver2 * sp;
		//bank���㣬����heading
		heading = atan2(-q.x*q.z + q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);// atan2(double y, double x) �����Ի��ȱ�ʾ�� y/x �ķ�����
		bank = 0.0f;
	}
	else {
		//֮ǰ�Ѿ��ڼ������������ʱ������Χ�����ˣ���˲���Ҫʹ�ð�ȫ��asin����
		pitch = asin(sp);
		heading = atan2(q.x*q.z + q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}


//�ӹ���-������Ԫ����ŷ����
void EulerAngles::fromInertialQuaternionToObject(const Quaternion &q) {
	float sp = -2.0f*(q.y*q.z + q.w*q.x);//����sin(pitch)
	if (fabs(sp) > 0.9999f) {
		//�����Ϸ������·���
		pitch = kPiOver2 * sp;
		//bank���㣬����heading
		heading = atan2(-q.x*q.z -q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);// atan2(double y, double x) �����Ի��ȱ�ʾ�� y/x �ķ�����
		bank = 0.0f;
	}
	else {
		//֮ǰ�Ѿ��ڼ������������ʱ������Χ�����ˣ���˲���Ҫʹ�ð�ȫ��asin����
		pitch = asin(sp);
		heading = atan2(q.x*q.z - q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y - q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}
//����-��������ϵ�任����ŷ����
//����ƽ�ƣ������������
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3 &m) {
	float sp = -m.m32;

	if (fabs(sp) > 9.9999f) {
		//�����Ϸ������·���
		pitch = kPiOver2 * sp;

		//bank���㣬����heading
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
		//�����Ϸ������·���
		pitch = kPiOver2 * sp;

		//bank���㣬����heading
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else {
		heading = atan2(m.m31, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m12, m.m22);
	}
}

//������ת������ŷ����
void EulerAngles::fromRotationMatrix(const RotationMatrix &m) {
	float sp = -m.m32;

	//���������
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

