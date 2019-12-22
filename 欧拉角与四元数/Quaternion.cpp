#include <assert.h>
#include <math.h>

#include "Quaternion.h"
#include "MathUtil.h"
#include "Vector3.h"
#include "EulerAngles.h"

const Quaternion kQuaternionIdentity = { 1.0f, 0.0f, 0.0f, 0.0f };
//q = [cos(theta / 2)  sin(theta / 2)nx sin(theta / 2)ny sin(theta / 2)nz]
void Quaternion::setToRotateAboutX(float theta) {
	float thetaOver2 = theta * 0.5f;

	w = cos(thetaOver2);
	x = sin(thetaOver2);
	y = 0.0f;
	z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta) {
	float thetaOver2 = theta * 0.5f;

	w = cos(thetaOver2);
	x = 0.0f;
	y = sin(thetaOver2);
	z = 0.0f;
}



void Quaternion::setToRotateAboutZ(float theta) {
	float thetaOver2 = theta * 0.5f;

	w = cos(thetaOver2);
	x = 0.0f;
	y = 0.0f;
	z = sin(thetaOver2);
}
//q = [cos(theta / 2)  sin(theta / 2)nx sin(theta / 2)ny sin(theta / 2)nz]
void Quaternion::setToRotateAboutAxis( const Vector3 &axis, float theta ) {
	//��ת������׼��
	assert(fabs(vectorMag(axis) - 1.0f) < 0.01f);

	//�����Ǻ�sinֵ
	float thetaOver2 = theta * 0.5f;
	float sinThetaOver2 = sin(thetaOver2);

	w = cos(thetaOver2);
	x = axis.x * sinThetaOver2;
	y = axis.y * sinThetaOver2;
	z = axis.z * sinThetaOver2;
}

//ִ�� ����-���� ��ת����Ԫ��
//��λ������ŷ���Ǹ���
void Quaternion::setToRotateObjectToInertial(const EulerAngles &orientation) {

	float sp, sb, sh;
	float cp, cb, ch;

	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &ch, orientation.heading*0.5f);

	w = ch * cp*cb + sh * sp*sb;
	x = ch * sp*cb + sh * cp*sb;
	y = -ch * sp*sb + sh * cp*cb;
	z = -sh * sp*cb + ch * cp*sb;
}


void Quaternion::setToRotateInertialToObject(const EulerAngles &orientation) {
	float sp, sb, sh;
	float cp, cb, ch;

	sinCos(&sp, &cp, orientation.pitch*0.5f);
	sinCos(&sb, &cb, orientation.bank*0.5f);
	sinCos(&sh, &ch, orientation.heading*0.5f);

	w = ch * cp*cb + sh * sp*sb;
	x = -ch * sp*cb - sh * cp*sb;
	y = ch * sp*sb - sh * cp*cb;
	z = sh * sp*cb - ch * cp*sb;
}


Quaternion& Quaternion::operator*=(const Quaternion& a) {
	*this = *this*a;
	return *this;
}


//������Ԫ����
//��ֹ����������������Ԫ���������ܵ����������
void Quaternion::normalize() {
	float mag = (float)sqrt(w*w + x * x + y * y + z*z);
	//��ֹ�������
	if (mag > 0.0f) {
		float oneOverMag = 1.0f / mag;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	}
	else {
		assert(false);
		identity();
	}
}


//q = [cos(theta / 2)  sin(theta / 2)nx sin(theta / 2)ny sin(theta / 2)nz]
//������ת��
float Quaternion::getRotationAngle()const {
	//w = cos(theta/2)
	float thetaOver2 = safeAcos(w);
	return thetaOver2 * 2.0f;
}

//��ȡ��ת��
Vector3 Quaternion::getRotationAxis()const {
	float sinThetaOver2Sq = 1.0f - w * w;

	if (sinThetaOver2Sq < 0.0f) {
		return Vector3(1.0f, 0.0f, 0.0f);
	}

	return Vector3(x / sqrt(sinThetaOver2Sq),
		y / sqrt(sinThetaOver2Sq),
		z / sqrt(sinThetaOver2Sq)
	);
}

float dotProduct(const Quaternion &a, const Quaternion &b) {
	return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

Quaternion slerp(const Quaternion &q0, const Quaternion &q1, float t) {
	if (t <= 0.0f) return q0;
	if (t >= 1.0f) return q1;

	//�õ�˼�����Ԫ���нǵ�cosֵ
	float cosOmega = dotProduct(q0, q1);

	//�����˽��Ϊ������ʹ��-q1,��Ԫ��q,-q��ʾ��ͬ����ת�������ܲ�����ͬ��slerp����
	//ѡ����ȷ��һ����ʹ����ǽ�����ת
	float q1w = q1.w;
	float q1x = q1.x;
	float q1y = q1.y;
	float q1z = q1.z;
	if (cosOmega < 0.0f) {
		q1w = -q1w;
		q1x = -q1x;
		q1y = -q1y;
		q1z = -q1z;
		cosOmega = -cosOmega;
	}

	//ʹ�õ���������λ��Ԫ������˽��Ӧ��<=1.0
	assert(cosOmega < 1.1f);

	//�����ֵƬ
	float k0, k1;
	if (cosOmega > 0.9999f) {
		//�ǳ��ӽ��������Բ�ֵ����ֹ����
		k0 = 1.0f - t;
		k1 = t;
	}
	else {
		float sinOmega = sqrt(1.0f - cosOmega * cosOmega);
		float omega = atan2(sinOmega, cosOmega);

		float oneOverSinOmega = 1.0f / sinOmega;
		//�����ֵ����
		k0 = sin((1.0f - t)*omega)*oneOverSinOmega;
		k1 = sin(t*omega)*oneOverSinOmega;
	}

	//��ֵ
	Quaternion res;
	res.x = k0 * q0.x + k1 * q1.x;
	res.y = k0 * q0.y + k1 * q1.y;
	res.z = k0 * q0.z + k1 * q1.z;
	res.w = k0 * q0.w + k1 * q1.w;

	return res;
}

//��Ԫ������
Quaternion conjugate(const Quaternion &q) {
	Quaternion result;

	//��ת����ͬ
	result.w = q.w;

	//��ת���෴
	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;

	return result;
}

Quaternion pow(const Quaternion &q, float exponent) {

	//��鵥λ��Ԫ������ֹ����
	if (fabs(q.w) > 0.9999f) {
		return q;
	}

	//��ȡ���alpha=theta/2
	float alpha = acos(q.w);

	//������alphaֵ
	float newAlpha = alpha * exponent;

	//������wֵ
	Quaternion result;
	result.w = cos(newAlpha);

	float mult = sin(newAlpha) / sin(alpha);
	result.x = q.x*mult;
	result.y = q.y*mult;
	result.z = q.z*mult;

	return result;
}

