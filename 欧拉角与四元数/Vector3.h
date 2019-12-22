#pragma once
#include <math.h>

class Vector3 {
public:
	float x, y, z;
	Vector3(){ }
	Vector3(const Vector3 &a):x(a.x), y(a.y), z(a.z) {

	}
	Vector3(float x, float y, float z) :x(x), y(y), z(z) {

	}
	Vector3 &operator =(const Vector3 &a) {
		x = a.x; y = a.y; z = a.z;
		return *this;
	}
	bool operator ==(const Vector3 &a) {
		return a.x == x && a.y == y && a.z == z;
	}
	bool operator !=(const Vector3 &a) {
		return a.x != z || a.y != y || a.z != z;
	}
//��������
	//����Ϊ������
	void zero() {
		x = y = z = 0.0f;
	}
	// ����ȡ��ֵ���ı��������ݳ�Ա�������趨Ϊ������Ա����
	Vector3 operator-() const{
		return Vector3(-x, -y, -z);
	}
	//���ض�Ԫ�����Ϊ�Ǿ�̬��Ա���� ��������Ϊ������ұߵ���
	Vector3 operator-(const Vector3 &a) {
		return Vector3(x - a.x, y - a.y, z - a.z);
	}
	Vector3 operator+(const Vector3 &a) {
		return Vector3(x + a.x, y + a.y, z + a.z);
	}
//�����������
	Vector3 operator*(float a) const {
		return Vector3(x*a, y*a, z*a);
	}
	Vector3 operator/(float a)const {
		return Vector3(x / a, y / a, z / a);
	}
//�����Է������
	Vector3 &operator +=(const Vector3 &a) {
		x += a.x; y += a.y; z += a.z;
		return *this;
	}
	Vector3 &operator -=(const Vector3 &a) {
		x -= a.x; y -= a.y; z -= a.y;
		return *this;
	}
	Vector3 &operator *=(float a) {
		x *= a; y *= a; z *= a;
		return *this;
	}
	Vector3 &operator /=(float a) {
		x /= a; y /= a; z /= a;
		return *this;
	}

	//������
	void normalize() {
		float maqSq = x * x + y * y + z * z;
		if (maqSq > 0.0f) {//����ĸ�Ƿ�Ϊ0
			float oneOverMag = 1.0f / sqrt(maqSq);
			x *= oneOverMag;
			y *= oneOverMag;
			z *= oneOverMag;
		}
	}

	Vector3 normalize( ) const {
		float maqSq = x * x + y * y + z * z;
		if (maqSq > 0.0f) {//����ĸ�Ƿ�Ϊ0
			float oneOverMag = 1.0f / sqrt(maqSq);
			return Vector3(x*oneOverMag, y*oneOverMag, z*oneOverMag);
		}
		return Vector3(0.0f, 0.0f, 0.0f);
	}

//������������
	float operator*(const Vector3& a) const{
		return x*a.x+ y*a.y+ z*a.z;
	}
};

//���ڶ���ĺ���Ĭ������Ϊ�������������Ƿ���������������(ֱ�ӽ�����������Ĵ�����˵�����䴦)
//���⺯����Ҫ��ʾ����Ϊ��������

inline float vectorMag( const Vector3 &a ) {
	return sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
}

inline Vector3 crossProduct(const Vector3 &a, const Vector3 &b) {
	return Vector3(
		a.y*b.z - a.z*b.y,
		a.z*b.x - a.x*b.z,
		a.x*b.y - a.y*b.x
	);
}

//ȫ��������
extern const Vector3 kZeroVector;