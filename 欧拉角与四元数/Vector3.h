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
//向量运算
	//重置为零向量
	void zero() {
		x = y = z = 0.0f;
	}
	// 这里取负值不改变自身数据成员，所以设定为常量成员函数
	Vector3 operator-() const{
		return Vector3(-x, -y, -z);
	}
	//重载二元运算符为非静态成员函数 函数参数为运算符右边的数
	Vector3 operator-(const Vector3 &a) {
		return Vector3(x - a.x, y - a.y, z - a.z);
	}
	Vector3 operator+(const Vector3 &a) {
		return Vector3(x + a.x, y + a.y, z + a.z);
	}
//与标量的运算
	Vector3 operator*(float a) const {
		return Vector3(x*a, y*a, z*a);
	}
	Vector3 operator/(float a)const {
		return Vector3(x / a, y / a, z / a);
	}
//重载自反运算符
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

	//正交化
	void normalize() {
		float maqSq = x * x + y * y + z * z;
		if (maqSq > 0.0f) {//检查分母是否为0
			float oneOverMag = 1.0f / sqrt(maqSq);
			x *= oneOverMag;
			y *= oneOverMag;
			z *= oneOverMag;
		}
	}

	Vector3 normalize( ) const {
		float maqSq = x * x + y * y + z * z;
		if (maqSq > 0.0f) {//检查分母是否为0
			float oneOverMag = 1.0f / sqrt(maqSq);
			return Vector3(x*oneOverMag, y*oneOverMag, z*oneOverMag);
		}
		return Vector3(0.0f, 0.0f, 0.0f);
	}

//与向量的运算
	float operator*(const Vector3& a) const{
		return x*a.x+ y*a.y+ z*a.z;
	}
};

//类内定义的函数默认声明为内联函数，但是否内联处理看编译器(直接将整个函数体的代码插人调用语句处)
//类外函数需要显示声明为内联函数

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

//全局零向量
extern const Vector3 kZeroVector;