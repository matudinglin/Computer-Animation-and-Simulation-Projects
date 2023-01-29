#pragma once

#include <cmath>
#include <vector>
#include <iostream>
using std::cout;
using std::endl;
using std::sqrt;
using std::vector;


// Some Useful structs & functions 

struct Vector3d
{
	double x, y, z;

	Vector3d() = default;
	Vector3d(double _x, double _y, double _z) :x(_x), y(_y), z(_z) {};
	~Vector3d() = default;

	Vector3d operator-() const { return Vector3d(-x, -y, -z); }
	Vector3d operator-(const Vector3d& v) const { return Vector3d(x - v.x, y - v.y, z - v.z); }
	friend Vector3d operator-(double c, const Vector3d& v)
	{
		return Vector3d(1.0 - v.x, 1.0 - v.y, 1.0 - v.z);
	}
	Vector3d& operator-=(const Vector3d& v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}
	Vector3d operator+(const Vector3d& v) const { return Vector3d(x + v.x, y + v.y, z + v.z); }
	friend Vector3d operator+(double c, const Vector3d& v)
	{
		return Vector3d(1.0 + v.x, 1.0 + v.y, 1.0 + v.z);
	}
	Vector3d& operator+=(const Vector3d& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	Vector3d operator*(double c) const { return Vector3d(c * x, c * y, c * z); }
	friend Vector3d operator*(double c, const Vector3d& v)
	{
		return Vector3d(c * v.x, c * v.y, c * v.z);
	}
	Vector3d& operator*=(double c)
	{
		x *= c;
		y *= c;
		z *= c;
		return *this;
	}
	Vector3d operator/(double c) const { return Vector3d(x / c, y / c, z / c); }
	friend Vector3d operator/(double c, const Vector3d& v)
	{
		return Vector3d(c / v.x, c / v.y, c / v.z);
	}
	Vector3d& operator/=(double c)
	{
		x /= c;
		y /= c;
		z /= c;
		return *this;
	}


	double length() const
	{
		double temp = x * x + y * y + z * z;
		return sqrt(temp);
	}

	Vector3d normalized()
	{
		Vector3d temp = *this / length();
		return temp;
	}
};

double dot(const Vector3d& v1, const Vector3d& v2);
Vector3d cross(const Vector3d& v1, const Vector3d& v2);


struct Spring
{
	int x1, y1, z1;
	int x2, y2, z2;
	double rest;

	Spring() = default;
	Spring(int _x1, int _y1, int _z1, int _x2, int _y2, int _z2, double _rest)
		:x1(_x1), y1(_y1), z1(_z1), x2(_x2), y2(_y2), z2(_z2), rest(_rest) {};
	~Spring() = default;

};

vector<Spring> createSprings();