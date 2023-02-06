#include "helper.h"
#include "jello.h"

const double EPSILON = 1.0 / DBL_MAX;

double dot(const Vector3d& v1, const Vector3d& v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }

Vector3d cross(const Vector3d& v1, const Vector3d& v2)
{
	Vector3d v;
	v.x = v1.y * v2.z - v1.z * v2.y;
	v.y = v1.z * v2.x - v1.x * v2.z;
	v.z = v1.x * v2.y - v1.y * v2.x;
	return v;
}

Vector3d trilinearInterpolation(double x, double y, double z,
								double x1, double y1, double z1,
								double x2, double y2, double z2,
								Vector3d f000, Vector3d f001,
								Vector3d f010, Vector3d f011,
								Vector3d f100, Vector3d f101,
								Vector3d f110, Vector3d f111) 
{
	double xd = (x - x1) / (x2 - x1);
	double yd = (y - y1) / (y2 - y1);
	double zd = (z - z1) / (z2 - z1);

	Vector3d c000 = f000 * (1 - xd) + f100 * xd;
	Vector3d c001 = f001 * (1 - xd) + f101 * xd;
	Vector3d c010 = f010 * (1 - xd) + f110 * xd;
	Vector3d c011 = f011 * (1 - xd) + f111 * xd;

	Vector3d c00 = c000 * (1 - yd) + c010 * yd;
	Vector3d c01 = c001 * (1 - yd) + c011 * yd;

	Vector3d c0 = c00 * (1 - zd) + c01 * zd;

	return c0;
}


void createSprings(vector<Spring>& springs)
{
	// create structural springs
	{
		double rest = 1.0 / 7.0;
		for (int i = 0; i < 8; ++i)
		{
			for (int j = 0; j < 8; ++j)
			{
				for (int k = 0; k < 7; ++k)
				{
					springs.emplace_back(i, j, k, i, j, k + 1, rest);
				}
			}
		}
		for (int i = 0; i < 8; ++i)
		{
			for (int k = 0; k < 8; ++k)
			{
				for (int j = 0; j < 7; ++j)
				{
					springs.emplace_back(i, j, k, i, j + 1, k, rest);
				}
			}
		}
		for (int k = 0; k < 8; ++k)
		{
			for (int j = 0; j < 8; ++j)
			{
				for (int i = 0; i < 7; ++i)
				{
					springs.emplace_back(i, j, k, i + 1, j, k, rest);
				}
			}
		}


	}

	// create shear springs
	{
		double rest = 1.0 / 7.0 * sqrt(2);
		for (int i = 0; i < 7; ++i)
		{
			for (int j = 0; j < 7; ++j)
			{
				for (int k = 0; k < 7; ++k)
				{
					springs.emplace_back(i, j, k, i + 1, j + 1, k, rest);
					springs.emplace_back(i, j + 1, k, i + 1, j, k, rest);
					springs.emplace_back(i, j, k + 1, i + 1, j + 1, k + 1, rest);
					springs.emplace_back(i, j + 1, k + 1, i + 1, j, k + 1, rest);

					springs.emplace_back(i, j, k, i, j + 1, k + 1, rest);
					springs.emplace_back(i, j + 1, k, i, j, k + 1, rest);
					springs.emplace_back(i + 1, j, k, i + 1, j + 1, k + 1, rest);
					springs.emplace_back(i + 1, j + 1, k, i + 1, j, k + 1, rest);

					springs.emplace_back(i, j, k, i + 1, j, k + 1, rest);
					springs.emplace_back(i + 1, j, k, i, j, k + 1, rest);
					springs.emplace_back(i, j + 1, k, i + 1, j + 1, k + 1, rest);
					springs.emplace_back(i + 1, j + 1, k, i, j + 1, k + 1, rest);
				}
			}
		}
		rest = 1.0 / 7.0 * sqrt(3);
		for (int i = 0; i < 7; ++i)
		{
			for (int j = 0; j < 7; ++j)
			{
				for (int k = 0; k < 7; ++k)
				{
					springs.emplace_back(i, j, k, i + 1, j + 1, k + 1, rest);
					springs.emplace_back(i, j, k + 1, i + 1, j + 1, k, rest);
					springs.emplace_back(i, j + 1, k, i + 1, j, k + 1, rest);
					springs.emplace_back(i, j + 1, k + 1, i + 1, j, k, rest);
				}
			}
		}
	}

	// create bend springs
	{
		double rest = 1.0 / 7.0 * 2.0;
		for (int i = 0; i < 8; ++i)
		{
			for (int j = 0; j < 8; ++j)
			{
				for (int k = 0; k < 6; ++k)
				{
					springs.emplace_back(i, j, k, i, j, k + 2, rest);
				}
			}
		}
		for (int i = 0; i < 8; ++i)
		{
			for (int k = 0; k < 8; ++k)
			{
				for (int j = 0; j < 6; ++j)
				{
					springs.emplace_back(i, j, k, i, j + 2, k, rest);
				}
			}
		}
		for (int k = 0; k < 8; ++k)
		{
			for (int j = 0; j < 8; ++j)
			{
				for (int i = 0; i < 6; ++i)
				{
					springs.emplace_back(i, j, k, i + 2, j, k, rest);
				}
			}
		}
	}
};

void createPlanes(vector<Plane>& planes)
{
	planes.emplace_back(1, 0, 0, 2, -1);
	planes.emplace_back(1, 0, 0, -2, 1);
	planes.emplace_back(0, 1, 0, 2, -1);
	planes.emplace_back(0, 1, 0, -2, 1);
	planes.emplace_back(0, 0, 1, 2, -1);
	planes.emplace_back(0, 0, 1, -2, 1);
}



//void createCollisionSprings(vector<Spring>& springs, const vector<Plane> &planes, const struct world* jello)
//{
//
//}