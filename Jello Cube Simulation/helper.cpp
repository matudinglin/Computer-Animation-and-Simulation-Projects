#include "helper.h"
#include "jello.h"

double dot(const Vector3d& v1, const Vector3d& v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }

Vector3d cross(const Vector3d& v1, const Vector3d& v2)
{
	Vector3d v;
	v.x = v1.y * v2.z - v1.z * v2.y;
	v.y = v1.z * v2.x - v1.x * v2.z;
	v.z = v1.x * v2.y - v1.y * v2.x;
	return v;
}

void createOriginalSprings(vector<Spring>& springs)
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


//void createCollisionSprings(vector<Spring>& springs, const vector<Plane> &planes, const struct world* jello)
//{
//
//}