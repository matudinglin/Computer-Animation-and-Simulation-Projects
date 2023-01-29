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

vector<Spring> createSprings()
{
	vector<Spring> springs;

	// create structural springs
	{
		double rest = 1.0 / 7.0;
		for (int i = 0; i < 8; ++i)
		{
			for (int j = 0; j < 8; ++j)
			{
				for (int k = 0; k < 8; ++k)
				{
					for (int l = 0; l < 7; ++l)
					{
						springs.emplace_back(i + l, j, k, i + l + 1, j, k, rest);
						springs.emplace_back(i, j + l, k, i, j + l + 1, k, rest);
						springs.emplace_back(i, j, k + l, i, j, k + l + 1, rest);
					}
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
				for (int k = 0; k < 8; ++k)
				{
					for (int l = 0; l < 6; ++l)
					{
						springs.emplace_back(i + l, j, k, i + l + 2, j, k, rest);
						springs.emplace_back(i, j + l, k, i, j + l + 2, k, rest);
						springs.emplace_back(i, j, k + l, i, j, k + l + 2, rest);
					}
				}
			}
		}
	}


	return springs;
};