/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"
#include "helper.h"

/* Computes acceleration to every control Vector3d of the jello cube,
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world* jello, struct Vector3d a[8][8][8])
{
	// initialize 
	for (int i = 0; i < 8; ++i)
		for (int j = 0; j < 8; ++j)
			for (int k = 0; k < 8; ++k)
			{
				a[i][j][k] = Vector3d(0, 0, 0);
			}

	double hook = jello->kElastic;
	double damp = jello->dElastic;
	Vector3d *forceField = jello->forceField;

	// add collision force
	vector<Plane> planes;
	planes.emplace_back(1, 0, 0, 2, -1);
	planes.emplace_back(1, 0, 0, -2, 1);
	planes.emplace_back(0, 1, 0, 2, -1);
	planes.emplace_back(0, 1, 0, -2, 1);
	planes.emplace_back(0, 0, 1, 2, -1);
	planes.emplace_back(0, 0, 1, -2, 1);
	//if (jello->incPlanePresent) planes.emplace_back(jello->a, jello->b, jello->c, jello->d, -1);
	for (int i = 0; i < 8; ++i)
		for (int j = 0; j < 8; ++j)
			for (int k = 0; k < 8; ++k)
			{
				Vector3d pos = jello->p[i][j][k];
				for (const Plane& plane : planes)
				{
					if (plane.checkInside(pos))
					{
						// point i, j, k collide with plane 
						Vector3d springF, dampF;
						// add spring force
						double R = 0.0;
						Vector3d normal = Vector3d(plane.a, plane.b, plane.c).normalized();
						double L = (plane.a * pos.x + plane.b * pos.y + plane.c * pos.z)
							/ sqrt(plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);
						springF = -hook * (L - R) * normal;
						a[i][j][k] += springF;
						// add damping force
						dampF = -damp * dot(jello->v[i][j][k], normal) * normal;
						a[i][j][k] += dampF;
					}
				}
			}

	// add spring force
	for (const auto& spring : springs)
	{
		Vector3d springF, dampF;
		// add spring force
		double R = spring.rest;
		int xa = spring.x1, ya = spring.y1, za = spring.z1;
		int xb = spring.x2, yb = spring.y2, zb = spring.z2;
		Vector3d vecL = (jello->p[xa][ya][za] - jello->p[xb][yb][zb]);
		double L = vecL.length();
		springF = -hook * (L - R) * (vecL / L);
		a[xa][ya][za] += springF;
		a[xb][yb][zb] += -springF;
		// add damping force
		Vector3d vecVa = jello->v[xa][ya][za], vecVb = jello->v[xb][yb][zb];
		dampF = -damp * dot(vecVa - vecVb, vecL) / L * (vecL / L);
		a[xa][ya][za] += dampF;
		a[xb][yb][zb] += -dampF;
	}
	
	// add force field
	int unit = jello->resolution;
	for (int i = 0; i < 8; ++i)
		for (int j = 0; j < 8; ++j)
			for (int k = 0; k < 8; ++k)
			{

			}

	auto res = jello->resolution;
	if (jello->resolution != 0)
	{
		double grid = 4.0 / double(jello->resolution - 1);
		// for every point 
		for (int i = 0; i < 8; ++i)
			for (int j = 0; j < 8; ++j)
				for (int k = 0; k < 8; ++k)
				{
					Vector3d pos = jello->p[i][j][k];
					int fi, fj, fk;
					fi = (pos.x+2) / grid; fj = (pos.y+2) / grid; fk = (pos.z +2) / grid;
					Vector3d f000 = jello->forceField[fi * res + fj * res + fk * res];
					Vector3d f001 = jello->forceField[fi * res + fj * res + (fk + 1) * res];
					Vector3d f010 = jello->forceField[fi * res + (fj + 1)* res + fk * res];
					Vector3d f011 = jello->forceField[fi * res + (fj + 1) * res + (fk + 1) * res];
					Vector3d f100 = jello->forceField[(fi+1) * res + fj * res + fk * res];
					Vector3d f101 = jello->forceField[(fi + 1) * res + fj * res + (fk + 1) * res];
					Vector3d f110 = jello->forceField[(fi + 1) * res + (fj + 1) * res + fk * res];
					Vector3d f111 = jello->forceField[(fi + 1) * res + (fj + 1) * res + (fk + 1) * res];
					// Trilinear interpolation
					a[i][j][k] += 0.1*trilinearInterpolation(pos.x, pos.y, pos.z,
						f000.x, f111.x,
						f000.y, f111.y,
						f000.z, f111.z,
						f000, f001,
						f010, f011,
						f100, f101,
						f110, f111);
				}
	}

	// compute acceleration for every control point 
	for (int i = 0; i < 8; ++i)
		for (int j = 0; j < 8; ++j)
			for (int k = 0; k < 8; ++k)
			{
				a[i][j][k] /= jello->mass;
			}
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world* jello)
{
	int i, j, k;
	Vector3d a[8][8][8];

	computeAcceleration(jello, a);

	for (i = 0; i <= 7; i++)
		for (j = 0; j <= 7; j++)
			for (k = 0; k <= 7; k++)
			{
				jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
				jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
				jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
				jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
				jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
				jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

			}
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world* jello)
{
	Vector3d F1p[8][8][8], F1v[8][8][8],
		F2p[8][8][8], F2v[8][8][8],
		F3p[8][8][8], F3v[8][8][8],
		F4p[8][8][8], F4v[8][8][8];

	Vector3d a[8][8][8];


	struct world buffer;

	int i, j, k;

	buffer = *jello; // make a copy of jello

	computeAcceleration(jello, a);

	for (i = 0; i <= 7; i++)
		for (j = 0; j <= 7; j++)
			for (k = 0; k <= 7; k++)
			{
				pMULTIPLY(jello->v[i][j][k], jello->dt, F1p[i][j][k]);
				pMULTIPLY(a[i][j][k], jello->dt, F1v[i][j][k]);
				pMULTIPLY(F1p[i][j][k], 0.5, buffer.p[i][j][k]);
				pMULTIPLY(F1v[i][j][k], 0.5, buffer.v[i][j][k]);
				pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
				pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
			}

	computeAcceleration(&buffer, a);

	for (i = 0; i <= 7; i++)
		for (j = 0; j <= 7; j++)
			for (k = 0; k <= 7; k++)
			{
				// F2p = dt * buffer.v;
				pMULTIPLY(buffer.v[i][j][k], jello->dt, F2p[i][j][k]);
				// F2v = dt * a(buffer.p,buffer.v);     
				pMULTIPLY(a[i][j][k], jello->dt, F2v[i][j][k]);
				pMULTIPLY(F2p[i][j][k], 0.5, buffer.p[i][j][k]);
				pMULTIPLY(F2v[i][j][k], 0.5, buffer.v[i][j][k]);
				pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
				pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
			}

	computeAcceleration(&buffer, a);

	for (i = 0; i <= 7; i++)
		for (j = 0; j <= 7; j++)
			for (k = 0; k <= 7; k++)
			{
				// F3p = dt * buffer.v;
				pMULTIPLY(buffer.v[i][j][k], jello->dt, F3p[i][j][k]);
				// F3v = dt * a(buffer.p,buffer.v);     
				pMULTIPLY(a[i][j][k], jello->dt, F3v[i][j][k]);
				pMULTIPLY(F3p[i][j][k], 1.0, buffer.p[i][j][k]);
				pMULTIPLY(F3v[i][j][k], 1.0, buffer.v[i][j][k]);
				pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
				pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
			}

	computeAcceleration(&buffer, a);


	for (i = 0; i <= 7; i++)
		for (j = 0; j <= 7; j++)
			for (k = 0; k <= 7; k++)
			{
				// F3p = dt * buffer.v;
				pMULTIPLY(buffer.v[i][j][k], jello->dt, F4p[i][j][k]);
				// F3v = dt * a(buffer.p,buffer.v);     
				pMULTIPLY(a[i][j][k], jello->dt, F4v[i][j][k]);

				pMULTIPLY(F2p[i][j][k], 2, buffer.p[i][j][k]);
				pMULTIPLY(F3p[i][j][k], 2, buffer.v[i][j][k]);
				pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], F1p[i][j][k], buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], F4p[i][j][k], buffer.p[i][j][k]);
				pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], jello->p[i][j][k], jello->p[i][j][k]);

				pMULTIPLY(F2v[i][j][k], 2, buffer.p[i][j][k]);
				pMULTIPLY(F3v[i][j][k], 2, buffer.v[i][j][k]);
				pSUM(buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], F1v[i][j][k], buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], F4v[i][j][k], buffer.p[i][j][k]);
				pMULTIPLY(buffer.p[i][j][k], 1.0 / 6, buffer.p[i][j][k]);
				pSUM(buffer.p[i][j][k], jello->v[i][j][k], jello->v[i][j][k]);
			}

	return;
}
