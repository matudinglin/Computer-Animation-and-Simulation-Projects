/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _JELLO_H_
#define _JELLO_H_

#include "openGL-headers.h"
#include "pic.h"
#include "helper.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>

using namespace std::chrono;

#define pi 3.141592653589793238462643383279 

// camera angles
extern double Theta;
extern double Phi;
extern double R;

// number of images saved to disk so far
extern int sprite;

// mouse control
extern int g_vMousePos[2];
extern int g_iLeftMouseButton, g_iMiddleMouseButton, g_iRightMouseButton;

// these variables control what is displayed on the screen
extern int shear, bend, structural, pause, viewingMode, saveScreenToFile;

// springs
extern vector<Spring> springs;
extern vector<Plane> planes;

// interaction
extern Vector3d mouseForce;

// FPS
extern system_clock::time_point lastTime;
extern int frames;

struct world
{
	char integrator[10]; // "RK4" or "Euler"
	double dt; // timestep, e.g.. 0.001
	int n; // display only every nth timepoint
	double kElastic; // Hook's elasticity coefficient for all springs except collision springs
	double dElastic; // Damping coefficient for all springs except collision springs
	double kCollision; // Hook's elasticity coefficient for collision springs
	double dCollision; // Damping coefficient collision springs
	double mass; // mass of each of the 512 control points, mass assumed to be equal for every control Vector3d
	int incPlanePresent; // Is the inclined plane present? 1 = YES, 0 = NO (always NO in this assignment)
	double a, b, c, d; // inclined plane has equation a * x + b * y + c * z + d = 0; if no inclined plane, these four fields are not used
	int resolution; // resolution for the 3d grid specifying the external force field; value of 0 means that there is no force field
	struct Vector3d* forceField; // pointer to the array of values of the force field
	struct Vector3d p[8][8][8]; // position of the 512 control points
	struct Vector3d v[8][8][8]; // velocities of the 512 control points
};

extern struct world jello;

// computes crossproduct of three vectors, which are given as points
// struct Vector3d vector1, vector2, dest
// result goes into dest
#define CROSSPRODUCTp(vector1,vector2,dest)\
  CROSSPRODUCT( (vector1).x, (vector1).y, (vector1).z,\
                (vector2).x, (vector2).y, (vector2).z,\
                (dest).x, (dest).y, (dest).z )

// computes crossproduct of three vectors, which are specified by floating-Vector3d coordinates
// double x1,y1,z1,x2,y2,z2,x,y,z
// result goes into x,y,z
#define CROSSPRODUCT(x1,y1,z1,x2,y2,z2,x,y,z)\
\
  x = (y1) * (z2) - (y2) * (z1);\
  y = (x2) * (z1) - (x1) * (z2);\
  z = (x1) * (y2) - (x2) * (y1)

// normalizes vector dest
// struct Vector3d dest
// result returned in dest
// must declare a double variable called 'length' somewhere inside the scope of the NORMALIZE macrp
// macro will change that variable
#define pNORMALIZE(dest)\
\
  length = sqrt((dest).x * (dest).x + (dest).y * (dest).y + (dest).z * (dest).z);\
  (dest).x /= length;\
  (dest).y /= length;\
  (dest).z /= length;

// copies vector source to vector dest
// struct Vector3d source,dest
#define pCPY(source,dest)\
\
  (dest).x = (source).x;\
  (dest).y = (source).y;\
  (dest).z = (source).z;

// assigns values x,y,z to Vector3d vector dest
// struct Vector3d dest
// double x,y,z
#define pMAKE(x,y,z,dest)\
\
  (dest).(x) = (x);\
  (dest).(y) = (y);\
  (dest).(z) = (z);

// sums points src1 and src2 to dest
// struct Vector3d src1,src2,dest
#define pSUM(src1,src2,dest)\
\
  (dest).x = (src1).x + (src2).x;\
  (dest).y = (src1).y + (src2).y;\
  (dest).z = (src1).z + (src2).z;

// dest = src2 - src1
// struct Vector3d src1,src2,dest
#define pDIFFERENCE(src1,src2,dest)\
\
  (dest).x = (src1).x - (src2).x;\
  (dest).y = (src1).y - (src2).y;\
  (dest).z = (src1).z - (src2).z;

// mulitplies components of Vector3d src by scalar and returns the result in dest
// struct Vector3d src,dest
// double scalar
#define pMULTIPLY(src,scalar,dest)\
\
  (dest).x = (src).x * (scalar);\
  (dest).y = (src).y * (scalar);\
  (dest).z = (src).z * (scalar);

#endif

