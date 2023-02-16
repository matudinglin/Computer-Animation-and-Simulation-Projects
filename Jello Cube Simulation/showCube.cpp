/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"



int pointMap(int side, int i, int j)
{
	int r;

	switch (side)
	{
	case 1: //[i][j][0] bottom face
		r = 64 * i + 8 * j;
		break;
	case 6: //[i][j][7] top face
		r = 64 * i + 8 * j + 7;
		break;
	case 2: //[i][0][j] front face
		r = 64 * i + j;
		break;
	case 5: //[i][7][j] back face
		r = 64 * i + 56 + j;
		break;
	case 3: //[0][i][j] left face
		r = 8 * i + j;
		break;
	case 4: //[7][i][j] right face
		r = 448 + 8 * i + j;
		break;
	}

	return r;
}

void showCube(struct world* jello)
{
	int i, j, k, ip, jp, kp;
	Vector3d r1, r2, r3; // aux variables

	/* normals buffer and counter for Gourad shading*/
	struct Vector3d normal[8][8];
	int counter[8][8];

	int face;
	double faceFactor, length;

	if (fabs(jello->p[0][0][0].x) > 10)
	{
		printf("Your cube somehow escaped way out of the box.\n");
		//exit(0);
	}


#define NODE(face,i,j) (*((struct Vector3d * )(jello->p) + pointMap((face),(i),(j))))


#define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\


	if (viewingMode == 0) // render wireframe
	{
		glLineWidth(1);
		glPointSize(5);
		glDisable(GL_LIGHTING);
		for (i = 0; i <= 7; i++)
			for (j = 0; j <= 7; j++)
				for (k = 0; k <= 7; k++)
				{
					if (i * j * k * (7 - i) * (7 - j) * (7 - k) != 0) // not surface Vector3d
						continue;

					glBegin(GL_POINTS); // draw Vector3d
					glColor4f(0, 0, 0, 0);
					glVertex3f(jello->p[i][j][k].x, jello->p[i][j][k].y, jello->p[i][j][k].z);
					glEnd();

					//
					//if ((i!=7) || (j!=7) || (k!=7))
					//  continue;

					glBegin(GL_LINES);
					// structural
					if (structural == 1)
					{
						glColor4f(0, 0, 1, 1);
						PROCESS_NEIGHBOUR(1, 0, 0);
						PROCESS_NEIGHBOUR(0, 1, 0);
						PROCESS_NEIGHBOUR(0, 0, 1);
						PROCESS_NEIGHBOUR(-1, 0, 0);
						PROCESS_NEIGHBOUR(0, -1, 0);
						PROCESS_NEIGHBOUR(0, 0, -1);
					}

					// shear
					if (shear == 1)
					{
						glColor4f(0, 1, 0, 1);
						PROCESS_NEIGHBOUR(1, 1, 0);
						PROCESS_NEIGHBOUR(-1, 1, 0);
						PROCESS_NEIGHBOUR(-1, -1, 0);
						PROCESS_NEIGHBOUR(1, -1, 0);
						PROCESS_NEIGHBOUR(0, 1, 1);
						PROCESS_NEIGHBOUR(0, -1, 1);
						PROCESS_NEIGHBOUR(0, -1, -1);
						PROCESS_NEIGHBOUR(0, 1, -1);
						PROCESS_NEIGHBOUR(1, 0, 1);
						PROCESS_NEIGHBOUR(-1, 0, 1);
						PROCESS_NEIGHBOUR(-1, 0, -1);
						PROCESS_NEIGHBOUR(1, 0, -1);

						PROCESS_NEIGHBOUR(1, 1, 1)
							PROCESS_NEIGHBOUR(-1, 1, 1)
							PROCESS_NEIGHBOUR(-1, -1, 1)
							PROCESS_NEIGHBOUR(1, -1, 1)
							PROCESS_NEIGHBOUR(1, 1, -1)
							PROCESS_NEIGHBOUR(-1, 1, -1)
							PROCESS_NEIGHBOUR(-1, -1, -1)
							PROCESS_NEIGHBOUR(1, -1, -1)
					}

					// bend
					if (bend == 1)
					{
						glColor4f(1, 0, 0, 1);
						PROCESS_NEIGHBOUR(2, 0, 0);
						PROCESS_NEIGHBOUR(0, 2, 0);
						PROCESS_NEIGHBOUR(0, 0, 2);
						PROCESS_NEIGHBOUR(-2, 0, 0);
						PROCESS_NEIGHBOUR(0, -2, 0);
						PROCESS_NEIGHBOUR(0, 0, -2);
					}
					glEnd();
				}
		glEnable(GL_LIGHTING);
	}

	else
	{
		glPolygonMode(GL_FRONT, GL_FILL);

		for (face = 1; face <= 6; face++)
			// face == face of a cube
			// 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
		{

			if ((face == 1) || (face == 3) || (face == 5))
				faceFactor = -1; // flip orientation
			else
				faceFactor = 1;


			for (i = 0; i <= 7; i++) // reset buffers
				for (j = 0; j <= 7; j++)
				{
					normal[i][j].x = 0; normal[i][j].y = 0; normal[i][j].z = 0;
					counter[i][j] = 0;
				}

			/* process triangles, accumulate normals for Gourad shading */

			for (i = 0; i <= 6; i++)
				for (j = 0; j <= 6; j++) // process block (i,j)
				{
					pDIFFERENCE(NODE(face, i + 1, j), NODE(face, i, j), r1); // first triangle
					pDIFFERENCE(NODE(face, i, j + 1), NODE(face, i, j), r2);
					CROSSPRODUCTp(r1, r2, r3); pMULTIPLY(r3, faceFactor, r3);
					pNORMALIZE(r3);
					pSUM(normal[i + 1][j], r3, normal[i + 1][j]);
					counter[i + 1][j]++;
					pSUM(normal[i][j + 1], r3, normal[i][j + 1]);
					counter[i][j + 1]++;
					pSUM(normal[i][j], r3, normal[i][j]);
					counter[i][j]++;

					pDIFFERENCE(NODE(face, i, j + 1), NODE(face, i + 1, j + 1), r1); // second triangle
					pDIFFERENCE(NODE(face, i + 1, j), NODE(face, i + 1, j + 1), r2);
					CROSSPRODUCTp(r1, r2, r3); pMULTIPLY(r3, faceFactor, r3);
					pNORMALIZE(r3);
					pSUM(normal[i + 1][j], r3, normal[i + 1][j]);
					counter[i + 1][j]++;
					pSUM(normal[i][j + 1], r3, normal[i][j + 1]);
					counter[i][j + 1]++;
					pSUM(normal[i + 1][j + 1], r3, normal[i + 1][j + 1]);
					counter[i + 1][j + 1]++;
				}

			// transparent
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			/* the actual rendering */
			for (j = 1; j <= 7; j++)
			{
				if (faceFactor > 0)
					glFrontFace(GL_CCW); // the usual definition of front face
				else
					glFrontFace(GL_CW); // flip definition of orientation

				glBegin(GL_TRIANGLE_STRIP);
				for (i = 0; i <= 7; i++)
				{
					glNormal3f(normal[i][j].x / counter[i][j], normal[i][j].y / counter[i][j], normal[i][j].z / counter[i][j]);
					glVertex3f(NODE(face, i, j).x, NODE(face, i, j).y, NODE(face, i, j).z);
					glNormal3f(normal[i][j - 1].x / counter[i][j - 1], normal[i][j - 1].y / counter[i][j - 1], normal[i][j - 1].z / counter[i][j - 1]);
					glVertex3f(NODE(face, i, j - 1).x, NODE(face, i, j - 1).y, NODE(face, i, j - 1).z);
				}
				glEnd();

			}
			glDisable(GL_BLEND);
		}
	} // end for loop over faces
	glFrontFace(GL_CCW);
}

void showIncPlane()
{
	// show inclined plane
	if (jello.incPlanePresent)
	{
		Plane incPlane = Plane(jello.a, jello.b, jello.c, jello.d, -1);
		// find intersections
		vector<Vector3d> intersections;
		{
			auto solveX = [&incPlane](double y, double z)->double {return (-incPlane.d - incPlane.b * y - incPlane.c * z) / incPlane.a; };
			double axisX[4][2] = { {2,2}, {2,-2},{-2,-2},{-2,2} };
			for (int i = 0; i < 4; ++i)
			{
				double x = solveX(axisX[i][0], axisX[i][1]);
				if (x <= 2 && x >= -2) intersections.emplace_back(x, axisX[i][0], axisX[i][1]);
			}
		}
		{
			auto solveY = [&incPlane](double x, double z)->double {return (-incPlane.d - incPlane.a * x - incPlane.c * z) / incPlane.b; };
			double axisY[4][2] = { {2,2}, {2,-2},{-2,-2},{-2,2} };
			for (int i = 0; i < 4; ++i)
			{
				double y = solveY(axisY[i][0], axisY[i][1]);
				if (y <= 2 && y >= -2) intersections.emplace_back(axisY[i][0], y, axisY[i][1]);
			}
		}
		{
			auto solveZ = [&incPlane](double x, double y)->double {return (-incPlane.d - incPlane.a * x - incPlane.b * y) / incPlane.c; };
			double axisZ[4][2] = { {2,2}, {2,-2},{-2,-2},{-2,2} };
			for (int i = 0; i < 4; ++i)
			{
				double z = solveZ(axisZ[i][0], axisZ[i][1]);
				if (z <= 2 && z >= -2) intersections.emplace_back(axisZ[i][0], axisZ[i][1], z);
			}
		}

		glColor4f(0.5, 0, 0, 0.7);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glDisable(GL_CULL_FACE);
		glBegin(GL_TRIANGLE_STRIP);
		for (const Vector3d& inter : intersections)
		{
			glVertex3f(inter.x, inter.y, inter.z);
		}
		glEnd();
		glEnable(GL_CULL_FACE);
		glDisable(GL_BLEND);
	}
}

void showBoundingBox()
{
	int i, j;

	glColor4f(0.6, 0.6, 0.6, 0);

	glDisable(GL_CULL_FACE);
	glEnable(GL_TEXTURE_2D);
	
	glBindTexture(GL_TEXTURE_2D, textureIndices[0]);
	glBegin(GL_QUADS);
	// back
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-2.0f, -2.0f, -2.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-2.0f, 2.0f, -2.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(2.0f, 2.0f, -2.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(2.0f, -2.0f, -2.0f);

	// left
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-2.0f, -2.0f, 2.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(-2.0f, 2.0f, 2.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(-2.0f, 2.0f, -2.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-2.0f, -2.0f, -2.0f);

	// right
	glTexCoord2f(0.0f, 0.0f); glVertex3f(2.0f, -2.0f, -2.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(2.0f, 2.0f, -2.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(2.0f, 2.0f, 2.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(2.0f, -2.0f, 2.0f);

	// up
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-2.0f, 2.0f, -2.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(2.0f, 2.0f, -2.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(2.0f, 2.0f, 2.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-2.0f, 2.0f, 2.0f);
	glEnd();


	glBindTexture(GL_TEXTURE_2D, textureIndices[1]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f); glVertex3f(-2.0f, -2.0f, -2.0f);
	glTexCoord2f(0.0f, 1.0f); glVertex3f(2.0f, -2.0f, -2.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex3f(2.0f, -2.0f, 2.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex3f(-2.0f, -2.0f, 2.0f);
	glEnd();
	
	glDisable(GL_TEXTURE_2D);
	
	return;
}

