/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  Your name:
  <write your name here>

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"
#include "helper.h"

#include <stdint.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// camera parameters
double Theta = PI / 2.0;  // y-axis
double Phi = 0; 
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton, g_iMiddleMouseButton, g_iRightMouseButton;

// number of images saved to disk so far
int sprite = 0;

// these variables control what is displayed on screen
int shear = 0, bend = 0, structural = 1, pause = 0, viewingMode = 0, saveScreenToFile = 1;

struct world jello;

int windowWidth, windowHeight;

// springs
vector<Spring> springs;
vector<Plane> planes;

// interaction
Vector3d mouseForce;

// FPS
system_clock::time_point lastTime = system_clock::now();
int frames = 0;

// textureIndices
struct TextureImage
{
	int textureWidth, textureHeight;
	unsigned char* data;
};
GLuint textureIndices[5];
TextureImage backTexture;
TextureImage topTexture;
TextureImage downTexture;
TextureImage leftTexture;
TextureImage rightTexture;

void LoadTexture()
{
	glGenTextures(5, textureIndices);
	
	glBindTexture(GL_TEXTURE_2D, textureIndices[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, backTexture.textureWidth,
		backTexture.textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE,
		backTexture.data);

	glBindTexture(GL_TEXTURE_2D, textureIndices[1]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, topTexture.textureWidth,
		topTexture.textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE,
		topTexture.data);

	glBindTexture(GL_TEXTURE_2D, textureIndices[2]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, downTexture.textureWidth,
		downTexture.textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE,
		downTexture.data);

	glBindTexture(GL_TEXTURE_2D, textureIndices[3]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, leftTexture.textureWidth,
		leftTexture.textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE,
		leftTexture.data);

	glBindTexture(GL_TEXTURE_2D, textureIndices[4]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rightTexture.textureWidth,
		rightTexture.textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE,
		rightTexture.data);
}


void myinit()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(90.0, 1.0, 0.01, 1000.0);

	// set background color to grey
	glClearColor(0.5, 0.5, 0.5, 0.0);

	glCullFace(GL_BACK);
	glEnable(GL_CULL_FACE);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	 
	// texture
	LoadTexture();

	// jello code
	createSprings(springs);
	createPlanes(planes);
	if (jello.incPlanePresent) planes.emplace_back(jello.a, jello.b, jello.c, jello.d, -1);

	return;
}

void reshape(int w, int h)
{
	// Prevent a divide by zero, when h is zero.
	// You can't make a window of zero height.
	if (h == 0)
		h = 1;

	glViewport(0, 0, w, h);

	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set the perspective
	double aspectRatio = 1.0 * w / h;
	gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	windowWidth = w;
	windowHeight = h;

	glutPostRedisplay();
}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// camera parameters are Phi, Theta, R
	gluLookAt(R * cos(Phi) * cos(Theta), R * sin(Phi) * cos(Theta), R * sin(Theta),
		0.0, 0.0, 0.0, 0.0, 1.0, 0.0);


	/* Lighting */
	/* You are encouraged to change lighting parameters or make improvements/modifications
	   to the lighting model .
	   This way, you will personalize your assignment and your assignment will stick out.
	*/

	// global ambient light
	GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };

	GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

	GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
	GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

	GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
	GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

	GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
	GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

	GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
	GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

	GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
	GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

	GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

	GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
	GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };
	
	//// white light
	//GLfloat lKa0[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

	//GLfloat lKa1[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKd1[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKs1[] = { 1.0, 1.0, 1.0, 1.0 };

	//GLfloat lKa2[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKd2[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKs2[] = { 1.0, 1.0, 1.0, 1.0 };

	//GLfloat lKa3[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKd3[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKs3[] = { 1.0, 1.0, 1.0, 1.0 };

	//GLfloat lKa4[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKd4[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKs4[] = { 1.0, 1.0, 1.0, 1.0 };

	//GLfloat lKa5[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKd5[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKs5[] = { 1.0, 1.0, 1.0, 1.0 };

	//GLfloat lKa6[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

	//GLfloat lKa7[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKd7[] = { 1.0, 1.0, 1.0, 1.0 };
	//GLfloat lKs7[] = { 1.0, 1.0, 1.0, 1.0 };

	// light positions and directions
	GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
	GLfloat lP1[] = { 1.999, -1.999, -1.999, 1.0 };
	GLfloat lP2[] = { 1.999, 1.999, -1.999, 1.0 };
	GLfloat lP3[] = { -1.999, 1.999, -1.999, 1.0 };
	GLfloat lP4[] = { -1.999, -1.999, 1.999, 1.0 };
	GLfloat lP5[] = { 1.999, -1.999, 1.999, 1.0 };
	GLfloat lP6[] = { 1.999, 1.999, 1.999, 1.0 };
	GLfloat lP7[] = { -1.999, 1.999, 1.999, 1.0 };

	// jelly material color
	GLfloat mKa[] = { 0.2, 0.2, 0.2, 0.85 };
	GLfloat mKd[] = { 0.5, 0.5, 0.5, 0.85 };
	GLfloat mKs[] = { 1.0, 1.0, 1.0, 0.85 };
	GLfloat mKe[] = { 0.2, 0.2, 0.2, 0.85 };

	/* set up lighting */
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
	glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

	// set up cube color
	glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
	glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
	glMaterialf(GL_FRONT, GL_SHININESS, 120);

	// macro to set up light i
#define LIGHTSETUP(i)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
  glEnable(GL_LIGHT##i)

	LIGHTSETUP(0);
	LIGHTSETUP(1);
	LIGHTSETUP(2);
	LIGHTSETUP(3);
	LIGHTSETUP(4);
	LIGHTSETUP(5);
	LIGHTSETUP(6);
	LIGHTSETUP(7);


	glEnable(GL_DEPTH_TEST);

	// show the bounding box
	showBoundingBox();

	// show the inclined plane
	showIncPlane();

	// show the cube, the rendering order matters when blending is enable
	glEnable(GL_LIGHTING);
	showCube(&jello);
	glDisable(GL_LIGHTING);

	// show FPS
	system_clock::time_point currentTime = system_clock::now();
	frames++;
	duration<double> deltaTime = currentTime - lastTime;
	if (deltaTime.count() >= 1.0)
	{
		cout << "FPS:" << int(double(frames) / deltaTime.count()) << endl;
		frames = 0;
		lastTime = system_clock::now();
	}

	glutSwapBuffers();
}

void doIdle()
{
	char s[30] = "results/picxxxx.ppm";
	int i;

	// save screen to file
	s[11] = 48 + (sprite / 1000);
	s[12] = 48 + (sprite % 1000) / 100;
	s[13] = 48 + (sprite % 100) / 10;
	s[14] = 48 + sprite % 10;

	if (saveScreenToFile == 1)
	{
		saveScreenshot(windowWidth, windowHeight, s);
		saveScreenToFile = 0; // save only once, change this if you want continuos image generation (i.e. animation)
		sprite++;
	}

	if (sprite >= 300) // allow only 300 snapshots
	{
		exit(0);
	}

	if (pause == 0)
	{
		// insert code which appropriately performs one step of the cube simulation:
		if (strcmp(jello.integrator, "RK4") == 0)
			RK4(&jello);
		else if (strcmp(jello.integrator, "Euler") == 0)
			Euler(&jello);
		else
		{
			printf("Wrong integrator!\n");
			//exit(0);
		}
	}

	glutPostRedisplay();
}

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		printf("Oops! You didn't say the jello world file!\n");
		printf("Usage: %s [worldfile]\n", argv[0]);
		//exit(0);
	}

	readWorld(argv[1], &jello);

	// load textureIndices image
	backTexture.data = stbi_load("texture/back.png", &backTexture.textureWidth, &backTexture.textureHeight, nullptr, 3);
	topTexture.data = stbi_load("texture/top.png", &topTexture.textureWidth, &topTexture.textureHeight, nullptr, 3);
	downTexture.data = stbi_load("texture/down.png", &downTexture.textureWidth, &downTexture.textureHeight, nullptr, 3);
	leftTexture.data = stbi_load("texture/left.png", &leftTexture.textureWidth, &leftTexture.textureHeight, nullptr, 3);
	rightTexture.data = stbi_load("texture/right.png", &rightTexture.textureWidth, &rightTexture.textureHeight, nullptr, 3);

	glutInit(&argc, argv);

	/* double buffered window, use depth testing, 640x480 */
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	windowWidth = 1280;
	windowHeight = 960;
	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Jello cube");

	/* tells glut to use a particular display function to redraw */
	glutDisplayFunc(display);

	/* replace with any animate code */
	glutIdleFunc(doIdle);

	/* callback for mouse drags */
	glutMotionFunc(mouseMotionDrag);

	/* callback for window size changes */
	glutReshapeFunc(reshape);

	/* callback for mouse movement */
	glutPassiveMotionFunc(mouseMotion);

	/* callback for mouse button changes */
	glutMouseFunc(mouseButton);

	/* register for keyboard events */
	glutKeyboardFunc(keyboardFunc);

	/* do initialization */
	myinit();

	/* forever sink in the black hole */
	glutMainLoop();

	// free image
	stbi_image_free(backTexture.data);
	stbi_image_free(topTexture.data);
	stbi_image_free(downTexture.data);
	stbi_image_free(leftTexture.data);
	stbi_image_free(rightTexture.data);

	return(0);
}

