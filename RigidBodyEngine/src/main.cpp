#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "../lib/glew/glew.h"
#include "../lib/freeglut/freeglut.h"
#include "Core\RigidBody.h"
#include "Core\Simulator.h"
#include "Core\WindowInfo.h"
#include "Core\Color.h"
#include "Core\maths\Quaternion.h"
#include "Core\maths\MathsUtils.h"

using namespace std;
using namespace Core;

void InitGL();
void SetCameraGL();

void RenderGL();	
void TimerGL(int id);
void KeyPressed(unsigned char key,int x, int y);
void ReshapeWindow(int width, int height);

void SafeExit();

tkSimulator *sim;
tkWindowInfo *window;

int main(int argc, char **argv)
{
	sim = new tkSimulator();
	sim->Init();

	window = new tkWindowInfo("Rigid Body - paper from Baraff & Witkin");
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow(window->GetTitle());
	glutReshapeWindow(window->GetWidth(), window->GetHeight());
	glutDisplayFunc(RenderGL);
	glutKeyboardFunc(KeyPressed);
	glutReshapeFunc(ReshapeWindow);
	glutTimerFunc(sim->ct, TimerGL, 1);

	InitGL();
	glutMainLoop();

	return 0;
}

void SafeExit()
{
	delete window;
	delete sim;
	exit(0);
}

void SetCameraGL()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, window->GetAspect(), 0.01f, 1000.0f);
	glTranslatef(0.0f, -5.0f, -35.0f);
	glViewport(0, 0, window->GetWidth(), window->GetHeight());
}

void InitGL()
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);

	SetCameraGL();
}

void RenderGL()
{
	glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glShadeModel(GL_SMOOTH);

	for (int i = 0; i < sim->NBODIES; i++)
	{
		float magOmega = sim->rBodies[i].omega.Magnitude();
		tkVec3 rotation = tkQuat::EulerAngles(sim->rBodies[i].q);
		glColor3f(Color::Green().R, Color::Green().G, Color::Green().B);

		glLoadIdentity();
		glTranslatef(sim->rBodies[i].position.x, sim->rBodies[i].position.y, sim->rBodies[i].position.z);
		glRotatef(magOmega, rotation.x, rotation.y, rotation.z);
		glScalef(sim->rBodies[i].a, sim->rBodies[i].b, sim->rBodies[i].c);
		glutSolidCube(1);
	}

	glutSwapBuffers();
}

void KeyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		SafeExit();
	}
}

void TimerGL(int id)
{
	if (id == 1)
	{
		sim->SimulationLoop();
		glutPostRedisplay();
		glutTimerFunc(sim->ct, TimerGL, 1);
	}
}

void ReshapeWindow(int width, int height)
{
	window->SetDimensions(width, height);

	SetCameraGL();
}