#include "Renderer.h"

using namespace Core;

void tkRenderer::InitGLUT()
{
	window = new tkWindowInfo("Rigid Body - paper from Baraff & Witkin");

	int argc = 1;
	char *argv[] = { "dummy", NULL };
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow(window->GetTitle());
	glutReshapeWindow(window->GetWidth(), window->GetHeight());
}