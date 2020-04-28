//
//  GlutMachine.h
//  RiBo Engine
//
//  Created by Angelo Moro on 10/12/2019
//

#ifndef __GLUT_MACHINE_H__
#define __GLUT_MACHINE_H__

#include <string.h>
#include <iostream>
#include "freeglut.h"
#include "WindowInfo.h"
#include "IListener.h"

class tkWindowInfo;

using namespace std;

namespace Core
{
	class tkGlutMachine {
	public:
		tkGlutMachine(int argc, char **argv);
		~tkGlutMachine();

		void InitGLUT(unsigned int t);

		static void SetListener(Core::IListener *& iListener);
		static void KeyPressed(unsigned char key, int x, int y);
		static void RunGLUT();

	private:
		static void Display(void);
		static void SetCameraGL(void);
		static void ReshapeWindow(int width, int height);
		static void TimerGL(int id);

		static tkWindowInfo* window;
		static IListener *listener;
		static unsigned int fixedT;
	};
}

#endif /*!__GLUT_MACHINE_H__*/
