/*
 * main.cpp
 *
 *  Created on: May 31, 2016
 *      Author: hatem
 */

#include <iostream>
#include "MainWindowWrapper.h"
#include "PlannerTestDraw.h"

using namespace  Graphics;

int main(int argc, char** argv)
{
	PlannerTestDraw car;
	WindowParams pms;
	DisplayParams dpms;
	dpms.centerRotX = 10;
	dpms.centerRotY = 30;
	dpms.translateX = 10;
	dpms.translateY = 30;
	MainWindowWrapper wrapper(&car);
	wrapper.UpdateParams(pms, dpms);
	wrapper.InitOpenGLWindow(argc, argv);

//	glutInit(&argc, argv);
//	MainGlWindow mw;
//	glutMainLoop();


}


