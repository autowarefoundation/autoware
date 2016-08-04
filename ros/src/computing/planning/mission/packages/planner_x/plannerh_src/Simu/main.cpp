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
	PlannerTestDraw* pSimulator = new PlannerTestDraw();
	WindowParams pms;
	DisplayParams dpms;
	dpms.centerRotX = 0;
	dpms.centerRotY = 0;
	dpms.translateX = 0;
	dpms.translateY = 0;
	MainWindowWrapper wrapper(pSimulator);
	wrapper.UpdateParams(pms, dpms);
	wrapper.InitOpenGLWindow(argc, argv);

//	delete pSimulator;
//	pSimulator = 0;

//	glutInit(&argc, argv);
//	MainGlWindow mw;
//	glutMainLoop();


}


