/*
 * main.cpp
 *
 *  Created on: May 31, 2016
 *      Author: hatem
 */

#include <iostream>
#include "op_simu/AlternativeVisualizer.h"
#include "op_simu/MainWindowWrapper.h"
#include "op_simu/PlannerTestDraw.h"

using namespace  Graphics;
#define USE_ALT_VISUALIZER 1

int main(int argc, char** argv)
{
	DrawObjBase* pSimulator =  0;
	if(USE_ALT_VISUALIZER == 1)
		pSimulator = new AlternativeVisualizer();
	else
		pSimulator = new PlannerTestDraw();

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


