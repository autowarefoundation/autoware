/*
// *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "MainWindowWrapper.h"
#include "DrawingHelpers.h"
#include <iostream>
#include <cmath>
#include "GL/freeglut_ext.h"


#define USE_INFO_WINDOW_

using namespace std;

namespace OP_TESTING_NS
{

WindowParams MainWindowWrapper::m_params;
DisplayParams MainWindowWrapper::m_DisplayParam;

WindowParams::WindowParams()
{
	title = "Testing UI";

	x = 10;
	y = 10;
	w = 400;
	h = 200;
	info_ratio = 0.25;

	UI_CONST.GAP = 20;
	UI_CONST.MAX_ZOOM = 600000.0;
	UI_CONST.MIN_ZOOM = 2.01;
	UI_CONST.INIT_ZOOM = 7.0;
	UI_CONST.FOLLOW_CONST_ZOOM = 2.5;

	ReCalcSimuWindow();

	bNew = true;
	bGPU = true;
}

void WindowParams::ReCalcSimuWindow()
{
	simu_window.x = UI_CONST.GAP;
	simu_window.y = UI_CONST.GAP;
	simu_window.w = w - UI_CONST.GAP * 2.0;
	simu_window.h = h - UI_CONST.GAP * 2.0;
}

DisplayParams::DisplayParams()
{

	prev_x = prev_y = -999999;
	currRotationZ = currRotationX = currRotationY  = 0;

	zoom = 15;
	centerRotX = 5;
	centerRotY = 5;

	eye[0] = 0.0; eye[1] = 0.0; eye[2] = zoom;
	at[0] = 0.0; at[1] = 0.0; at[2] = 0.0;
	up[0] = 0.0; up[1] = 1.0; up[2] = 0.0;

	bDisplayMode = DISPLAY_TOP_FREE;

	bLeftDown =	bRightDown = bCenterDown = false;;

	prespective_z   = 100;
	prespective_fov = 45;

	translateX = 5;
	translateY = 5;

	actualViewX = 0;
	actualViewY = 0;

	bFullScreen = false;

	bSelectPosition = 0;
	StartPos[0] = StartPosFinal[0] = 0;
	StartPos[1] = StartPosFinal[1] = 0;
	StartPos[2] = StartPosFinal[2] = 0;
	GoalPos[0] = GoalPosFinal[0] = 0;
	GoalPos[1] = GoalPosFinal[1] = 0;
	GoalPos[2] = GoalPosFinal[1] = 0;
	SimulatedCarPos[0] = SimulatedCarPos[1] = SimulatedCarPos[2] = 0;

}

int MainWindowWrapper::m_MainWindow = 0;
int MainWindowWrapper::m_SimuWindow = 0;
int MainWindowWrapper::m_InfoWindow = 0;
int MainWindowWrapper::m_PopupMenu = 0;
DrawObjBase* MainWindowWrapper::m_DrawAndControl = 0;

MainWindowWrapper::MainWindowWrapper(DrawObjBase* pDraw)
{
	m_DrawAndControl = pDraw;
}

MainWindowWrapper::~MainWindowWrapper(){}

void MainWindowWrapper::CleanUp()
{
	if(m_DrawAndControl)
		delete m_DrawAndControl;
}

void MainWindowWrapper::InitOpenGLWindow(int argc, char** argv)
{
	if(m_params.bGPU)
		glutInitDisplayMode(GLUT_RGBA | GLUT_MULTISAMPLE);
	else
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_SINGLE);
	glutInitWindowSize(m_params.w, m_params.h );
	glutInitWindowPosition(m_params.x, m_params.y);
	glutInit(&argc, argv);

	m_MainWindow = glutCreateWindow(m_params.title.c_str());
	glutReshapeFunc(MainWindowWrapper::MainReshape);
	glutDisplayFunc(MainWindowWrapper::MainDisplay);
	glutKeyboardFunc(MainWindowWrapper::KeyboardExitCommand);

	m_SimuWindow = glutCreateSubWindow(m_MainWindow, m_params.simu_window.x, m_params.simu_window.y, m_params.simu_window.w, m_params.simu_window.h);
	glutReshapeFunc(MainWindowWrapper::SimuReshape);
	glutDisplayFunc(MainWindowWrapper::SimuDisplay);
	glutKeyboardFunc(MainWindowWrapper::KeyboardCommand);
	glutSpecialFunc(MainWindowWrapper::KeyboardSpecialCommand);

	glutPostRedisplay();

	KeyboardCommand('f', 0,0);

	atexit(CleanUp);

	cout << "Before The OpenGL Main Loop" << endl;

	glutMainLoop();
}

void MainWindowWrapper::InitLighting()
{
	float light_ambient[] = { 0.5,1.0, 0.5, 0.0 };
	float defuse[] = {1.0, 1.0, 1.0,1.0};
	float mat_specular[] = { 1.0, 1.0, 1.0, 0.0 };
	float mat_shininess[] = { 50.0 };
	//float light_position[4] = { 0.0, 0.0, 20.0, 0.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, defuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, light_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
	//glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
}

void MainWindowWrapper::UpdateParams(const WindowParams& params, const DisplayParams& display_params)
{
	m_params = params;
	m_DisplayParam = display_params;
	FromScreenToModelCoordinate(m_params.simu_window.w/2.0, m_params.simu_window.h/2.0,
				m_DisplayParam.initScreenToCenterMargin[0], m_DisplayParam.initScreenToCenterMargin[1]);
}

void MainWindowWrapper::MainReshape(int width,  int height)
{
	m_params.w = width;
	m_params.h = height;
	m_params.ReCalcSimuWindow();

	glViewport(0,0, m_params.w, m_params.h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, m_params.w, m_params.h, 0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glutPostWindowRedisplay(m_SimuWindow);
}

void MainWindowWrapper::SimuReshape(int width,  int height)
{
	glutPositionWindow(m_params.simu_window.x, m_params.simu_window.y);
	glutReshapeWindow(m_params.simu_window.w, m_params.simu_window.h);

	glViewport(0,0, m_params.simu_window.w, m_params.simu_window.h);

	//this should rely on the map not the window
	double y_max = sqrt(pow(m_params.simu_window.w,2) + pow(m_params.simu_window.h,2))/2.0;
	if(y_max > 0.05 && y_max < (m_params.simu_window.w*m_params.simu_window.h))
		m_DisplayParam.prespective_z = y_max;

	m_DisplayParam.prespective_z = 10000;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double aspect_ratio = (double)m_params.simu_window.w/(double)m_params.simu_window.h;

	gluPerspective(m_DisplayParam.prespective_fov,aspect_ratio,0.05,m_DisplayParam.prespective_z);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if(m_DisplayParam.bDisplayMode == DISPLAY_FOLLOW && m_DrawAndControl)
	{
		m_DisplayParam.eye[0] = m_DisplayParam.centerRotX;
		m_DisplayParam.eye[1] = m_DisplayParam.centerRotY;
		m_DisplayParam.at[0] = m_DisplayParam.centerRotX;
		m_DisplayParam.at[1] = m_DisplayParam.centerRotY;
	}


	gluLookAt(m_DisplayParam.eye[0], m_DisplayParam.eye[1], m_DisplayParam.zoom,
			m_DisplayParam.at[0], m_DisplayParam.at[1], m_DisplayParam.at[2],
			m_DisplayParam.up[0], m_DisplayParam.up[1],m_DisplayParam.up[2]);

	InitLighting();

}

void MainWindowWrapper::FromScreenToModelCoordinate(int sx, int sy, double& modelX, double& modelY)
{
	double whole = (double)(m_params.simu_window.w + m_params.simu_window.h)/2.0;
	double actualViewX = tan(m_DisplayParam.prespective_fov*DEG2RAD) * m_DisplayParam.zoom / whole;

	modelX = sx * actualViewX ;
	modelY = sy * actualViewX ;
}

void MainWindowWrapper::FromModelToScreenCoordinate(double modelX, double modelY, int& sx, int& sy)
{
	double actualViewX = tan(m_DisplayParam.prespective_fov*DEG2RAD) * m_DisplayParam.zoom / m_params.simu_window.w;

	sx = modelX / actualViewX ;
	sy = modelY / actualViewX ;
}

void MainWindowWrapper::SimuDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(m_DrawAndControl)
		m_DrawAndControl->DrawSimu();

	glutSwapBuffers();
}

void MainWindowWrapper::MainDisplay()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_LIGHTING);
	glPushMatrix();
	glBegin(GL_POLYGON);
	glColor3f(0.1,0.1,0.1);
	glVertex2i(0, m_params.h);
	glVertex2i(m_params.w,m_params.h);
	glColor3f(0.9,0.9,0.9);
	glVertex2i(m_params.w, 0);
	glVertex2i(0,0);
	glEnd();
	glPopMatrix();
	glEnable(GL_LIGHTING);

	glPushMatrix();
	glColor3f(0.9, 0.2, 0.2);
	glTranslated(m_params.simu_window.x, m_params.UI_CONST.GAP-1, 0);
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)"Command Input Window");
	glPopMatrix();

	glutSwapBuffers();
}

void MainWindowWrapper::KeyboardExitCommand(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
		break;
	}
}

void MainWindowWrapper::KeyboardCommand(unsigned char key, int x, int y)
{
	//cout << "Char : " << (int)key <<  endl;
	if(m_DrawAndControl)
		m_DrawAndControl->OnKeyboardPress(0, key);

	switch (key)
	{
	case 27:
		exit(0);
		break;
	}
}

void MainWindowWrapper::KeyboardSpecialCommand(int key, int x, int y)
{
	//cout << "Control : " << key << endl;

	if(m_DrawAndControl)
		m_DrawAndControl->OnKeyboardPress(key, 0);
}

}
