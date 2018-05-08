/*
 * MainWindowWrapper.cpp
 *
 *  Created on: May 31, 2016
 *      Author: hatem
 */

#include "op_simu/MainWindowWrapper.h"
#include "op_simu/DrawingHelpers.h"
#include <iostream>
#include <cmath>
#include "op_utility/UtilityH.h"
#include "GL/freeglut_ext.h"

using namespace std;
using namespace UtilityHNS;
#include "op_planner/MatrixOperations.h"

namespace Graphics {

WindowParams MainWindowWrapper::m_params;
DisplayParams MainWindowWrapper::m_DisplayParam;

WindowParams::WindowParams()
{
	title = "Simple Simulator";

	x = 10;
	y = 10;
	w = 1200;
	h = 800;
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
	info_window.x = w*(1.0-info_ratio) - UI_CONST.GAP;
	info_window.y = UI_CONST.GAP;
	info_window.w = w*info_ratio;
	info_window.h = h - 2.0*UI_CONST.GAP;

	simu_window.x = UI_CONST.GAP;
	simu_window.y = UI_CONST.GAP;
	simu_window.w = w - UI_CONST.GAP * 3 - info_window.w;
	simu_window.h = h - UI_CONST.GAP * 2;
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
	glutMotionFunc(MainWindowWrapper::MouseMove);
	glutMouseFunc(MainWindowWrapper::MouseCommand);
	glutKeyboardFunc(MainWindowWrapper::KeyboardCommand);
	glutSpecialFunc(MainWindowWrapper::KeyboardSpecialCommand);
	if(m_DrawAndControl)
		m_DrawAndControl->LoadMaterials();
	CreateRightClickMenu();

	m_InfoWindow = glutCreateSubWindow(m_MainWindow, m_params.info_window.x, m_params.info_window.y, m_params.info_window.w, m_params.info_window.h);
	glutReshapeFunc(MainWindowWrapper::InfoReshape);
	glutDisplayFunc(MainWindowWrapper::InfoDisplay);
	glutKeyboardFunc(MainWindowWrapper::KeyboardExitCommand);

	//RedisplayAll();
	glutPostRedisplay();

	KeyboardCommand('f', 0,0);

	atexit(CleanUp);

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

	glutPostWindowRedisplay(m_InfoWindow);
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
//	if(aspect_ratio < m_params.simu_window.h/m_params.simu_window.w)
//		aspect_ratio = m_params.simu_window.h/m_params.simu_window.w;

	gluPerspective(m_DisplayParam.prespective_fov,aspect_ratio,0.05,m_DisplayParam.prespective_z);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//double x = 0, y =0;

	if(m_DisplayParam.bDisplayMode == DISPLAY_FOLLOW && m_DrawAndControl)
	{
//			m_DisplayParam.eye[0] = m_DrawAndControl->m_followX-m_DisplayParam.translateX;
//			m_DisplayParam.eye[1] = m_DrawAndControl->m_followY-m_DisplayParam.translateY;
//			m_DisplayParam.at[0] = m_DrawAndControl->m_followX-m_DisplayParam.translateX;
//			m_DisplayParam.at[1] = m_DrawAndControl->m_followY-m_DisplayParam.translateY;
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

	//glClearColor(1,1,1,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if(m_DisplayParam.bDisplayMode == DISPLAY_FOLLOW && m_DrawAndControl)
	{
		//glTranslated(-m_DisplayParam.translateX, -m_DisplayParam.translateY, 0);
		m_DisplayParam.centerRotX = m_DrawAndControl->m_followX;
		m_DisplayParam.centerRotY = m_DrawAndControl->m_followY;


		int yaw = (m_DisplayParam.currRotationZ/8)%360;
		int roll = (m_DisplayParam.currRotationX/8)%360;
		int pitch = (m_DisplayParam.currRotationY/8)%360;

		glTranslated(m_DisplayParam.centerRotX, m_DisplayParam.centerRotY, 0);
		glRotated(yaw, 0,0,1);
		glRotated(roll, 1,0,0);
		glRotated(pitch, 0,1,0);
		glTranslated(-m_DisplayParam.centerRotX, -m_DisplayParam.centerRotY, 0);
	}
	else if(m_DisplayParam.bDisplayMode == DISPLAY_FREE)
	{
		glTranslated(-m_DisplayParam.translateX, -m_DisplayParam.translateY, 0);

		int yaw = (m_DisplayParam.currRotationZ/8)%360;
		int roll = (m_DisplayParam.currRotationX/8)%360;
		int pitch = (m_DisplayParam.currRotationY/8)%360;

		glTranslated(m_DisplayParam.centerRotX, m_DisplayParam.centerRotY, 0);
		glRotated(yaw, 0,0,1);
		glRotated(roll, 1,0,0);
		glRotated(pitch, 0,1,0);
		glTranslated(-m_DisplayParam.centerRotX, -m_DisplayParam.centerRotY, 0);
	}
	else if(m_DisplayParam.bDisplayMode == DISPLAY_TOP_FREE)
	{
		glTranslated(-m_DisplayParam.translateX, -m_DisplayParam.translateY, 0);

		int yaw = (m_DisplayParam.currRotationZ/8)%360;
		//int roll = (m_DisplayParam.currRotationX/8)%360;
		//int pitch = (m_DisplayParam.currRotationY/8)%360;


		glTranslated(m_DisplayParam.centerRotX, m_DisplayParam.centerRotY, 0);
		glRotated(yaw, 0,0,1);
		//glRotated(roll, 1,0,0);
		//glRotated(pitch, 0,1,0);
		glTranslated(-m_DisplayParam.centerRotX, -m_DisplayParam.centerRotY, 0);
	}


//	glPushMatrix();
//	glTranslated(5, 5, 0);
//	glutSolidTeapot(3);
//	glPopMatrix();

//

	DrawingHelpers::DrawCustomOrigin(m_DisplayParam.centerRotX, m_DisplayParam.centerRotY, 0, 0, 0, 0, 2.5 );

	if(m_DrawAndControl)
		m_DrawAndControl->DrawSimu();
	else
		DrawingHelpers::DrawGrid(m_DisplayParam.centerRotX-5, m_DisplayParam.centerRotY-5, 10, 10, 1);

	int yaw = (m_DisplayParam.currRotationZ/8)%360;
	PlannerHNS::Mat3 rotationMat(-yaw*DEG2RAD);
	PlannerHNS::Mat3 translationMat(m_DisplayParam.centerRotX, m_DisplayParam.centerRotY);
	PlannerHNS::Mat3 invTranslationMat(-m_DisplayParam.centerRotX, -m_DisplayParam.centerRotY);

	if(m_DisplayParam.bSelectPosition == 1)
	{
		double sx=0, sy=0;
		FromScreenToModelCoordinate(m_DisplayParam.StartPos[0]-m_params.simu_window.w/2.0,
				m_params.simu_window.h/2.0 - m_DisplayParam.StartPos[1],sx,sy);

		PlannerHNS::GPSPoint sp(sx+m_DisplayParam.translateX, sy+m_DisplayParam.translateY, 0, 0);
		sp = translationMat * sp;
		sp = rotationMat * sp;
		sp = invTranslationMat  * sp;
		m_DisplayParam.StartPosFinal[0] = sp.x;
		m_DisplayParam.StartPosFinal[1] = sp.y;
		m_DisplayParam.StartPosFinal[2] = m_DisplayParam.StartPos[2];

	}
	else if(m_DisplayParam.bSelectPosition == 2)
	{
		double gx=0,gy=0;
		FromScreenToModelCoordinate(m_DisplayParam.GoalPos[0]-m_params.simu_window.w/2.0,
						m_params.simu_window.h/2.0 - m_DisplayParam.GoalPos[1],gx,gy);

		PlannerHNS::GPSPoint gp(gx+m_DisplayParam.translateX, gy+m_DisplayParam.translateY, 0, m_DisplayParam.GoalPos[2]);
		gp = translationMat * gp;
		gp = rotationMat * gp;
		gp = invTranslationMat * gp;
		m_DisplayParam.GoalPosFinal[0] = gp.x;
		m_DisplayParam.GoalPosFinal[1] = gp.y;
		m_DisplayParam.GoalPosFinal[2] = m_DisplayParam.GoalPos[2];
	}
	else if(m_DisplayParam.bSelectPosition == 3)
	{
		double x=0,y=0;
		FromScreenToModelCoordinate(m_DisplayParam.SimulatedCarPos[0]-m_params.simu_window.w/2.0,
						m_params.simu_window.h/2.0 - m_DisplayParam.SimulatedCarPos[1],x,y);

		PlannerHNS::GPSPoint gp(x+m_DisplayParam.translateX, y+m_DisplayParam.translateY, 0, m_DisplayParam.SimulatedCarPos[2]);
		gp = translationMat * gp;
		gp = rotationMat * gp;
		gp = invTranslationMat * gp;
		m_DisplayParam.SimulatedCarPosFinal[0] = gp.x;
		m_DisplayParam.SimulatedCarPosFinal[1] = gp.y;
		m_DisplayParam.SimulatedCarPosFinal[2] = m_DisplayParam.SimulatedCarPos[2];
	}

	glPushMatrix();
	DrawingHelpers::DrawArrow(m_DisplayParam.StartPosFinal[0], m_DisplayParam.StartPosFinal[1], m_DisplayParam.StartPosFinal[2]);
	DrawingHelpers::DrawArrow(m_DisplayParam.GoalPosFinal[0], m_DisplayParam.GoalPosFinal[1], m_DisplayParam.GoalPosFinal[2]);
	DrawingHelpers::DrawArrow(m_DisplayParam.SimulatedCarPosFinal[0], m_DisplayParam.SimulatedCarPosFinal[1], m_DisplayParam.SimulatedCarPosFinal[2]);
	glPopMatrix();


	glutSwapBuffers();
}

void MainWindowWrapper::InfoReshape(int width,  int height)
{
	glutPositionWindow(m_params.info_window.x, m_params.info_window.y);
	glutReshapeWindow(m_params.info_window.w, m_params.info_window.h);

	glViewport(0,0, m_params.info_window.w, m_params.info_window.h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(0, m_params.info_window.w, m_params.info_window.h, 0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//cout << "Info: width = " << width << ", height = " << height << ", x=" <<m_params.info_window.x << ", y=" <<m_params.info_window.y<<  endl;
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
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)"Simulation View");
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.9, 0.2, 0.2);
	glTranslated(m_params.info_window.x, m_params.UI_CONST.GAP-1, 0);
	DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)"Info View");
	glPopMatrix();

	glutSwapBuffers();
}

void MainWindowWrapper::InfoDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_LIGHTING);
	glPushMatrix();
	glBegin(GL_POLYGON);
	glColor3f(0.1,0.1,0.1);
	glVertex2i(0,0);
	glVertex2i(0,m_params.info_window.h);
	glColor3f(0.3,0.3,0.3);
	glVertex2i(m_params.info_window.w,m_params.info_window.h);
	glVertex2i(m_params.info_window.w,0);
	glEnd();
	glPopMatrix();
	glEnable(GL_LIGHTING);

	//cout << "InfoDisplay" << endl;

	if(m_DisplayParam.bSelectPosition == 1)
	{
		glPushMatrix();
		glColor3f(0.9, 0.2, 0.2);
		glTranslated(10, m_params.info_window.h - 40, 0);
		DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)"Start Position");
		glPopMatrix();
	}
	else if(m_DisplayParam.bSelectPosition == 2)
	{
		glPushMatrix();
		glColor3f(0.9, 0.2, 0.2);
		glTranslated(10, m_params.info_window.h - 40, 0);
		DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)"Goal Position");
		glPopMatrix();
	}
	else if(m_DisplayParam.bSelectPosition == 3)
	{
		glPushMatrix();
		glColor3f(0.9, 0.2, 0.2);
		glTranslated(10, m_params.info_window.h - 40, 0);
		DrawingHelpers::DrawString(0, 0, GLUT_BITMAP_TIMES_ROMAN_24, (char*)"Simulation Positions");
		glPopMatrix();
	}

	if(m_DrawAndControl)
		m_DrawAndControl->DrawInfo(m_params.info_window.w/2, m_params.info_window.h/2, m_params.info_window.w, m_params.info_window.h);

	glutSwapBuffers();
}

void MainWindowWrapper::RedisplayAll()
{
//	glutSetWindow(m_InfoWindow);
//	glutPostRedisplay();
//	glutSetWindow(m_SimuWindow);
//	glutPostRedisplay();
//	glutSetWindow(m_MainWindow);
//	glutPostRedisplay();
}

void MainWindowWrapper::MouseMove(int x, int y)
{
	if(m_DisplayParam.bSelectPosition == 1)
	{
		if(m_DisplayParam.bLeftDown)
		{
			m_DisplayParam.StartPos[2] = atan2(m_DisplayParam.StartPos[1] - y, x - m_DisplayParam.StartPos[0]);
		}
	}
	else if(m_DisplayParam.bSelectPosition == 2)
	{
		if(m_DisplayParam.bLeftDown)
		{
			m_DisplayParam.GoalPos[2] = atan2(m_DisplayParam.GoalPos[1] - y, x - m_DisplayParam.GoalPos[0]);
		}
	}
	else if(m_DisplayParam.bSelectPosition == 3)
	{
		if(m_DisplayParam.bLeftDown)
		{
			m_DisplayParam.SimulatedCarPos[2] = atan2(m_DisplayParam.SimulatedCarPos[1] - y, x - m_DisplayParam.SimulatedCarPos[0]);
		}
	}
	else if(m_DisplayParam.bLeftDown)
	{
		m_DisplayParam.currRotationZ += x-m_DisplayParam.prev_x;
		m_DisplayParam.prev_x = x;

		//if(m_DisplayParam.bFreeDisplay)
		{
			double zdir = (m_DisplayParam.currRotationZ/8)%360 * DEG2RAD;
			double xRatio = cos(zdir);
			double yRatio = sin(zdir);

			m_DisplayParam.currRotationX += (y-m_DisplayParam.prev_y) * xRatio;
			m_DisplayParam.currRotationY += (m_DisplayParam.prev_y - y) * yRatio;

			m_DisplayParam.prev_y = y;
		}
	}
	else if (m_DisplayParam.bRightDown)
	{
	}
	else if (m_DisplayParam.bCenterDown)
	{

		m_DisplayParam.actualViewX = (tan(m_DisplayParam.prespective_fov/2.0*DEG2RAD) * m_DisplayParam.zoom * 2.0)/m_params.simu_window.w;
		m_DisplayParam.actualViewY = (tan(m_DisplayParam.prespective_fov/2.0*DEG2RAD) * m_DisplayParam.zoom * 2.0)/m_params.simu_window.h;

		m_DisplayParam.translateX += (m_DisplayParam.prev_x-x)*m_DisplayParam.actualViewX;
		m_DisplayParam.translateY += (y-m_DisplayParam.prev_y)*m_DisplayParam.actualViewY;

		m_DisplayParam.prev_x = x;
		m_DisplayParam.prev_y = y;

	}
	else
	{

	}

}

void MainWindowWrapper::MouseCommand(int button, int state, int x, int y)
{
	//cout << x << ", " << y << endl;
	if(m_DisplayParam.bSelectPosition == 1)
	{
		if(button == 0 && state == 0 && !m_DisplayParam.bLeftDown)
		{
			m_DisplayParam.bLeftDown = true;
			m_DisplayParam.StartPos[0] = x;
			m_DisplayParam.StartPos[1] = y;
			//cout << "Left Down" << endl;
		}
		else if(button == 0 && state == 1)
		{
			m_DisplayParam.bLeftDown = false;
			m_DisplayParam.bSelectPosition = 0;
		}
	}
	else if(m_DisplayParam.bSelectPosition == 2)
	{
		if(button == 0 && state == 0 && !m_DisplayParam.bLeftDown)
		{
			m_DisplayParam.bLeftDown = true;
			m_DisplayParam.GoalPos[0] = x;
			m_DisplayParam.GoalPos[1] = y;
			//cout << "Left Down" << endl;
		}
		else if(button == 0 && state == 1)
		{
			m_DisplayParam.bLeftDown = false;
			m_DisplayParam.bSelectPosition = 0;
			m_DrawAndControl->UpdatePlaneStartGoal(m_DisplayParam.StartPosFinal[0],
					m_DisplayParam.StartPosFinal[1], m_DisplayParam.StartPosFinal[2],
					m_DisplayParam.GoalPosFinal[0], m_DisplayParam.GoalPosFinal[1], m_DisplayParam.GoalPosFinal[2]);
		}
	}
	else if(m_DisplayParam.bSelectPosition == 3)
	{
//		if(button == 0 && state == 1)
//		{
//			int yaw = (m_DisplayParam.currRotationZ/8)%360;
//			PlannerHNS::Mat3 rotationMat(-yaw*DEG2RAD);
//			PlannerHNS::Mat3 translationMat(m_DisplayParam.centerRotX, m_DisplayParam.centerRotY);
//			PlannerHNS::Mat3 invTranslationMat(-m_DisplayParam.centerRotX, -m_DisplayParam.centerRotY);
//
//			double gx=0,gy=0;
//			FromScreenToModelCoordinate(x-m_params.simu_window.w/2.0, m_params.simu_window.h/2.0 - y,gx,gy);
//
//			PlannerHNS::GPSPoint gp(gx+m_DisplayParam.translateX, gy+m_DisplayParam.translateY, 0, 0);
//			gp = translationMat * gp;
//			gp = rotationMat * gp;
//			gp = invTranslationMat * gp;
//
//			m_DisplayParam.bSelectPosition = 0;
//
//		}

		if(button == 0 && state == 0 && !m_DisplayParam.bLeftDown)
		{
			m_DisplayParam.bLeftDown = true;
			m_DisplayParam.SimulatedCarPos[0] = x;
			m_DisplayParam.SimulatedCarPos[1] = y;
			//cout << "Left Down" << endl;
		}
		else if(button == 0 && state == 1)
		{
			m_DisplayParam.bLeftDown = false;
			m_DisplayParam.bSelectPosition = 0;
			m_DrawAndControl->AddSimulatedCarPos(m_DisplayParam.SimulatedCarPosFinal[0],
					m_DisplayParam.SimulatedCarPosFinal[1], m_DisplayParam.SimulatedCarPosFinal[2]);
		}
	}
	else if(button == 0 && state == 0 && !m_DisplayParam.bLeftDown)
	{
		m_DisplayParam.bLeftDown = true;
		m_DisplayParam.prev_x = x;
		m_DisplayParam.prev_y = y;
		//cout << "Left Down" << endl;
	}
	else if(button == 0 && state == 1)
	{
		m_DisplayParam.bLeftDown = false;
	}
	else if(button == 2 && state == 0 && !m_DisplayParam.bRightDown)
	{
		m_DisplayParam.bRightDown = true;
		m_DisplayParam.prev_x = x;
		m_DisplayParam.prev_y = y;
		//cout << "Right Down" << endl;
	}
	else if(button == 2 && state == 1)
	{
		m_DisplayParam.bRightDown = false;

	}
	else if(button == 1 && state == 0 && !m_DisplayParam.bCenterDown)
	{
		m_DisplayParam.bCenterDown = true;
		m_DisplayParam.prev_x = x;
		m_DisplayParam.prev_y = y;
		//cout << "Right Down" << endl;
	}
	else if(button == 1 && state == 1)
	{
		m_DisplayParam.bCenterDown = false;

	}
	else if(button == 3)
	{
		m_DisplayParam.zoom-=1;
		if(m_DisplayParam.zoom < 2)
			m_DisplayParam.zoom = 2;
	}
	else if (button == 4)
	{
		m_DisplayParam.zoom+=1;
		if(m_DisplayParam.zoom > m_DisplayParam.prespective_z/10.0)
			m_DisplayParam.zoom = m_DisplayParam.prespective_z/10.0;
	}

	if(m_DrawAndControl)
		m_DrawAndControl->OnLeftClick(x,y);

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
	//cout << "Char : " << key <<  endl;
	switch (key)
	{
	case 27:
		exit(0);
		break;
	case 'e':
	{
			m_DisplayParam.bDisplayMode = DISPLAY_FREE;
			m_DisplayParam.translateX = 0;
			m_DisplayParam.translateY = 0;
	}
		break;
	case 'c':
	{
			m_DisplayParam.bDisplayMode = DISPLAY_FOLLOW;
	}
		break;
	case 't':
	{
			m_DisplayParam.bDisplayMode = DISPLAY_TOP_FREE;
	}
		break;
	case 'f':
	{
		if(m_DisplayParam.bFullScreen)
		{
			glutSetWindow(m_MainWindow);
			glutFullScreenToggle();
			m_DisplayParam.bFullScreen = false;
		}
		else
		{
			m_DisplayParam.bFullScreen = true;
		}
	}
		break;
	case 'r':
	{
		if(m_DrawAndControl)
			m_DrawAndControl->Reset();
	}
	break;
	case 'p':
	{
		if(m_DisplayParam.bSelectPosition == 1)
		{
			m_DisplayParam.bSelectPosition = 0;
		}
		else
		{
			m_DisplayParam.bSelectPosition = 1;
		}

	}
	break;
	case 'o':
	{
		if(m_DisplayParam.bSelectPosition == 2)
		{
			m_DisplayParam.bSelectPosition = 0;
		}
		else
		{
			m_DisplayParam.bSelectPosition = 2;
		}
	}
	break;
	case 'u':
	{
		if(m_DisplayParam.bSelectPosition == 3)
		{
			m_DisplayParam.bSelectPosition = 0;
		}
		else
		{
			m_DisplayParam.bSelectPosition = 3;
		}
	}
	break;
	}

	if(m_DrawAndControl)
		m_DrawAndControl->OnKeyboardPress(CHAR_KEY, key);
}

void MainWindowWrapper::KeyboardSpecialCommand(int key, int x, int y)
{
	//cout << "Control : " << key << endl;
	switch (key)
	{
	case 101: // Up
	{
//		double prevAngle = angle;
//		angle += rand()%5;
//		//double a = circ_angle->CalcAngle(3.0*DEG2RAD);
//		angle = UtilityH::GetCircularAngle(prevAngle*DEG2RAD, angle*DEG2RAD) * RAD2DEG;
//		cout << endl << "angle = " << angle << endl;
	}
	break;
	case 103: //Down
	{
//		double prevAngle = angle;
//		angle -= rand()%5;
//		//double a = circ_angle->CalcAngle(357.0*DEG2RAD);
//		angle = UtilityH::GetCircularAngle(prevAngle*DEG2RAD, angle*DEG2RAD) * RAD2DEG;
//		cout << "angle = " << angle << endl;
	}
	break;
	case 102: //Right
	{

	}
	break;
	case 100: //Left
	{

	}
	break;
	case 112: //left shift key
	{

	}
	break;
	case 114: //left Ctrl key
	{

	}
	break;
	case 113: //Right shift key
	{

	}
	break;
	case 115: // right Ctrl key
	{

	}
	break;
	case 116: //ALT key
	{

	}
	break;
	default:
		//m_RandomObstacle =0;
		break;
	}
}

void MainWindowWrapper::MenuCommand(int value)
{
	KeyboardCommand(value, 0,0);
}

void MainWindowWrapper::ProcessMenuStatus(int status, int x, int y)
{
	if (status != GLUT_MENU_IN_USE)
	{
		ModifyPopupMenu();
	}
}

void MainWindowWrapper::ModifyPopupMenu()
{
//	glutSetMenu(m_PopupMenu);
//
//	if(m_DisplayParam.bDisplayMode == DISPLAY_FOLLOW)
//		glutChangeToMenuEntry(5, "> Follow CAM(c)", 1004);
//	else if(m_DisplayParam.bDisplayMode == DISPLAY_TOP_FREE)
//		glutChangeToMenuEntry(6, "> Top CAM   (t)", 1005);
//	else if(m_DisplayParam.bDisplayMode == DISPLAY_FREE)
//		glutChangeToMenuEntry(7, "> Free CAM  (e)", 1006);

//	if(m_DisplayParam.bFullScreen)
//	{
//		glutChangeToMenuEntry(8, "Hide V Window", 'h');
//	}
}

void MainWindowWrapper::CreateRightClickMenu()
{
	if(m_PopupMenu==0)
		m_PopupMenu = glutCreateMenu(MenuCommand);
	else
		glutSetMenu(m_PopupMenu);

	glutAddMenuEntry("Start/Stop  (s)", 's');
	glutAddMenuEntry("Restart     (r)", 'r');

	glutAddMenuEntry("------------", 0);

	glutAddMenuEntry("Full Screen (f)", 'f');
	glutAddMenuEntry("Follow CAM  (c)", 'c');
	glutAddMenuEntry("Top CAM     (t)", 't');
	glutAddMenuEntry("Free CAM    (e)", 'e');

	glutAddMenuEntry("------------", 0);

	glutAddMenuEntry("Start Pose  (p)", 0);
	glutAddMenuEntry("End   Pose  (o)", 0);
	glutAddMenuEntry("Add Sim Car (u)", 0);
	glutAddMenuEntry("Delete Sim Cars (n)", 0);
	glutAddMenuEntry("Save Sim Points(v)", 0);
	glutAddMenuEntry("Load Sim Points(l)", 0);

	glutAddMenuEntry("------------", 0);

	glutAddMenuEntry("Exit        (Esc)", 27);

	glutAttachMenu(GLUT_RIGHT_BUTTON);
	//glutMenuStatusFunc(ProcessMenuStatus);
}



} /* namespace Graphics */
