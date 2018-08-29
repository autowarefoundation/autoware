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

#ifndef WINDOWWRAPPER_TEST
#define WINDOWWRAPPER_TEST

#include <string>
#include "DrawObjBase.h"
#include <vector>
#include <ros/ros.h>


namespace OP_TESTING_NS
{

enum DisplayMode{DISPLAY_FREE, DISPLAY_TOP_FREE, DISPLAY_FOLLOW};

class WindowParams
{
public:
	int x;
	int y;
	int w;
	int h;
	double info_ratio;
	std::string title;
	struct info_window
	{
		int x;
		int y;
		int w;
		int h;
	}info_window;

	struct simu_window
	{
		int x;
		int y;
		int w;
		int h;
	}simu_window;

	struct UI_CONST
	{
		int GAP;
		double MAX_ZOOM;
		double MIN_ZOOM;
		double INIT_ZOOM;
		double FOLLOW_CONST_ZOOM;

	}UI_CONST;

	bool bNew;
	bool bGPU;


	WindowParams();
	void ReCalcSimuWindow();
};

class DisplayParams
{
public:

	int prev_x;
	int prev_y;
	int currRotationZ; //degrees, scaled radians
	int currRotationX; //degrees, scaled radians
	int currRotationY; //degrees, scaled radians

	double centerRotX;
	double centerRotY;

	double translateX;
	double translateY;

	float eye[3];
	float at[3] ;
	float up[3] ;

	bool bFullScreen;
	DisplayMode  bDisplayMode;
	double prespective_z;
	double prespective_fov;

	bool bLeftDown;
	bool bRightDown;
	bool bCenterDown;
	int  bSelectPosition; //0 nothing,  1 start , 2 goal, 3 simulation
	double StartPos[3];
	double GoalPos[3];
	double StartPosFinal[3];
	double GoalPosFinal[3];

	double SimulatedCarPos[3];
	double SimulatedCarPosFinal[3];


	double zoom;
	double actualViewX;
	double actualViewY;
	double initScreenToCenterMargin[2];

	DisplayParams();
};

class MainWindowWrapper {
public:

	MainWindowWrapper(DrawObjBase* pDraw);
	virtual ~MainWindowWrapper();

	void InitOpenGLWindow(int argc, char** argv);
	void UpdateParams(const WindowParams& params, const DisplayParams& display_params);
	static void MainReshape(int width,  int height);
	static void SimuReshape(int width,  int height);

	static void MainDisplay();
	static void SimuDisplay();

	static void KeyboardCommand(unsigned char key, int x, int y);
	static void KeyboardExitCommand(unsigned char key, int x, int y);
	static void KeyboardSpecialCommand(int key, int x, int y);

	static void InitLighting();
	static void FromScreenToModelCoordinate(int sx, int sy, double& modelX, double& modelY);
	static void FromModelToScreenCoordinate(double modelX, double modelY, int& sx, int& sy);

	static void DrawGrid(const double& x, const double& y, const double& w, const double& h, const double& cell_l);

	static void CleanUp();

private:
	static DrawObjBase* m_DrawAndControl;
	static int m_MainWindow;
	static int m_SimuWindow;
	static int m_InfoWindow;
	static int m_PopupMenu;
	static WindowParams m_params;
	static DisplayParams m_DisplayParam;



private:
	//this area for code testing , remove later


};

}

#endif
