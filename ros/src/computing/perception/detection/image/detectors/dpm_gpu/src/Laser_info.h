///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////Laser_info.h   Laser_radar information & definition header  /////////////////////////////////////////

//OpenCV library
//#include "cv.h"			
//#include "cxcore.h"
//#include "highgui.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#ifdef _DEBUG
    //Debugモードの場合
    #pragma comment(lib,"cv200d.lib") 
    #pragma comment(lib,"cxcore200d.lib") 
    #pragma comment(lib,"cvaux200d.lib") 
    #pragma comment(lib,"highgui200d.lib") 
#else
    //Releaseモードの場合
    #pragma comment(lib,"cv200.lib") 
    #pragma comment(lib,"cxcore200.lib") 
    #pragma comment(lib,"cvaux200.lib") 
    #pragma comment(lib,"highgui200.lib") 
#endif
//C++ library
#include <stdio.h>	

#include "switch_float.h"

#ifndef INCLUDED_L_INFO_
#define INCLUDED_L_INFO_

#define m_PI 3.1415926535

//camera parameter 
#define VA 59.5
#define cxp 0.135
#define cyp -0.005
#define czp 0.12
#define cha -0.95
#define cva -5.0

typedef struct {
	int SNUM;				//number of scan point
	int *CCLASS;			//car class (for discrimination)
	int *Ctype;				//channel type of each scanpoint
	FLOAT *XYZdata;			//(X,Y,Z) data of each scanpoint
	int *XYIM;				//(X,Y) image coordinate of each scanpoint
}SCANDATA;

#endif
