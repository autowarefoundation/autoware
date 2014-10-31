///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////MODEL_info.h   Detector-Model information & definition header  /////////////////////////////////////////

//OpenCV library
//#include "cv.h"
//#include "cxcore.h"
//#include "highgui.h"
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
//Add messi 2012/11/16
#include "opencv2/legacy/legacy.hpp"
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
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDED_Minfo_
#define INCLUDED_Minfo_

/////////////////////
//file information///
/////////////////////

//#define IM_NAME		"test5.jpg"				//Image name
//#define MNAME			"M1.avi"				//source movie name

//#define IN_S_NAME		"C:\\TESTIM_2010_2_3\\"			//Input-Image(successive)
//#define IN_S_NAME		"C:\\Users\\kawano\\Desktop\\re\\"
#define IN_S_NAME		"CAR_TRACKING/Test_Images/Daytime_Image_PNG/"			//Input-Image(successive)
#define OUTMNAME		"Out.avi"				//output movie name
#define OUT_NAME		"Out_Image/res"		//Result name
#define EX_NAME			".png"

//#define F_NAME_COM		"car_comp.csv"			//file name (component)
#define F_NAME_COM		"./CAR_TRACKING/car_comp.csv"			//file name (component)
//#define F_NAME_ROOT		"car_root.csv"			//file name (root_filter)
#define F_NAME_ROOT		"./CAR_TRACKING/car_root.csv"			//file name (root_filter)
//#define F_NAME_PART		"car_part.csv"			//file name (part_filter)
#define F_NAME_PART		"./CAR_TRACKING/car_part.csv"			//file name (part_filter)

///////////////////////
//struct information///
///////////////////////

//struct for model component information
typedef struct {

	//basic information
	//from xxxcomp.csv
	int numcomponent;	//number of component
	int sbin;			//cell size
	int interval;		//interval (for hierachical detection)
	int max_X;
	int max_Y;
	//from calculation
	int padx;			//pad information
	int pady;
	int max_scale;
	//image size information
	int IM_WIDTH;
	int IM_HEIGHT;

	//per root
	int *ridx;			//root index information
	int *oidx;			//offsetindex information
	double *offw;		//offset weight
	int *rsize;			//root size
	int *numpart;		//number of part filter per component

	//per part
	int **pidx;			//part index information
	int **didx;			//define index of part
	int **psize;

	//defs
	double *def;	//defs
	int *anchor;	//anchor

	//least_square info
	double **x1;
	double **y1;
	double **x2;
	double **y2;

	bool ini;	//flag for initialization
	double ratio;	//ratio of zooming image

}Model_info;

//struct for root_filter_information
typedef struct {
	int NoR;				//number of root filter
	int **root_size;		//size of root filter
	double **rootfilter;	//weight of root filter
	int *rootsym;			//symmetric information
}Rootfilters;

//struct for part_filter_information
typedef struct {
	int NoP;				//number of part filter
	int **part_size;		//size of part filter
	double **partfilter;	//weight of root filter
	int *part_partner;		//symmetric-partner information
	int *part_sym;			//symmetric information of part filter
}Partfilters;


//model information
typedef struct {
	Model_info *MI;
	Rootfilters *RF;
	Partfilters *PF;
}MODEL;

//Particle filter informations
typedef struct {
	int *partner;
	CvConDensation **condens;
	int *se_num;
	int **L_P;
	double **L_VX;
	double **L_VY;
	double **ave_p;
}PINFO;

//Result of Detection
typedef struct {
	int num;
	int *point;
	int *OR_point;
	IplImage **IM;
	int *type;
	double *scale;
	double *score;
}RESULT;

#endif
