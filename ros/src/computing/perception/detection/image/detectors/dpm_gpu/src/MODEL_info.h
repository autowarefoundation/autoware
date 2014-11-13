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
#include <opencv2/legacy/legacy.hpp>
#if !defined(ROS)
#ifdef _DEBUG
    // case of Debug mode
    #pragma comment(lib,"cv200d.lib") 
    #pragma comment(lib,"cxcore200d.lib") 
    #pragma comment(lib,"cvaux200d.lib") 
    #pragma comment(lib,"highgui200d.lib") 
#else
    // case of Release mode
    #pragma comment(lib,"cv200.lib") 
    #pragma comment(lib,"cxcore200.lib") 
    #pragma comment(lib,"cvaux200.lib") 
    #pragma comment(lib,"highgui200.lib") 
#endif
#endif
#include <stdio.h>	

#include "switch_float.h"
#include "switch_release.h"

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

#if defined(ROS)

#define XSTR(x) #x
#define STR(x) XSTR(x)

#if defined(CAR_DETECTOR)

#define	F_NAME_COM STR(GPU_MODEL_PATH)	"car_comp.csv"
#define	F_NAME_ROOT STR(GPU_MODEL_PATH)	"car_root.csv"
#define	F_NAME_PART STR(GPU_MODEL_PATH)	"car_part.csv"

#elif defined(PEDESTRIAN_DETECTOR)

#define	F_NAME_COM STR(GPU_MODEL_PATH)	"person_comp.csv"
#define	F_NAME_ROOT STR(GPU_MODEL_PATH)	"person_root.csv"
#define	F_NAME_PART STR(GPU_MODEL_PATH)	"person_part.csv"

#else
#error Invalid detector should be Car or Pedestrian
#endif

#else
#ifdef RELEASE

#define F_NAME_COM "/usr/local/geye_with_cam/bin/car_detecter/car_comp.csv" //file name (component)
#define F_NAME_ROOT	"/usr/local/geye_with_cam/bin/car_detecter/car_root.csv" //file name (root_filter)
#define F_NAME_PART	"/usr/local/geye_with_cam/bin/car_detecter/car_part.csv" //file name (part_filter)

#else

//#define F_NAME_COM		"car_comp.csv"			//file name (component)
#define F_NAME_COM "./CAR_TRACKING/car_comp.csv" //file name (component)
//#define F_NAME_ROOT		"car_root.csv"			//file name (root_filter)
#define F_NAME_ROOT "./CAR_TRACKING/car_root.csv"			//file name (root_filter)
//#define F_NAME_PART		"car_part.csv"			//file name (part_filter)
#define F_NAME_PART	"./CAR_TRACKING/car_part.csv"			//file name (part_filter)

#endif /* ifdef RELEASE */
#endif

///////////////////////
//struct information///
///////////////////////

#ifndef _MODEL_INFO
#define _MODEL_INFO
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
	FLOAT *offw;		//offset weight
	int *rsize;			//root size
	int *numpart;		//number of part filter per component

	//per part
	int **pidx;			//part index information
	int **didx;			//define index of part
	int **psize;

	//defs
	FLOAT *def;	//defs
	int *anchor;	//anchor

	//least_square info
	FLOAT **x1;
	FLOAT **y1;
	FLOAT **x2;
	FLOAT **y2;

	bool ini;	//flag for initialization
	FLOAT ratio;	//ratio of zooming image 

}Model_info;

//struct for root_filter_information
typedef struct {
	int NoR;				//number of root filter
	int **root_size;		//size of root filter
	FLOAT **rootfilter;	//weight of root filter
	int *rootsym;			//symmetric information
}Rootfilters;

//struct for part_filter_information
typedef struct {
	int NoP;				//number of part filter
	int **part_size;		//size of part filter
	FLOAT **partfilter;	//weight of root filter
	int *part_partner;		//symmetric-partner information
	int *part_sym;			//symmetric information of part filter
}Partfilters;


//model information
typedef struct {
	Model_info *MI;
	Rootfilters *RF;
	Partfilters *PF;
}MODEL;

#endif

//Particle filter informations
typedef struct {
	int *partner;
	CvConDensation ** condens;
	int *se_num;
	int **L_P;
	FLOAT **L_VX;
	FLOAT **L_VY;
	FLOAT **ave_p;
}PINFO;

//Result of Detection
typedef struct {
	int num;
	int *point;
	int *OR_point;
	IplImage **IM;
	int *type;
	FLOAT *scale;
	FLOAT *score;
}RESULT;

#endif
