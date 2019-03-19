/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////MODEL_info.h   Detector-Model information & definition header  /////////////////////////////////////////

//OpenCV library
#ifndef INCLUDED_Minfo_
#define INCLUDED_Minfo_

#include <opencv/cv.h>
#include "switch_float.h"

/////////////////////
//file information///
/////////////////////

//struct for model component information
struct Model_info {
	//basic information
	//from xxxcomp.csv
	int numcomponent;	//number of component
	int sbin;		//cell size
	int interval;	//interval (for hierachical detection)
	int max_X;
	int max_Y;
	//from calculation
	int padx;	//pad information
	int pady;
	int max_scale;
	//image size information
	int IM_WIDTH;
	int IM_HEIGHT;

	//per root
	int *ridx;	//root index information
	int *oidx;	//offsetindex information
	FLOAT *offw;	//offset weight
	int *rsize;	//root size
	int *numpart;	//number of part filter per component

	//per part
	int **pidx;	//part index information
	int **didx;	//define index of part
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
};

//struct for root_filter_information
struct Rootfilters {
	int NoR;		//number of root filter
	int **root_size;	//size of root filter
	FLOAT **rootfilter;	//weight of root filter
	int *rootsym;		//symmetric information
};

//struct for part_filter_information
struct Partfilters {
	int NoP;		//number of part filter
	int **part_size;	//size of part filter
	FLOAT **partfilter;	//weight of root filter
	int *part_partner;	//symmetric-partner information
	int *part_sym;		//symmetric information of part filter
};

//model information
struct MODEL {
	Model_info *MI;
	Rootfilters *RF;
	Partfilters *PF;
};

//Result of Detection
struct RESULT {
	int num;
	int *point;
	int *OR_point;
	IplImage **IM;
	int *type;
	FLOAT *scale;
	FLOAT *score;
};

#endif
