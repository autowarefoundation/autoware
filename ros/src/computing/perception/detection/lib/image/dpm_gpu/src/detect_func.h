///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////detect_func.h functions about object-detection (to extend detect.cc) /////////////////////////////////////////

#include <cstdio>
#include "MODEL_info.h"		//File information

#ifndef INCLUDED_DFunctions_
#define INCLUDED_DFunctions_

#include "switch_float.h"

//featurepyramid.cpp

//matrix initialization
FLOAT *ini_scales(Model_info *MI,IplImage *IM,int X,int Y);					//initialize scales
extern int *ini_featsize(Model_info *MI);								//initialize feature size

//feature calculatation
extern FLOAT **calc_f_pyramid(IplImage *Image,Model_info *MI,int *FTSIZE,FLOAT *scale);

//getboxes.cpp

//get boundary box coordinate
extern FLOAT *get_boxes(FLOAT **features,FLOAT *scales,int *FSIZE,MODEL *MO,int *Dnum,FLOAT *A_SCORE,FLOAT thresh);

//release matrix
extern void free_features(FLOAT **features,Model_info *MI);		//release features

//nms.cpp
extern FLOAT *nms(FLOAT *boxes,FLOAT overlap,int *num,MODEL *MO);	//Non_maximum suppression function (extended to detect.cc)

//tracking.cpp
extern RESULT *get_new_rects(IplImage *Image,MODEL *MO,FLOAT *boxes,int *NUM);	//get new_rectangle pixel_point

#endif
