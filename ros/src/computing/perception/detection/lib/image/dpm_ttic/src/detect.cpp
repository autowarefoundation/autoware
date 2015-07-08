/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////detect.cpp   Detect car from an image  ///////////////////////////////////////////////////////////////////////

//OpenCV library
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

//C++ library
#include <cstdio>
#include <cstdlib>
#include <time.h>
#include <opencv2/legacy/legacy.hpp>

//ORIGINAL header files
#include "MODEL_info.h"		//File information
#include "detect_func.h"	//functions
#include "Common.h"

#include "switch_float.h"

//definiton of functions//

//resize Image (IplImage)
IplImage *ipl_resize(IplImage *IM,FLOAT ratio);

//create and resize Iplimage
IplImage *ipl_cre_resize(IplImage *IM,int width,int height);

//initialize accumulated score
FLOAT *ini_ac_score(IplImage *IM);

//detect object and return rectangle-box coorinate (extended to main.cpp)
FLOAT *detect(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE);

//detect car-boundary-boxes
RESULT *car_detection(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE,FLOAT overlap);

//resize Image (IplImage)
IplImage *ipl_resize(IplImage *IM,FLOAT ratio)
{
	IplImage *R_I;	//Output (Resized Image)

	//parameters
	const int height = IM->height;
	const int width = IM->width;

	const int UpY = height/10;
	const int NEW_Y = height-UpY-height/10;

	const int depth = IM->depth;
	const int nChannels = IM->nChannels;

	//set ROI
	CvRect REC = cvRect(0,UpY,width,NEW_Y);
	cvSetImageROI(IM,REC);			//change ROI of Image

	if((int)((FLOAT)IM->height*ratio)==IM->height)
	{
		R_I =cvCreateImage(cvSize(width,NEW_Y),depth,nChannels);
		cvCopy(IM,R_I);		//copy
	}
	else
	{
		R_I = cvCreateImage(cvSize((int)((FLOAT)width*ratio),(int)((FLOAT)NEW_Y*ratio)),depth,nChannels);
		cvResize(IM,R_I);	//resize
	}
	cvResetImageROI(IM);

	return(R_I);
}

IplImage *ipl_cre_resize(IplImage *IM,int width,int height)
{
	IplImage *R_I = cvCreateImage(cvSize(width,height),IM->depth,IM->nChannels);
	cvResize(IM,R_I);	//resize
	return(R_I);
}

FLOAT *ini_ac_score(IplImage *IM)
{
	int L = IM->height*IM->width;
	FLOAT *A_SCORE = (FLOAT *)calloc(L,sizeof(FLOAT));
	for(int ii=0;ii<L;ii++)
		*(A_SCORE+ii)=-100.0;
	return (A_SCORE);
}

FLOAT *detect(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE)
{
	//for time measurement
	struct timeval tv;
	struct timeval tv_calc_f_pyramid_start, tv_calc_f_pyramid_end;

	//initialize scale information for hierachical detection
	FLOAT *scales=ini_scales(MO->MI,IM,IM->width,IM->height);

	//initialize feature-size matrix
	int *featsize=ini_featsize(MO->MI);
	//calculate feature pyramid

	gettimeofday(&tv_calc_f_pyramid_start, NULL);
	FLOAT **feature=calc_f_pyramid(IM,MO->MI,featsize,scales);
	gettimeofday(&tv_calc_f_pyramid_end, NULL);
	tvsub(&tv_calc_f_pyramid_end, &tv_calc_f_pyramid_start, &tv);
	printf("\n");
	printf("calc_f_pyramid %f[ms]\n", tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0);


	//detect boundary boxes
	FLOAT *boxes = get_boxes(feature,scales,featsize,MO,D_NUMS,A_SCORE,thresh);

	s_free(scales);						//release scale-information
	s_free(featsize);					//release feat size information
	free_features(feature,MO->MI);

	return boxes;
}

RESULT *car_detection(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE,FLOAT overlap)
{
	FLOAT *boxes = detect(IM,MO,thresh,D_NUMS,A_SCORE);	//detect high-score region
	FLOAT *rects = nms(boxes,overlap,D_NUMS,MO);		//get boundary-rectangles of car
	RESULT *CUR = get_new_rects(IM,MO,rects,D_NUMS);	//get current result

	s_free(boxes);
	s_free(rects);

	return CUR;
}
