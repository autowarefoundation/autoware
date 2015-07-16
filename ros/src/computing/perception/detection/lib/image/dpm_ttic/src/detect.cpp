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
#include "Common.h"

#include "switch_float.h"
#include "tracking.hpp"
#include "nms.hpp"
#include "get_boxes.hpp"
#include "featurepyramid.hpp"

//definiton of functions//

//create and resize Iplimage
//initialize accumulated score

//detect car-boundary-boxes

FLOAT *ini_ac_score(IplImage *IM)
{
	int L = IM->height*IM->width;
	FLOAT *A_SCORE = (FLOAT *)calloc(L,sizeof(FLOAT));
	for(int ii=0;ii<L;ii++)
		*(A_SCORE+ii)=-100.0;
	return (A_SCORE);
}

static FLOAT *detect(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE)
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

	free(scales);
	free(featsize);
	free_features(feature, MO->MI);

	return boxes;
}

RESULT *car_detection(IplImage *image, MODEL *model, FLOAT thresh, int *D_NUMS,FLOAT *A_SCORE,FLOAT overlap)
{
	FLOAT *boxes = detect(image,model,thresh,D_NUMS,A_SCORE);	//detect high-score region
	FLOAT *rects = nms(boxes,overlap,D_NUMS,model);		//get boundary-rectangles of car
	RESULT *result = get_new_rects(image,model,rects,D_NUMS);	//get current result

	s_free(boxes);
	s_free(rects);

	return result;
}
