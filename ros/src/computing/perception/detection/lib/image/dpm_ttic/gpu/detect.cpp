/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////detect.cpp   Detect car from an image

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <cstdio>
#include <cstdlib>
#include <time.h>

//ORIGINAL header files
#include "MODEL_info.h"		//File information
#include "featurepyramid.hpp"
#include "detect.hpp"
#include "nms.hpp"
#include "get_boxes.hpp"
#include "tracking.hpp"

#include "switch_float.h"
#include "switch_release.h"

//definiton of functions//

//initialize accumulated score
size_t gpu_size_A_SCORE;

//detect and save detected boxes
static FLOAT *detect(IplImage *IM,GPUModel *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE)
{
	/* for measurement */
	struct timeval tv, tv_calc_f_pyramid_start, tv_calc_f_pyramid_end;
	float time_calc_f_pyramid = 0;

	//initialize scale information for hierachical detection
	FLOAT *scales = gpu_init_scales(MO->MI,IM,IM->width,IM->height);

	//initialize feature-size matrix
	int *featsize = gpu_init_featsize(MO->MI);

	//calculate feature pyramid
	gettimeofday(&tv_calc_f_pyramid_start, NULL);
	FLOAT **feature = gpu_calc_f_pyramid(IM,MO->MI,featsize,scales);
	gettimeofday(&tv_calc_f_pyramid_end, NULL);
	tvsub(&tv_calc_f_pyramid_end, &tv_calc_f_pyramid_start, &tv);

	time_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	time_calc_f_pyramid += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
#ifdef PRINT_INFO
	printf("\n");
	printf("calc_f_pyramid %f[ms]\n", time_calc_f_pyramid);
#endif

	//detect boundary boxes
	FLOAT *boxes = dpm_ttic_gpu_get_boxes(feature,scales,featsize,MO,D_NUMS,A_SCORE,thresh);

	free(scales);
	free(featsize);
	gpu_free_features(feature,MO->MI);

	return boxes;
}

//detect car-boundary-boxes
RESULT *dpm_ttic_gpu_car_detection(IplImage *IM,GPUModel *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE,FLOAT overlap)
{
	FLOAT *boxes = detect(IM,MO,thresh,D_NUMS,A_SCORE);	//detect high-score region
	FLOAT *rects = dpm_ttic_gpu_nms(boxes,overlap,D_NUMS,MO);		//get boundary-rectangles of car
	RESULT *result = dpm_ttic_gpu_get_new_rects(IM,MO,rects,D_NUMS);	//get current result

	free(boxes);
	free(rects);
	return result;
}
