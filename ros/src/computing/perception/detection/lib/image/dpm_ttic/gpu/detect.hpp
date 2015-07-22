#ifndef _DETECT_H_
#define _DETECT_H_

#include <opencv/cv.h>
#include "switch_float.h"

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

extern size_t gpu_size_A_SCORE;
extern RESULT *dpm_ttic_gpu_car_detection(IplImage *IM,GPUModel *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE,FLOAT overlap);

#endif /* _DETECT_H_ */
