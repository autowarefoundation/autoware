#ifndef _FEATURE_PYRAMID_H_
#define _FEATURE_PYRAMID_H_

#include <opencv/cv.h>
#include "switch_float.h"
#include "MODEL_info.h"

extern int *gpu_init_featsize(Model_info *MI);
extern FLOAT *gpu_init_scales(Model_info *MI, IplImage *IM,int X,int Y);
extern void gpu_free_features(FLOAT **features,Model_info *MI);
extern FLOAT **gpu_calc_f_pyramid(IplImage *Image,Model_info *MI,int *FTSIZE,FLOAT *scale);

#endif /* _FEATURE_PYRAMID_H_ */
