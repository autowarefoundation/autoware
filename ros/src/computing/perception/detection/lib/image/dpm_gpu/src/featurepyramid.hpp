#ifndef _FEATURE_PYRAMID_H_
#define _FEATURE_PYRAMID_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include "switch_float.h"
#include "MODEL_info.h"

extern FLOAT *ini_scales(Model_info *MI, IplImage *IM,int X,int Y);
extern int *ini_featsize(Model_info *MI);
extern void free_features(FLOAT **features,Model_info *MI);

extern FLOAT **calc_f_pyramid(IplImage *Image,Model_info *MI,int *FTSIZE,FLOAT *scale);

#endif /* _FEATURE_PYRAMID_H_ */
