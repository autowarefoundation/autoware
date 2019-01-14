#ifndef _TRACKING_H_
#define _TRACKING_H_

#include <opencv/cv.h>
#include "detect.hpp"
#include "switch_float.h"

//get object_rectangles
extern RESULT *dpm_ttic_gpu_get_new_rects(IplImage *Image,GPUModel *MO,FLOAT *boxes,int *NUM);

#endif /* _TRACKING_H_ */
