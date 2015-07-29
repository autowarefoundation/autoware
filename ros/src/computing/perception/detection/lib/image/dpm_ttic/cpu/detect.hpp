#ifndef _DETECT_H_
#define _DETECT_H_

#include "switch_float.h"

extern RESULT *dpm_ttic_cpu_car_detection(IplImage *image, MODEL *model, FLOAT thresh, int *D_NUMS,
					  FLOAT *A_SCORE,FLOAT overlap);

#endif /* _DETECT_H_ */
