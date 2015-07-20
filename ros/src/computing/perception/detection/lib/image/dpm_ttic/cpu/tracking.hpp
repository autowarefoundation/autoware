#ifndef _TRACKING_H_
#define _TRACKING_H_

#include "MODEL_info.h"
#include "switch_float.h"

//get new_rectangle pixel_point
extern RESULT *dpm_ttic_cpu_get_new_rects(IplImage *Image,MODEL *MO,FLOAT *boxes,int *NUM);

#endif /* _TRACKING_H_ */
