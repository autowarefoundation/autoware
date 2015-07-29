#ifndef _GET_BOXES_H_
#define _GET_BOXES_H_

#include "switch_float.h"

//Object-detection function (extended to main)
FLOAT *dpm_ttic_cpu_get_boxes(FLOAT **features,FLOAT *scales,int *FSIZE,MODEL *MO,int *Dnum,FLOAT *A_SCORE,FLOAT thresh);

#endif /* _GET_BOXES_H_ */
