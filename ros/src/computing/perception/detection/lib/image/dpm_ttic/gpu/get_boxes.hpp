#ifndef _GET_BOXES_H_
#define _GET_BOXES_H_

#include "switch_float.h"
#include "MODEL_info.h"

extern FLOAT *dpm_ttic_gpu_get_boxes(FLOAT **features,FLOAT *scales,int *FSIZE,GPUModel *MO,int *Dnum,FLOAT *A_SCORE,FLOAT thresh);

#endif /* _GET_BOXES_H_ */
