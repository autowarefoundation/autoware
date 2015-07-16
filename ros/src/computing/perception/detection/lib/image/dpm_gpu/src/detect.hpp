#ifndef _DETECT_H_
#define _DETECT_H_

#include "switch_float.h"
#include "MODEL_info.h"

extern size_t size_A_SCORE;

extern FLOAT *ini_ac_score(IplImage *IM);
extern RESULT *car_detection(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE,FLOAT overlap);

#endif /* _DETECT_H_ */
