#ifndef _NMS_H_
#define _NMS_H_

#include "switch_float.h"

//Non_maximum suppression function (extended to detect.cc)
extern FLOAT *dpm_ttic_cpu_nms(FLOAT *boxes,FLOAT overlap,int *num,MODEL *MO);

#endif /* _NMS_H_ */
