#ifndef _FEATURE_PYRAMID_H_
#define _FEATURE_PYRAMID_H_

#include "switch_float.h"

//initialize feature size information matrix (extended to main)
extern int *dpm_ttic_cpu_ini_featsize(Model_info *MI);
//release features
extern void dpm_ttic_cpu_free_features(FLOAT **features,Model_info *MI);
//initialize scales (extended to main)
extern FLOAT *dpm_ttic_cpu_ini_scales(Model_info *MI,IplImage *IM,int X,int Y);
//calculate feature pyramid (extended to detect.c)
extern FLOAT **dpm_ttic_cpu_calc_f_pyramid(IplImage *Image,Model_info *MI,int *FTSIZE,FLOAT *scale);

#endif /* _FEATURE_PYRAMID_H_ */
