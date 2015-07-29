#ifndef _RESIZE_GPU_H_
#define _RESIZE_GPU_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cuda.h>
#include "switch_float.h"

extern FLOAT *dpm_ttic_gpu_Ipl_to_FLOAT_forGPU(IplImage *Input);
extern void dpm_ttic_gpu_resize_byGPU(FLOAT *org_image, int *org_image_size, int *resized_image_size,
				      int interval, int LEN, CUstream *stream_array);
extern void *dpm_ttic_gpu_calc_resized_image_size(int *org_image_size, int *resized_image_size, int interval,
						  FLOAT sc, int max_scale, FLOAT *scale_array);

extern void dpm_ttic_gpu_create_resized_image_texref(void);
extern void dpm_ttic_gpu_cleanup_about_resize(void);

#endif /* _RESIZE_GPU_H_ */
