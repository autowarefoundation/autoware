#ifndef _RESIZE_GPU_H_
#define _RESIZE_GPU_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cuda.h>
#include "switch_float.h"

extern FLOAT *Ipl_to_FLOAT_forGPU(IplImage *Input);
extern void resize_byGPU(FLOAT *org_image, int *org_image_size, int *resized_image_size,
			 int interval, int LEN, CUstream *stream_array);
extern void *calc_resized_image_size(int *org_image_size, int *resized_image_size, int interval,
				     FLOAT sc, int max_scale, FLOAT *scale_array);

extern void create_resized_image_texref(void);
extern void cleanup_about_resize(void);

#endif /* _RESIZE_GPU_H_ */
