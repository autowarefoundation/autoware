#ifndef _RESIZEIMG_GPU_H_
#define _RESIZEIMG_GPU_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <cuda.h>
#include "for_use_gpu.h"

extern int calculateWidthStep(int width, int channel, size_t bytes);

extern void uploadImageToGPU1D(IplImage* img);

extern void cleanImageFromGPU1D();

extern void resizeLaunchUsingTex(IplImage* img, float scale, size_t type,
        CvLSVMFeatureMapGPU *dev_out, CUstream stream);

extern void resizeGPUStream(const int numStep, IplImage* img, float* scale,
        CvLSVMFeatureMapGPU **devs_img, CUstream *stream);

#endif // _RESIZE_IMG_GPU_H_
