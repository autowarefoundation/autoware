#ifndef _FEATUREPYRAMID_GPU_H_
#define _FEATUREPYRAMID_GPU_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "for_use_gpu.h"
#include "cuda_check.h"

extern int calculateHistogramGPULaunch(const int k, CvLSVMFeatureMapGPU *dev_img,
        CvLSVMFeatureMapGPU *dev_r, CvLSVMFeatureMapGPU *dev_alfa,
        CUstream stream);

extern int getFeatureMapsGPULaunch(const int k, CvLSVMFeatureMapGPU *dev_r,
        CvLSVMFeatureMapGPU *dev_alfa, CUdeviceptr *dev_nearest,
        CUdeviceptr *dev_w, CvLSVMFeatureMapGPU *dev_map, CUstream stream);

extern int calculateNormGPULaunch(CvLSVMFeatureMapGPU *dev_map_in,
        CvLSVMFeatureMapGPU *dev_norm, CUstream stream);

extern int normalizeGPULaunch(const float alfa, CvLSVMFeatureMapGPU *dev_map_in,
        CvLSVMFeatureMapGPU *dev_norm, CvLSVMFeatureMapGPU *dev_map_out,
        CUstream stream);

extern int PCAFeatureMapsAddNullableBorderGPULaunch(CvLSVMFeatureMapGPU *dev_map_in,
        CvLSVMFeatureMapGPU *dev_map_out, const int bx, const int by,
        CUstream stream);

extern int getFeatureMapsGPUStream(const int numStep, const int k,
        CvLSVMFeatureMapGPU **devs_img, CvLSVMFeatureMapGPU **devs_map,
        CUstream *streams);

extern int normalizeAndTruncateGPUStream(const int numStep, const float alfa,
        CvLSVMFeatureMapGPU **devs_map_in, CvLSVMFeatureMapGPU **devs_map_out,
        CUstream *streams);

extern int PCAFeatureMapsGPUStream(const int numStep, const int bx, const int by,
        CvLSVMFeatureMapGPU **devs_map_in, CvLSVMFeatureMap **feature_maps,
        CUstream *streams);

extern int getFeaturePyramid(IplImage * image, CvLSVMFeaturePyramid **maps,
        const int bx, const int by);

// Add for gpu
template<typename T>
int allocFeatureMapObjectGPU(CvLSVMFeatureMapGPU **obj, const int sizeX,
        const int sizeY, const int p)
{
    CUresult res;
    (*obj) = (CvLSVMFeatureMapGPU *) malloc(sizeof(CvLSVMFeatureMapGPU));
    (*obj)->sizeX = sizeX;
    (*obj)->sizeY = sizeY;
    (*obj)->numFeatures = p;

    res = cuMemAlloc(&(*obj)->map, sizeof(T) * sizeX * sizeY * p);
    CUDA_CHECK(res, "cuMemAlloc(map)");

    return LATENT_SVM_OK;
}

#endif /* _FEATUREPYRAMID_GPU_H_ */
