#include "for_use_gpu.h"
#include "routine_gpu.hpp"
#include "cuda_check.h"

// Add for gpu
int freeFeatureMapObjectGPU(CvLSVMFeatureMapGPU **obj)
{
    CUresult res;
    if (*obj == NULL)
        return LATENT_SVM_MEM_NULL;
    res = cuMemFree((*obj)->map);
    CUDA_CHECK(res, "cuMemFree(map)");
    free(*obj);
    (*obj) = NULL;
    return LATENT_SVM_OK;
}

int freeFeaturePyramidObjectGPU(CvLSVMFeaturePyramidGPU **obj)
{
    int i;
    if (*obj == NULL)
        return LATENT_SVM_MEM_NULL;
    for (i = 0; i < (*obj)->numLevels; i++)
    {
        freeFeatureMapObjectGPU(&((*obj)->pyramid[i]));
    }
    free((*obj)->pyramid);
    free(*obj);
    (*obj) = NULL;
    return LATENT_SVM_OK;
}
