#ifndef _ROUTINE_GPU_H_
#define _ROUTINE_GPU_H_

int freeFeatureMapObjectGPU(CvLSVMFeatureMapGPU **obj);

#ifdef __cplusplus
extern "C"
#endif
int freeFeaturePyramidObjectGPU(CvLSVMFeaturePyramidGPU **obj);

#endif /* _ROUTINE_GPU_H_ */
