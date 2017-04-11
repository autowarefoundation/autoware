#ifndef _MATCHING_GPU_H_
#define _MATCHING_GPU_H_

#include "for_use_gpu.h"

extern CvLSVMFeatureMap* featureMapBorderPartFilter(CvLSVMFeatureMap *map,
        int maxXBorder, int maxYBorder);

extern int thresholdFunctionalScore(const CvLSVMFilterObject **all_F, int n,
        const CvLSVMFeaturePyramid *H, float b, CvLSVMFeatureMap *map[],
        float scoreThreshold, float **score, CvPoint **points, int **levels,
        int *kPoints, CvPoint ***partsDisplacement);

#endif /* _MATCHING_GPU_H_ */
