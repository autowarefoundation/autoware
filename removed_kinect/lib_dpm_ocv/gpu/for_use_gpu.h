#ifndef _FOR_USE_GPU_H_
#define _FOR_USE_GPU_H_

#include <cuda.h>
#include <opencv2/objdetect/objdetect.hpp>

// Macros in OpenCV private header files
#define LATENT_SVM_MEM_NULL 2
#define LAMBDA 10 // ORIGINAL 10
#define SIDE_LENGTH 8 //ORIGINAL 8
#define VAL_OF_TRUNCATE 0.2f
#define LATENT_SVM_OK 0
#define NUM_SECTOR 9 // ORIGINAL 9
#define LATENT_SVM_SEARCH_OBJECT_FAILED -5
#define LATENT_SVM_FAILED_SUPERPOSITION -6

// Data structures in OpenCV private header files
typedef struct{
    int sizeX;
    int sizeY;
    int numFeatures;
    float *map;
} CvLSVMFeatureMap;

typedef struct{
    int numLevels;
    CvLSVMFeatureMap **pyramid;
} CvLSVMFeaturePyramid;

// Add for gpu
typedef struct
{
    int sizeX;
    int sizeY;
    int numFeatures;
    CUdeviceptr map;
} CvLSVMFeatureMapGPU;

typedef struct
{
    int numLevels;
    CvLSVMFeatureMapGPU **pyramid;
} CvLSVMFeaturePyramidGPU;

typedef struct
{
    float *score;
    int *x;
    int *y;
    int size;
} CvLSVMFilterDisposition;

extern int Lambda;
extern int Side_Length;
extern float Val_Of_Truncate;

#ifdef __cplusplus
extern "C"
{
#endif

// define variables for using GPU

extern CUdevice *dev;
extern CUcontext *ctx;

extern CUfunction *BilinearKernelTex32F_func;
extern CUfunction *calculateHistogram_func;
extern CUfunction *getFeatureMaps_func;
extern CUfunction *calculateNorm_func;
extern CUfunction *normalizeAndTruncate_func;
extern CUfunction *PCAFeatureMapsAddNullableBorder_func;
extern CUfunction *ConvolutionKernel_func;
extern CUfunction *DistanceTransformTwoDimensionalProblem_func;

extern CUmodule *module;
extern int *NR_MAXTHREADS_X, *NR_MAXTHREADS_Y;

extern int device_num;
extern int max_threads_num;

// functions for using GPU and to calculate on GPU
extern void init_cuda(void);

extern void clean_cuda(void);

#ifdef __cplusplus
}
#endif

#endif // _FOR_USE_GPU_H_
