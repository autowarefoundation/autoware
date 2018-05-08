#include "for_use_gpu.h"
#include "cuda_check.h"
#include <stdio.h>
#include <math.h>
#include <string>
#include <unistd.h>

#define XSTR(x) #x
#define STR(x) XSTR(x)

CUdevice *dev;
CUcontext *ctx;

CUfunction *DistanceTransformTwoDimensionalProblem_func;
CUfunction *ConvolutionKernel_func;
CUfunction *BilinearKernelTex32F_func;
CUfunction *calculateHistogram_func;
CUfunction *getFeatureMaps_func;
CUfunction *calculateNorm_func;
CUfunction *normalizeAndTruncate_func;
CUfunction *PCAFeatureMapsAddNullableBorder_func;

CUmodule *module;
int *NR_MAXTHREADS_X, *NR_MAXTHREADS_Y;
int device_num;
int max_threads_num;

void init_cuda(void)
{
    CUresult res;
    std::string cubin_path(STR(CUBIN_PATH));

    // initnialize GPU
    res = cuInit(0);
    CUDA_CHECK(res, "cuInit()");

    // count the number of usable GPU
    res = cuDeviceGetCount(&device_num);
    CUDA_CHECK(res, "cuDeviceGetCount()");

    // unsupported multi GPU
    device_num = 1;

    // get device
    dev = (CUdevice*) malloc(device_num * sizeof(CUdevice));

    for (int i = 0; i < device_num; i++)
    {
        res = cuDeviceGet(&dev[i], i);
        CUDA_CHECK(res, "cuDeviceGet(&dev[%d])", i);
    }

    ctx = (CUcontext*) malloc(device_num * sizeof(CUcontext));

    module = (CUmodule*) malloc(device_num * sizeof(CUmodule));

    ConvolutionKernel_func = (CUfunction*) malloc(
            device_num * sizeof(CUfunction));
    DistanceTransformTwoDimensionalProblem_func = (CUfunction*) malloc(
            device_num * sizeof(CUfunction));
    BilinearKernelTex32F_func = (CUfunction*) malloc(
            device_num * sizeof(CUfunction));
    calculateHistogram_func = (CUfunction*) malloc(
            device_num * sizeof(CUfunction));
    getFeatureMaps_func = (CUfunction*) malloc(device_num * sizeof(CUfunction));
    calculateNorm_func = (CUfunction*) malloc(device_num * sizeof(CUfunction));
    normalizeAndTruncate_func = (CUfunction*) malloc(
            device_num * sizeof(CUfunction));
    PCAFeatureMapsAddNullableBorder_func = (CUfunction*) malloc(
            device_num * sizeof(CUfunction));

    for (int i = 0; i < device_num; i++)
    {
        res = cuCtxCreate(&ctx[i], 0, dev[i]);
        CUDA_CHECK(res, "cuCtxCreate(&ctx[%d])", i);
    }

    for (int i = 0; i < device_num; i++)
    {

        res = cuCtxSetCurrent(ctx[i]);
        CUDA_CHECK(res, "cuCtxSetCurrent(ctx[%d])", i);

        // load .cubin file
        res = cuModuleLoad(&module[i], cubin_path.c_str());
        CUDA_CHECK(res, "cuModuleLoad(&module[%d]), cubin path=%s", i, cubin_path.c_str());

        res = cuModuleGetFunction(&ConvolutionKernel_func[i], module[i],
                "ConvolutionKernel");
        CUDA_CHECK(res, "cuModuleGetFunction(ConvolutionKernel)");

        res = cuModuleGetFunction(
                &DistanceTransformTwoDimensionalProblem_func[i], module[i],
                "DistanceTransformTwoDimensionalProblemKernel");
        CUDA_CHECK(res, "cuModuleGetFunction(DistanceTransformTwoDimensionalProblemKernel)");

        res = cuModuleGetFunction(&BilinearKernelTex32F_func[i], module[i],
                "BilinearKernelTex32F");
        CUDA_CHECK(res, "cuModuleGetFunction(BilinearKernelTex32F)");

        res = cuModuleGetFunction(&calculateHistogram_func[i], module[i],
                "calculateHistogram");
        CUDA_CHECK(res, "cuModuleGetFunction(calculateHistogram)");

        res = cuModuleGetFunction(&getFeatureMaps_func[i], module[i],
                "getFeatureMaps");
        CUDA_CHECK(res, "cuModuleGetFunction(getFeatureMaps)");

        res = cuModuleGetFunction(&calculateNorm_func[i], module[i],
                "calculateNorm");
        CUDA_CHECK(res, "cuModuleGetFunction(calculateNorm)");

        res = cuModuleGetFunction(&normalizeAndTruncate_func[i], module[i],
                "normalizeAndTruncate");
        CUDA_CHECK(res, "cuModuleGetFunction(normalizeAndTruncate)");

        res = cuModuleGetFunction(&PCAFeatureMapsAddNullableBorder_func[i],
                module[i], "PCAFeatureMapsAddNullableBorder");
        CUDA_CHECK(res, "cuModuleGetFunction(PCAFeatureMapsAddNullableBorder)");
    }

    NR_MAXTHREADS_X = (int*) malloc(device_num * sizeof(int));
    NR_MAXTHREADS_Y = (int*) malloc(device_num * sizeof(int));

    for (int i = 0; i < device_num; i++)
    {
        // get max thread num per block
        max_threads_num = 0;
        res = cuDeviceGetAttribute(&max_threads_num,
                CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, dev[i]);
        CUDA_CHECK(res, "cuDeviceGetAttribute(CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK)");

        NR_MAXTHREADS_X[i] = (int) sqrt((double) max_threads_num);
        NR_MAXTHREADS_Y[i] = (int) sqrt((double) max_threads_num);
    }

}

void clean_cuda(void)
{
    CUresult res;

    for (int i = 0; i < device_num; i++)
    {
        res = cuModuleUnload(module[i]);
        CUDA_CHECK(res, "cuModuleUnload(module[%d])", i);
    }

    for (int i = 0; i < device_num; i++)
    {
        res = cuCtxDestroy(ctx[i]);
        CUDA_CHECK(res, "cuCtxDestroy(ctx[%d])", i);
    }

    free(NR_MAXTHREADS_X);
    free(NR_MAXTHREADS_Y);
    free(ConvolutionKernel_func);
    free(DistanceTransformTwoDimensionalProblem_func);
    free(BilinearKernelTex32F_func);
    free(calculateHistogram_func);
    free(getFeatureMaps_func);
    free(calculateNorm_func);
    free(normalizeAndTruncate_func);
    free(PCAFeatureMapsAddNullableBorder_func);
    free(module);
    free(dev);
    free(ctx);
}
