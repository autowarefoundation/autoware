#include <cmath>
#include <cuda.h>
#include <unistd.h>

#include "featurepyramid_gpu.hpp"
#include "for_use_gpu.h"
#include "resizeimg_gpu.h"
#include "cuda_check.h"
#include "routine_gpu.hpp"

extern int allocFeatureMapObject(CvLSVMFeatureMap **obj, const int sizeX, const int sizeY,
                                const int p);
extern "C" {
extern int allocFeaturePyramidObject(CvLSVMFeaturePyramid **obj, const int numLevels);
};

/*
// Launch GPU kernel of calculate histogram
//
// API
//int calculateHistogramGPULaunch(const int k, CvLSVMFeatureMapGPU *dev_img,
          CvLSVMFeatureMapGPU *dev_r, CvLSVMFeatureMapGPU *dev_alfa,
          CUstream stream)
// INPUT
// k
// dev_img
// stream
// OUTPUT
// dev_r
// dev_alfa
// RESULT
// Error status
*/
int calculateHistogramGPULaunch(const int k, CvLSVMFeatureMapGPU *dev_img,
        CvLSVMFeatureMapGPU *dev_r, CvLSVMFeatureMapGPU *dev_alfa,
        CUstream stream)
{
    int width, height, width_step, numChannels;
    int thread_num_x, thread_num_y, thread_num_z;
    int block_num_x, block_num_y, block_num_z;
    int sharedMemBytes;

    CUresult res;

    width = dev_img->sizeX;
    height = dev_img->sizeY;
    numChannels = dev_img->numFeatures;
    width_step = calculateWidthStep(width, numChannels, sizeof(float));

    void *calc_hist_kernel_arg[] =
    { (void *) &dev_img->map, (void *) &dev_r->map, (void *) &dev_alfa->map,
            (void *) &width, (void *) &height, (void *) &width_step,
            (void *) &numChannels };

    thread_num_x =
            (width < std::sqrt(max_threads_num)) ? width : std::sqrt(max_threads_num);
    thread_num_y =
            (height < std::sqrt(max_threads_num)) ? height : std::sqrt(max_threads_num);
    thread_num_z = 1;
    block_num_x = width / thread_num_x;
    block_num_y = height / thread_num_y;
    block_num_z = 1;
    if (width % thread_num_x != 0)
        block_num_x++;
    if (height % thread_num_y != 0)
        block_num_y++;

    sharedMemBytes = 0;

    res = cuLaunchKernel(calculateHistogram_func[0], block_num_x, block_num_y,
            block_num_z, thread_num_x, thread_num_y, thread_num_z,
            sharedMemBytes, stream, calc_hist_kernel_arg, NULL);
    CUDA_CHECK(res, "cuLaunchKernel(calculateHistogram)");

    return LATENT_SVM_OK;
}

/*
// Launch GPU kernel of get feature maps
//
// API
//int getFeatureMapsGPULaunch(const int k, CvLSVMFeatureMapGPU *dev_r,
          CvLSVMFeatureMapGPU *dev_alfa, CUdeviceptr *dev_nearest,
          CUdeviceptr *dev_w, CvLSVMFeatureMapGPU *dev_map, CUstream stream)
// INPUT
// k
// dev_r
// dev_alfa
// dev_nearest
// dev_w
// stream
// OUTPUT
// dev_map
// RESULT
// Error status
*/
int getFeatureMapsGPULaunch(const int k, CvLSVMFeatureMapGPU *dev_r,
        CvLSVMFeatureMapGPU *dev_alfa, CUdeviceptr *dev_nearest,
        CUdeviceptr *dev_w, CvLSVMFeatureMapGPU *dev_map, CUstream stream)
{
    int sizeX, sizeY;

    int p, width, height;
    int thread_num_x, thread_num_y, thread_num_z;
    int block_num_x, block_num_y, block_num_z;
    int sharedMemBytes;

    CUresult res;

    p = 3 * NUM_SECTOR;
    width  = dev_r->sizeX;
    height = dev_r->sizeY;
    sizeX = dev_map->sizeX;
    sizeY = dev_map->sizeY;

    void *get_feature_maps_kernel_arg[] =
    { (void *) &dev_r->map, (void *) &dev_alfa->map, (void *) dev_nearest,
            (void *) dev_w, (void *) &dev_map->map, (void *) &width,
            (void *) &height, (void *) &k, (void *) &p };

    thread_num_x =
            (sizeX < std::sqrt(max_threads_num)) ? sizeX : std::sqrt(max_threads_num);
    thread_num_y =
            (sizeY < std::sqrt(max_threads_num)) ? sizeY : std::sqrt(max_threads_num);
    thread_num_z = 1;
    block_num_x = sizeX / thread_num_x;
    block_num_y = sizeY / thread_num_y;
    block_num_z = k * k;
    if (sizeX % thread_num_x != 0)
        block_num_x++;
    if (sizeY % thread_num_y != 0)
        block_num_y++;

    sharedMemBytes = 0;

    res = cuLaunchKernel(getFeatureMaps_func[0], block_num_x, block_num_y,
            block_num_z, thread_num_x, thread_num_y, thread_num_z,
            sharedMemBytes, stream, get_feature_maps_kernel_arg, NULL);
    CUDA_CHECK(res, "cuLaunchKernel(getFeatureMaps)");

    return LATENT_SVM_OK;
}

/*
// Launch GPU kernel of calculate norm
//
// API
//int calculateNormGPULaunch(CvLSVMFeatureMapGPU *dev_map_in,
          CvLSVMFeatureMapGPU *dev_norm, CUstream stream)
// INPUT
// dev_map_in
// stream
// OUTPUT
// dev_norm
// RESULT
// Error status
*/
int calculateNormGPULaunch(CvLSVMFeatureMapGPU *dev_map_in,
        CvLSVMFeatureMapGPU *dev_norm, CUstream stream)
{
    int sizeX, sizeY, xp;
    int thread_num_x, thread_num_y, thread_num_z;
    int block_num_x, block_num_y, block_num_z;
    int sharedMemBytes;
    CUresult res;

    sizeX = dev_map_in->sizeX;
    sizeY = dev_map_in->sizeY;
    xp = dev_map_in->numFeatures;

    void *calc_norm_kernel_arg[] =
    { (void *) &dev_map_in->map, (void *) &dev_norm->map, (void *) &sizeX,
            (void *) &sizeY, (void *) &xp, };

    thread_num_x =
            (sizeX < std::sqrt(max_threads_num)) ? sizeX : std::sqrt(max_threads_num);
    thread_num_y =
            (sizeY < std::sqrt(max_threads_num)) ? sizeY : std::sqrt(max_threads_num);
    thread_num_z = 1;
    block_num_x = sizeX / thread_num_x;
    block_num_y = sizeY / thread_num_y;
    block_num_z = 1;
    if (sizeX % thread_num_x != 0)
        block_num_x++;
    if (sizeY % thread_num_y != 0)
        block_num_y++;

    sharedMemBytes = 0;

    res = cuLaunchKernel(calculateNorm_func[0], block_num_x, block_num_y,
            block_num_z, thread_num_x, thread_num_y, thread_num_z,
            sharedMemBytes, stream, calc_norm_kernel_arg, NULL);
    CUDA_CHECK(res, "cuLaunchKernel(calcuateNorm)");

    return LATENT_SVM_OK;
}

/*
// Launch GPU kernel of normalize
//
// API
// int normalizeGPULaunch(const int alfa, CvLSVMFeatureMapGPU *dev_map_in,
           CvLSVMFeatureMapGPU *dev_norm, CvLSVMFeatureMapGPU *dev_map_out,
           CUstream stream);
// INPUT
// alfa
// dev_map_in
// dev_norm
// stream
// OUTPUT
// dev_map_out
// RESULT
// Error status
*/
int normalizeGPULaunch(const float alfa, CvLSVMFeatureMapGPU *dev_map_in,
        CvLSVMFeatureMapGPU *dev_norm, CvLSVMFeatureMapGPU *dev_map_out,
        CUstream stream)
{
    int sizeX, sizeY;
    int thread_num_x, thread_num_y, thread_num_z;
    int block_num_x, block_num_y, block_num_z;
    int sharedMemBytes;
    CUresult res;

    sizeX = dev_map_in->sizeX;
    sizeY = dev_map_in->sizeY;

    void *normalize_kernel_arg[] =
    { (void *) &dev_map_in->map, (void *) &dev_norm->map,
            (void *) &dev_map_out->map, (void *) &sizeX, (void *) &sizeY,
            (void *) &alfa, };

    thread_num_x =
            (sizeX < std::sqrt(max_threads_num)) ? sizeX : std::sqrt(max_threads_num);
    thread_num_y =
            (sizeY < std::sqrt(max_threads_num)) ? sizeY : std::sqrt(max_threads_num);
    thread_num_z = 1;
    block_num_x = sizeX / thread_num_x;
    block_num_y = sizeY / thread_num_y;
    block_num_z = NUM_SECTOR * 2;
    if (sizeX % thread_num_x != 0)
        block_num_x++;
    if (sizeY % thread_num_y != 0)
        block_num_y++;

    sharedMemBytes = 0;

    res = cuLaunchKernel(normalizeAndTruncate_func[0], block_num_x, block_num_y,
            block_num_z, thread_num_x, thread_num_y, thread_num_z,
            sharedMemBytes, stream, normalize_kernel_arg, NULL);
    CUDA_CHECK(res, "cuLaunchKernel(normalizeAndTruncate)");

    return LATENT_SVM_OK;
}

/*
// Launch GPU kernel of PCA feature maps
//
// API
// int PCAFeatureMapsAddNullableBorderGPULaunch(CvLSVMFeatureMapGPU *dev_map_in,
           CvLSVMFeatureMapGPU *dev_map_out, const int bx, const int by,
           CUstream stream);
// INPUT
// dev_map_in
// bx
// by
// stream
// OUTPUT
// dev_map_out
// RESULT
// Error status
*/
int PCAFeatureMapsAddNullableBorderGPULaunch(CvLSVMFeatureMapGPU *dev_map_in,
        CvLSVMFeatureMapGPU *dev_map_out, const int bx, const int by,
        CUstream stream)
{
    int sizeX, sizeY, p;
    int thread_num_x, thread_num_y, thread_num_z;
    int block_num_x, block_num_y, block_num_z;
    int sharedMemBytes;
    CUresult res;

    sizeX = dev_map_in->sizeX;
    sizeY = dev_map_in->sizeY;
    p = dev_map_in->numFeatures;

    void *pca_kernel_arg[] =
    { (void *) &dev_map_in->map, (void *) &dev_map_out->map, (void *) &sizeX,
            (void *) &sizeY, (void *) &p, (void *) &bx, (void *) &by };

    thread_num_x =
            (sizeX < std::sqrt(max_threads_num)) ? sizeX : std::sqrt(max_threads_num);
    thread_num_y =
            (sizeY < std::sqrt(max_threads_num)) ? sizeY : std::sqrt(max_threads_num);
    thread_num_z = 1;
    block_num_x = sizeX / thread_num_x;
    block_num_y = sizeY / thread_num_y;
    block_num_z = 1;
    if (sizeX % thread_num_x != 0)
        block_num_x++;
    if (sizeY % thread_num_y != 0)
        block_num_y++;

    sharedMemBytes = 0;

    res = cuLaunchKernel(PCAFeatureMapsAddNullableBorder_func[0], block_num_x,
            block_num_y, block_num_z, thread_num_x, thread_num_y, thread_num_z,
            sharedMemBytes, stream, pca_kernel_arg, NULL);
    CUDA_CHECK(res, "cuLaunchKernel(PCAFeatureMaps)");

    return LATENT_SVM_OK;
}


/*
// Getting feature map for the selected subimage in GPU
//
// API
//int getFeatureMapsGPUStream(const int numStep, const int k,
          CvLSVMFeatureMapGPU **devs_img, CvLSVMFeatureMapGPU **devs_map,
          CUstream *streams)
// INPUT
// numStep
// k
// devs_img
// streams
// OUTPUT
// devs_map
// RESULT
// Error status
*/
int getFeatureMapsGPUStream(const int numStep, const int k,
        CvLSVMFeatureMapGPU **devs_img, CvLSVMFeatureMapGPU **devs_map,
        CUstream *streams)
{
    int sizeX, sizeY;
    int p, px;
    int height, width;
    int i, j;

    int *nearest;
    float *w, a_x, b_x;

    int size_r, size_alfa, size_nearest, size_w, size_map;

    CUresult res;
    CvLSVMFeatureMapGPU **devs_r, **devs_alfa;
    CUdeviceptr dev_nearest, dev_w;

    px = 3 * NUM_SECTOR;
    p = px;

    size_nearest = k;
    size_w = k * 2;

    devs_r = (CvLSVMFeatureMapGPU **) malloc(
            sizeof(CvLSVMFeatureMapGPU*) * numStep);
    devs_alfa = (CvLSVMFeatureMapGPU **) malloc(
            sizeof(CvLSVMFeatureMapGPU*) * numStep);
    nearest = (int *) malloc(sizeof(int) * size_nearest);
    w = (float *) malloc(sizeof(float) * size_w);

    // initialize "nearest" and "w"
    for (i = 0; i < k / 2; i++)
    {
        nearest[i] = -1;
    }/*for(i = 0; i < k / 2; i++)*/
    for (i = k / 2; i < k; i++)
    {
        nearest[i] = 1;
    }/*for(i = k / 2; i < k; i++)*/

    for (j = 0; j < k / 2; j++)
    {
        b_x = k / 2 + j + 0.5f;
        a_x = k / 2 - j - 0.5f;
        w[j * 2] = 1.0f / a_x * ((a_x * b_x) / (a_x + b_x));
        w[j * 2 + 1] = 1.0f / b_x * ((a_x * b_x) / (a_x + b_x));
    }/*for(j = 0; j < k / 2; j++)*/
    for (j = k / 2; j < k; j++)
    {
        a_x = j - k / 2 + 0.5f;
        b_x = -j + k / 2 - 0.5f + k;
        w[j * 2] = 1.0f / a_x * ((a_x * b_x) / (a_x + b_x));
        w[j * 2 + 1] = 1.0f / b_x * ((a_x * b_x) / (a_x + b_x));
    }/*for(j = k / 2; j < k; j++)*/

    res = cuMemAlloc(&dev_nearest, sizeof(int) * size_nearest);
    CUDA_CHECK(res, "cuMemAlloc(dev_nearest): %zd bytes", sizeof(int) * size_nearest);
    res = cuMemAlloc(&dev_w, sizeof(float) * size_w);
    CUDA_CHECK(res, "cuMemAlloc(dev_w): %zd bytes", sizeof(float) * size_w);

    res = cuMemcpyHtoDAsync(dev_nearest, nearest, sizeof(int) * size_nearest,
            streams[numStep - 1]);
    CUDA_CHECK(res, "cuMemcpyHtoDAsync(dev_nearest, nearest, %zd bytes)", sizeof(int) * size_nearest);
    res = cuMemcpyHtoDAsync(dev_w, w, sizeof(float) * size_w,
            streams[numStep - 1]);
    CUDA_CHECK(res, "cuMemcpyHtoDAsync(dev_w, w, %zd bytes)", sizeof(float) * size_w);

    // allocate device memory
    for (i = 0; i < numStep; i++)
    {
        width = devs_img[i]->sizeX;
        height = devs_img[i]->sizeY;

        allocFeatureMapObjectGPU<float>(&devs_r[i], width, height, 1);
        allocFeatureMapObjectGPU<int>(&devs_alfa[i], width, height, 2);
    }

    // excute async
    for (i = 0; i < numStep; i++)
    {
        // initialize "map", "r" and "alfa"
        width = devs_img[i]->sizeX;
        height = devs_img[i]->sizeY;
        sizeX = width / k;
        sizeY = height / k;
        size_map = sizeX * sizeY * p;
        size_r = width * height;
        size_alfa = width * height * 2;

        // initilize device memory value of 0
        res = cuMemsetD32Async(devs_map[i]->map, 0, size_map, streams[i]);
        CUDA_CHECK(res, "cuMemset(dev_map[%d]->map)", i);
        res = cuMemsetD32Async(devs_r[i]->map, 0, size_r, streams[i]);
        CUDA_CHECK(res, "cuMemset(dev_r[%d]->map)", i);
        res = cuMemsetD32Async(devs_alfa[i]->map, 0, size_alfa, streams[i]);
        CUDA_CHECK(res, "cuMemset(dev_alfa[%d]->map)", i);

        // launch kernel
        calculateHistogramGPULaunch(k, devs_img[i], devs_r[i], devs_alfa[i],
                streams[i]);
    }

    for (i = 0; i < numStep; i++)
    {
        getFeatureMapsGPULaunch(k, devs_r[i], devs_alfa[i], &dev_nearest,
                &dev_w, devs_map[i], streams[i]);
    }

    // free device memory
    res = cuMemFree(dev_nearest);
    CUDA_CHECK(res, "cuMemFree(dev_nearest)");
    res = cuMemFree(dev_w);
    CUDA_CHECK(res, "cuMemFree(dev_w)");

    for (i = 0; i < numStep; i++)
    {
        freeFeatureMapObjectGPU(&devs_r[i]);
        freeFeatureMapObjectGPU(&devs_alfa[i]);
    }

    free(nearest);
    free(w);
    free(devs_r);
    free(devs_alfa);

    return LATENT_SVM_OK;
}

/*
// Feature map Normalization and Truncation in GPU
//
// API
//int normalizeAndTruncateGPUStream(const int numStep, const float alfa,
          CvLSVMFeatureMapGPU **devs_map_in, CvLSVMFeatureMapGPU **devs_map_out,
          CUstream *streams)
// INPUT
// numStep
// alfa
// devs_map_in
// streams
// OUTPUT
// devs_map_out
// RESULT
// Error status
*/
int normalizeAndTruncateGPUStream(const int numStep, const float alfa,
        CvLSVMFeatureMapGPU **devs_map_in, CvLSVMFeatureMapGPU **devs_map_out,
        CUstream *streams)
{

    int sizeX, sizeY, newSizeX, newSizeY, pp;
    int size_norm, size_map_out;
    int i;
    CUresult res;
    CvLSVMFeatureMapGPU **devs_norm;

    pp = NUM_SECTOR * 12;

    devs_norm = (CvLSVMFeatureMapGPU **) malloc(
            sizeof(CvLSVMFeatureMapGPU*) * (numStep));

    // allocate device memory
    for (i = 0; i < numStep; i++)
    {
        sizeX = devs_map_in[i]->sizeX;
        sizeY = devs_map_in[i]->sizeY;
        newSizeX = sizeX - 2;
        newSizeY = sizeY - 2;

        allocFeatureMapObjectGPU<float>(&devs_norm[i], sizeX, sizeY, 1);
    }

    // exucute async
    for (i = 0; i < numStep; i++)
    {
        sizeX = devs_map_in[i]->sizeX;
        sizeY = devs_map_in[i]->sizeY;
        newSizeX = sizeX - 2;
        newSizeY = sizeY - 2;
        size_norm = sizeX * sizeY;
        size_map_out = newSizeX * newSizeY * pp;

        // initilize device memory value of 0
        res = cuMemsetD32Async(devs_norm[i]->map, 0, size_norm, streams[i]);
        CUDA_CHECK(res, "cuMemset(dev_norm)");
        res = cuMemsetD32Async(devs_map_out[i]->map, 0, size_map_out,
                streams[i]);
        CUDA_CHECK(res, "cuMemset(dev_map_out)");

        // launch kernel
        calculateNormGPULaunch(devs_map_in[i], devs_norm[i], streams[i]);

    }

    for (i = 0; i < numStep; i++)
    {
        // launch kernel
        normalizeGPULaunch(alfa, devs_map_in[i], devs_norm[i], devs_map_out[i],
                streams[i]);
    }

    // synchronize cuda stream
    for (i = 0; i < numStep; i++)
    {
        CUresult res = cuStreamSynchronize(streams[i]);
        CUDA_CHECK(res, "cuStreamSynchronize(streams[%d])", i);
    }

    // free device memory
    for (i = 0; i < numStep; i++)
    {
        freeFeatureMapObjectGPU(&devs_norm[i]);
    }

    free(devs_norm);

    return LATENT_SVM_OK;
}

/*
// Feature map reduction in GPU
// In each cell we reduce dimension of the feature vector
// according to original paper special procedure
//
// API
//int PCAFeatureMapsGPUStream(const int numStep, const int bx, const int by,
          CvLSVMFeatureMapGPU **devs_map_in, CvLSVMFeatureMap **feature_maps,
          CUstream *streams)
// INPUT
// numStep
// bx
// by
// devs_map_in
// streams
// OUTPUT
// feature_maps
// RESULT
// Error status
*/
int PCAFeatureMapsGPUStream(const int numStep, const int bx, const int by,
        CvLSVMFeatureMapGPU **devs_map_in, CvLSVMFeatureMap **feature_maps,
        CUstream *streams)
{

    int sizeX, sizeY, pp;
    int size_map_pca;
    int i;
    CUresult res;
    CvLSVMFeatureMapGPU **devs_map_pca;

    pp = NUM_SECTOR * 3 + 4;

    devs_map_pca = (CvLSVMFeatureMapGPU **) malloc(
            sizeof(CvLSVMFeatureMapGPU*) * (numStep));

    // allocate memory
    for (i = 0; i < numStep; i++)
    {
        sizeX = devs_map_in[i]->sizeX + 2 * bx;
        sizeY = devs_map_in[i]->sizeY + 2 * by;

        size_map_pca = sizeX * sizeY * pp;

        allocFeatureMapObject(&feature_maps[i], sizeX, sizeY, pp);
        allocFeatureMapObjectGPU<float>(&devs_map_pca[i], sizeX, sizeY, pp);
    }

    // exucute async
    for (i = 0; i < numStep; i++)
    {
        sizeX = devs_map_pca[i]->sizeX;
        sizeY = devs_map_pca[i]->sizeY;
        size_map_pca = sizeX * sizeY * pp;

        // initilize device memory value of 0
        res = cuMemsetD32Async(devs_map_pca[i]->map, 0, size_map_pca,
                streams[i]);
        CUDA_CHECK(res, "cuMemset(dev_map_pca[%d]->map)", i);

        // launch kernel
        PCAFeatureMapsAddNullableBorderGPULaunch(devs_map_in[i],
                devs_map_pca[i], bx, by, streams[i]);
    }

    for (i = 0; i < numStep; i++)
    {
        sizeX = devs_map_pca[i]->sizeX;
        sizeY = devs_map_pca[i]->sizeY;
        size_map_pca = sizeX * sizeY * pp;

        // copy memory from device to host
        res = cuMemcpyDtoHAsync(feature_maps[i]->map, devs_map_pca[i]->map,
                sizeof(float) * size_map_pca, streams[i]);
        CUDA_CHECK(res, "cuMemcpyDtoHAsync(feature_maps[%d]->map)", i);
    }

    // free device memory
    for (i = 0; i < numStep; i++)
    {
        freeFeatureMapObjectGPU(&devs_map_pca[i]);
    }

    free(devs_map_pca);

    return LATENT_SVM_OK;
}

/*
// Property Message
//
// API
//static int getPathOfFeaturePyramidGPUStream(IplImage * image, float step,
          int numStep, int startIndex, int sideLength, int bx, int by,
          CvLSVMFeaturePyramid **maps)
// INPUT
// image
// step
// numStep
// startIndex
// sideLength
// bx
// by
// OUTPUT
// maps
// RESULT
// Error status
*/
static int getPathOfFeaturePyramidGPUStream(IplImage * image, float step,
        int numStep, int startIndex, int sideLength, int bx, int by,
        CvLSVMFeaturePyramid **maps)
{
    CvLSVMFeatureMap **feature_maps;

    int i;
    int width, height, numChannels, sizeX, sizeY, p, pp, newSizeX, newSizeY;
    float *scales;
    CvLSVMFeatureMapGPU **devs_img, **devs_map_pre_norm, **devs_map_pre_pca;
    CUstream *streams;
    CUresult res;

    scales = (float *) malloc(sizeof(float) * (numStep));
    devs_img = (CvLSVMFeatureMapGPU **) malloc(
            sizeof(CvLSVMFeatureMapGPU*) * (numStep));
    devs_map_pre_norm = (CvLSVMFeatureMapGPU **) malloc(
            sizeof(CvLSVMFeatureMapGPU*) * (numStep));
    devs_map_pre_pca = (CvLSVMFeatureMapGPU **) malloc(
            sizeof(CvLSVMFeatureMapGPU*) * (numStep));
    streams = (CUstream *) malloc(sizeof(CUstream) * (numStep));
    feature_maps = (CvLSVMFeatureMap **) malloc(
            sizeof(CvLSVMFeatureMap *) * (numStep));

    // allocate device memory
    for (i = 0; i < numStep; i++)
    {
        scales[i] = 1.0f / powf(step, (float) i);
        width  = (int) (((float) image->width ) * scales[i] + 0.5);
        height = (int) (((float) image->height) * scales[i] + 0.5);
        numChannels = image->nChannels;
        sizeX = width  / sideLength;
        sizeY = height / sideLength;
        p  = NUM_SECTOR * 3;
        pp = NUM_SECTOR * 12;
        newSizeX = sizeX - 2;
        newSizeY = sizeY - 2;

        allocFeatureMapObjectGPU<float>(&devs_img[i], width, height,
                numChannels);
        allocFeatureMapObjectGPU<float>(&devs_map_pre_norm[i], sizeX, sizeY, p);
        allocFeatureMapObjectGPU<float>(&devs_map_pre_pca[i], newSizeX,
                newSizeY, pp);
        res = cuStreamCreate(&streams[i], CU_STREAM_DEFAULT);
        CUDA_CHECK(res, "cuStreamCreate(&streams[%d])", i);
    }

    // excute main function
    resizeGPUStream(numStep, image, scales, devs_img, streams);

    getFeatureMapsGPUStream(numStep, sideLength, devs_img, devs_map_pre_norm,
            streams);

    normalizeAndTruncateGPUStream(numStep, Val_Of_Truncate, devs_map_pre_norm,
            devs_map_pre_pca, streams);

    PCAFeatureMapsGPUStream(numStep, bx, by, devs_map_pre_pca, feature_maps,
            streams);

    // synchronize cuda stream
    for (i = 0; i < numStep; i++)
    {
        CUresult res = cuStreamSynchronize(streams[i]);
        CUDA_CHECK(res, "cuStreamSynchronize(streams[%d])", i);

        res = cuStreamDestroy(streams[i]);
        CUDA_CHECK(res, "cuStreamDestroy(streams[%d])", i);
    }

    for (i = 0; i < numStep; i++)
    {
        (*maps)->pyramid[startIndex + i] = feature_maps[i];
    }/*for(i = 0; i < numStep; i++)*/

    // free device memory
    for (i = 0; i < numStep; i++)
    {
        freeFeatureMapObjectGPU(&devs_img[i]);
        freeFeatureMapObjectGPU(&devs_map_pre_norm[i]);
        freeFeatureMapObjectGPU(&devs_map_pre_pca[i]);
    }

    free(scales);
    free(devs_img);
    free(devs_map_pre_norm);
    free(devs_map_pre_pca);
    free(streams);
    free(feature_maps);

    return LATENT_SVM_OK;
}

int getFeaturePyramid(IplImage * image, CvLSVMFeaturePyramid **maps,
        const int bx, const int by)
{
    IplImage *imgResize;
    float step;
    unsigned int numStep;
    unsigned int maxNumCells;
    unsigned int W, H;

    if (image->depth == IPL_DEPTH_32F)
    {
        imgResize = image;
    }
    else
    {
        imgResize = cvCreateImage(cvSize(image->width, image->height),
                IPL_DEPTH_32F, 3);
        cvConvert(image, imgResize);
    }

    W = imgResize->width;
    H = imgResize->height;

    step = powf(2.0f, 1.0f / ((float) Lambda));
    maxNumCells = W / Side_Length;
    if (maxNumCells > H / Side_Length)
    {
        maxNumCells = H / Side_Length;
    }
    numStep = (int) (logf((float) maxNumCells / (5.0f)) / logf(step)) + 1;

    allocFeaturePyramidObject(maps, numStep + Lambda);

#ifdef PROFILE
    TickMeter tm;

    tm.start();
    cout << "(featurepyramid.cpp)getPathOfFeaturePyramid START " << endl;
#endif

    uploadImageToGPU1D(imgResize);

    getPathOfFeaturePyramidGPUStream(imgResize, step , Lambda, 0,
            Side_Length / 2, bx, by, maps);

    getPathOfFeaturePyramidGPUStream(imgResize, step, numStep, Lambda,
            Side_Length , bx, by, maps);

    cleanImageFromGPU1D();

#ifdef PROFILE
    tm.stop();
    cout << "(featurepyramid.cpp)getPathOfFeaturePyramid END time = "
            << tm.getTimeSec() << " sec" << endl;
#endif

    if (image->depth != IPL_DEPTH_32F)
    {
        cvReleaseImage(&imgResize);
    }

    return LATENT_SVM_OK;
}
