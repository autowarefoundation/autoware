#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <iostream>

#include "for_use_gpu.h"
#include "resizeimg_gpu.h"
#include "cuda_check.h"

using namespace cv;

extern CUmodule *module;

CUdeviceptr dev_image;
char *image;

int calculateWidthStep(int width, int channel, size_t bytes)
{
    int widthStep = width * channel * bytes;
    int mod = widthStep % 4;

    switch (mod)
    {
    case 0:
        widthStep += 0;
        break;
    case 1:
        widthStep += 3;
        break;
    case 2:
        widthStep += 2;
        break;
    case 3:
        widthStep += 1;
        break;
    default:
        break;
    }
    return widthStep;
}

void uploadImageToGPU1D(IplImage* img)
{
    CUresult res;
    CUtexref image_texref;
    int width, height, nChannels, size_img;

    width  = img->width;
    height = img->height;
    nChannels = img->nChannels;

    size_img = width * height * nChannels;

    res = cuMemAlloc(&dev_image, sizeof(float) * size_img);
    CUDA_CHECK(res, "cuMemAlloc(&dev_image): %zd bytes", sizeof(float) * size_img);

    res = cuMemcpyHtoD(dev_image, img->imageData, sizeof(float) * size_img);
    CUDA_CHECK(res, "cuMemcpyHtoD(dev_image, img->imageData): %zd bytes", sizeof(float) * size_img);

    res = cuModuleGetTexRef(&image_texref, module[0], "texRef");
    CUDA_CHECK(res, "cuModuleGetTexRef(&image_texref)");

    res = cuTexRefSetAddress(NULL, image_texref, dev_image,
            sizeof(float) * size_img);
    CUDA_CHECK(res, "cuTexRefSetAddress(image_texref)");

    res = cuTexRefSetFlags(image_texref, CU_TRSF_READ_AS_INTEGER);
    CUDA_CHECK(res, "cuTexRefSetFlags(image_texref)");

    res = cuTexRefSetFormat(image_texref, CU_AD_FORMAT_FLOAT, 1);
    CUDA_CHECK(res, "cuTexRefSetFormat(image_texref)");
}

void cleanImageFromGPU1D()
{
    CUresult res;

    res = cuMemFree(dev_image);
    CUDA_CHECK(res, "cuMemFree(dev_image)");
}

void resizeLaunchUsingTex(IplImage* img, float scale, size_t type,
        CvLSVMFeatureMapGPU *dev_img, CUstream stream)
{
    int W, H, tW, tH;
    int nChannels;
    int widthStepIn, widthStepOut;
    int thread_num_x, thread_num_y, thread_num_z;
    int block_num_x, block_num_y, block_num_z;
    int sharedMemBytes;
    CUresult res;

    W = img->width;
    H = img->height;

    tW = dev_img->sizeX;
    tH = dev_img->sizeY;

    nChannels = img->nChannels;

    widthStepIn = img->widthStep;
    widthStepOut = calculateWidthStep(tW, nChannels, sizeof(float));

    void *kernel_arg[] =
    { (void *) &dev_img->map, (void *) &W, (void *) &H, (void *) &tW,
            (void *) &tH, (void *) &nChannels, (void *) &widthStepIn,
            (void *) &widthStepOut };

    thread_num_x = (tW < sqrt(max_threads_num)) ? tW : sqrt(max_threads_num);
    thread_num_y = (tH < sqrt(max_threads_num)) ? tH : sqrt(max_threads_num);
    thread_num_z = 1;
    block_num_x = tW / thread_num_x;
    block_num_y = tH / thread_num_y;
    block_num_z = nChannels;
    if (tW % thread_num_x != 0)
        block_num_x++;
    if (tH % thread_num_y != 0)
        block_num_y++;

    sharedMemBytes = 0;

    res = cuLaunchKernel(BilinearKernelTex32F_func[0], block_num_x, block_num_y,
            block_num_z, thread_num_x, thread_num_y, thread_num_z,
            sharedMemBytes, stream, kernel_arg, NULL);
    CUDA_CHECK(res, "cuLaunchKernel(BilinearKernelTex32F)");

}

void resizeGPUStream(const int numStep, IplImage* img, float* scale,
        CvLSVMFeatureMapGPU **devs_img, CUstream *stream)
{
    for (int i = 0; i < numStep; i++)
    {
        resizeLaunchUsingTex(img, scale[i], sizeof(float), devs_img[i],
                stream[i]);
    }
}

