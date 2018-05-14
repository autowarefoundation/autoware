/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////resize.cpp   resize image (Input and Output must be double-array) ////////////////////////////////////////////

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <pthread.h>
#include <cuda.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

//ORIGINAL header files
#include "common.hpp"

#include "switch_float.h"
#include "switch_release.h"
#include "drvapi_error_string.h"

//#define USE_PTHREAD

/* cuda error handling macro */
#define MY_CUDA_CHECK(res, text)                                        \
  if ((res) != CUDA_SUCCESS) {                                          \
    printf("%s failed: res = %d\n->%s\n", (text), (res), getCudaDrvErrorString((res))); \
    exit(1);                                                            \
  }

/* structure to pass some data to pthread */
typedef struct {
  int      *src_size;
  int      *dst_size;
  CUstream  stream;
  int       level;
} resize_thread_arg;

/*********************************************************************/
/* global variable for GPU */
/*********************************************************************/
static CUarray      org_image_dev;
static CUdeviceptr  resized_image_dev;
static CUdeviceptr  image_idx_incrementer_dev;
static int         *image_idx_incrementer; // Look-Up Table
extern CUdevice    *dev;
extern CUmodule    *module;
extern CUcontext   *ctx;
extern CUfunction  *func_resize;
static int          sum_size_image;

/*********************************************************************/
/* sub function to devide pixel values into row-major array by color */
/*********************************************************************/
FLOAT *Ipl_to_FLOAT_forGPU(IplImage *Input)	//get intensity data (FLOAT) of input
{
	const int width     = Input->width;
#ifdef PRINT_INFO
	printf("%d\n",width);
#endif
	const int height    = Input->height;
#ifdef PRINT_INFO
	printf("%d\n",height);
#endif
	const int nChannels = Input->nChannels;
#ifdef PRINT_INFO
	printf("%d\n",nChannels);
#endif
	const int SQ        = height*width;
	const int WS        = Input->widthStep;

	FLOAT *Output = (FLOAT *)malloc(sizeof(FLOAT)*height*width*nChannels);
#ifdef PRINT_INFO
	printf("%d",height*width*nChannels);
#endif

	FLOAT *R     = Output;
	FLOAT *G     = Output + SQ;
	FLOAT *B     = Output + 2*SQ;
	char  *IDATA = Input->imageData;

	//pick intensity of pixel (color)
    for(int y=0; y<height; y++)
      {
        for(int x=0; x<width; x++)
          {
            int XT = x*3;

            int pp = WS*y + XT;
            *(B++) = (FLOAT)(unsigned char)IDATA[pp];	//B
            pp++;
            *(G++) = (FLOAT)(unsigned char)IDATA[pp];	//G
            pp++;
            *(R++) = (FLOAT)(unsigned char)IDATA[pp];	//R
          }
      }

	return(Output);
} /* Ipl_to_FLOAT_forGPU() */

/*********************************************/
/* upload original image to GPU
   and get its texture reference */
/*********************************************/
static void upload_org_image_toGPU(FLOAT *org_image, int org_image_size[3])
{
  CUresult res;
  int src_height   = org_image_size[0];
  int src_width    = org_image_size[1];
  int src_nChannel = org_image_size[2];

  /* create CUDA Array to deal with original image in GPU */
  CUDA_ARRAY3D_DESCRIPTOR desc = {0};
  desc.Width       = src_width;
  desc.Height      = src_height;
  desc.Depth       = src_nChannel;
#ifdef USE_FLOAT_AS_DECIMAL
  desc.Format      = CU_AD_FORMAT_FLOAT;
  desc.NumChannels = 1;
#else
  desc.Format      = CU_AD_FORMAT_UNSIGNED_INT32;
  desc.NumChannels = 2;
#endif
  desc.Flags       = CUDA_ARRAY3D_LAYERED;

  res = cuArray3DCreate(&org_image_dev, &desc);
  MY_CUDA_CHECK(res, "cuArray3DCreate(org_image)");

  /* upload data to GPU */
  CUDA_MEMCPY3D pCopy = {0};
  /* src config */
  pCopy.srcMemoryType = CU_MEMORYTYPE_HOST;
  pCopy.srcHost       = org_image;
  pCopy.srcPitch      = src_width * sizeof(FLOAT);
  pCopy.srcHeight     = src_height;
  /* dst config */
  pCopy.dstMemoryType = CU_MEMORYTYPE_ARRAY;
  pCopy.dstArray      = org_image_dev;
  /* 3D copy performing config */
  pCopy.WidthInBytes  = src_width *  sizeof(FLOAT);
  pCopy.Height        = src_height;
  pCopy.Depth         = src_nChannel;

  res = cuMemcpy3D(&pCopy);
  MY_CUDA_CHECK(res, "cuMemcpy3D(org_image)");

  CUtexref org_image_texref;
  /* get handle of texture reference */
  res = cuModuleGetTexRef(&org_image_texref, module[0], "org_image");
  MY_CUDA_CHECK(res, "cuModuleGetTexRef(org_image)");

  /* bind CUDA Array to texture reference */
  res = cuTexRefSetArray(org_image_texref, org_image_dev, CU_TRSA_OVERRIDE_FORMAT);
  MY_CUDA_CHECK(res, "cuTexRefSetArray(org_image)");

  /* configuration for automatic interpolation */
  res = cuTexRefSetFilterMode(org_image_texref, CU_TR_FILTER_MODE_LINEAR);
  MY_CUDA_CHECK(res, "cuTexRefSetFilterMode(MODE_LINEAR)");

  /* configuration in order to return border value when outside region access is occur */
  res = cuTexRefSetAddressMode(org_image_texref, 0, CU_TR_ADDRESS_MODE_CLAMP);
  MY_CUDA_CHECK(res, "cuTexRefSetAddressMode(dim0)");

  res = cuTexRefSetAddressMode(org_image_texref, 1, CU_TR_ADDRESS_MODE_CLAMP);
  MY_CUDA_CHECK(res, "cuTexRefSetAddressMode(dim1)");

  res = cuTexRefSetAddressMode(org_image_texref, 2, CU_TR_ADDRESS_MODE_CLAMP);
  MY_CUDA_CHECK(res, "cuTexRefSetAddressMode(dim2)");

  /* other configurations */
  res = cuTexRefSetFlags(org_image_texref, CU_TRSF_READ_AS_INTEGER);
  MY_CUDA_CHECK(res, "cuTexRefSetFlags(org_image)");

#ifdef USE_FLOAT_AS_DECIMAL
  res = cuTexRefSetFormat(org_image_texref, CU_AD_FORMAT_FLOAT, 1);
#else
  res = cuTexRefSetFormat(org_image_texref, CU_AD_FORMAT_UNSIGNED_INT32, 2);
#endif
  MY_CUDA_CHECK(res, "cuTexRefSetFormat(org_image)");

} /* upload_org_image_toGPU() */


/**********************************************************/
/* create Look-Up Table(LUT) and get texture reference
   to adjust pointer position in GPU */
/**********************************************************/
static void  make_image_idx_incrementer(int *resized_image_size, int LEN)
{
  CUresult res;
  res = cuMemHostAlloc((void **)&image_idx_incrementer,
                       LEN*sizeof(int),
                       CU_MEMHOSTALLOC_PORTABLE);
  MY_CUDA_CHECK(res, "cuMemHostAlloc(image_idx_incrementer)");
  memset(image_idx_incrementer, 0, LEN*sizeof(int));  // zero clear

  sum_size_image = 0;  // zero clear global variable
  for (int level=0; level<LEN; level++)
    {
      int height = resized_image_size[level*3];
      int width  = resized_image_size[level*3 + 1];
      int depth  = resized_image_size[level*3 + 2];

      /* save total image size until this level */
      image_idx_incrementer[level] = sum_size_image;

      /* add this level's image size */
      sum_size_image += height * width * depth;
    }

  CUdeviceptr image_idx_incrementer_dev;
  /* allocate GPU memory region for LUT */
  res = cuMemAlloc(&image_idx_incrementer_dev, LEN*sizeof(int));
  MY_CUDA_CHECK(res, "cuMemAlloc(image_idx_incrementer)");

  /* upload data to GPU */
  res = cuMemcpyHtoD(image_idx_incrementer_dev, image_idx_incrementer, LEN*sizeof(int));
  MY_CUDA_CHECK(res, "cuMemcpyHtoD(image_idx_incrementer)");

  CUtexref image_idx_incrementer_texref;
  /* get handle of texture reference */
  res = cuModuleGetTexRef(&image_idx_incrementer_texref, module[0], "image_idx_incrementer");
  MY_CUDA_CHECK(res, "cuModuleGetTexRef(image_idx_incrementer)");

  /* bind to texture reference */
  res = cuTexRefSetAddress(NULL, image_idx_incrementer_texref, image_idx_incrementer_dev, LEN*sizeof(int));
  MY_CUDA_CHECK(res, "cuTexRefSetAddress(image_idx_incrementer)");

  /* configure texture memory */
  res = cuTexRefSetFlags(image_idx_incrementer_texref, CU_TRSF_READ_AS_INTEGER);
  MY_CUDA_CHECK(res, "cuTexRefSetFlags(image_idx_incrementer_texref)");

  res = cuTexRefSetFormat(image_idx_incrementer_texref, CU_AD_FORMAT_SIGNED_INT32, 1);
  MY_CUDA_CHECK(res, "cuTexRefSetFormat(image_idx_incrementer_texref)");

} /* make_image_idx_incrementer() */


/***************************************************************/
/* image resizing function using bilinear interpolation method */
/***************************************************************/
static void *bilinear_resizing(void *arg)
{
  resize_thread_arg *this_arg = (resize_thread_arg *)arg;
  //  FLOAT    *src_top  = this_arg->src_top;
  int      *src_size = this_arg->src_size;
  //  FLOAT *dst_top = this_arg->dst_top;
  int      *dst_size = this_arg->dst_size;
  int       level    = this_arg->level;
  CUstream  stream   = this_arg->stream;

  const int src_height = src_size[0];
  const int src_width  = src_size[1];
  const int dst_height = dst_size[0];
  const int dst_width  = dst_size[1];
  const int nChannels  = dst_size[2];

  const FLOAT hfactor = (FLOAT)src_height/dst_height;
  const FLOAT wfactor = (FLOAT)src_width/dst_width;

  CUresult res;

  /* attach CUDA Context 0 on this pthread */
  res = cuCtxSetCurrent(ctx[0]);
  MY_CUDA_CHECK(res, "cuCtxSetCurrent(ctx[0]");


  /* CUDA kernel argument */
  void *kernel_arg[] = {
    (void *)&src_height,
    (void *)&src_width,
    &resized_image_dev,
    (void *)&dst_height,
    (void *)&dst_width,
    (void *)&hfactor,
    (void *)&wfactor,
    (void *)&level
  };

  /* define CUDA kernel shape */
  int max_thread_num = 0;
  res = cuDeviceGetAttribute(&max_thread_num, CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, dev[0]);
  MY_CUDA_CHECK(res, "cuDeviceGetAttribute()");

  int thread_num_x = (dst_width < sqrt(max_thread_num)) ? dst_width : sqrt(max_thread_num);
  int thread_num_y = (dst_height < sqrt(max_thread_num)) ? dst_height : sqrt(max_thread_num);

  int block_num_x = dst_width / thread_num_x;
  int block_num_y = dst_height / thread_num_y;
  int block_num_z = nChannels;
  if (dst_width % thread_num_x != 0) block_num_x++;
  if (dst_height % thread_num_y != 0) block_num_y++;

  int sharedMemBytes = 0;

  /* execute GPU function */
  res = cuLaunchKernel(
                       func_resize[0], // call function
                       block_num_x,    // gridDimX
                       block_num_y,    // gridDimY
                       block_num_z,    // gridDimZ
                       thread_num_x,   // blockDimX
                       thread_num_y,   // blockDimY
                       1,              // blockDimZ
                       sharedMemBytes, // sharedMemBytes
                       stream,         // hStream
                       kernel_arg,     // kernel Parameter
                       NULL            // extra
                       );
  MY_CUDA_CHECK(res, "cuLaunchKernel(resize)");

  // res = cuStreamSynchronize(stream);
  // MY_CUDA_CHECK(res, "cuStreamSynchronize(stream)");

  return (void *)NULL;
} /* bilinear_resizing() */

/* main function (resize) */
void resize_byGPU(FLOAT *org_image,
                  int *org_image_size,
                  //                  FLOAT **resized_image,
                  int *resized_image_size,
                  int interval,
                  int LEN,
                  CUstream *stream_array)
{
  /* pthread handler */
  /* to calculate all resized image, the required number of threads is (LEN - interval) */
  pthread_t *thread = (pthread_t *)calloc(LEN - interval, sizeof(pthread_t));

  /* structure to carry data to pthread function */
  resize_thread_arg *args = (resize_thread_arg *)calloc(LEN - interval, sizeof(resize_thread_arg));
  int thread_count = 0;


  /* upload original image data to GPU */
  upload_org_image_toGPU(org_image, org_image_size);

  /* create Look-Up Table to adjust pointer in GPU */
  make_image_idx_incrementer(resized_image_size, LEN);

  CUresult res;

  /* allocate GPU memory resion for resized image */
  res = cuMemAlloc(&resized_image_dev, sum_size_image*sizeof(FLOAT));
  MY_CUDA_CHECK(res, "cuMemAlloc(resized_image)");

#ifdef USE_FLOAT_AS_DECIMAL
  res = cuMemsetD32(resized_image_dev, 0, sum_size_image);
  MY_CUDA_CHECK(res, "cuMemsetD32(resized_image_dev)");
#else
  res = cuMemsetD32(resized_image_dev, 0, sum_size_image*2);
  MY_CUDA_CHECK(res, "cuMemsetD32(resized_image_dev)");
#endif

#ifdef DEBUG
  printf("\n/*************************************************************************/\n");
  printf("!!!!!!!!!!!!!!!!!!!!!!!!!!Debug MODE is ON !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  printf(" -> pthread is not created\n");
  printf("/*************************************************************************/\n");
#endif

  /* resizing */
  for (int level=0; level<interval; level++)
    {
      /* assign data for pthread function */
      args[thread_count].src_size = org_image_size;
      args[thread_count].dst_size = &resized_image_size[level*3];
      args[thread_count].stream   = stream_array[level];
      args[thread_count].level    = level;

#ifdef DEBUG
      bilinear_resizing((void *)&args[thread_count]);
#else
      pthread_create(&thread[thread_count], NULL, bilinear_resizing, (void *)&args[thread_count]);
#endif
      thread_count++;
    }

  /* extra resizing */
  for (int level=2*interval; level<LEN; level++)
    {
      /* assign data for pthread function */
      args[thread_count].src_size = org_image_size;
      args[thread_count].dst_size = &resized_image_size[level*3];
      args[thread_count].stream   = stream_array[level];
      args[thread_count].level    = level;

#ifdef DEBUG
      bilinear_resizing((void *)&args[thread_count]);
#else
      pthread_create(&thread[thread_count], NULL, bilinear_resizing, (void *)&args[thread_count]);
#endif
      thread_count++;
    }

#ifndef DEBUG
  /* wait for all pthread complete its work */
  //  for (int counter=0; counter<LEN-interval; counter++)
  for (int counter=0; counter<thread_count; counter++)
    {
      pthread_join(thread[counter], NULL);
    }
#endif

  /* (interval <= level < 2*interval) use same resize scale as (0 <= level < interval) */
  for (int level=interval; level<2*interval; level++)
    {
      /* wait until copy source is ready */
      res = cuStreamSynchronize(stream_array[level-interval]);
      MY_CUDA_CHECK(res, "cuStreamSynchronize()");

      unsigned long long int src_ptr = (unsigned long long int)resized_image_dev + (unsigned long long int)(image_idx_incrementer[level - interval]*sizeof(FLOAT));
      unsigned long long int dst_ptr = (unsigned long long int)resized_image_dev + (unsigned long long int)(image_idx_incrementer[level]*sizeof(FLOAT));
      int copy_size = resized_image_size[level*3] * resized_image_size[level*3 + 1] * resized_image_size[level*3 + 2] * sizeof(FLOAT);
      res = cuMemcpyDtoDAsync((CUdeviceptr)dst_ptr,
                              (CUdeviceptr)src_ptr,
                              copy_size,
                              stream_array[level]);
    }

  /* cleanup */
  free(thread);
  free(args);

  res = cuArrayDestroy(org_image_dev);
  MY_CUDA_CHECK(res, "cuMemFree(org_image_dev)");

  res = cuMemFreeHost(image_idx_incrementer);
  MY_CUDA_CHECK(res, "cuMemFreeHost(image_idx_incrementer)");

} /* resize_byGPU()  */

/* calculate each image size after resizing */
void calc_resized_image_size(int *org_image_size,
                             int *resized_image_size,
                             int interval,
                             FLOAT sc,
                             int max_scale,
                             FLOAT *scale_array)
{
 const int org_height    = org_image_size[0];
 const int org_width     = org_image_size[1];
 const int org_nChannels = org_image_size[2];

  for (int ii=0; ii<interval; ii++)
    {
      /* resizing rate */
      FLOAT scale = 1.0/pow(sc, ii);

      /* calculte and assign resized image size */
      if (scale==1.0)
        {
          resized_image_size[ii*3]     = org_height;
          resized_image_size[ii*3 + 1] = org_width;
          resized_image_size[ii*3 + 2] = org_nChannels;
        }
      else
        {
          resized_image_size[ii*3]     = (int)((FLOAT)org_height*scale + 0.5);
          resized_image_size[ii*3 + 1] = (int)((FLOAT)org_width*scale + 0.5);
          resized_image_size[ii*3 + 2] = org_nChannels;
        }

      memcpy(resized_image_size + (ii+interval)*3, resized_image_size + (ii)*3, 3*sizeof(int));

      /* save scale */
      scale_array[ii] = scale*2;
      scale_array[ii+interval] = scale;

      /* extra resizing  */
      const FLOAT extra_scale = 0.5;
      for (int jj=ii+interval; jj<max_scale; jj+=interval)
        {
          resized_image_size[(jj+interval)*3]     = (int)((FLOAT)resized_image_size[jj*3]*extra_scale + 0.5);
          resized_image_size[(jj+interval)*3 + 1] = (int)((FLOAT)resized_image_size[jj*3 + 1]*extra_scale + 0.5);
          resized_image_size[(jj+interval)*3 + 2] = resized_image_size[jj*3 + 2];

          /* save scale */
          scale_array[jj+interval] = 0.5*scale_array[jj];
        }
    }

  return;
} /* calc_resized_image_size() */

/*************************************************/
/* free GPU memory region allocated in this file */
/*************************************************/
void create_resized_image_texref(void)
{
  CUresult res;
  CUtexref resized_image_texref;

  /* get handle to texture memory on GPU */
#ifdef USE_FLOAT_AS_DECIMAL
  {
    res = cuModuleGetTexRef(&resized_image_texref, module[0], "resized_image");
    MY_CUDA_CHECK(res, "cuModuleGetTexRef(resized_image)");
  }
#else
  {
    res = cuModuleGetTexRef(&resized_image_texref, module[0], "resized_image_double");
    MY_CUDA_CHECK(res, "cuModuleGetTexRef(resized_image)");
  }
#endif

  /* bind to texture memory on GPU */
  res = cuTexRefSetAddress(NULL, resized_image_texref, resized_image_dev, sum_size_image*sizeof(FLOAT));
  MY_CUDA_CHECK(res, "cuTexRefSetAddress(resized_image_dev)");

  /* texture memory configuration */
  res = cuTexRefSetFlags(resized_image_texref, CU_TRSF_READ_AS_INTEGER);
  MY_CUDA_CHECK(res, "cuTexRefSetFlags(resized_image_texref)");

#ifdef USE_FLOAT_AS_DECIMAL
  {
    res = cuTexRefSetFormat(resized_image_texref, CU_AD_FORMAT_FLOAT, 1);
    MY_CUDA_CHECK(res, "cuTexRefSetFormat(resized_image_texref)");
  }
#else
  {
    res = cuTexRefSetFormat(resized_image_texref, CU_AD_FORMAT_UNSIGNED_INT32, 2);
    MY_CUDA_CHECK(res, "cuTexRefSetFormat(resized_image_texref)");
  }
#endif

} /* create_resized_image_texref() */

/* free GPU memory region allocated in this file */
void cleanup_about_resize(void)
{
  CUresult res;
  res = cuMemFree(resized_image_dev);
  MY_CUDA_CHECK(res, "cuMemFree(resized_image_dev)");

  res = cuMemFree(image_idx_incrementer_dev);
  MY_CUDA_CHECK(res, "cuMemFree(image_idx_incrementer)");
} /* cleanup_about_resize() */
