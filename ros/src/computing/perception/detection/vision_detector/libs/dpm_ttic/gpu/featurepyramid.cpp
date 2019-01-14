/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////featurepyramid.cpp   calculate HOG-feature pyramid ///////////////////////////////////////////////////////////

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include <time.h>
#include <iostream>
#include <cuda.h>

using namespace std;

//Header files
#include "MODEL_info.h"		//File information
#include "common.hpp"

#include "switch_float.h"
#include "switch_release.h"
#include "drvapi_error_string.h"
#include "resize_GPU.hpp"

extern CUcontext *ctx;
extern CUfunction *func_calc_hist;
extern CUfunction *func_calc_norm;
extern CUfunction *func_calc_feat;
extern CUmodule *module;

//definition of constant
#define eps 0.0001

//definition of sin and cos
static const FLOAT Hcos[9] = {1.0000 , 0.9397, 0.7660, 0.5000, 0.1736, -0.1736, -0.5000, -0.7660, -0.9397};
static const FLOAT Hsin[9] = {0.0000 , 0.3420, 0.6428, 0.8660, 0.9848, 0.9848, 0.8660, 0.6428, 0.3420};

//definition of structure
struct thread_data {
	FLOAT *IM;
	int    ISIZE[3];
	int    FSIZE[2];
	int    F_C;
	int    sbin;
	FLOAT *Out;
};

struct thread_data_forGPU {
	//  FLOAT *IM;
	int    ISIZE[3];
	int    FSIZE[2];
	int    F_C;
	int    sbin;
	FLOAT *Out;
	CUdeviceptr hist_dev;
	CUdeviceptr norm_dev;
	CUdeviceptr feat_dev;
	CUstream stream;
};

//main function to calculate feature pyramid


//external function

//resize.cc
extern FLOAT *resize(FLOAT *src,int *sdims,int *odims,FLOAT scale); //resize image

// resize_GPU.cc

//inline functions

//return maximum number (integer)
static inline int max_i(int x,int y)
{
	return (x >= y ? x : y);
}

//return minimum number (integer)
static inline int min_i(int x,int y)
{
	return (x <= y ? x : y);
}

//return minimum number (FLOAT)
static inline FLOAT min_2(FLOAT x)
{
	return (x <= 0.2 ? x :0.2);
}

//initialization functions

//initialize scales
FLOAT *gpu_init_scales(Model_info *MI,IplImage *IM,int X,int Y) //X,Y length of image
{
	int interval,max_scale;

	if(MI->ini)
	{
		//calculate max scale
		//MI->interval/=2;	//reduce calculation time
		const int sbin = MI->sbin;
		interval = MI->interval;
		const FLOAT sc = pow(2.0,(1/(double)interval));// represent down-scale rate
		const int numcomponent = MI->numcomponent;
		//max_scale = 1+int(floor(log(minsize/(5*FLOAT(sbin)))/log(sc)));
		max_scale = 36;
		const int L_NUM = interval+max_scale;

		FLOAT MRY =(FLOAT)MI->rsize[0];
		FLOAT MRX =(FLOAT)MI->rsize[1];

		for(int kk=1;kk<numcomponent;kk++)
		{
			if(MI->rsize[kk*2]<MRY) MRY=MI->rsize[kk*2];
			if(MI->rsize[kk*2+1]<MRX) MRX=MI->rsize[kk*2+1];
		}

		MRY/=2;
		MRX/=2;

		FLOAT height =(FLOAT)IM->height/(FLOAT)sbin;
		FLOAT width = (FLOAT)IM->width/(FLOAT)sbin;
		FLOAT sc_step =1/sc;   // down-scale rate

		for(int kk=0;kk<L_NUM;kk++)
		{
			height*=sc_step;
			width*=sc_step;
			if(height<MRY || width<MRX)
			{
				max_scale = kk-interval-1;
				break;
			}
		}

		if(max_scale<interval) max_scale = interval;
		MI->max_scale=max_scale;
#ifdef PRINT_INFO
		printf("max_scale:%d\n",max_scale);
#endif
		MI->IM_HEIGHT=IM->height;

		MI->IM_WIDTH=IM->width;

		MI->ini=false;
	}
	else
	{
		interval = MI->interval;
		max_scale = MI->max_scale;
        MI->IM_HEIGHT = IM->height;
        MI->IM_WIDTH = IM->width;
	}

	//return
	FLOAT *scales = (FLOAT*)calloc((max_scale+interval),sizeof(FLOAT));		//Model information
	return(scales);
}

//initialize feature size matrix
int *gpu_init_featsize(Model_info *MI)
{
	const int LofFeat = MI->max_scale+MI->interval;
	int *featsize = (int*)calloc(LofFeat*2,sizeof(FLOAT)); // feature size information matrix
	return(featsize);
}

//calculate HOG features from Image
//HOG features are calculated for each block(BSL*BSL pixels)
#ifdef ORIGINAL
static FLOAT *calc_feature
(
 FLOAT *SRC,                    // resized image
 int *ISIZE,                    // resized image size (3 dimension)
 int *FTSIZE,                   // size of feature(output)
 int sbin                       // block size desicion element for each filter
 )
{
	/* input size */
	const int height  = ISIZE[0]; //{268,268,134,67,233,117,203,203,177,154,89,203,154,77}
	const int width   = ISIZE[1]; //{448,112,224,390,195,340,170,296,257,148,340,257,129}
	const int dims[2] = {height, width};

	/* size of Histgrams and Norm calculation space size */
	const int blocks[2] = {(int)floor(double(height)/double(sbin)+0.5), (int)floor(double(width)/double(sbin)+0.5)}; //{67,112}....sbine=4

	/* Output features size(Output) */
	int out[3] = {max_i(blocks[0]-2, 0), max_i(blocks[1]-2, 0), 27+4};

	/* Visible range (eliminate border blocks) */
	const int visible[2] = {blocks[0]*sbin, blocks[1]*sbin};

	/* HOG Histgram and Norm */
	FLOAT *hist = (FLOAT *)calloc(blocks[0]*blocks[1]*18, sizeof(FLOAT)); // HOG histgram
	FLOAT *norm = (FLOAT *)calloc(blocks[0]*blocks[1], sizeof(FLOAT));    // Norm

	/* feature(Output) */
	FLOAT *feat = (FLOAT *)calloc(out[0]*out[1]*out[2], sizeof(FLOAT));

	// for time measurement
	struct timeval hist_start, hist_end;
	struct timeval tv;
	float histCreate = 0.;

	gettimeofday(&hist_start, NULL);

	for (int x=1; x<visible[1]-1; x++) {
		for (int y=1; y<visible[0]-1; y++) {

			/* first color channel */
			FLOAT *s  = SRC + min_i(x, dims[1]-2)*dims[0] + min_i(y, dims[0]-2);
			FLOAT  dy = *(s+1) - *(s-1);
			FLOAT  dx = *(s+dims[0]) - *(s-dims[0]);
			FLOAT  v  = dx*dx + dy*dy;

			/* second color channel */
			s += dims[0]*dims[1];
			FLOAT dy2 = *(s+1) - *(s-1);
			FLOAT dx2 = *(s+dims[0]) - *(s-dims[0]);
			FLOAT v2  = dx2*dx2 + dy2*dy2;

			/* third color channel */
			s += dims[0]*dims[1];
			FLOAT dy3 = *(s+1) - *(s-1);
			FLOAT dx3 = *(s+dims[0]) - *(s-dims[0]);
			FLOAT v3  = dx3*dx3 + dy3*dy3;

			/* pick channel with strongest gradient */
			if (v2 > v) {
				v  = v2;
				dx = dx2;
				dy = dy2;
			}
			if (v3 > v) {
				v  = v3;
				dx = dx3;
				dy = dy3;
			}

			/* snap to one of 18 orientations */
			FLOAT best_dot = 0;
			int   best_o   = 0;
			for (int o=0; o<9; o++)
			{
				FLOAT dot = Hcos[o]*dx + Hsin[o]*dy;
				if (dot > best_dot) {
					best_dot = dot;
					best_o   = o;
				}
				else if (-dot > best_dot) {
					best_dot = -dot;
					best_o   = o + 9;
				}
			}

			/*add to 4 histgrams aroud pixel using linear interpolation*/
			FLOAT xp  = ((FLOAT)x+0.5)/(FLOAT)sbin - 0.5;
			FLOAT yp  = ((FLOAT)y+0.5)/(FLOAT)sbin - 0.5;
			int   ixp = (int)floor(xp);
			int   iyp = (int)floor(yp);
			FLOAT vx0 = xp - ixp;
			FLOAT vy0 = yp - iyp;
			FLOAT vx1 = 1.0 - vx0;
			FLOAT vy1 = 1.0 - vy0;
			v = sqrt(v);

			if (ixp >= 0 && iyp >= 0) {
				*(hist + ixp*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += vx1*vy1*v;
			}

			if (ixp+1 < blocks[1] && iyp >= 0) {
				*(hist + (ixp+1)*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += vx0*vy1*v;
			}

			if (ixp >= 0 && iyp+1 < blocks[0]) {
				*(hist + ixp*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += vx1*vy0*v;
			}

			if (ixp+1 < blocks[1] && iyp+1 < blocks[0]) {
				*(hist + (ixp+1)*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += vx0*vy0*v;
			}
		}
	}
	/*****************************************************************/
	// for time measurement
	/*****************************************************************/
	gettimeofday(&hist_end, NULL);
	tvsub(&hist_end, &hist_start, &tv);
	histCreate = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	printf("histCreate %f\n", histCreate);
	fflush(stdout);
	/*****************************************************************/
	/*****************************************************************/

	/* compute energy in each block by summing over orientations */
#if 1
	for (int o=0; o<9; o++) {
		FLOAT *src1 = hist + o*blocks[0]*blocks[1];
		FLOAT *src2 = hist + (o+9)*blocks[0]*blocks[1];
		FLOAT *dst  = norm;
		FLOAT *end  = norm + blocks[0]*blocks[1];

		while(dst < end) {
			*(dst++) += (*src1 + *src2) * (*src1 + *src2);
			src1++;
			src2++;
		}
	}
#else
	for (int o=0; o<18; o++) {
		FLOAT *src1 = hist + o*blocks[0]*blocks[1];
		FLOAT *dst  = norm;
		FLOAT *end  = norm + blocks[0]*blocks[1];

		while(dst < end) {
			*(dst++) += (*src1) * (*src1);
			src1++;
		}
	}
#endif

	/* compute featuers */
	for (int x=0; x<out[1]; x++) {
		for (int y=0; y<out[0]; y++) {
			FLOAT *dst = feat + x*out[0] + y;
			FLOAT *src, *p, n1, n2, n3, n4;

			p = norm + (x+1)*blocks[0] + y+1;
			n1 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

			p = norm + (x+1)*blocks[0] + y;
			n2 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

			p = norm + x*blocks[0] + y+1;
			n3 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

			p = norm + x*blocks[0] + y;
			n4 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

			FLOAT t1 = 0;
			FLOAT t2 = 0;
			FLOAT t3 = 0;
			FLOAT t4 = 0;

			/* contrast-sensitive features */
			src = hist + (x+1)*blocks[0] + (y+1);
			for (int o=0; o<18; o++) {
				FLOAT h1 = min_2(*src * n1);
				FLOAT h2 = min_2(*src * n2);
				FLOAT h3 = min_2(*src * n3);
				FLOAT h4 = min_2(*src * n4);

				*dst = 0.5 * (h1 + h2 + h3 + h4);

				t1 += h1;
				t2 += h2;
				t3 += h3;
				t4 += h4;

				dst += out[0]*out[1];
				src += blocks[0]*blocks[1];
			}

			/* contrast-insensitive features */
			src = hist + (x+1)*blocks[0] + (y+1);
			for (int o=0; o<9; o++) {
				FLOAT sum = *src + *(src + 9*blocks[0]*blocks[1]);
				FLOAT h1 = min_2(sum * n1);
				FLOAT h2 = min_2(sum * n2);
				FLOAT h3 = min_2(sum * n3);
				FLOAT h4 = min_2(sum * n4);

				*dst = 0.5 * (h1 + h2 + h3 + h4);

				dst += out[0]*out[1];
				src += blocks[0]*blocks[1];
			}

			/* texture features */
			*dst = 0.2357 * t1;
			dst += out[0]*out[1];

			*dst = 0.2357 * t2;
			dst += out[0]*out[1];

			*dst = 0.2357 * t3;
			dst += out[0]*out[1];

			*dst = 0.2357 * t4;
		}
	}

	free(hist);
	free(norm);

	/* size of feature(output) */
	*FTSIZE     = out[0];
	*(FTSIZE+1) = out[1];

	//	printf("feat%f\n",*(feat));
	return(feat);
}
#endif


/* error handling macro */
#define MY_CUDA_CHECK(res, text)                \
  if ((res) != CUDA_SUCCESS) {                  \
    printf("%s failed: res = %d\n->%s\n", (text), (res), getCudaDrvErrorString((res))); \
  exit(1);                                      \
  }

#define USE_STREAM

static pthread_barrier_t barrier;


static void calc_feature_byGPU
(
 // FLOAT *SRC,                    // resized image
 int *ISIZE,                    // resized image size (3 dimension)
 int *FTSIZE,                   // size of feature(output)
 int sbin,                      // block size desicion element for each filter
 int level,
 CUdeviceptr hist_dev,
 CUdeviceptr norm_dev,
 CUdeviceptr feat_dev,
 CUstream stream
 )
{
  /* rename argument */
  //  FLOAT *resized_image      = SRC;

  /* input size */
  const int height  = ISIZE[0]; //{268,268,134,67,233,117,203,203,177,154,89,203,154,77}
  const int width   = ISIZE[1]; //{448,112,224,390,195,340,170,296,257,148,340,257,129}

  /* size of Histgrams and Norm calculation space size */
  const int blocks[2] = {(int)floor(double(height)/double(sbin)+0.5), (int)floor(double(width)/double(sbin)+0.5)}; //{67,112}....sbine=4

  /* Output features size(Output) */
  int out[3] = {max_i(blocks[0]-2, 0), max_i(blocks[1]-2, 0), 27+4};

  /* Visible range (eliminate border blocks) */
  const int visible[2] = {blocks[0]*sbin, blocks[1]*sbin};

  /* attach CUDA Context on this p-thread */
  CUresult res;
  res = cuCtxSetCurrent(ctx[0]);
  MY_CUDA_CHECK(res, "cuCtxSetCurrent(ctx[0])");




  void *kernel_args_hist[] = {
    &hist_dev,
    (void *)&sbin,
    (void *)&visible[0],
    (void *)&visible[1],
    (void *)&level
  };


  /* decide CUDA block shape */
  int max_thread_num = 0;
  // MY_CUDA_CHECK(res, "cuDeviceGetAttribure()");

  //  max_thread_num = 128;
  max_thread_num = 1024;

  int thread_num_x = (visible[1]-1 < sqrt(max_thread_num)) ?
    visible[1]-1 : sqrt(max_thread_num);
  int thread_num_y = (visible[0]-1 < sqrt(max_thread_num)) ?
    visible[0]-1 : sqrt(max_thread_num);

  int block_num_x = (visible[1]-1) / thread_num_x;
  int block_num_y = (visible[0]-1) / thread_num_y;
  if ((visible[1]-1) % thread_num_x != 0) block_num_x++;
  if ((visible[0]-1) % thread_num_y != 0) block_num_y++;

  int sharedMemBytes = 0;

  /* p-thread barrier */
  pthread_barrier_wait(&barrier);

  /* execute GPU function */
#ifdef USE_STREAM
  res = cuLaunchKernel(
                       func_calc_hist[0], // call function
                       block_num_x,       // gridDimX
                       block_num_y,       // gridDimY
                       1,                 // gridDimZ
                       thread_num_x,      // blockDimX
                       thread_num_y,      // blockDimY
                       1,                 // blockDimZ
                       sharedMemBytes,    // sharedMemBytes
                       stream,            // hStream
                       kernel_args_hist,  // kernel Parameter
                       NULL               // extra
                       );
#else
  res = cuLaunchKernel(
                       func_calc_hist[0], // call function
                       block_num_x,       // gridDimX
                       block_num_y,       // gridDimY
                       1,                 // gridDimZ
                       thread_num_x,      // blockDimX
                       thread_num_y,      // blockDimY
                       1,                 // blockDimZ
                       sharedMemBytes,    // sharedMemBytes
                       NULL,              // hStream
                       kernel_args_hist,  // kernel Parameter
                       NULL               // extra
                       );

#endif
  MY_CUDA_CHECK(res, "cuLaunchKernel(calc_hist)");

#ifdef USE_STREAM
  /* p-thread barrier in order to enqueue Launch command in breadth first order */
  pthread_barrier_wait(&barrier);

  /* synchronize CUDA Stream */
  /*
     A CUDA operation is dispatched from the engine queue
     if preceding calls in the same stream have completed.
     So, there is no need to synchronize CUDA Stream here.
  */
  // res = cuStreamSynchronize(stream);
  // MY_CUDA_CHECK(res, "cuStreamSynchronize(stream)");
#else
  /* synchronize GPU threads */
  res = cuCtxSynchronize();
  MY_CUDA_CHECK(res, "cuCtxSynchronize(calc_hist)");
#endif




  /* compute energy in each block by summing over orientations */
  void *kernel_args_norm[] = {
    &hist_dev,
    &norm_dev,
    (void *)&blocks[0],
    (void *)&blocks[1],
    (void *)&level
  };

  /* decide CUDA block shape */
  thread_num_x = (blocks[1] < sqrt(max_thread_num)) ?
    blocks[1] : sqrt(max_thread_num);
  thread_num_y = (blocks[0] < sqrt(max_thread_num)) ?
    blocks[0] : sqrt(max_thread_num);

  block_num_x = blocks[1] / thread_num_x;
  block_num_y = blocks[0] / thread_num_y;
  if (blocks[1] % thread_num_x != 0) block_num_x++;
  if (blocks[0] % thread_num_y != 0) block_num_y++;

  sharedMemBytes = 0;

  /* execute GPU function */
#ifdef USE_STREAM
  res = cuLaunchKernel(
                       func_calc_norm[0], // call function
                       block_num_x,       // gridDimX
                       block_num_y,       // gridDimY
                       1,                 // gridDimZ
                       thread_num_x,      // blockDimX
                       thread_num_y,      // blockDimY
                       1,                 // blockDimZ
                       sharedMemBytes,    // sharedMemBytes
                       stream,            // hStream
                       kernel_args_norm,  // kernel Parameter
                       NULL               // extra
                       );
#else
  res = cuLaunchKernel(
                       func_calc_norm[0], // call function
                       block_num_x,       // gridDimX
                       block_num_y,       // gridDimY
                       1,                 // gridDimZ
                       thread_num_x,      // blockDimX
                       thread_num_y,      // blockDimY
                       1,                 // blockDimZ
                       sharedMemBytes,    // sharedMemBytes
                       NULL,              // hStream
                       kernel_args_norm,  // kernel Parameter
                       NULL               // extra
                       );
#endif
  MY_CUDA_CHECK(res, "cuLaunchKernel(calc_norm)");

#ifdef USE_STREAM
  /* p-thread barrier in order to enqueue Launch command in breadth first order */
  pthread_barrier_wait(&barrier);

  /* synchronize CUDA Stream */
  /*
     A CUDA operation is dispatched from the engine queue
     if preceding calls in the same stream have completed.
     So, there is no need to synchronize CUDA Stream here.
  */
  // res = cuStreamSynchronize(stream);
  // MY_CUDA_CHECK(res, "cuStreamSynchronize(stream)");
#else
  /* synchronize GPU threads */
  res = cuCtxSynchronize();
  MY_CUDA_CHECK(res, "cuCtxSynchronize(calc_norm)");
#endif



  /* compute featuers */
  void *kernel_args_feat[] = {
    &hist_dev,
    &norm_dev,
    &feat_dev,
    (void *)&out[0],
    (void *)&out[1],
    (void *)&blocks[0],
    (void *)&blocks[1],
    (void *)&level
  };

  /* decide CUDA block shape */
  thread_num_x = (out[1] < sqrt(max_thread_num)) ?
    out[1] : sqrt(max_thread_num);
  thread_num_y = (out[0] < sqrt(max_thread_num)) ?
    out[0] : sqrt(max_thread_num);

  if (thread_num_x == 0) thread_num_x++;
  if (thread_num_y == 0) thread_num_y++;

  block_num_x = out[1] / thread_num_x;
  block_num_y = out[0] / thread_num_y;
  if (out[1] % thread_num_x != 0 || block_num_x == 0) block_num_x++;
  if (out[0] % thread_num_y != 0 || block_num_y == 0) block_num_y++;

  sharedMemBytes = 0;

  /* execute GPU function */
#ifdef USE_STREAM
  res = cuLaunchKernel(
                       func_calc_feat[0], // call function
                       block_num_x,       // gridDimX
                       block_num_y,       // gridDimY
                       1,                 // gridDimZ
                       thread_num_x,      // blockDimX
                       thread_num_y,      // blockDimY
                       1,                 // blockDimZ
                       sharedMemBytes,    // sharedMemBytes
                       stream,            // hStream
                       kernel_args_feat,  // kernel Parameter
                       NULL               // extra
                       );
#else
  res = cuLaunchKernel(
                       func_calc_feat[0], // call function
                       block_num_x,       // gridDimX
                       block_num_y,       // gridDimY
                       1,                 // gridDimZ
                       thread_num_x,      // blockDimX
                       thread_num_y,      // blockDimY
                       1,                 // blockDimZ
                       sharedMemBytes,    // sharedMemBytes
                       NULL,              // hStream
                       kernel_args_feat,  // kernel Parameter
                       NULL               // extra
                       );
#endif
  MY_CUDA_CHECK(res, "cuLaunchKernel(calc_feat)");

#ifdef USE_STREAM
  /* p-thread barrier in order to enqueue Launch command in breadth first order */
  pthread_barrier_wait(&barrier);

  /* synchronize CUDA Stream */
  res = cuStreamSynchronize(stream);
  MY_CUDA_CHECK(res, "cuStreamSynchronize(stream)");
#else
  /* synchronize GPU threads */
  res = cuCtxSynchronize();
  MY_CUDA_CHECK(res, "cuCtxSynchronize(calc_feat)");
#endif




  //    free(hist);
  //    free(norm);

  /* size of feature(output) */
  *FTSIZE     = out[0];
  *(FTSIZE+1) = out[1];


  //	printf("feat%f\n",*(feat));
  //  return(feat);

}



//sub functions

// get pixel-intensity(FLOAT)  of image(IplImage)

FLOAT *Ipl_to_FLOAT(IplImage *Input)	//get intensity data (FLOAT) of input
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
	for(int x=0; x<width; x++)
	{
		int XT = x*3;
		for(int y=0; y<height; y++)
		{
			int pp = WS*y + XT;
			*(B++) = (FLOAT)(unsigned char)IDATA[pp];	//B
			pp++;
			*(G++) = (FLOAT)(unsigned char)IDATA[pp];	//G
			pp++;
			*(R++) = (FLOAT)(unsigned char)IDATA[pp];	//R
		}
	}
	return(Output);
}

// feature calculation
#ifdef ORIGINAL
static void* feat_calc(void *thread_arg)
{
	thread_data *args  = (thread_data *)thread_arg;
	FLOAT       *Out   = calc_feature(args->IM, args->ISIZE, args->FSIZE, args->sbin);
	args->Out = Out;
	pthread_exit((void*)thread_arg);
}
#endif

static void* feat_calc_forGPU(void *thread_arg)
{
	thread_data_forGPU *args     = (thread_data_forGPU *)thread_arg;
	//  FLOAT       *IM       = args->IM;
	int         *ISIZE    = args->ISIZE;
	int         *FSIZE    = args->FSIZE;
	int          sbin     = args->sbin;
	int          level    = args->F_C;
	CUdeviceptr  hist_dev = args->hist_dev;
	CUdeviceptr  norm_dev = args->norm_dev;
	CUdeviceptr  feat_dev = args->feat_dev;
	CUstream     stream   = args->stream;
	calc_feature_byGPU(//IM,
		ISIZE,
		FSIZE,
		sbin,
		level,
		hist_dev,
		norm_dev,
		feat_dev,
		stream
		);

	pthread_exit((void*)thread_arg);
}


#ifdef ORIGINAL
//void initialize thread data
static void ini_thread_data(thread_data *TD,FLOAT *IM,int *INSIZE,int sbin,int level)
{
	TD->IM       = IM;
	memcpy(TD->ISIZE, INSIZE, sizeof(int)*3);
	TD->FSIZE[0] = 0;
	TD->FSIZE[1] = 0;
	TD->sbin     = sbin;
	TD->F_C      = level;
}
#else
static void ini_thread_data_fouGPU(thread_data_forGPU *TD,
				   int *INSIZE,
				   int sbin,
				   int level,
				   CUdeviceptr hist_dev,
				   CUdeviceptr norm_dev,
				   CUdeviceptr feat_dev,
				   CUstream stream
	)
{
	//  TD->IM       = IM;
	memcpy(TD->ISIZE, INSIZE, sizeof(int)*3);
	TD->FSIZE[0] = 0;
	TD->FSIZE[1] = 0;
	TD->sbin     = sbin;
	TD->F_C      = level;
	TD->hist_dev = hist_dev;
	TD->norm_dev = norm_dev;
	TD->feat_dev = feat_dev;
	TD->stream   = stream;
}
#endif

//#define ORIGINAL
//calculate feature pyramid (extended to main.cpp)

//calculate feature pyramid
FLOAT **gpu_calc_f_pyramid
(
 IplImage *Image,
 Model_info *MI,
 int *FTSIZE,
 FLOAT *scale
 )	//calculate feature pyramid
{
  /* constant parameters */
  const int   max_scale = MI->max_scale;
  const int   interval  = MI->interval;
  const int   sbin      = MI->sbin;                     // for part filter
  const int   sbin2     = (int)floor((double)sbin/2.0); // for root filter
  const int   LEN       = max_scale + interval;
  const FLOAT sc        = pow(2, (1.0/(double)interval));

  int org_image_size[3]  = {Image->height, Image->width, Image->nChannels}; // original image size // (元INSIZE)

  /* Original image (FLOAT) */
  // pickup brightness values of each channel from IplImage // (originally D_I)
  FLOAT *org_image = dpm_ttic_gpu_Ipl_to_FLOAT_forGPU(Image);

#ifdef ORIGINAL
  /* features */
  FLOAT **feat = (FLOAT**)malloc(sizeof(FLOAT*)*LEN); //Model information
#endif

  /* thread for feature calculation */
#ifdef ORIGINAL
  thread_data *td = (thread_data *)calloc(LEN, sizeof(thread_data));
#else
  thread_data_forGPU *td = (thread_data_forGPU *)calloc(LEN, sizeof(thread_data_forGPU));
#endif
  pthread_t   *ts = (pthread_t *)calloc(LEN, sizeof(pthread_t));

  //  FLOAT **resized_image      = (FLOAT**)calloc(LEN, sizeof(FLOAT*)); // resized image // (originally RIM_S)
  int    *resized_image_size = (int*)calloc(LEN*3, sizeof(int));     // resized image size // (originally RI_S)
  int     t_count            = 0;

  CUresult res;
  res = cuCtxSetCurrent(ctx[0]);
  MY_CUDA_CHECK(res, "cuCtxCurrent(ctx[0])");


  /*****************************************************************/
  // for time measurement
  /*****************************************************************/
  CUevent resize_start, resize_end;
  res = cuEventCreate(&resize_start, CU_EVENT_DEFAULT);
  MY_CUDA_CHECK(res, "cuEventCreate(resize_start)");
  res = cuEventCreate(&resize_end, CU_EVENT_DEFAULT);
  MY_CUDA_CHECK(res, "cuEventCreate(resize_end)");

  cuEventRecord(resize_start, NULL);
  MY_CUDA_CHECK(res, "cuEventRecord(resize_start)");

  /* calculate resized image size */
  dpm_ttic_gpu_calc_resized_image_size(org_image_size,
				       resized_image_size,
				       interval,
				       sc,
				       max_scale,
				       scale);

#ifndef ORIGINAL
  /* create CUDA Stream for each p-thread */
  CUstream *stream = (CUstream *)malloc(LEN*sizeof(CUstream));
  for (int level=0; level<LEN; level++)
    {
      res = cuStreamCreate(&stream[level], CU_STREAM_DEFAULT);
      MY_CUDA_CHECK(res, "cuStreamCreate(stream)");
    }
#endif


  /* image resizing on GPU */
  dpm_ttic_gpu_resize_byGPU(org_image,
			    org_image_size,
			    resized_image_size,
			    interval,
			    LEN,
			    stream);

  /*****************************************************************/
  // for time measurement
  /*****************************************************************/
  cuEventRecord(resize_end, NULL);
  MY_CUDA_CHECK(res, "cuEventRecord(resize_end)");
  cuEventSynchronize(resize_end);
  MY_CUDA_CHECK(res, "cuEventSynchronize(resize_end)");

  float elapsed_time;
  res = cuEventElapsedTime(&elapsed_time, resize_start, resize_end);
  MY_CUDA_CHECK(res, "cuEventElapsedTime(resize)");
#ifdef PRINT_INFO
  printf("\n-----resize image : %f", elapsed_time);
#endif

  res = cuEventDestroy(resize_start);
  MY_CUDA_CHECK(res, "cuEventDestroy(resize_start)");
  cuEventDestroy(resize_end);
  MY_CUDA_CHECK(res, "cuEventDestroy(resize_end)");
  /*****************************************************************/
  /*****************************************************************/



  /*****************************************************************/
  // for time measurement
  /*****************************************************************/
  CUevent pre_launch_start, pre_launch_end;
  res = cuEventCreate(&pre_launch_start, CU_EVENT_DEFAULT);
  MY_CUDA_CHECK(res, "cuEventCreate(pre_launch_start)");
  res = cuEventCreate(&pre_launch_end, CU_EVENT_DEFAULT);
  MY_CUDA_CHECK(res, "cuEventCreate(pre_launch_end)");

  cuEventRecord(pre_launch_start, NULL);
  MY_CUDA_CHECK(res, "cuEventRecord(pre_launch_start)");
  /*****************************************************************/
  /*****************************************************************/

#ifndef ORIGINAL
  /*prepare for launch GPU kernel from each p-thread */

  /* allocat array to save pointer info */
  //  int *image_idx_incrementer;
  unsigned long long int *hist_ptr_incrementer;
  unsigned long long int *norm_ptr_incrementer;
  unsigned long long int *feat_ptr_incrementer;

  // res = cuMemHostAlloc((void **)&image_idx_incrementer,
  //                      LEN*sizeof(int),
  //                      CU_MEMHOSTALLOC_PORTABLE);
  // MY_CUDA_CHECK(res, "cuMemHostAlloc(image_idx_incrementer)");

  res = cuMemHostAlloc((void **)&hist_ptr_incrementer,
                       LEN*sizeof(unsigned long long int),
                       CU_MEMHOSTALLOC_PORTABLE);
  MY_CUDA_CHECK(res, "cuMemHostAlloc(hist_ptr_incrementer)");

  res = cuMemHostAlloc((void **)&norm_ptr_incrementer,
                       LEN*sizeof(unsigned long long int),
                       CU_MEMHOSTALLOC_PORTABLE);
  MY_CUDA_CHECK(res, "cuMemHostAlloc(norm_ptr_incrementer)");

  res = cuMemHostAlloc((void **)&feat_ptr_incrementer,
                       LEN*sizeof(unsigned long long int),
                       CU_MEMHOSTALLOC_PORTABLE);
  MY_CUDA_CHECK(res, "cuMemHostAlloc(feat_ptr_incrementer)");

  //  int sum_size_image = 0;
  int sum_size_hist = 0;
  int sum_size_norm = 0;
  int sum_size_feat = 0;

  /* save pointer infomation to make it easy to access memory region in GPU */
  for (int level=0; level<LEN; level++)
    {
      int sbin_inner = (level < interval) ? sbin2 : sbin;

      /* size of image of each level */
      int height_inner = resized_image_size[level*3];
      int width_inner  = resized_image_size[level*3 + 1];

      /* size of Histgram and Norm caluculation space */
      int blocks_inner[2] = {
        (int)floor((double)height_inner/(double)sbin_inner + 0.5),
        (int)floor((double)width_inner/(double)sbin_inner + 0.5)
      };

      /* size of Output features size */
      int out_size[3] = {
        max_i(blocks_inner[0]-2, 0),
        max_i(blocks_inner[1]-2, 0),
        27+4
      };

      /* save sum size until prior level */
      //      image_idx_incrementer[level] = sum_size_image/sizeof(FLOAT);
      hist_ptr_incrementer[level]  = sum_size_hist;
      norm_ptr_incrementer[level]  = sum_size_norm;
      feat_ptr_incrementer[level]  = sum_size_feat;

      /* increment this level's size */
      //      sum_size_image += height_inner*width_inner*depth_inner*sizeof(FLOAT);
      sum_size_hist  += blocks_inner[0]*blocks_inner[1]*18*sizeof(FLOAT);
      sum_size_norm  += blocks_inner[0]*blocks_inner[1]*sizeof(FLOAT);
      sum_size_feat  += out_size[0]*out_size[1]*out_size[2]*sizeof(FLOAT);
    }


  /* allocate CPU memory of Histgram, Norm and Output feature*/
  FLOAT *dst_hist = (FLOAT *)calloc(sum_size_hist, 1);
  memset(dst_hist, 0, sum_size_hist); // zero clear
  FLOAT *dst_norm = (FLOAT *)calloc(sum_size_norm, 1);
  memset(dst_norm, 0, sum_size_norm); // zero clear
  FLOAT *dst_feat = (FLOAT *)calloc(sum_size_feat, 1);
  memset(dst_feat, 0, sum_size_feat); // zero clear

  FLOAT **hist = (FLOAT **)calloc(LEN, sizeof(FLOAT *)); // histgram
  FLOAT **norm = (FLOAT **)calloc(LEN, sizeof(FLOAT *)); // norm
  FLOAT **feat = (FLOAT **)calloc(LEN, sizeof(FLOAT *)); // Model information

  unsigned long long int ptr_hist = (unsigned long long int)dst_hist;
  unsigned long long int ptr_norm = (unsigned long long int)dst_norm;
  unsigned long long int ptr_feat = (unsigned long long int)dst_feat;

  for (int level=0; level<LEN; level++)
    {
      /* distribute memory regions */
      hist[level] = (FLOAT *)(ptr_hist + hist_ptr_incrementer[level]);
      norm[level] = (FLOAT *)(ptr_norm + norm_ptr_incrementer[level]);
      feat[level] = (FLOAT *)(ptr_feat + feat_ptr_incrementer[level]);
    }


  /* allocate GPU memory */
  // CUdeviceptr resized_image_dev;
  // res = cuMemAlloc(&resized_image_dev, sum_size_image);
  // MY_CUDA_CHECK(res, "cuMemAlloc(resized_image_dev)");

  CUdeviceptr resized_image_size_dev;
  res = cuMemAlloc(&resized_image_size_dev, LEN*3*sizeof(int));
  MY_CUDA_CHECK(res, "cuMemAlloc(resized_image_size_dev)");

  CUdeviceptr hist_dev;
  res = cuMemAlloc(&hist_dev, sum_size_hist);
  MY_CUDA_CHECK(res, "cuMemAlloc(hist_dev)");

  // CUdeviceptr image_idx_incrementer_dev;
  // res = cuMemAlloc(&image_idx_incrementer_dev, LEN*sizeof(int));
  // MY_CUDA_CHECK(res, "cuMemAlloc(image_idx_incrementer)");

  CUdeviceptr hist_ptr_incrementer_dev;
  res = cuMemAlloc(&hist_ptr_incrementer_dev, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuMemAlloc(hist_ptr_incrementer_dev)");

  CUdeviceptr norm_dev;
  res = cuMemAlloc(&norm_dev, sum_size_norm);
  MY_CUDA_CHECK(res, "cuMemAlloc(norm_dev)");

  CUdeviceptr norm_ptr_incrementer_dev;
  res = cuMemAlloc(&norm_ptr_incrementer_dev, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuMemAlloc(norm_ptr_incrementer_dev)");

  CUdeviceptr feat_dev;
  res = cuMemAlloc(&feat_dev, sum_size_feat);
  MY_CUDA_CHECK(res, "cuMemAlloc(feat_dev)");

  CUdeviceptr feat_ptr_incrementer_dev;
  res = cuMemAlloc(&feat_ptr_incrementer_dev, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuMemAlloc(feat_ptr_incrementer_dev)");

  /* upload resized image data */
  // unsigned long long int ptr_resized_image_dev = (unsigned long long int)resized_image_dev;
  // for (int level=0; level<LEN; level++)
  //   {
  //     int height_inner = resized_image_size[level*3];
  //     int width_inner  = resized_image_size[level*3 + 1];
  //     int depth_inner  = resized_image_size[level*3 + 2];

  //     res = cuMemcpyHtoD((CUdeviceptr)ptr_resized_image_dev,
  //                        (void *)(&resized_image[level][0]),
  //                        height_inner*width_inner*depth_inner*sizeof(FLOAT)
  //                        );
  //     MY_CUDA_CHECK(res, "cuMemcpyHtoD(resized_image)");

  //     ptr_resized_image_dev += (unsigned long long int)height_inner*width_inner*depth_inner*sizeof(FLOAT);
  //   }

  /* upload resized image size to GPU */
  res = cuMemcpyHtoD(resized_image_size_dev, resized_image_size, LEN*3*sizeof(int));
  MY_CUDA_CHECK(res, "cuMemcpyHtoD(resized_image_size)");

  /* upload other data to GPU */
  res = cuMemcpyHtoD(hist_dev, &hist[0][0], sum_size_hist);
  MY_CUDA_CHECK(res, "cuMemcpyHtoD(hist_dev)");

  // res = cuMemcpyHtoD(image_idx_incrementer_dev, image_idx_incrementer, LEN*sizeof(int));
  // MY_CUDA_CHECK(res, "cuMemcpyHtoD(image_idx_incrementer_dev)");

  res = cuMemcpyHtoD(hist_ptr_incrementer_dev, hist_ptr_incrementer, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuMemcpyHtoD(hist_ptr_incrementer_dev)");

  res = cuMemcpyHtoD(norm_dev, &norm[0][0], sum_size_norm);
  MY_CUDA_CHECK(res, "cuMemcpyHtoD(norm_dev)");

  res = cuMemcpyHtoD(norm_ptr_incrementer_dev, norm_ptr_incrementer, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuMemcpyHtoD(norm_ptr_incrementer_dev)");

  res = cuMemcpyHtoD(feat_dev, &feat[0][0], sum_size_feat);
  MY_CUDA_CHECK(res, "cuMemcpyHtoD(feat_dev)");

  res = cuMemcpyHtoD(feat_ptr_incrementer_dev, feat_ptr_incrementer, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuMemcpyHtoD(feat_ptr_incrementer_dev)");

  /* get handle to texture memory on GPU*/
  dpm_ttic_gpu_create_resized_image_texref();

  CUtexref resized_image_size_texref;
  res = cuModuleGetTexRef(&resized_image_size_texref, module[0], "resized_image_size");
  MY_CUDA_CHECK(res, "cuModuleGetTexRef(resized_image_size)");

  // CUtexref image_idx_incrementer_texref;
  // res = cuModuleGetTexRef(&image_idx_incrementer_texref, module[0], "image_idx_incrementer");
  // MY_CUDA_CHECK(res, "cuModuleGetTexRef(image_idx_incrementer)");

  CUtexref hist_ptr_incrementer_texref;
  res = cuModuleGetTexRef(&hist_ptr_incrementer_texref, module[0], "hist_ptr_incrementer");
  MY_CUDA_CHECK(res, "cuModuleGetTexRef(hist_ptr_incrementer)");

  CUtexref norm_ptr_incrementer_texref;
  res = cuModuleGetTexRef(&norm_ptr_incrementer_texref, module[0], "norm_ptr_incrementer");
  MY_CUDA_CHECK(res, "cuModuleGetTexRef(norm_ptr_incrementer)");

  CUtexref feat_ptr_incrementer_texref;
  res = cuModuleGetTexRef(&feat_ptr_incrementer_texref, module[0], "feat_ptr_incrementer");
  MY_CUDA_CHECK(res, "cuModuleGetTexRef(feat_ptr_incrementer)");

  /* bind to texture memory on GPU */
  // res = cuTexRefSetAddress(NULL, resized_image_texref, resized_image_dev, sum_size_image);
  // MY_CUDA_CHECK(res, "cuTexRefSetAddress(resized_image_dev)");

  res = cuTexRefSetAddress(NULL, resized_image_size_texref, resized_image_size_dev, LEN*3*sizeof(int));
  MY_CUDA_CHECK(res, "cuTexRefSetAddress(resized_image_size_dev)");

  // res = cuTexRefSetAddress(NULL, image_idx_incrementer_texref, image_idx_incrementer_dev, LEN*sizeof(int));
  // MY_CUDA_CHECK(res, "cuTexRefSetAddress(image_idx_incrementer_dev)");

  res = cuTexRefSetAddress(NULL, hist_ptr_incrementer_texref, hist_ptr_incrementer_dev, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuTexRefSetAddress(hist_ptr_incrementer_dev)");

  res = cuTexRefSetAddress(NULL, norm_ptr_incrementer_texref, norm_ptr_incrementer_dev, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuTexRefSetAddress(norm_ptr_incrementer_dev)");

  res = cuTexRefSetAddress(NULL, feat_ptr_incrementer_texref, feat_ptr_incrementer_dev, LEN*sizeof(unsigned long long int));
  MY_CUDA_CHECK(res, "cuTexRefSetAddress(feat_ptr_incrementer_dev)");

  /* texture memory configuration */
  // //  res = cuTexRefSetFlags(resized_image_texref, CU_TRSF_NORMALIZED_COORDINATES);
  // res = cuTexRefSetFlags(resized_image_texref, CU_TRSF_READ_AS_INTEGER);
  // MY_CUDA_CHECK(res, "cuTexRefSetFlags(resized_image_texref)");

  //  res = cuTexRefSetFlags(resized_image_size_texref, CU_TRSF_NORMALIZED_COORDINATES);
  res = cuTexRefSetFlags(resized_image_size_texref, CU_TRSF_READ_AS_INTEGER);
  MY_CUDA_CHECK(res, "cuTexRefSetFlags(resized_image_size_texref)");

  // //  res = cuTexRefSetFlags(image_idx_incrementer_texref, CU_TRSF_NORMALIZED_COORDINATES);
  // res = cuTexRefSetFlags(image_idx_incrementer_texref, CU_TRSF_READ_AS_INTEGER);
  // MY_CUDA_CHECK(res, "cuTexRefSetFlags(image_idx_incrementer_texref)");

  //  res = cuTexRefSetFlags(hist_ptr_incrementer_texref, CU_TRSF_NORMALIZED_COORDINATES);
  res = cuTexRefSetFlags(hist_ptr_incrementer_texref, CU_TRSF_READ_AS_INTEGER);
  MY_CUDA_CHECK(res, "cuTexRefSetFlags(hist_ptr_incrementer_texref)");

  //  res = cuTexRefSetFlags(norm_ptr_incrementer_texref, CU_TRSF_NORMALIZED_COORDINATES);
  res = cuTexRefSetFlags(norm_ptr_incrementer_texref, CU_TRSF_READ_AS_INTEGER);
  MY_CUDA_CHECK(res, "cuTexRefSetFlags(norm_ptr_incrementer_texref)");

  //  res = cuTexRefSetFlags(feat_ptr_incrementer_texref, CU_TRSF_NORMALIZED_COORDINATES);
  res = cuTexRefSetFlags(feat_ptr_incrementer_texref, CU_TRSF_READ_AS_INTEGER);
  MY_CUDA_CHECK(res, "cuTexRefSetFlags(feat_ptr_incrementer_texref)");

// #ifdef USE_FLOAT_AS_DECIMAL
//   {
//     res = cuTexRefSetFormat(resized_image_texref, CU_AD_FORMAT_FLOAT, 1);
//     MY_CUDA_CHECK(res, "cuTexRefSetFormat(resized_image_texref)");
//   }
// #else
//   {
//     res = cuTexRefSetFormat(resized_image_texref, CU_AD_FORMAT_UNSIGNED_INT32, 2);
//     MY_CUDA_CHECK(res, "cuTexRefSetFormat(resized_image_texref)");
//   }
// #endif

  res = cuTexRefSetFormat(resized_image_size_texref, CU_AD_FORMAT_SIGNED_INT32, 1);
  MY_CUDA_CHECK(res, "cuTexRefSetFormat(resized_image_size_texref)");

  // res = cuTexRefSetFormat(image_idx_incrementer_texref, CU_AD_FORMAT_SIGNED_INT32, 1);
  // MY_CUDA_CHECK(res, "cuTexRefSetFormat(image_idx_incrementer_texref)");

  res = cuTexRefSetFormat(hist_ptr_incrementer_texref, CU_AD_FORMAT_UNSIGNED_INT32, 2);
  MY_CUDA_CHECK(res, "cuTexRefSetFormat(hist_ptr_incrementer_texref)");

  res = cuTexRefSetFormat(norm_ptr_incrementer_texref, CU_AD_FORMAT_UNSIGNED_INT32, 2);
  MY_CUDA_CHECK(res, "cuTexRefSetFormat(norm_ptr_incrementer_texref)");

  res = cuTexRefSetFormat(feat_ptr_incrementer_texref, CU_AD_FORMAT_UNSIGNED_INT32, 2);
  MY_CUDA_CHECK(res, "cuTexRefSetFormat(feat_ptr_incrementer_texref)");


  /*****************************************************************/
  // for time measurement
  /*****************************************************************/
  cuEventRecord(pre_launch_end, NULL);
  MY_CUDA_CHECK(res, "cuEventRecord(pre_launch_end)");
  cuEventSynchronize(pre_launch_end);
  MY_CUDA_CHECK(res, "cuEventSynchronize(pre_launch_end)");

  float time_pre_launch;
  res = cuEventElapsedTime(&time_pre_launch, pre_launch_start, pre_launch_end);
  MY_CUDA_CHECK(res, "cuEventElapsedTime(pre_launch)");
#ifdef PRINT_INFO
  printf("\n-----pre launch   : %f\n", time_pre_launch);
#endif

  res = cuEventDestroy(pre_launch_start);
  MY_CUDA_CHECK(res, "cuEventDestroy(pre_launch_start)");
  cuEventDestroy(pre_launch_end);
  MY_CUDA_CHECK(res, "cuEventDestroy(pre_launch_end)");
  /*****************************************************************/
  /*****************************************************************/





  /* create p-thread barrier */
  pthread_barrier_init(&barrier, NULL, LEN);
  /***************************/
#endif  // ifdef ORIGINAL


  /* calculate HOG feature for each resized image */
  for(int ii=0; ii<interval; ii++)
    {
      /* features for root filter(global features)? */
      /* "first" 2x interval */
#ifdef ORIGINAL
      ini_thread_data(
                      &td[t_count],
                      resized_image[ii],
                      resized_image_size + ii*3,
                      sbin2,
                      ii
                      );  //initialize thread
#else
      ini_thread_data_fouGPU(
                             &td[t_count],
                             //                             resized_image[ii],
                             resized_image_size + ii*3,
                             sbin2,
                             ii,
                             hist_dev,
                             norm_dev,
                             feat_dev,
                             stream[ii]
                             );  //initialize thread
#endif

#ifdef ORIGINAL
      if( pthread_create(&ts[t_count], NULL, feat_calc, (void*)&td[t_count]))
#else
        if( pthread_create(&ts[t_count], NULL, feat_calc_forGPU, (void*)&td[t_count]))
#endif
          {printf("Error thread\n"); exit(0);}
      //      feat_calc((void *)&td[t_count]);
      t_count++;

      /* features for part filter(local features)? */
      /* "second" 1x interval */
#ifdef ORIGINAL
      ini_thread_data(
                      &td[t_count],
                      resized_image[ii+interval],
                      resized_image_size + ii*3,
                      sbin,
                      ii+interval
                      );	//initialize thread
#else
      ini_thread_data_fouGPU(
                             &td[t_count],
                             //                             resized_image[ii+interval],
                             resized_image_size + ii*3,
                             sbin,
                             ii+interval,
                             hist_dev,
                             norm_dev,
                             feat_dev,
                             stream[ii+interval]
                             );	//initialize thread
#endif

#ifdef ORIGINAL
      if(pthread_create(&ts[t_count], NULL, feat_calc, (void*)&td[t_count]))
#else
        if(pthread_create(&ts[t_count], NULL, feat_calc_forGPU, (void*)&td[t_count]))
#endif
      {printf("Error thread\n"); exit(0);}
      //      feat_calc((void *)&td[t_count]);
      t_count++;

      /* remained resolutions (for root_only) */
      for(int jj=ii+interval; jj<max_scale; jj+=interval)
        {
#ifdef ORIGINAL
          ini_thread_data(
                          &td[t_count],
                          resized_image[jj+interval],
                          resized_image_size + (jj+interval)*3,
                          sbin,
                          jj+interval
                          ); //initialize thread
#else
          ini_thread_data_fouGPU(
                                 &td[t_count],
                                 //                                 resized_image[jj+interval],
                                 resized_image_size + (jj+interval)*3,
                                 sbin,
                                 jj+interval,
                                 hist_dev,
                                 norm_dev,
                                 feat_dev,
                                 stream[jj+interval]
                                 ); //initialize thread
#endif

#ifdef ORIGINAL
            if(pthread_create(&ts[t_count], NULL, feat_calc, (void*)&td[t_count]))
#else
              if(pthread_create(&ts[t_count], NULL, feat_calc_forGPU, (void*)&td[t_count]))
#endif
            {printf("Error thread\n"); exit(0);}
          //          feat_calc((void *)&td[t_count]);
          t_count++;
        }
    }


  /* get thread data */
  for(int ss=0; ss<LEN; ss++)
    {
      pthread_join(ts[ss], NULL);
#ifdef ORIGINAL
      feat[td[ss].F_C] = td[ss].Out; // assemble features into one array
#endif
      memcpy(&FTSIZE[td[ss].F_C*2], td[ss].FSIZE, sizeof(int)*2);

    }

#ifndef ORIGINAL
  /* destroy CUDA Stream for each p-thread */
  for (int level=0; level<LEN; level++)
    {
      res = cuStreamDestroy(stream[level]);
      MY_CUDA_CHECK(res, "cuStreamDestory(stream)");
    }
  free(stream);

  /* destroy p-thread barrier */
  pthread_barrier_destroy(&barrier);
  /****************************/



  /* download output feature from GPU */
  res = cuMemcpyDtoH((void *)&feat[0][0], feat_dev, sum_size_feat);
  MY_CUDA_CHECK(res, "cuMemcpyDtoH(feat_dev)");
#endif

  /* release original image */
  s_free(org_image);

  /* release resized image */
//  for(int ss=0; ss<interval; ss++) s_free(resized_image[ss]);
//  free(&resized_image[0][0]);
  //  for(int ss=interval*2; ss<LEN; ss++) s_free(resized_image[ss]);
  //  s_free(resized_image);
  s_free(resized_image_size);

#ifndef ORIGINAL
  /* release array to save pointer info */
  // res = cuMemFreeHost(image_idx_incrementer);
  // MY_CUDA_CHECK(res, "cuMemFreeHost(image_idx_incrementer)");

  res = cuMemFreeHost(hist_ptr_incrementer);
  MY_CUDA_CHECK(res, "cuMemFreeHost(hist_ptr_incrementer)");

  res = cuMemFreeHost(norm_ptr_incrementer);
  MY_CUDA_CHECK(res, "cuMemFreeHost(norm_ptr_incrementer)");

  res = cuMemFreeHost(feat_ptr_incrementer);
  MY_CUDA_CHECK(res, "cuMemFreeHost(feat_ptr_incrementer)");

  /* release GPU memory */
  // res = cuMemFree(resized_image_dev);
  // MY_CUDA_CHECK(res, "cuMemFree(resized_image_dev)");

  res = cuMemFree(resized_image_size_dev);
  MY_CUDA_CHECK(res, "cuMemFree(resized_image_size_dev)");

  res = cuMemFree(hist_dev);
  MY_CUDA_CHECK(res, "cuMemFree(hist_dev)");

  // res = cuMemFree(image_idx_incrementer_dev);
  // MY_CUDA_CHECK(res, "cuMemFree(image_idx_incrementer_dev)");

  res = cuMemFree(hist_ptr_incrementer_dev);
  MY_CUDA_CHECK(res, "cuMemFree(hist_ptr_incrementer_dev)");

  res = cuMemFree(norm_dev);
  MY_CUDA_CHECK(res, "cuMemFree(norm_dev)");

  res = cuMemFree(norm_ptr_incrementer_dev);
  MY_CUDA_CHECK(res, "cuMemFree(norm_ptr_incrementer_dev)");

  res = cuMemFree(feat_dev);
  MY_CUDA_CHECK(res, "cuMemFree(feat_dev)");

  res = cuMemFree(feat_ptr_incrementer_dev);
  MY_CUDA_CHECK(res, "cuMemFree(feat_ptr_incrementer_dev)");

  dpm_ttic_gpu_cleanup_about_resize();
#endif

  /* free CPU memory region */
  free(dst_hist);
  free(hist);

  free(dst_norm);
  free(norm);

  /* release thread information */
  s_free(td);
  s_free(ts);

  return(feat);
}

//release function
void gpu_free_features(FLOAT **features,Model_info *MI)
{
	if(features != NULL)
	{
		free(&features[0][0]);
		s_free(features);
	}
}
