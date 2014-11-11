///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////get_boxes.cpp  detect boundary-boxes-coordinate of oject /////////////////////////////////////////////////////

//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Header files
#include "MODEL_info.h"				//File information
#include "get_boxes_func.h"			//external functions 
#include "Common.h"

#include "for_use_GPU.h"
#include "switch_float.h"
#include "switch_release.h"

int max_numpart = 0;
int max_RL_S = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//definiton of functions//

//subfunctions for detection

FLOAT *padarray(FLOAT *feature,int *size,int padx,int pady); //padd zeros to image 
FLOAT *flip_feat(FLOAT *feat,int *size); //filip feature order (to reduce calculation time)
int *get_gmpc(FLOAT *score,FLOAT thresh,int *ssize,int *GMN); //get good matches

//calculate root rectangle-cooridnate 
FLOAT *rootbox(int x,int y,FLOAT scale,int padx,int pady,int *rsize);	

//calculate @art rectangle-cooridnate 
FLOAT *partbox(int x,int y,int ax,int ay,FLOAT scale,int padx,int pady,int *psize,int *lx,int *ly,int *ssize);

//calculate accumulated HOG detector score
//void calc_a_score(FLOAT *ac_score,FLOAT *score,int *ssize,int *rsize,Model_info *MI,FLOAT scale);
void calc_a_score_GPU(FLOAT *ac_score, FLOAT **score, int *ssize_start, Model_info *MI, FLOAT scale, int *size_score_array, int NoC);

//Object-detection function (extended to main)
FLOAT *get_boxes(FLOAT **features,FLOAT *scales,int *FSIZE,MODEL *MO,int *Dnum,FLOAT *A_SCORE,FLOAT thresh);

//release-functions
void free_rootmatch(FLOAT **rootmatch, MODEL *MO);
void free_partmatch(FLOAT **partmatch, MODEL *MO);
void free_boxes(FLOAT **boxes,int LofFeat);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//sub functions
//padd zeros to image 
FLOAT *padarray(FLOAT *feature,int *size,int padx,int pady)
{
  const int NEW_Y=size[0]+pady*2;
  const int NEW_X=size[1]+padx*2;
  const int L=NEW_Y*padx;
  const int SPL=size[0]+pady;
  const int M_S = sizeof(FLOAT)*size[0];
  
  CUresult res;
  FLOAT *new_feature;
  
  res = cuMemHostAlloc((void **)&new_feature, NEW_Y*NEW_X * size[2] * sizeof(FLOAT), CU_MEMHOSTALLOC_DEVICEMAP); // feature (pad-added)
  if(res != CUDA_SUCCESS) {
    printf("cuMemHostAlloc(new_feature) failed: res = %s\n", conv(res));
    exit(1);
  }
  
  memset(new_feature, 0, NEW_Y*NEW_X * size[2] * sizeof(FLOAT));  // zero clear
  

  FLOAT *P=new_feature;
  FLOAT *S=feature;
  for(int ii=0;ii<size[2];ii++)	//31
    {
      P+=L; //row-all
      for(int jj=0;jj<size[1];jj++)	
		{
          P+=pady;
          //memcpy_s(P,M_S,S,M_S);
          memcpy(P, S,M_S);
          S+=size[0];
          P+=SPL;
		}
      P+=L; //row-all
	}
  size[0]=NEW_Y;
  size[1]=NEW_X;
  
  return(new_feature);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//flip feat 
FLOAT *flip_feat(FLOAT *feat,int *size)
{
	const int ORD[31]={10,9,8,7,6,5,4,3,2,1,18,17,16,15,14,13,12,11,19,27,26,25,24,23,22,21,20,30,31,28,29};
	const int S_S =sizeof(FLOAT)*size[0];

#ifndef ORIGINAL
    CUresult res;
#if 0
    res = cuCtxPushCurrent(ctx);
    if(res != CUDA_SUCCESS){
      printf("cuCtxPushCurrent() failed: res = %s\n", conv(res));
      exit(1);
    }
#endif
#endif

#ifdef ORIGINAL
	FLOAT *fliped = (FLOAT*)calloc(size[0]*size[1]*size[2],sizeof(FLOAT)); // feature (pad-added)
#else
    FLOAT *fliped;
    res = cuMemHostAlloc((void **)&fliped, size[0]*size[1]*size[2]*sizeof(FLOAT), CU_MEMHOSTALLOC_DEVICEMAP);
    if(res != CUDA_SUCCESS){
      printf("cuMemHostAlloc(fliped) failed: res = %s\n", conv(res));
      exit(1);
    }

    memset(fliped, 0, size[0]*size[1]*size[2]*sizeof(FLOAT));

#endif

	FLOAT *D=fliped;
	int DIM = size[0]*size[1];
	for(int ii=0;ii<size[2];ii++)
	{
		FLOAT *S=feat+DIM*(*(ORD+ii));

		for(int jj=1;jj<=size[1];jj++)
		{
			FLOAT *P=S-*(size)*jj; //*(size)=size[0]
			//memcpy_s(D,S_S,P,S_S);
            memcpy(D, P,S_S);
			D+=*size;
			P+=*size;
		}
	}

#ifndef ORIGINAL
#if 0
    res = cuCtxPopCurrent(&ctx);
    if(res != CUDA_SUCCESS){
      printf("cuCtxPopCurrent(ctx) failed: res = %s\n", conv(res));
      exit(1);
    }
#endif
#endif

	return(fliped);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//get good-matched pixel-coordinate 
int *get_gmpc(FLOAT *score,FLOAT thresh,int *ssize,int *GMN)
{
  const int L = ssize[0]*ssize[1];
  FLOAT *P=score;
  int NUM=0;
  FLOAT MAX_SCORE=thresh;
  
  for(int ii=0;ii<L;ii++)
    {
      if(*P>thresh) 
        {
          NUM++;	
          if(*P>MAX_SCORE) MAX_SCORE=*P;
        }
      P++;
    }
  
  int *Out =(int*)malloc(NUM*2*sizeof(int));		
  P=score;
  
  int ii=0,qq=0;
  while(qq<NUM)
    {
      if(*P>thresh)
        {
          Out[2*qq]=ii%ssize[0];		//Y coordinate of match
          Out[2*qq+1]=ii/ssize[0];	//X coordinate of match
          qq++;
        }
      ii++;
      P++;
    }
  //printf("MAX_SCORE %f\n",MAX_SCORE);
  *GMN = NUM;
  return(Out);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//get root-box pixel coordinate 
FLOAT *rootbox(int x,int y,FLOAT scale,int padx,int pady,int *rsize)
{
  FLOAT *Out=(FLOAT*)malloc(sizeof(FLOAT)*4);
  Out[0]=((FLOAT)y-(FLOAT)pady+1)*scale;	//Y1
  Out[1]=((FLOAT)x-(FLOAT)padx+1)*scale;	//X1
  Out[2]=Out[0]+(FLOAT)rsize[0]*scale-1.0;	//Y2
  Out[3]=Out[1]+(FLOAT)rsize[1]*scale-1.0;	//X2
  return(Out);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//get part-box pixel coordinate 
//partbox(PA,x,y,ax[pp],ay[pp],scale,padx,pady,Pd_size,Ix[pp],Iy[pp],Isize);
FLOAT *partbox(int x,int y,int ax,int ay,FLOAT scale,int padx,int pady,int *psize,int *lx,int *ly,int *ssize)
{
  FLOAT *Out=(FLOAT*)malloc(sizeof(FLOAT)*4);
  int probex = (x-1)*2+ax;
  int probey = (y-1)*2+ay;
  int P = probey+probex*ssize[0];
  
  FLOAT px = (FLOAT)lx[P]+1.0;
  FLOAT py = (FLOAT)ly[P]+1.0;
  
  Out[0]=((py-2.0)/2.0+1.0-(FLOAT)pady)*scale;	//Y1
  Out[1]=((px-2.0)/2.0+1.0-(FLOAT)padx)*scale;	//X1
  Out[2]=Out[0]+(FLOAT)psize[0]*scale/2.0-1.0;		//Y2
  Out[3]=Out[1]+(FLOAT)psize[1]*scale/2.0-1.0;		//X2
  return(Out);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//calculate accumulated HOG detector score
void calc_a_score(FLOAT *ac_score,FLOAT *score,int *ssize,int *rsize,Model_info *MI,FLOAT scale)
{
  const int IHEI = MI->IM_HEIGHT;
  const int IWID = MI->IM_WIDTH;
  int pady_n = MI->pady;
  int padx_n = MI->padx;
  int block_pad = (int)(scale/2.0);

  int RY = (int)((FLOAT)rsize[0]*scale/2.0-1.0+block_pad);
  int RX = (int)((FLOAT)rsize[1]*scale/2.0-1.0+block_pad);
  
  for(int ii=0;ii<IWID;ii++)
    {
      int Xn=(int)((FLOAT)ii/scale+padx_n);
      
      for(int jj=0;jj<IHEI;jj++)
        {
          int Yn =(int)((FLOAT)jj/scale+pady_n);

          if(Yn<ssize[0] && Xn<ssize[1])
            {
              FLOAT sc = score[Yn+Xn*ssize[0]]; //get score of pixel
              
              int Im_Y = jj+RY;
              int Im_X = ii+RX;
              if(Im_Y<IHEI && Im_X<IWID)
                {
                  FLOAT *PP=ac_score+Im_Y+Im_X*IHEI; //consider root rectangle size
                  if(sc>*PP) *PP=sc;                 //save max score
                }
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

extern size_t size_A_SCORE;

void calc_a_score_GPU(
  FLOAT *ac_score,
  FLOAT **score,
  int *ssize_start,
  Model_info *MI,
  FLOAT scale, 
  int *size_score_array,
  int NoC
                      )
{
  CUresult res;

  const int IHEI = MI->IM_HEIGHT;
  const int IWID = MI->IM_WIDTH;
  int pady_n = MI->pady;
  int padx_n = MI->padx;
  int block_pad = (int)(scale/2.0);

  struct timeval tv;  

  int *RY_array, *RX_array;
  res = cuMemHostAlloc((void**)&RY_array, NoC*sizeof(int), CU_MEMHOSTALLOC_DEVICEMAP);
  if(res != CUDA_SUCCESS) {
    printf("cuMemHostAlloc(RY_array) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemHostAlloc((void**)&RX_array, NoC*sizeof(int), CU_MEMHOSTALLOC_DEVICEMAP);
  if(res != CUDA_SUCCESS) {
    printf("cuMemHostAlloc(RX_array) failed: res = %s\n", conv(res));
    exit(1);
  }

  for(int jj=0; jj<NoC; jj++) {
    int rsize[2] = {MI->rsize[jj*2], MI->rsize[jj*2+1]};
    
    RY_array[jj] = (int)((FLOAT)rsize[0]*scale/2.0-1.0+block_pad);
    RX_array[jj] = (int)((FLOAT)rsize[1]*scale/2.0-1.0+block_pad);
  }
  
  CUdeviceptr ac_score_dev, score_dev;
  CUdeviceptr ssize_dev, size_score_dev;
  CUdeviceptr RY_dev, RX_dev;


  int size_score=0;
  for(int jj=0; jj<NoC; jj++) {
    size_score += size_score_array[jj];
  }

  /* allocate GPU memory */
  res = cuMemAlloc(&ac_score_dev, size_A_SCORE);
  if(res != CUDA_SUCCESS) {
    printf("cuMemAlloc(ac_score) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemAlloc(&score_dev, size_score);
  if(res != CUDA_SUCCESS) {
    printf("cuMemAlloc(score) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemAlloc(&ssize_dev, NoC*sizeof(int));
  if(res != CUDA_SUCCESS) {
    printf("cuMemAlloc(ssize) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemAlloc(&size_score_dev, NoC*sizeof(int));
  if(res != CUDA_SUCCESS) {
    printf("cuMemAlloc(size_score) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemAlloc(&RY_dev, NoC*sizeof(int));
  if(res != CUDA_SUCCESS) {
    printf("cuMemAlloc(RY) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemAlloc(&RX_dev, NoC*sizeof(int));
  if(res != CUDA_SUCCESS) {
    printf("cuMemAlloc(RX) failed: res = %s\n", conv(res));
    exit(1);
  }

  gettimeofday(&tv_memcpy_start, NULL);
  /* upload date to GPU */
  res = cuMemcpyHtoD(ac_score_dev, ac_score, size_A_SCORE);
  if(res != CUDA_SUCCESS) {
    printf("cuMemcpyHtoD(ac_score) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemcpyHtoD(score_dev, score, size_score);
  if(res != CUDA_SUCCESS) {
    printf("cuMemcpyHtoD(score) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemcpyHtoD(ssize_dev, ssize_start, NoC*sizeof(int));
  if(res != CUDA_SUCCESS) {
    printf("cuMemcpyHtoD(ssize) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemcpyHtoD(size_score_dev, size_score_array, NoC*sizeof(int));
  if(res != CUDA_SUCCESS) {
    printf("cuMemcpyHtoD(size_score) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemcpyHtoD(RY_dev, RY_array, NoC*sizeof(int));
  if(res != CUDA_SUCCESS) {
    printf("cuMemcpyHtoD(RY) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemcpyHtoD(RX_dev, RX_array, NoC*sizeof(int));
  if(res != CUDA_SUCCESS) {
    printf("cuMemcpyHtoD(RX) failed: res = %s\n", conv(res));
    exit(1);
  }

  gettimeofday(&tv_memcpy_end, NULL);
  tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
  time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;


  void* kernel_args[] = {
    (void*)&IWID,
    (void*)&IHEI,
    (void*)&scale,
    (void*)&padx_n,
    (void*)&pady_n,
    &RX_dev,
    &RY_dev,
    &ac_score_dev,
    &score_dev,
    &ssize_dev,
    (void*)&NoC,
    &size_score_dev
  };




  int sharedMemBytes = 0;

  /* define CUDA block shape */
  int max_threads_num = 0;
  int thread_num_x, thread_num_y;
  int block_num_x, block_num_y;

  res = cuDeviceGetAttribute(&max_threads_num, CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, dev[0]);
  if(res != CUDA_SUCCESS){
    printf("\ncuDeviceGetAttribute() failed: res = %s\n", conv(res));
    exit(1);
  }
  
  NR_MAXTHREADS_X[0] = (int)sqrt((double)max_threads_num/NoC);
  NR_MAXTHREADS_Y[0] = (int)sqrt((double)max_threads_num/NoC);
  
  thread_num_x = (IWID < NR_MAXTHREADS_X[0]) ? IWID : NR_MAXTHREADS_X[0];
  thread_num_y = (IHEI < NR_MAXTHREADS_Y[0]) ? IHEI : NR_MAXTHREADS_Y[0];
  
  block_num_x = IWID / thread_num_x;
  block_num_y = IHEI / thread_num_y;
  if(IWID % thread_num_x != 0) block_num_x++;
  if(IHEI % thread_num_y != 0) block_num_y++;

  gettimeofday(&tv_kernel_start, NULL);
  /* launch GPU kernel */
  res = cuLaunchKernel(
                       func_calc_a_score[0], // call function
                       block_num_x,       // gridDimX
                       block_num_y,       // gridDimY
                       1,                 // gridDimZ
                       thread_num_x,      // blockDimX
                       thread_num_y,      // blockDimY
                       NoC,               // blockDimZ
                       sharedMemBytes,    // sharedMemBytes
                       NULL,              // hStream
                       kernel_args,       // kernelParams
                       NULL               // extra
                       );
  if(res != CUDA_SUCCESS) { 
    printf("cuLaunchKernel(calc_a_score) failed : res = %s\n", conv(res));
    exit(1);
  }

  res = cuCtxSynchronize();
  if(res != CUDA_SUCCESS) {
    printf("cuCtxSynchronize(calc_a_score) failed: res = %s\n", conv(res));
    exit(1);
  }
  gettimeofday(&tv_kernel_end, NULL);
  tvsub(&tv_kernel_end, &tv_kernel_start, &tv);
  time_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;


  gettimeofday(&tv_memcpy_start, NULL);
  /* download data from GPU */
  res = cuMemcpyDtoH(ac_score, ac_score_dev, size_A_SCORE);
  if(res != CUDA_SUCCESS) {
    printf("cuMemcpyDtoH(ac_score) failed: res = %s\n", conv(res));
    exit(1);
  }
  gettimeofday(&tv_memcpy_end, NULL);
  tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
  time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;


  /* free GPU memory */
  res = cuMemFree(ac_score_dev);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFree(ac_score_dev) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemFree(score_dev);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFree(score_dev) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemFree(ssize_dev);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFree(ssize_dev) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemFree(size_score_dev);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFree(size_score_dev) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemFree(RY_dev);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFree(RY_dev) failed: res = %s\n", conv(res));
    exit(1);
  }

  res = cuMemFree(RX_dev);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFree(RX_dev) failed: res = %s\n", conv(res));
    exit(1);
  }

  /* free CPU memory */
  res = cuMemFreeHost(RY_array);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFreeHost(RY_array) failed: res = %s\n", conv(res));
    exit(1);
  } 

  res = cuMemFreeHost(RX_array);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFreeHost(RX_array) failed: res = %s\n", conv(res));
    exit(1);
  } 

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//detect boundary box 
FLOAT *get_boxes(FLOAT **features,FLOAT *scales,int *FSIZE,MODEL *MO,int *Dnum,FLOAT *A_SCORE,FLOAT thresh)
{
  //constant parameters
  const int max_scale = MO->MI->max_scale;
  const int interval = MO->MI->interval;
  const int sbin = MO->MI->sbin;
  const int padx = MO->MI->padx;
  const int pady = MO->MI->pady;
  const int NoR = MO->RF->NoR;
  const int NoP = MO->PF->NoP;
  const int NoC = MO->MI->numcomponent;
  const int *numpart = MO->MI->numpart;
  const int LofFeat=(max_scale+interval)*NoC;
  const int L_MAX = max_scale+interval;

  /* for measurement */
  struct timeval tv;
  struct timeval tv_make_c_start, tv_make_c_end;
  struct timeval tv_nucom_start, tv_nucom_end;
  struct timeval tv_box_start, tv_box_end;
  float time_box=0;
  struct timeval tv_root_score_start, tv_root_score_end;
  float time_root_score = 0;
  struct timeval tv_part_score_start, tv_part_score_end;
  float time_part_score = 0;
  struct timeval tv_dt_start, tv_dt_end;
  float time_dt = 0;
  struct timeval tv_calc_a_score_start, tv_calc_a_score_end;
  float time_calc_a_score = 0;

  gettimeofday(&tv_make_c_start, NULL);

  int **RF_size = MO->RF->root_size;
  int *rootsym = MO->RF->rootsym;	
  int *part_sym = MO->PF->part_sym;	
  int **part_size = MO->PF->part_size;
  FLOAT **rootfilter = MO->RF->rootfilter;
  FLOAT **partfilter=MO->PF->partfilter;
  int **psize = MO->MI->psize;
  
  int **rm_size_array = (int **)malloc(sizeof(int *)*L_MAX);
  int **pm_size_array = (int **)malloc(sizeof(int *)*L_MAX);
  pm_size_array = (int **)malloc(sizeof(int *)*L_MAX);

  FLOAT **Tboxes=(FLOAT**)calloc(LofFeat,sizeof(FLOAT*)); //box coordinate information(Temp)
  int  *b_nums =(int*)calloc(LofFeat,sizeof(int)); //length of Tboxes 
  int count = 0;                                   
  int D_NUMS=0;                 //number of detected boundary box
  CUresult res;
  
  /* matched score (root and part) */
  FLOAT ***rootmatch,***partmatch = nullptr;

  int *new_PADsize;  // need new_PADsize[L_MAX*3]
  size_t SUM_SIZE_feat = 0;

  FLOAT **featp2 = (FLOAT **)malloc(L_MAX*sizeof(FLOAT *));


  if(featp2 == NULL) {  // error semantics
    printf("allocate featp2 failed\n");
    exit(1);
  }


  /* allocate required memory for new_PADsize */
  new_PADsize = (int *)malloc(L_MAX*3*sizeof(int));
  if(new_PADsize == NULL) {     // error semantics
    printf("allocate new_PADsize failed\n");
    exit(1);
  }
  
  
  /*******************************************************************/
  /* do padarray once and reuse it at calculating root and part time */

  /* calculate sum of size of padded feature */   
  for(int tmpL=0; tmpL<L_MAX; tmpL++) {
    
    int PADsize[3] = { FSIZE[tmpL*2], FSIZE[tmpL*2+1], 31 };
    int NEW_Y = PADsize[0] + pady*2;
    int NEW_X = PADsize[1] + padx*2;
    SUM_SIZE_feat += (NEW_X*NEW_Y*PADsize[2])*sizeof(FLOAT);

  }

  /* allocate region for padded feat in a lump */
  FLOAT *dst_feat;
  res = cuMemHostAlloc((void **)&dst_feat, SUM_SIZE_feat, CU_MEMHOSTALLOC_DEVICEMAP);
  if(res != CUDA_SUCCESS) {
    printf("cuMemHostAlloc(dst_feat) failed: res = %s\n", conv(res));
    exit(1);
  }

  memset(dst_feat, 0, SUM_SIZE_feat);  // zero clear

  /* distribute allocated region */
  unsigned long long int pointer_feat = (unsigned long long int)dst_feat;
  for(int tmpL=0; tmpL<L_MAX; tmpL++) {

    featp2[tmpL] = (FLOAT *)pointer_feat;
    int PADsize[3] = { FSIZE[tmpL*2], FSIZE[tmpL*2+1], 31 };
    int NEW_Y = PADsize[0] + pady*2;
    int NEW_X = PADsize[1] + padx*2;
    pointer_feat += (unsigned long long int)(NEW_X*NEW_Y*PADsize[2]*sizeof(FLOAT));

  }

  /* copy feat to feat2 */
  for(int tmpL=0; tmpL<L_MAX; tmpL++) {

    int PADsize[3] = { FSIZE[tmpL*2], FSIZE[tmpL*2+1], 31 };
    int NEW_Y = PADsize[0] + pady*2;
    int NEW_X = PADsize[1] + padx*2;
    int L = NEW_Y*padx;
    int SPL = PADsize[0] + pady;
    int M_S = sizeof(FLOAT)*PADsize[0];
    FLOAT *P = featp2[tmpL];
    FLOAT *S = features[tmpL];

    for(int ii=0; ii<PADsize[2]; ii++) 
      {
        P += L; 
        for(int jj=0; jj<PADsize[1]; jj++)
          {
            P += pady;
            memcpy(P, S, M_S);
            S += PADsize[0];
            P += SPL;
          }
        P += L;
      }

    new_PADsize[tmpL*3] = NEW_Y;
    new_PADsize[tmpL*3 + 1] = NEW_X;
    new_PADsize[tmpL*3 + 2] = PADsize[2];

  }

  /* do padarray once and reuse it at calculating root and part time */
  /*******************************************************************/





  /* allocation in a lump */
  int *dst_rm_size = (int *)malloc(sizeof(int)*NoC*2*L_MAX);
  if(dst_rm_size == NULL) {
    printf("allocate dst_rm_size failed\n");
    exit(1);
  }

  /* distribution to rm_size_array[L_MAX] */
  unsigned long long int ptr = (unsigned long long int)dst_rm_size;
  for(int i=0; i<L_MAX; i++) {
    rm_size_array[i] = (int *)ptr;
    ptr += (unsigned long long int)(NoC*2*sizeof(int));
  }

  /* allocation in a lump */
  int *dst_pm_size = (int *)malloc(sizeof(int)*NoP*2*L_MAX);  
  if(dst_pm_size == NULL) {
    printf("allocate dst_pm_size failed\n");
    exit(1);
  }

  /* distribution to pm_size_array[L_MAX] */
  ptr = (unsigned long long int)dst_pm_size;
  for(int i=0; i<L_MAX; i++) {
    pm_size_array[i] = (int *)ptr;
    ptr += (unsigned long long int)(NoP*2*sizeof(int));
  }

  
  ///////level
  
  
  for (int level=interval; level<L_MAX; level++)  // feature's loop(A's loop) 1level 1picture
    {
      /**************************************************************************/      
      /* loop conditon */
      if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
		{
          Tboxes[count]=NULL;
          count++;
          continue;
		}
      /* loop conditon */
      /**************************************************************************/      


    }  //for (level)  // feature's loop(A's loop) 1level 1picture




  ///////root calculation/////////
  /* calculate model score (only root) */


  gettimeofday(&tv_root_score_start, NULL);
  rootmatch = fconvsMT_GPU(
                           featp2,
                           SUM_SIZE_feat, 
                           rootfilter, 
                           rootsym, 
                           1, 
                           NoR, 
                           new_PADsize,  
                           RF_size, rm_size_array, 
                           L_MAX, 
                           interval, 
                           FSIZE, 
                           padx, 
                           pady, 
                           MO->MI->max_X, 
                           MO->MI->max_Y, 
                           ROOT
                           );   
  gettimeofday(&tv_root_score_end, NULL);
  tvsub(&tv_root_score_end, &tv_root_score_start, &tv);
  time_root_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
  
  

  ///////part calculation/////////
  if(NoP>0)
    {
      /* calculate model score (only part) */
      gettimeofday(&tv_part_score_start, NULL);
      partmatch = fconvsMT_GPU(
                               featp2,
                               SUM_SIZE_feat,
                               partfilter, 
                               part_sym, 
                               1, 
                               NoP, 
                               new_PADsize, 
                               part_size, 
                               pm_size_array, 
                               L_MAX, 
                               interval, 
                               FSIZE, 
                               padx, 
                               pady, 
                               MO->MI->max_X,
                               MO->MI->max_Y, 
                               PART
                               );
      gettimeofday(&tv_part_score_end, NULL);		
      tvsub(&tv_part_score_end, &tv_part_score_start, &tv);
      time_part_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
      
    }

     
  res = cuCtxSetCurrent(ctx[0]);
  if(res != CUDA_SUCCESS) {
    printf("cuCtxSetCurrent(ctx[0]) failed: res = %s\n", conv(res));
    exit(1);
  }

  gettimeofday(&tv_make_c_end, NULL);

  gettimeofday(&tv_nucom_start, NULL);
    
  count = 0;
  D_NUMS = 0;

  /////nucom loop start 

  // for (int level=interval; level<L_MAX; level++)  // feature's loop(A's loop) 1level 1picture
  //   {
  //     /* parameters (related for level) */
  //     int L=level-interval;
  //     /* matched score size matrix */
  //     FLOAT scale=(FLOAT)sbin/scales[level];
      
  //     /**************************************************************************/      
  //     /* loop conditon */
      
  //     if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
  //       {
  //         Tboxes[count]=NULL;
  //         count++;
  //         continue;
  //       }
      
  //     /* loop conditon */
  //     /**************************************************************************/
      
  int **RL_array = (int **)malloc((L_MAX-interval)*sizeof(int*));
  int *dst_RL = (int *) malloc(NoC*(L_MAX-interval)*sizeof(int));

  int **RI_array = (int **)malloc((L_MAX-interval)*sizeof(int*));
  int *dst_RI = (int *)malloc(NoC*(L_MAX-interval)*sizeof(int));

  int **OI_array = (int **)malloc((L_MAX-interval)*sizeof(int*));
  int *dst_OI = (int *)malloc((NoC)*(L_MAX-interval)*sizeof(int));

  int **RL_S_array = (int **)malloc((L_MAX-interval)*sizeof(int*));
  int *dst_RL_S = (int *)malloc(NoC*(L_MAX-interval)*sizeof(int));


  FLOAT **OFF_array = (FLOAT **)malloc((L_MAX-interval)*sizeof(FLOAT*));
  FLOAT *dst_OFF = (FLOAT *)malloc(NoC*(L_MAX-interval)*sizeof(FLOAT));

  FLOAT ***SCORE_array = (FLOAT ***)malloc((L_MAX-interval)*sizeof(FLOAT **));
  FLOAT **sub_dst_SCORE = (FLOAT **)malloc(NoC*(L_MAX-interval)*sizeof(FLOAT*));

  
  
  unsigned long long int pointer_RL = (unsigned long long int)dst_RL;
  unsigned long long int pointer_RI = (unsigned long long int)dst_RI;
  unsigned long long int pointer_OI = (unsigned long long int)dst_OI;
  unsigned long long int pointer_RL_S = (unsigned long long int)dst_RL_S;
  unsigned long long int pointer_OFF = (unsigned long long int)dst_OFF;
  unsigned long long int pointer_SCORE = (unsigned long long int)sub_dst_SCORE;
  for (int level=interval; level<L_MAX; level++) {
    
    int L=level-interval;        
    
    RL_array[L] = (int *)pointer_RL;
    pointer_RL += (unsigned long long int)NoC*sizeof(int);
    
    RI_array[L] = (int *)pointer_RI;
    pointer_RI += (unsigned long long int)NoC*sizeof(int);
    
    OI_array[L] = (int *)pointer_OI;
    pointer_OI += (unsigned long long int)NoC*sizeof(int);
    
    RL_S_array[L] = (int *)pointer_RL_S;
    pointer_RL_S += (unsigned long long int)NoC*sizeof(int);
    
    OFF_array[L] = (FLOAT *)pointer_OFF;
    pointer_OFF += (unsigned long long int)NoC*sizeof(FLOAT);

    SCORE_array[L] = (FLOAT **)pointer_SCORE;
    pointer_SCORE += (unsigned long long int)NoC*sizeof(FLOAT*);
    
  }




      int sum_RL_S = 0;
      int sum_SNJ = 0;
      /* prepare for parallel execution */
      for(int level=interval; level<L_MAX; level++) {
        int L = level - interval;
        /**************************************************************************/      
        /* loop conditon */
        
        if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
          {
            continue;
          }
        
        /* loop conditon */
        /**************************************************************************/
        
        for(int jj=0; jj<NoC; jj++) {
          
          /* root score + offset */
          RL_array[L][jj] = rm_size_array[level][jj*2]*rm_size_array[level][jj*2+1];  //length of root-matching 
          RI_array[L][jj] = MO->MI->ridx[jj];  //root-index
          OI_array[L][jj] =  MO->MI->oidx[jj];  //offset-index
          RL_S_array[L][jj] =sizeof(FLOAT)*RL_array[L][jj];

          
          OFF_array[L][jj] = MO->MI->offw[RI_array[L][jj]];  //offset information
          
          
          /* search max values */
          max_RL_S = (max_RL_S < RL_S_array[L][jj]) ? RL_S_array[L][jj] : max_RL_S;
          max_numpart = (max_numpart < numpart[jj]) ? numpart[jj] : max_numpart;
        }
      }

      sum_RL_S = max_RL_S*NoC*(L_MAX-interval);

      /* root matching size */
      sum_SNJ = sizeof(int*)*max_numpart*NoC*(L_MAX-interval);

      /* consolidated allocation for SCORE_array and distribute region */
      FLOAT *dst_SCORE = (FLOAT *)malloc(sum_RL_S);
      pointer_SCORE = (unsigned long long int)dst_SCORE;
      for(int level=interval; level<L_MAX; level++) {
        int L = level - interval;
        /**************************************************************************/      
        /* loop conditon */
        
        if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
          {
            continue;
          }
        
        /* loop conditon */
        /**************************************************************************/
        
        for(int jj=0; jj<NoC; jj++) {
          SCORE_array[L][jj] = (FLOAT *)pointer_SCORE;
          pointer_SCORE += (unsigned long long int)max_RL_S;
        }
      }

      /* add offset */
      for(int level=interval; level<L_MAX; level++) {
        int L = level - interval;
        /**************************************************************************/      
        /* loop conditon */
        
        if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
          {
            continue;
          }
        
        /* loop conditon */
        /**************************************************************************/
        
        for(int jj=0; jj<NoC; jj++) {
          memcpy(SCORE_array[L][jj], rootmatch[level][jj], RL_S_array[L][jj]);
          FLOAT *SC_S = SCORE_array[L][jj];
          FLOAT *SC_E = SCORE_array[L][jj]+RL_array[L][jj];
          while(SC_S<SC_E) *(SC_S++)+=OFF_array[L][jj];
        }
      }


      /* anchor matrix */  // consolidated allocation 
      int ***ax_array = (int ***)malloc((L_MAX-interval)*sizeof(int **));
      int **sub_dst_ax = (int **)malloc(NoC*(L_MAX-interval)*sizeof(int *));
      int *dst_ax = (int *)malloc(sum_SNJ);

      int ***ay_array = (int ***)malloc((L_MAX-interval)*sizeof(int **));
      int **sub_dst_ay = (int **)malloc(NoC*(L_MAX-interval)*sizeof(int *));
      int *dst_ay = (int *)malloc(sum_SNJ);

      /* boudary index */  // consolidated allocation
      int ****Ix_array =(int ****)malloc((L_MAX-interval)*sizeof(int ***));
      int ***sub_dst_Ix = (int ***)malloc(NoC*(L_MAX-interval)*sizeof(int **));
      int **dst_Ix = (int **)malloc(sum_SNJ);

      int ****Iy_array = (int ****)malloc((L_MAX-interval)*sizeof(int ***));
      int ***sub_dst_Iy = (int ***)malloc(NoC*(L_MAX-interval)*sizeof(int **));
      int **dst_Iy = (int **)malloc(sum_SNJ);

      /* distribute region */
      unsigned long long int pointer_ax = (unsigned long long int)sub_dst_ax;
      unsigned long long int pointer_ay = (unsigned long long int)sub_dst_ay;
      unsigned long long int pointer_Ix = (unsigned long long int)sub_dst_Ix;
      unsigned long long int pointer_Iy = (unsigned long long int)sub_dst_Iy;
      for(int level=interval; level<L_MAX; level++) {
        int L = level - interval;
        /**************************************************************************/      
        /* loop conditon */
        
        if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
          {
            continue;
          }
        
        /* loop conditon */
        /**************************************************************************/
        

        ax_array[L] = (int **)pointer_ax;
        pointer_ax += (unsigned long long int)(NoC*sizeof(int*));

        ay_array[L] = (int **)pointer_ay;
        pointer_ay += (unsigned long long int)(NoC*sizeof(int*));

        Ix_array[L] = (int ***)pointer_Ix;
        pointer_Ix += (unsigned long long int)(NoC*sizeof(int**));

        Iy_array[L] = (int ***)pointer_Iy;
        pointer_Iy += (unsigned long long int)(NoC*sizeof(int**));
      }


      pointer_ax = (unsigned long long int)dst_ax;
      pointer_ay = (unsigned long long int)dst_ay;
      pointer_Ix = (unsigned long long int)dst_Ix;
      pointer_Iy = (unsigned long long int)dst_Iy;
      for(int level=interval; level<L_MAX; level++) {
        int L = level - interval;
        /**************************************************************************/      
        /* loop conditon */
        
        if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
          {
            continue;
          }
        
        /* loop conditon */
        /**************************************************************************/


        for(int jj=0; jj<NoC; jj++) {
          unsigned long long int pointer_offset = sizeof(int*)*max_numpart;
          
          ax_array[L][jj] = (int *)pointer_ax;
          pointer_ax += pointer_offset;
          
          ay_array[L][jj] = (int *)pointer_ay;
          pointer_ay += pointer_offset;
          
          Ix_array[L][jj] = (int **)pointer_Ix;
          pointer_Ix += pointer_offset;
          
          Iy_array[L][jj] = (int **)pointer_Iy;
          pointer_Iy += pointer_offset;
        }
      }
      
      /* add parts */
      if(NoP>0)
        {
          /* arrays to store temporary loop variables */
          int tmp_array_size = 0;
          for(int level=interval; level<L_MAX; level++) {
            /**************************************************************************/      
            /* loop conditon */
            
            if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
              {
                continue;
              }
            
            /* loop conditon */
            /**************************************************************************/
            
            for(int jj=0; jj<NoC; jj++) {
              tmp_array_size += max_numpart*sizeof(int);
            }
          }

          int ***DIDX_array = (int ***)malloc((L_MAX-interval)*sizeof(int**));
          int **sub_dst_DIDX = (int **)malloc(NoC*(L_MAX-interval)*sizeof(int*));
          int *dst_DIDX = (int *)malloc(tmp_array_size);


          int ***DID_4_array = (int ***)malloc((L_MAX-interval)*sizeof(int **));
          int **sub_dst_DID_4 = (int **)malloc(NoC*(L_MAX-interval)*sizeof(int*));
          int *dst_DID_4;
          res = cuMemHostAlloc((void **)&dst_DID_4, tmp_array_size, CU_MEMHOSTALLOC_DEVICEMAP);
          if(res != CUDA_SUCCESS) {
            printf("cuMemHostAlloc(dst_DID_4) failed: res = %s\n", conv(res));
            exit(1);
          }
          
          
          int ***PIDX_array = (int ***)malloc((L_MAX-interval)*sizeof(int **));
          int **sub_dst_PIDX = (int **)malloc(NoC*(L_MAX-interval)*sizeof(int*));
          int *dst_PIDX;
          res = cuMemHostAlloc((void **)&dst_PIDX, tmp_array_size, CU_MEMHOSTALLOC_DEVICEMAP);
          if(res != CUDA_SUCCESS) {
            printf("cuMemHostAlloc(dst_PIDX) failed: res = %s\n", conv(res));
            exit(1);
          }

          /* distribute consolidated region */          
          unsigned long long int pointer_DIDX = (unsigned long long int)sub_dst_DIDX;
          unsigned long long int pointer_DID_4 = (unsigned long long int)sub_dst_DID_4;
          unsigned long long int pointer_PIDX = (unsigned long long int)sub_dst_PIDX;
          for(int level=interval; level<L_MAX; level++) {
            int L = level - interval;

            /**************************************************************************/      
            /* loop conditon */
            
            if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
              {
                continue;
              }
            
            /* loop conditon */
            /**************************************************************************/


            DIDX_array[L] = (int **)pointer_DIDX;
            pointer_DIDX += (unsigned long long int)(NoC*sizeof(int*));

            DID_4_array[L] = (int **)pointer_DID_4;
            pointer_DID_4 += (unsigned long long int)(NoC*sizeof(int*));

            PIDX_array[L] = (int **)pointer_PIDX;
            pointer_PIDX += (unsigned long long int)(NoC*sizeof(int*));
          }

          pointer_DIDX = (unsigned long long int)dst_DIDX;
          pointer_DID_4 = (unsigned long long int)dst_DID_4;
          pointer_PIDX = (unsigned long long int)dst_PIDX;
          for(int level=interval; level<L_MAX; level++) {
            int L = level - interval;
            /**************************************************************************/      
            /* loop conditon */
            
            if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
              {
                continue;
              }
            
            /* loop conditon */
            /**************************************************************************/
            
            
            for(int jj=0; jj<NoC; jj++) {
              unsigned long long int pointer_offset = (unsigned long long int)(max_numpart*sizeof(int));
              
              DIDX_array[L][jj] = (int *)pointer_DIDX;
              pointer_DIDX += pointer_offset;
              
              DID_4_array[L][jj] = (int *)pointer_DID_4;
              pointer_DID_4 += pointer_offset;
              
              PIDX_array[L][jj] = (int *)pointer_PIDX;
              pointer_PIDX += pointer_offset;
            }
          }

          
          /* prepare for parallel execution */
          int sum_size_index_matrix = 0;
          for(int level=interval; level<L_MAX; level++) {          
            int L = level - interval;
            /**************************************************************************/      
            /* loop conditon */
            
            if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
              {
                continue;
              }
            
            /* loop conditon */
            /**************************************************************************/
            
            
            for(int jj=0; jj<NoC; jj++) {
              for (int kk=0;kk<numpart[jj];kk++)
                {
                  /* assign values to each element */
                  DIDX_array[L][jj][kk] = MO->MI->didx[jj][kk];
                  DID_4_array[L][jj][kk] = DIDX_array[L][jj][kk]*4;
                  PIDX_array[L][jj][kk] = MO->MI->pidx[jj][kk];
                  
                  /* anchor */
                  ax_array[L][jj][kk] = MO->MI->anchor[DIDX_array[L][jj][kk]*2]+1;
                  ay_array[L][jj][kk] = MO->MI->anchor[DIDX_array[L][jj][kk]*2+1]+1;

                  int PSSIZE[2] ={pm_size_array[L][PIDX_array[L][jj][kk]*2], pm_size_array[L][PIDX_array[L][jj][kk]*2+1]}; // Cのサイズ
                  
                  
                  /* index matrix */
                  sum_size_index_matrix += sizeof(int)*PSSIZE[0]*PSSIZE[1];
                  
                }
            }
          }
          
          int *dst_Ix_kk = (int *)malloc(sum_size_index_matrix);
          int *dst_Iy_kk = (int *)malloc(sum_size_index_matrix);
          unsigned long long int pointer_Ix_kk = (unsigned long long int)dst_Ix_kk;
          unsigned long long int pointer_Iy_kk = (unsigned long long int)dst_Iy_kk;
          for(int level=interval; level<L_MAX; level++) {
            int L = level - interval;

            /**************************************************************************/      
            /* loop conditon */
            
            if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
              {
                continue;
              }
      
            /* loop conditon */
            /**************************************************************************/

            for(int jj=0; jj<NoC; jj++) {
              for (int kk=0;kk<numpart[jj];kk++)
                {
                  int PSSIZE[2] ={pm_size_array[L][PIDX_array[L][jj][kk]*2], pm_size_array[L][PIDX_array[L][jj][kk]*2+1]}; // Cのサイズ
                  
                  Ix_array[L][jj][kk] = (int *)pointer_Ix_kk;
                  Iy_array[L][jj][kk] = (int *)pointer_Iy_kk;
                  
                  pointer_Ix_kk += (unsigned long long int)(sizeof(int)*PSSIZE[0]*PSSIZE[1]);
                  pointer_Iy_kk += (unsigned long long int)(sizeof(int)*PSSIZE[0]*PSSIZE[1]);
                }
            }
            
          }


       
          gettimeofday(&tv_dt_start, NULL);
          FLOAT ****M_array = dt_GPU(                     
                                    Ix_array,      // int ****Ix_array
                                    Iy_array,      // int ****Iy_array
                                    PIDX_array,    // int ***PIDX_array
                                    pm_size_array, // int **size_array
                                    NoP,           // int NoP
                                    numpart,       // int *numpart
                                    NoC,           // int NoC
                                    interval,      // int interval
                                    L_MAX,         // int L_MAX
                                    FSIZE,         // int *FSIZE,
                                    padx,          // int padx,
                                    pady,          // int pady,
                                    MO->MI->max_X, // int max_X
                                    MO->MI->max_Y, // int max_Y
                                    MO->MI->def, // FLOAT *def
                                    tmp_array_size, // int tmp_array_size
                                    dst_PIDX, // int *dst_PIDX
                                    dst_DID_4 // int *DID_4
                                                         );
          gettimeofday(&tv_dt_end, NULL);
          tvsub(&tv_dt_end, &tv_dt_start, &tv);
          time_dt += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
         

          /* add part score */
          for(int level=interval; level<L_MAX; level++){
            int L = level - interval;          
            /**************************************************************************/      
            /* loop conditon */
            
            if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
              {
                continue;
              }
            
            /* loop conditon */
            /**************************************************************************/
          
  
            for(int jj=0; jj<NoC; jj++) {
              for(int kk=0; kk<numpart[jj]; kk++) {
                int PSSIZE[2] ={pm_size_array[L][PIDX_array[L][jj][kk]*2], pm_size_array[L][PIDX_array[L][jj][kk]*2+1]}; // Cのサイズ
                int R_S[2]={rm_size_array[level][jj*2], rm_size_array[level][jj*2+1]};
                
                add_part_calculation(SCORE_array[L][jj], M_array[L][jj][kk], R_S, PSSIZE, ax_array[L][jj][kk], ay_array[L][jj][kk]); 
                
              }
            }
          }
          
          s_free(M_array[0][0][0]);
          s_free(M_array[0][0]);
          s_free(M_array[0]);
          s_free(M_array);
          
          
          /* free temporary arrays */
          free(dst_DIDX);
          free(sub_dst_DIDX);
          free(DIDX_array);
          
          
          res = cuMemFreeHost(dst_DID_4);
          if(res != CUDA_SUCCESS) {
            printf("cuMemFreeHost(dst_DID_4) failed: res = %s\n", conv(res));
            exit(1);
          }
          free(sub_dst_DID_4);
          free(DID_4_array);

          
          res = cuMemFreeHost(dst_PIDX);
          if(res != CUDA_SUCCESS) {
            printf("cuMemFreeHost(dst_PIDX) failed: res = %s\n", conv(res));
            exit(1);
          }

          free(sub_dst_PIDX);
          free(PIDX_array);

          
 
          res = cuCtxSetCurrent(ctx[0]);
          if(res != CUDA_SUCCESS) {
            printf("cuCtxSetCurrent(ctx[0]) failed: res = %s\n", conv(res));
            exit(1);
           }

          
        } // start from if(NoP>0)
      
      

      ////nucom
      /* combine root and part score and detect boundary box for each-component */
      
      FLOAT *scale_array = (FLOAT *)malloc((L_MAX-interval)*sizeof(FLOAT));
      for(int level=interval; level<L_MAX; level++) {
        int L = level - interval;
        /**************************************************************************/      
        /* loop conditon */
        
        if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
          {
            Tboxes[count]=NULL;
            count++;
            continue;
          }
        
        /* loop conditon */
        /**************************************************************************/
        
        scale_array[L] = (FLOAT)sbin/scales[level];

      }
          

      for (int level=interval; level<L_MAX; level++)  // feature's loop(A's loop) 1level 1picture
        {
          /* parameters (related for level) */
          int L=level-interval;
          /* matched score size matrix */
          FLOAT scale=(FLOAT)sbin/scales[level];
          
          /**************************************************************************/      
          /* loop conditon */
          
          if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
            {
              Tboxes[count]=NULL;
              count++;
              continue;
            }
          
          /* loop conditon */
          /**************************************************************************/
          
          
          
          
          /* calculate accumulated score */
          gettimeofday(&tv_calc_a_score_start, NULL);
          
          calc_a_score_GPU(
                           A_SCORE,              // FLOAT *ac_score
                           SCORE_array[L],       // FLOAT **score
                           rm_size_array[level], // int *ssize_start
                           MO->MI,               // Model_info *MI
                           scale,                // FLOAT scale
                           RL_S_array[L],        // int *size_score_array
                           NoC                   // int NoC
                           );
          
          gettimeofday(&tv_calc_a_score_end, NULL);
          tvsub(&tv_calc_a_score_end, &tv_calc_a_score_start, &tv);
          time_calc_a_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
          
          
          for(int jj=0;jj<NoC;jj++)
            {
              
              int R_S[2]={rm_size_array[level][jj*2], rm_size_array[level][jj*2+1]};
              
              /* get all good matches */
              int GMN;
              int *GMPC = get_gmpc(SCORE_array[L][jj],thresh,R_S,&GMN);
              int RSIZE[2]={MO->MI->rsize[jj*2], MO->MI->rsize[jj*2+1]};
              
              int GL = (numpart[jj]+1)*4+3;  //31


              /* detected box coordinate(current level) */
              FLOAT *t_boxes = (FLOAT*)calloc(GMN*GL,sizeof(FLOAT));
              
              gettimeofday(&tv_box_start, NULL);
              
              ////////////////////////////////////////////////////////////////////////////////////
              ////////////////////////////////////////////////////////////////////////////////////
              ///////////////////////////// NO NEED TO USE GPU ///////////////////////////////////
              ////////////////////////////////////////////////////////////////////////////////////
              for(int kk=0;kk<GMN;kk++)
                {
                  FLOAT *P_temp = t_boxes+GL*kk;
                  int y = GMPC[2*kk];
                  int x = GMPC[2*kk+1];
                  
                  /* calculate root box coordinate */
                  FLOAT *RB =rootbox(x,y,scale,padx,pady,RSIZE);
                  memcpy(P_temp, RB,sizeof(FLOAT)*4);
                  s_free(RB);
                  P_temp+=4;
                  
                  for(int pp=0;pp<numpart[jj];pp++)
                    {
                      int PBSIZE[2]={psize[jj][pp*2], psize[jj][pp*2+1]};
                      int Isize[2]={pm_size_array[L][MO->MI->pidx[jj][pp]*2], pm_size_array[L][MO->MI->pidx[jj][pp]*2+1]};
                      
                      /* calculate part box coordinate */
                      FLOAT *PB = partbox(x,y,ax_array[L][jj][pp],ay_array[L][jj][pp],scale,padx,pady,PBSIZE,Ix_array[L][jj][pp],Iy_array[L][jj][pp],Isize);
                      memcpy(P_temp, PB,sizeof(FLOAT)*4);
                      P_temp+=4;
                      s_free(PB);
                    }
                  /* component number and score */
                  *(P_temp++)=(FLOAT)jj; //component number 
                  *(P_temp++)=SCORE_array[L][jj][x*R_S[0]+y]; //score of good match
                  *P_temp = scale;
                }
              ////////////////////////////////////////////////////////////////////////////////////
              ///////////////////////////// NO NEED TO USE GPU ///////////////////////////////////
              ////////////////////////////////////////////////////////////////////////////////////
              ////////////////////////////////////////////////////////////////////////////////////
              
              gettimeofday(&tv_box_end, NULL);
              tvsub(&tv_box_end, &tv_box_start, &tv);
              time_box += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
              
              
              
              
              
              /* save box information */
              if(GMN>0) Tboxes[count]=t_boxes;
              else Tboxes[count]=NULL;
              b_nums[count]=GMN;
              count++;
              D_NUMS+=GMN;			//number of detected box 
              
              /* release */
              s_free(GMPC);
              
            }
          ////numcom
         
        }
      ////level

          /* free temporary arrays */
          free(dst_RL);
          free(RL_array);
          
          free(dst_RI);
          free(RI_array);
          
          free(dst_OI);
          free(OI_array);
          
          free(dst_RL_S);
          free(RL_S_array);
          
          free(dst_OFF);
          free(OFF_array);
          
          free(dst_SCORE);
          free(sub_dst_SCORE);
          free(SCORE_array);
          
          free(dst_ax);
          free(sub_dst_ax);
          free(ax_array);
          
          free(dst_ay);
          free(sub_dst_ay);
          free(ay_array);
          
          free(Ix_array[0][0][0]);
          free(dst_Ix);
          free(sub_dst_Ix);
          free(Ix_array);
          
          free(Iy_array[0][0][0]);
          free(dst_Iy);
          free(sub_dst_Iy);
          free(Iy_array);


          free(scale_array);
 
 



  gettimeofday(&tv_nucom_end, NULL);

#if 1
  // tvsub(&tv_make_c_end, &tv_make_c_start, &tv);
  // make_c = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  // tvsub(&tv_nucom_end, &tv_nucom_start, &tv);
  // nucom = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  // printf("\n============================================\n");
  // printf("-- printed by get_boxex.cpp ----------------\n");
  // printf("make_c     : %f\n", make_c);
  // printf("nucom      : %f\n", nucom);
  // printf("----- breakdown of nucom -----\n");
  // printf("time_dt    : %f\n", time_dt);
  // printf("calc_a_score : %f\n", time_calc_score);
  // //printf("time box   : %f\n", time_box);
  // printf("------------------------------\n");
  // printf("============================================\n");
  // printf("\n");
  
#ifdef PRINT_INFO
  printf("root SCORE : %f\n", time_root_score);
  printf("part SCORE : %f\n", time_part_score);
  printf("dt  : %f\n", time_dt);
  printf("calc_a_score : %f\n", time_calc_a_score);
#endif

#endif
  



  res = cuCtxSetCurrent(ctx[0]);
  if(res != CUDA_SUCCESS) {
    printf("cuCtxSetCurrent(ctx[0]) failed: res = %s\n",conv(res));
    exit(1);
  }

  /* free memory regions */
  res = cuMemFreeHost((void *)featp2[0]);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFreeHost(featp2[0]) failed: res = %s\n", conv(res));
    exit(1);
  }
  
  s_free(featp2);
  
  
  res = cuMemFreeHost((void *)rootmatch[interval][0]);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFreeHost(rootmatch[0][0]) failed: res = %s\n", conv(res));
    exit(1);
  }
  s_free(rootmatch[0]);
  s_free(rootmatch);
  
  if (partmatch != nullptr) {
    res = cuMemFreeHost((void *)partmatch[0][0]);
    if(res != CUDA_SUCCESS) {
      printf("cuMemFreeHost(partmatch[0][0]) failed: res = %s\n", conv(res));
      exit(1);
    }
    
    s_free(partmatch[0]);
    s_free(partmatch);
    
    s_free(new_PADsize);
  }
  
  /* release */
  s_free(rm_size_array[0]);
  s_free(rm_size_array);
  s_free(pm_size_array[0]);
  s_free(pm_size_array);


  /* Output boundary-box coorinate information */
  int GL=(numpart[0]+1)*4+3;
  FLOAT *boxes=(FLOAT*)calloc(D_NUMS*GL,sizeof(FLOAT));		//box coordinate information(Temp)
  FLOAT *T1=boxes;
  
  for(int ii=0;ii<LofFeat;ii++)
    {
      int num_t = b_nums[ii]*GL;
      FLOAT *T2 = Tboxes[ii];
      if(num_t>0)
        {
          //memcpy_s(T1,sizeof(FLOAT)*num_t,T2,sizeof(FLOAT)*num_t);
          memcpy(T1, T2,sizeof(FLOAT)*num_t);
          T1+=num_t;
        }
    }
  
  
  FLOAT AS_OFF = abs(thresh);
  
  /////////////////////////////////
  /* accumulated score calculation */
  FLOAT max_ac = 0.0;
  
  
  /* add offset to accumulated score */
  for(int ii=0;ii<MO->MI->IM_HEIGHT*MO->MI->IM_WIDTH;ii++)
	{
      if(A_SCORE[ii]<thresh) A_SCORE[ii]=0.0;
      else
		{
          A_SCORE[ii]+=AS_OFF;
          if(A_SCORE[ii]>max_ac) max_ac=A_SCORE[ii];
		}
	}
  /* normalization */
  if(max_ac>0.0)
	{
      FLOAT ac_ratio = 1.0/max_ac;
      for(int ii=0;ii<MO->MI->IM_HEIGHT*MO->MI->IM_WIDTH;ii++){A_SCORE[ii]*=ac_ratio;}
	}
  
  /* release */
  free_boxes(Tboxes,LofFeat);
  s_free(b_nums);
  
  /* output result */
  *Dnum=D_NUMS;

  return(boxes);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////release functions //////

//free root-matching result 

void free_rootmatch(FLOAT **rootmatch, MODEL *MO)
{
  CUresult res;


  if(rootmatch!=NULL)
	{
      for(int ii=0;ii<MO->RF->NoR;ii++)
		{	
#ifdef ORIGINAL
          s_free(rootmatch[ii]);
#else
          res = cuMemFreeHost((void *)rootmatch[ii]);
          if(res != CUDA_SUCCESS){
            printf("cuMemFreeHost(rootmatch) failed: res = %s\n", conv(res));
            exit(1);
          }
#endif
		}
      s_free(rootmatch);
	}
  

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//free part-matching result 

void free_partmatch(FLOAT **partmatch, MODEL *MO)
{
  CUresult res;

  if(partmatch!=NULL)
	{
      for(int ii=0;ii<MO->PF->NoP;ii++)
		{
#ifdef ORIGINAL
          s_free(partmatch[ii]);
#else
          res = cuMemFreeHost((void *)partmatch[ii]);
          if(res != CUDA_SUCCESS){
            printf("cuMemFreeHost(partmatch) failed: res = %s\n", conv(res));
            exit(1);
          }
#endif
		}
      s_free(partmatch);
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//free detected boxes result 

void free_boxes(FLOAT **boxes, int LofFeat)
{
	if(boxes!=NULL)
	{
		for(int ii=0;ii<LofFeat;ii++)
		{
			s_free(boxes[ii]);
		}
		s_free(boxes);
	}
}
