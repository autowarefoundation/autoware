/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////get_boxes.cpp  detect boundary-boxes-coordinate of oject /////////////////////////////////////////////////////

#include <algorithm>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "MODEL_info.h"		//File information
#include "common.hpp"

#include "for_use_GPU.h"
#include "switch_float.h"
#include "switch_release.h"
#include "dt.hpp"
#include "dt_GPU.hpp"
#include "detect.hpp"
#include "fconvsMT.hpp"

#include <cuda_util.hpp>

int max_numpart = 0;
static int max_RL_S = 0;

//definiton of functions//

//get good-matched pixel-coordinate
static int *get_gmpc(FLOAT *score,FLOAT thresh,int *ssize,int *good_matched)
{
	const int limit = ssize[0]*ssize[1];
	int matches = 0;

	std::vector<int> indices;
	for(int i = 0; i < limit; i++) {
		if(score[i] > thresh) {
			indices.push_back(i);
			matches++;
		}
	}

	int *result = (int*)malloc(matches * 2 *sizeof(int));
	int i = 0;
	for (const int index : indices) {
		result[i*2] = index % ssize[0];
		result[i*2+1] = index / ssize[0];
		++i;
	}

	*good_matched = matches;
	return result;
}

//get root-box pixel coordinate
static FLOAT *rootbox(int x, int y, FLOAT scale, int pad_x, int pad_y, int *root_size)
{
	FLOAT *Out=(FLOAT*)malloc(sizeof(FLOAT)*4);

	Out[0]=((FLOAT)y-(FLOAT)pad_y+1)*scale;	//Y1
	Out[1]=((FLOAT)x-(FLOAT)pad_x+1)*scale;	//X1
	Out[2]=Out[0]+(FLOAT)root_size[0]*scale-1.0;	//Y2
	Out[3]=Out[1]+(FLOAT)root_size[1]*scale-1.0;	//X2

	return(Out);
}

//get part-box pixel coordinate
static FLOAT *partbox(int x,int y,int ax,int ay,FLOAT scale,int padx,int pady,int *psize,int *lx,int *ly,int *ssize)
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

static void calc_a_score_GPU(FLOAT *ac_score,  FLOAT **score,
			     int *ssize_start,  Model_info *MI,
			     FLOAT scale, int *size_score_array,
			     int NoC)
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
		printf("cuMemHostAlloc(RY_array) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemHostAlloc((void**)&RX_array, NoC*sizeof(int), CU_MEMHOSTALLOC_DEVICEMAP);
	if(res != CUDA_SUCCESS) {
		printf("cuMemHostAlloc(RX_array) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	for(int i = 0; i < NoC; i++) {
		int rsize[2] = {MI->rsize[i*2], MI->rsize[i*2+1]};

		RY_array[i] = (int)((FLOAT)rsize[0]*scale/2.0-1.0+block_pad);
		RX_array[i] = (int)((FLOAT)rsize[1]*scale/2.0-1.0+block_pad);
	}

	CUdeviceptr ac_score_dev, score_dev;
	CUdeviceptr ssize_dev, size_score_dev;
	CUdeviceptr RY_dev, RX_dev;

	int size_score=0;
	for(int i = 0; i < NoC; i++) {
		size_score += size_score_array[i];
	}

	/* allocate GPU memory */
	res = cuMemAlloc(&ac_score_dev, gpu_size_A_SCORE);
	if(res != CUDA_SUCCESS) {
		printf("cuMemAlloc(ac_score) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemAlloc(&score_dev, size_score);
	if(res != CUDA_SUCCESS) {
		printf("cuMemAlloc(score) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemAlloc(&ssize_dev, NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemAlloc(ssize) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemAlloc(&size_score_dev, NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemAlloc(size_score) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemAlloc(&RY_dev, NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemAlloc(RY) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemAlloc(&RX_dev, NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemAlloc(RX) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	gettimeofday(&tv_memcpy_start, nullptr);
	/* upload date to GPU */
	res = cuMemcpyHtoD(ac_score_dev, &ac_score[0], gpu_size_A_SCORE);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(ac_score) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(score_dev, &score[0][0], size_score);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(score) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(ssize_dev, &ssize_start[0], NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(ssize) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(size_score_dev, &size_score_array[0], NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(size_score) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(RY_dev, &RY_array[0], NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(RY) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(RX_dev, &RX_array[0], NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(RX) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	gettimeofday(&tv_memcpy_end, nullptr);
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
		printf("\ncuDeviceGetAttribute() failed: res = %s\n", cuda_response_to_string(res));
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

	gettimeofday(&tv_kernel_start, nullptr);
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
		nullptr,              // hStream
		kernel_args,       // kernelParams
		nullptr               // extra
		);
	if(res != CUDA_SUCCESS) {
		printf("cuLaunchKernel(calc_a_score) failed : res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuCtxSynchronize();
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSynchronize(calc_a_score) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}
	gettimeofday(&tv_kernel_end, nullptr);
	tvsub(&tv_kernel_end, &tv_kernel_start, &tv);
	time_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

	gettimeofday(&tv_memcpy_start, nullptr);
	/* download data from GPU */
	res = cuMemcpyDtoH(ac_score, ac_score_dev, gpu_size_A_SCORE);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyDtoH(ac_score) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	gettimeofday(&tv_memcpy_end, nullptr);
	tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
	time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

	/* free GPU memory */
	res = cuMemFree(ac_score_dev);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFree(ac_score_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemFree(score_dev);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFree(score_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemFree(ssize_dev);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFree(ssize_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemFree(size_score_dev);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFree(size_score_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemFree(RY_dev);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFree(RY_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemFree(RX_dev);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFree(RX_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	/* free CPU memory */
	res = cuMemFreeHost(RY_array);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFreeHost(RY_array) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemFreeHost(RX_array);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFreeHost(RX_array) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}
}

//free detected boxes result
static void free_boxes(FLOAT **boxes, int LofFeat)
{
	if (boxes == nullptr)
		return;

	for (int i = 0; i < LofFeat; i++) {
		s_free(boxes[i]);
	}

	s_free(boxes);
}

//detect boundary box
FLOAT *dpm_ttic_gpu_get_boxes(FLOAT **features,FLOAT *scales,int *feature_size, GPUModel *MO,
			      int *detected_count, FLOAT *acc_score, FLOAT thresh)
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

	gettimeofday(&tv_make_c_start, nullptr);

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
	int detected_boxes=0;
	CUresult res;

	/* matched score (root and part) */
	FLOAT ***rootmatch,***partmatch = nullptr;

	int *new_PADsize;  // need new_PADsize[L_MAX*3]
	size_t SUM_SIZE_feat = 0;

	FLOAT **featp2 = (FLOAT **)malloc(L_MAX*sizeof(FLOAT *));


	if(featp2 == nullptr) {  // error semantics
		printf("allocate featp2 failed\n");
		exit(1);
	}


	/* allocate required memory for new_PADsize */
	new_PADsize = (int *)malloc(L_MAX*3*sizeof(int));
	if(new_PADsize == nullptr) {     // error semantics
		printf("allocate new_PADsize failed\n");
		exit(1);
	}

	/* do padarray once and reuse it at calculating root and part time */

	/* calculate sum of size of padded feature */
	for(int tmpL=0; tmpL<L_MAX; tmpL++) {
		int PADsize[3] = { feature_size[tmpL*2], feature_size[tmpL*2+1], 31 };
		int NEW_Y = PADsize[0] + pady*2;
		int NEW_X = PADsize[1] + padx*2;
		SUM_SIZE_feat += (NEW_X*NEW_Y*PADsize[2])*sizeof(FLOAT);
	}

	/* allocate region for padded feat in a lump */
	FLOAT *dst_feat;
	res = cuMemHostAlloc((void **)&dst_feat, SUM_SIZE_feat, CU_MEMHOSTALLOC_DEVICEMAP);
	if(res != CUDA_SUCCESS) {
		printf("cuMemHostAlloc(dst_feat) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	memset(dst_feat, 0, SUM_SIZE_feat);  // zero clear

	/* distribute allocated region */
	uintptr_t pointer_feat = (uintptr_t)dst_feat;
	for(int tmpL=0; tmpL<L_MAX; tmpL++) {

		featp2[tmpL] = (FLOAT *)pointer_feat;
		int PADsize[3] = { feature_size[tmpL*2], feature_size[tmpL*2+1], 31 };
		int NEW_Y = PADsize[0] + pady*2;
		int NEW_X = PADsize[1] + padx*2;
		pointer_feat += (uintptr_t)(NEW_X*NEW_Y*PADsize[2]*sizeof(FLOAT));

	}

	/* copy feat to feat2 */
	for(int tmpL=0; tmpL<L_MAX; tmpL++) {

		int PADsize[3] = { feature_size[tmpL*2], feature_size[tmpL*2+1], 31 };
		int NEW_Y = PADsize[0] + pady*2;
		int NEW_X = PADsize[1] + padx*2;
		int L = NEW_Y*padx;
		int SPL = PADsize[0] + pady;
		int M_S = sizeof(FLOAT)*PADsize[0];
		FLOAT *P = featp2[tmpL];
		FLOAT *S = features[tmpL];

		for(int i=0; i<PADsize[2]; i++)
		{
			P += L;
			for(int j=0; j<PADsize[1]; j++)
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

	/* allocation in a lump */
	int *dst_rm_size = (int *)malloc(sizeof(int)*NoC*2*L_MAX);
	if(dst_rm_size == nullptr) {
		printf("allocate dst_rm_size failed\n");
		exit(1);
	}

	/* distribution to rm_size_array[L_MAX] */
	uintptr_t ptr = (uintptr_t)dst_rm_size;
	for(int i=0; i<L_MAX; i++) {
		rm_size_array[i] = (int *)ptr;
		ptr += (uintptr_t)(NoC*2*sizeof(int));
	}

	/* allocation in a lump */
	int *dst_pm_size = (int *)malloc(sizeof(int)*NoP*2*L_MAX);
	if(dst_pm_size == nullptr) {
		printf("allocate dst_pm_size failed\n");
		exit(1);
	}

	/* distribution to pm_size_array[L_MAX] */
	ptr = (uintptr_t)dst_pm_size;
	for(int i=0; i<L_MAX; i++) {
		pm_size_array[i] = (int *)ptr;
		ptr += (uintptr_t)(NoP*2*sizeof(int));
	}


	///////level
	for (int level=interval; level<L_MAX; level++)  // feature's loop(A's loop) 1level 1picture
	{
		if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
		{
			Tboxes[count]=nullptr;
			count++;
			continue;
		}
	}  //for (level)  // feature's loop(A's loop) 1level 1picture

	///////root calculation/////////
	/* calculate model score (only root) */

	gettimeofday(&tv_root_score_start, nullptr);
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
		feature_size,
		padx,
		pady,
		MO->MI->max_X,
		MO->MI->max_Y,
		ROOT
		);
	gettimeofday(&tv_root_score_end, nullptr);
	tvsub(&tv_root_score_end, &tv_root_score_start, &tv);
	time_root_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

	///////part calculation/////////
	if(NoP>0)
	{
		/* calculate model score (only part) */
		gettimeofday(&tv_part_score_start, nullptr);
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
			feature_size,
			padx,
			pady,
			MO->MI->max_X,
			MO->MI->max_Y,
			PART
			);
		gettimeofday(&tv_part_score_end, nullptr);
		tvsub(&tv_part_score_end, &tv_part_score_start, &tv);
		time_part_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

	}

	res = cuCtxSetCurrent(ctx[0]);
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSetCurrent(ctx[0]) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	gettimeofday(&tv_make_c_end, nullptr);
	gettimeofday(&tv_nucom_start, nullptr);

	count = 0;
	detected_boxes = 0;

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

	uintptr_t pointer_RL = (uintptr_t)dst_RL;
	uintptr_t pointer_RI = (uintptr_t)dst_RI;
	uintptr_t pointer_OI = (uintptr_t)dst_OI;
	uintptr_t pointer_RL_S = (uintptr_t)dst_RL_S;
	uintptr_t pointer_OFF = (uintptr_t)dst_OFF;
	uintptr_t pointer_SCORE = (uintptr_t)sub_dst_SCORE;
	for (int level=interval; level<L_MAX; level++) {

		int L=level-interval;

		RL_array[L] = (int *)pointer_RL;
		pointer_RL += (uintptr_t)NoC*sizeof(int);

		RI_array[L] = (int *)pointer_RI;
		pointer_RI += (uintptr_t)NoC*sizeof(int);

		OI_array[L] = (int *)pointer_OI;
		pointer_OI += (uintptr_t)NoC*sizeof(int);

		RL_S_array[L] = (int *)pointer_RL_S;
		pointer_RL_S += (uintptr_t)NoC*sizeof(int);

		OFF_array[L] = (FLOAT *)pointer_OFF;
		pointer_OFF += (uintptr_t)NoC*sizeof(FLOAT);

		SCORE_array[L] = (FLOAT **)pointer_SCORE;
		pointer_SCORE += (uintptr_t)NoC*sizeof(FLOAT*);
	}

	int sum_RL_S = 0;
	int sum_SNJ = 0;
	/* prepare for parallel execution */
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
		{
			continue;
		}

		for(int j=0; j<NoC; j++) {

			/* root score + offset */
			RL_array[L][j] = rm_size_array[level][j*2]*rm_size_array[level][j*2+1];  //length of root-matching
			RI_array[L][j] = MO->MI->ridx[j];  //root-index
			OI_array[L][j] =  MO->MI->oidx[j];  //offset-index
			RL_S_array[L][j] =sizeof(FLOAT)*RL_array[L][j];


			OFF_array[L][j] = MO->MI->offw[RI_array[L][j]];  //offset information


			/* search max values */
			max_RL_S = (max_RL_S < RL_S_array[L][j]) ? RL_S_array[L][j] : max_RL_S;
			max_numpart = (max_numpart < numpart[j]) ? numpart[j] : max_numpart;
		}
	}

	sum_RL_S = max_RL_S*NoC*(L_MAX-interval);

	/* root matching size */
	sum_SNJ = sizeof(int*)*max_numpart*NoC*(L_MAX-interval);

	/* consolidated allocation for SCORE_array and distribute region */
	FLOAT *dst_SCORE = (FLOAT *)malloc(sum_RL_S);
	pointer_SCORE = (uintptr_t)dst_SCORE;
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
		{
			continue;
		}

		for(int j=0; j<NoC; j++) {
			SCORE_array[L][j] = (FLOAT *)pointer_SCORE;
			pointer_SCORE += (uintptr_t)max_RL_S;
		}
	}

	/* add offset */
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
		{
			continue;
		}

		for(int j=0; j<NoC; j++) {
			memcpy(SCORE_array[L][j], rootmatch[level][j], RL_S_array[L][j]);
			FLOAT *SC_S = SCORE_array[L][j];
			FLOAT *SC_E = SCORE_array[L][j]+RL_array[L][j];
			while(SC_S<SC_E) *(SC_S++)+=OFF_array[L][j];
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
	uintptr_t pointer_ax = (uintptr_t)sub_dst_ax;
	uintptr_t pointer_ay = (uintptr_t)sub_dst_ay;
	uintptr_t pointer_Ix = (uintptr_t)sub_dst_Ix;
	uintptr_t pointer_Iy = (uintptr_t)sub_dst_Iy;
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
		{
			continue;
		}

		ax_array[L] = (int **)pointer_ax;
		pointer_ax += (uintptr_t)(NoC*sizeof(int*));

		ay_array[L] = (int **)pointer_ay;
		pointer_ay += (uintptr_t)(NoC*sizeof(int*));

		Ix_array[L] = (int ***)pointer_Ix;
		pointer_Ix += (uintptr_t)(NoC*sizeof(int**));

		Iy_array[L] = (int ***)pointer_Iy;
		pointer_Iy += (uintptr_t)(NoC*sizeof(int**));
	}

	pointer_ax = (uintptr_t)dst_ax;
	pointer_ay = (uintptr_t)dst_ay;
	pointer_Ix = (uintptr_t)dst_Ix;
	pointer_Iy = (uintptr_t)dst_Iy;
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
		{
			continue;
		}

		for(int j=0; j<NoC; j++) {
			uintptr_t pointer_offset = sizeof(int*)*max_numpart;

			ax_array[L][j] = (int *)pointer_ax;
			pointer_ax += pointer_offset;

			ay_array[L][j] = (int *)pointer_ay;
			pointer_ay += pointer_offset;

			Ix_array[L][j] = (int **)pointer_Ix;
			pointer_Ix += pointer_offset;

			Iy_array[L][j] = (int **)pointer_Iy;
			pointer_Iy += pointer_offset;
		}
	}

	/* add parts */
	if(NoP>0)
        {
		/* arrays to store temporary loop variables */
		int tmp_array_size = 0;
		for(int level=interval; level<L_MAX; level++) {
			if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
			{
				continue;
			}

			for(int j=0; j<NoC; j++) {
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
			printf("cuMemHostAlloc(dst_DID_4) failed: res = %s\n", cuda_response_to_string(res));
			exit(1);
		}


		int ***PIDX_array = (int ***)malloc((L_MAX-interval)*sizeof(int **));
		int **sub_dst_PIDX = (int **)malloc(NoC*(L_MAX-interval)*sizeof(int*));
		int *dst_PIDX;
		res = cuMemHostAlloc((void **)&dst_PIDX, tmp_array_size, CU_MEMHOSTALLOC_DEVICEMAP);
		if(res != CUDA_SUCCESS) {
			printf("cuMemHostAlloc(dst_PIDX) failed: res = %s\n", cuda_response_to_string(res));
			exit(1);
		}

		/* distribute consolidated region */
		uintptr_t pointer_DIDX = (uintptr_t)sub_dst_DIDX;
		uintptr_t pointer_DID_4 = (uintptr_t)sub_dst_DID_4;
		uintptr_t pointer_PIDX = (uintptr_t)sub_dst_PIDX;
		for(int level=interval; level<L_MAX; level++) {
			int L = level - interval;

			if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
			{
				continue;
			}

			DIDX_array[L] = (int **)pointer_DIDX;
			pointer_DIDX += (uintptr_t)(NoC*sizeof(int*));

			DID_4_array[L] = (int **)pointer_DID_4;
			pointer_DID_4 += (uintptr_t)(NoC*sizeof(int*));

			PIDX_array[L] = (int **)pointer_PIDX;
			pointer_PIDX += (uintptr_t)(NoC*sizeof(int*));
		}

		pointer_DIDX = (uintptr_t)dst_DIDX;
		pointer_DID_4 = (uintptr_t)dst_DID_4;
		pointer_PIDX = (uintptr_t)dst_PIDX;
		for(int level=interval; level<L_MAX; level++) {
			int L = level - interval;

			if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X)) {
				continue;
			}

			for(int j=0; j<NoC; j++) {
				uintptr_t pointer_offset = (uintptr_t)(max_numpart*sizeof(int));

				DIDX_array[L][j] = (int *)pointer_DIDX;
				pointer_DIDX += pointer_offset;

				DID_4_array[L][j] = (int *)pointer_DID_4;
				pointer_DID_4 += pointer_offset;

				PIDX_array[L][j] = (int *)pointer_PIDX;
				pointer_PIDX += pointer_offset;
			}
		}

		/* prepare for parallel execution */
		int sum_size_index_matrix = 0;
		for(int level=interval; level<L_MAX; level++) {
			int L = level - interval;

			if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X)) {
				continue;
			}

			for(int j=0; j<NoC; j++) {
				for (int k=0;k<numpart[j];k++) {
					/* assign values to each element */
					DIDX_array[L][j][k] = MO->MI->didx[j][k];
					DID_4_array[L][j][k] = DIDX_array[L][j][k]*4;
					PIDX_array[L][j][k] = MO->MI->pidx[j][k];

					/* anchor */
					ax_array[L][j][k] = MO->MI->anchor[DIDX_array[L][j][k]*2]+1;
					ay_array[L][j][k] = MO->MI->anchor[DIDX_array[L][j][k]*2+1]+1;

					int PSSIZE[2] ={pm_size_array[L][PIDX_array[L][j][k]*2], pm_size_array[L][PIDX_array[L][j][k]*2+1]}; // size of C

					/* index matrix */
					sum_size_index_matrix += sizeof(int)*PSSIZE[0]*PSSIZE[1];
				}
			}
		}

		int *dst_Ix_kk = (int *)malloc(sum_size_index_matrix);
		int *dst_Iy_kk = (int *)malloc(sum_size_index_matrix);
		uintptr_t pointer_Ix_kk = (uintptr_t)dst_Ix_kk;
		uintptr_t pointer_Iy_kk = (uintptr_t)dst_Iy_kk;
		for(int level=interval; level<L_MAX; level++) {
			int L = level - interval;

			if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
			{
				continue;
			}

			for(int j=0; j<NoC; j++) {
				for (int k=0;k<numpart[j];k++) {
					int PSSIZE[2] ={pm_size_array[L][PIDX_array[L][j][k]*2], pm_size_array[L][PIDX_array[L][j][k]*2+1]}; // size of C

					Ix_array[L][j][k] = (int *)pointer_Ix_kk;
					Iy_array[L][j][k] = (int *)pointer_Iy_kk;

					pointer_Ix_kk += (uintptr_t)(sizeof(int)*PSSIZE[0]*PSSIZE[1]);
					pointer_Iy_kk += (uintptr_t)(sizeof(int)*PSSIZE[0]*PSSIZE[1]);
				}
			}
		}

		gettimeofday(&tv_dt_start, nullptr);
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
			feature_size,         // int *feature_size,
			padx,          // int padx,
			pady,          // int pady,
			MO->MI->max_X, // int max_X
			MO->MI->max_Y, // int max_Y
			MO->MI->def, // FLOAT *def
			tmp_array_size, // int tmp_array_size
			dst_PIDX, // int *dst_PIDX
			dst_DID_4 // int *DID_4
			);
		gettimeofday(&tv_dt_end, nullptr);
		tvsub(&tv_dt_end, &tv_dt_start, &tv);
		time_dt += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

		/* add part score */
		for(int level=interval; level<L_MAX; level++){
			int L = level - interval;

			if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
			{
				continue;
			}

			for(int j=0; j<NoC; j++) {
				for(int k=0; k<numpart[j]; k++) {
					int PSSIZE[2] ={pm_size_array[L][PIDX_array[L][j][k]*2],
							pm_size_array[L][PIDX_array[L][j][k]*2+1]}; // Size of C
					int R_S[2]={rm_size_array[level][j*2], rm_size_array[level][j*2+1]};

					dpm_ttic_add_part_calculation(SCORE_array[L][j], M_array[L][j][k], R_S,
								      PSSIZE, ax_array[L][j][k], ay_array[L][j][k]);
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
			printf("cuMemFreeHost(dst_DID_4) failed: res = %s\n", cuda_response_to_string(res));
			exit(1);
		}
		free(sub_dst_DID_4);
		free(DID_4_array);

		res = cuMemFreeHost(dst_PIDX);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFreeHost(dst_PIDX) failed: res = %s\n", cuda_response_to_string(res));
			exit(1);
		}

		free(sub_dst_PIDX);
		free(PIDX_array);

		res = cuCtxSetCurrent(ctx[0]);
		if(res != CUDA_SUCCESS) {
			printf("cuCtxSetCurrent(ctx[0]) failed: res = %s\n", cuda_response_to_string(res));
			exit(1);
		}
        } // start from if(NoP>0)

	/* combine root and part score and detect boundary box for each-component */

	FLOAT *scale_array = (FLOAT *)malloc((L_MAX-interval)*sizeof(FLOAT));
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X))
		{
			Tboxes[count]=nullptr;
			count++;
			continue;
		}

		scale_array[L] = (FLOAT)sbin/scales[level];
	}

	for (int level=interval; level<L_MAX; level++)  // feature's loop(A's loop) 1level 1picture
        {
		/* parameters (related for level) */
		int L=level-interval;
		/* matched score size matrix */
		FLOAT scale=(FLOAT)sbin/scales[level];

		/* loop conditon */
		if(feature_size[level*2]+2*pady<MO->MI->max_Y ||(feature_size[level*2+1]+2*padx<MO->MI->max_X)) {
			Tboxes[count]=nullptr;
			count++;
			continue;
		}

		/* calculate accumulated score */
		gettimeofday(&tv_calc_a_score_start, nullptr);

		calc_a_score_GPU(
			acc_score,              // FLOAT *ac_score
			SCORE_array[L],       // FLOAT **score
			rm_size_array[level], // int *ssize_start
			MO->MI,               // Model_info *MI
			scale,                // FLOAT scale
			RL_S_array[L],        // int *size_score_array
			NoC                   // int NoC
			);

		gettimeofday(&tv_calc_a_score_end, nullptr);
		tvsub(&tv_calc_a_score_end, &tv_calc_a_score_start, &tv);
		time_calc_a_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

		for(int j = 0; j <NoC; j++) {
			int R_S[2]={rm_size_array[level][j*2], rm_size_array[level][j*2+1]};

			/* get all good matches */
			int GMN;
			int *GMPC = get_gmpc(SCORE_array[L][j],thresh,R_S,&GMN);
			int RSIZE[2]={MO->MI->rsize[j*2], MO->MI->rsize[j*2+1]};

			int GL = (numpart[j]+1)*4+3;  //31

			/* detected box coordinate(current level) */
			FLOAT *t_boxes = (FLOAT*)calloc(GMN*GL,sizeof(FLOAT));

			gettimeofday(&tv_box_start, nullptr);

			// NO NEED TO USE GPU 
			for(int k = 0;k < GMN;k++) {
				FLOAT *P_temp = t_boxes+GL*k;
				int y = GMPC[2*k];
				int x = GMPC[2*k+1];

				/* calculate root box coordinate */
				FLOAT *RB =rootbox(x,y,scale,padx,pady,RSIZE);
				memcpy(P_temp, RB,sizeof(FLOAT)*4);
				s_free(RB);
				P_temp+=4;

				for(int pp=0;pp<numpart[j];pp++) {
					int PBSIZE[2]={psize[j][pp*2], psize[j][pp*2+1]};
					int Isize[2]={pm_size_array[L][MO->MI->pidx[j][pp]*2], pm_size_array[L][MO->MI->pidx[j][pp]*2+1]};

					/* calculate part box coordinate */
					FLOAT *PB = partbox(x,y,ax_array[L][j][pp],ay_array[L][j][pp],scale,padx,pady,PBSIZE,Ix_array[L][j][pp],Iy_array[L][j][pp],Isize);
					memcpy(P_temp, PB,sizeof(FLOAT)*4);
					P_temp+=4;
					s_free(PB);
				}
				/* component number and score */
				*(P_temp++)=(FLOAT)j; //component number
				*(P_temp++)=SCORE_array[L][j][x*R_S[0]+y]; //score of good match
				*P_temp = scale;
			}

			//  NO NEED TO USE GPU
			gettimeofday(&tv_box_end, nullptr);
			tvsub(&tv_box_end, &tv_box_start, &tv);
			time_box += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

			/* save box information */
			if (GMN > 0)
				Tboxes[count] = t_boxes;
			else
				Tboxes[count] = nullptr;

			b_nums[count]=GMN;
			count++;
			detected_boxes+=GMN;			//number of detected box

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

	gettimeofday(&tv_nucom_end, nullptr);

#ifdef PRINT_INFO
	printf("root SCORE : %f\n", time_root_score);
	printf("part SCORE : %f\n", time_part_score);
	printf("dt  : %f\n", time_dt);
	printf("calc_a_score : %f\n", time_calc_a_score);
#endif
	res = cuCtxSetCurrent(ctx[0]);
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSetCurrent(ctx[0]) failed: res = %s\n",cuda_response_to_string(res));
		exit(1);
	}

	/* free memory regions */
	res = cuMemFreeHost((void *)featp2[0]);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFreeHost(featp2[0]) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	s_free(featp2);

	res = cuMemFreeHost((void *)rootmatch[interval][0]);
	if(res != CUDA_SUCCESS) {
		printf("cuMemFreeHost(rootmatch[0][0]) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}
	s_free(rootmatch[0]);
	s_free(rootmatch);

	if (partmatch != nullptr) {
		res = cuMemFreeHost((void *)partmatch[0][0]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFreeHost(partmatch[0][0]) failed: res = %s\n", cuda_response_to_string(res));
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
	FLOAT *boxes=(FLOAT*)calloc(detected_boxes*GL,sizeof(FLOAT));		//box coordinate information(Temp)

	FLOAT *T1 = boxes;
	for(int i = 0; i < LofFeat; i++) {
		int num_t = b_nums[i]*GL;
		if(num_t > 0) {
			FLOAT *T2 = Tboxes[i];
			//memcpy_s(T1,sizeof(FLOAT)*num_t,T2,sizeof(FLOAT)*num_t);
			memcpy(T1, T2,sizeof(FLOAT)*num_t);
			T1 += num_t;
		}
	}

	FLOAT abs_threshold = abs(thresh);

	/* accumulated score calculation */
	FLOAT max_score = 0.0;

	/* add offset to accumulated score */
	for(int i = 0; i < MO->MI->IM_HEIGHT*MO->MI->IM_WIDTH; i++) {
		if (acc_score[i] < thresh) {
			acc_score[i] = 0.0;
		} else {
			acc_score[i] += abs_threshold;

			if (acc_score[i] > max_score)
				max_score = acc_score[i];
		}
	}

	/* normalization */
	if (max_score > 0.0) {
		FLOAT ac_ratio = 1.0 / max_score;

		for (int i = 0; i < MO->MI->IM_HEIGHT*MO->MI->IM_WIDTH; i++) {
			acc_score[i] *= ac_ratio;
		}
	}

	/* release */
	free_boxes(Tboxes,LofFeat);
	s_free(b_nums);

	/* output result */
	*detected_count = detected_boxes;
	return boxes;
}
