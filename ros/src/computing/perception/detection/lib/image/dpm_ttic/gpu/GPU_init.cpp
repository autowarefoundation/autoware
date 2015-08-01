/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cstdio>
#include <cmath>

#include "for_use_GPU.h"
#include "drvapi_error_string.h"
#include <cuda_runtime_api.h>
#include "switch_release.h"
#include "GPU_init.hpp"

#define SIZE_FEATP2 100000000
#define SIZE_A_SIZE 1000
#define SIZE_B 100000
#define SIZE_B_DIMS 1000
#define SIZE_ERROR_ARRAY 100
#define SIZE_C 50000000
#define SIZE_PM 10000
#define SIZE_DEF 1000
#define SIZE_NUMPART 100
#define SIZE_PIDX 10000
#define SIZE_DID 10000
#define SIZE_M 30000000
#define SIZE_TMPM 30000000
#define SIZE_TMPIX 30000000
#define SIZE_TMPIY 30000000

/*** for debug(Linux) ***/
#include <unistd.h>

/* declaration of global variables */

//extern CUdevice dev;
CUdevice *dev;
CUcontext *ctx;
CUfunction *func_process_root, *func_process_part, *func_dt1d_x, *func_dt1d_y, *func_calc_a_score, *func_inverse_Q, *func_calc_hist, *func_calc_norm, *func_calc_feat, *func_resize;
CUmodule *module;
int *NR_MAXTHREADS_X, *NR_MAXTHREADS_Y;
CUdeviceptr *A_SIZE_dev, *featp2_dev, *B_dev, *B_dims_dev, *fconvs_error_array_dev, *fconvs_C_dev, *part_C_dev, *part_error_array_dev, *pm_size_array_dev;
CUdeviceptr *PIDX_array_dev, *def_array_dev, *DID_4_array_dev, *numpart_dev,*M_dev, *tmpM_dev, *tmpIx_dev, *tmpIy_dev;

/* init_cuda
   initialization device to use CUDA function
*/
void dpm_ttic_gpu_init_cuda(void)
{
    //const char file_name[43] = "./gccDebug/GPU_function.cubin";
#ifdef RELEASE
    const char file_name[256] = "/usr/local/geye_with_cam/bin/car_detecter/GPU_function.cubin";
#else
    const char file_name[43] = "./gccRelease/GPU_function.cubin";
#endif
    dpm_ttic_gpu_init_cuda_with_cubin(file_name);
}/* init_cuda */

void dpm_ttic_gpu_init_cuda_with_cubin(const char *cubin_path)
{
	/* initnialize GPU */
	CUresult res = cuInit(0);
	if(res != CUDA_SUCCESS){
		printf("\ncuInit failed: res = %s\n", getCudaDrvErrorString(res));
		exit(1);
	}

	/* count the number of usable GPU */
	res = cuDeviceGetCount(&device_num);
	if(res != CUDA_SUCCESS) {
		printf("cuDeviceGetCount() failed: res = %s\n", getCudaDrvErrorString(res));
		exit(1);
	}

	//    device_num = 2;
	printf("\ncar detection program %d GPUs found\n", device_num);

	/* get device */
	dev = (CUdevice*)malloc(device_num*sizeof(CUdevice));
	for(int i=0; i<device_num; i++) {
		//    res = cuDeviceGet(&dev[i], 0);
		res = cuDeviceGet(&dev[i], i);
		if(res != CUDA_SUCCESS) {
			printf("cuDeviceGet(dev[%d]) failed: res = %s\n", i, getCudaDrvErrorString(res));
			exit(1);
		}
		printf("car detection use GPU[No.%d]\n", i);
	}

	ctx = (CUcontext*)malloc(device_num*sizeof(CUcontext));

	module = (CUmodule*)malloc(device_num*sizeof(CUmodule));

	func_process_root = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_process_part = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_dt1d_x = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_dt1d_y = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_calc_a_score = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_inverse_Q = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_calc_hist    = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_calc_norm    = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_calc_feat    = (CUfunction*)malloc(device_num*sizeof(CUfunction));
	func_resize  = (CUfunction*)malloc(device_num*sizeof(CUfunction));

	for(int i=0; i<device_num; i++) {
		res = cuCtxCreate(&ctx[i], 0, dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuCtxCreate(ctx[%d]) failed: res = %s\n", i, getCudaDrvErrorString(res));
			exit(1);
		}
	}

	for(int i=0; i<device_num; i++) {
		res = cuCtxSetCurrent(ctx[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuCtxSetCurrent(ctx[%d]) failed: res = %s\n", i, getCudaDrvErrorString(res));
			exit(1);
		}

		/* load .cubin file */
		res = cuModuleLoad(&module[i], cubin_path);
		if(res != CUDA_SUCCESS){
			printf("\ncuModuleLoad failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_process_root[i], module[i], "process_root");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(process_root) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_process_part[i], module[i], "process_part");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(process_part) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_inverse_Q[i], module[i], "inverse_Q");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(inverse_Q) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_dt1d_x[i], module[i], "dt1d_x");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(dt1d_x) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_dt1d_y[i], module[i], "dt1d_y");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(dt1d_y) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_calc_a_score[i], module[i], "calc_a_score");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(calc_a_score) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_calc_hist[i], module[i], "calc_hist");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(calc_hist) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_calc_norm[i], module[i], "calc_norm");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(calc_norm) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_calc_feat[i], module[i], "calc_feat");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(calc_feat) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetFunction(&func_resize[i], module[i], "resize");
		if(res != CUDA_SUCCESS){
			printf("\ncuGetFunction(resize) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

	}

	/* allocate GPU memory */
	A_SIZE_dev = (CUdeviceptr *)malloc(device_num*sizeof(CUdeviceptr));
	featp2_dev = (CUdeviceptr *)malloc(device_num*sizeof(CUdeviceptr));
	B_dev = (CUdeviceptr *)malloc(device_num*sizeof(CUdeviceptr));
	B_dims_dev = (CUdeviceptr *)malloc(device_num*sizeof(CUdeviceptr));
	fconvs_error_array_dev = (CUdeviceptr *)malloc(device_num*sizeof(CUdeviceptr));
	fconvs_C_dev = (CUdeviceptr *)malloc(device_num*sizeof(CUdeviceptr));
	part_error_array_dev = (CUdeviceptr *)malloc(sizeof(CUdeviceptr) * device_num);
	part_C_dev = (CUdeviceptr *)malloc(sizeof(CUdeviceptr) * device_num);
	pm_size_array_dev = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));
	PIDX_array_dev = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));
	def_array_dev = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));
	DID_4_array_dev = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));
	numpart_dev  = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));
	M_dev = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));
	tmpM_dev = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));
	tmpIx_dev = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));
	tmpIy_dev  = (CUdeviceptr*)malloc(device_num*sizeof(CUdeviceptr));

	for(int i = 0; i<device_num; i++) {
		res = cuCtxSetCurrent(ctx[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuCtxSetCurrent(ctx[%d]) failed: res = %s\n", i, getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&featp2_dev[i], SIZE_FEATP2);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(featp2_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}


		res = cuMemAlloc(&A_SIZE_dev[i], SIZE_A_SIZE);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(A_SIZE_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&B_dev[i], SIZE_B);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(B_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&B_dims_dev[i], SIZE_B_DIMS);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(B_dims_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&fconvs_error_array_dev[i], SIZE_ERROR_ARRAY);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(fconvs_error_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&fconvs_C_dev[i], SIZE_C);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(fconvs_C_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&part_C_dev[i], SIZE_C);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(part_C_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&part_error_array_dev[i], SIZE_ERROR_ARRAY);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(part_error_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&pm_size_array_dev[i], SIZE_PM);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(pm_size_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&def_array_dev[i], SIZE_DEF);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(def_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&numpart_dev[i], SIZE_NUMPART);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(numpart_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&PIDX_array_dev[i], SIZE_PIDX);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(PIDX_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&DID_4_array_dev[i], SIZE_DID);
		if(res != CUDA_SUCCESS) {
			printf("cuMemAlloc(DID_4__array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&M_dev[i], SIZE_M);
		if(res != CUDA_SUCCESS){
			printf("cuMemAlloc(M_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&tmpM_dev[i], SIZE_TMPM);
		if(res != CUDA_SUCCESS){
			printf("cuMemAlloc(tmpM_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&tmpIx_dev[i], SIZE_TMPIX);
		if(res != CUDA_SUCCESS){
			printf("cuMemAlloc(tmpIx_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemAlloc(&tmpIy_dev[i], SIZE_TMPIY);
		if(res != CUDA_SUCCESS){
			printf("cuMemAlloc(tmpIy_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}
	}

	NR_MAXTHREADS_X = (int*)malloc(device_num*sizeof(int));
	NR_MAXTHREADS_Y = (int*)malloc(device_num*sizeof(int));

	for(int i = 0; i<device_num; i++) {
		/* get max thread num per block */
		int max_threads_num = 0;
		res = cuDeviceGetAttribute(&max_threads_num, CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, dev[i]);
		if(res != CUDA_SUCCESS){
			printf("\ncuDeviceGetAttribute() failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		NR_MAXTHREADS_X[i] = (int)sqrt((double)max_threads_num);
		NR_MAXTHREADS_Y[i] = (int)sqrt((double)max_threads_num);
	}

	res = cuCtxSetCurrent(ctx[0]);
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSetCurrent(ctx[0]) failed: res = %s\n", getCudaDrvErrorString(res));
		exit(1);
	}
}

/*****************************************************************/
/* clean_cuda

   cleaning up after using GPU
*/
/*****************************************************************/
void dpm_ttic_gpu_clean_cuda(void)
{
	CUresult res;

	for (int i=0; i < device_num; ++i) {
		res = cuCtxSetCurrent(ctx[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuCtxSetCurrent(ctx[%d]) failed: res = %s\n", i, getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(featp2_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(featp2_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(A_SIZE_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(A_SIZE_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(B_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(B_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(B_dims_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(B_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(fconvs_error_array_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(fconvs_error_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(fconvs_C_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(fconvs_C_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(part_C_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(part_C_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(part_error_array_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(part_error_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(pm_size_array_dev[i]);
		if(res != CUDA_SUCCESS){
			printf("cuMemFree(pm_size_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(def_array_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(def_array_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(numpart_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(numpart_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(M_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(M_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(tmpM_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(tmpM_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(tmpIx_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(tmpIx_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuMemFree(tmpIy_dev[i]);
		if(res != CUDA_SUCCESS) {
			printf("cuMemFree(tmpIy_dev) failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}

	}

	for (int i = 0; i < device_num; ++i) {
		res = cuModuleUnload(module[i]);
		if(res != CUDA_SUCCESS){
			printf("\ncuModuleUnload failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}
	}

	printf("module unloaded\n");

	for (int i=0; i < device_num; ++i) {
		res = cuCtxDestroy(ctx[i]);
		if(res != CUDA_SUCCESS){
			printf("\ncuCtxDestroy failed: res = %s\n", getCudaDrvErrorString(res));
			exit(1);
		}
	}

	printf("context destroyed\n");
	free(featp2_dev);
	free(A_SIZE_dev);
	free(B_dev);
	free(B_dims_dev);
	free(fconvs_error_array_dev);
	free(fconvs_C_dev);
	free(NR_MAXTHREADS_X);
	free(NR_MAXTHREADS_Y);
	free(func_process_root);
	free(func_process_part);
	free(func_dt1d_x);
	free(func_dt1d_y);
	free(func_calc_a_score);
	free(func_inverse_Q);
	free(func_calc_hist);
	free(func_calc_norm);
	free(func_calc_feat);
	free(func_resize);
	free(part_C_dev);
	free(part_error_array_dev);
	free(pm_size_array_dev);
	free(def_array_dev);
	free(numpart_dev);
	free(DID_4_array_dev);
	free(PIDX_array_dev);
	free(M_dev);
	free(tmpM_dev);
	free(tmpIx_dev);
	free(tmpIy_dev);
	free(module);
	free(dev);
	free(ctx);
	printf("clean_cuda finished\n");
}/* clean_cuda */
