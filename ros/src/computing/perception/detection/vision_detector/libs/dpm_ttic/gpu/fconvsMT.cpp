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

//C++ library (thread-functions are only supported by windows)
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

#include "MODEL_info.h"		//File information
#include "common.hpp"
#include "multithreading.h"
#include "for_use_GPU.h"
#include "drvapi_error_string.h"
#include "switch_release.h"

#include <cuda_util.hpp>

// use for dt_GPU
int part_error_array_num;
int *part_error_array;
size_t SUM_SIZE_C;
FLOAT *dst_C;

struct fconvs_partition {
	FLOAT **featp2;
	size_t SUM_SIZE_feat;
	int *A_SIZE;
	size_t SUM_SIZE_B;
	size_t SUM_SIZE_C;
	FLOAT *dst_C;
	FLOAT **filter;
	int len;
	int calc_flag;
	int error_array_num;
	int start;
	int *error_array;
	int *B_dimension;
	int interval;
	int L_MAX;
	int pid;
	int max_height;
	int max_width;
};

struct thread_data {
	FLOAT *A;
	FLOAT *B;
	FLOAT *C;
	FLOAT *F;
	FLOAT *T;
	int A_dims[3];
	int B_dims[3];
	int C_dims[2];
};

static CUT_THREADPROC fconvs_thread_func(void *p)
{
	fconvs_partition *pt = (fconvs_partition *)p;
	CUresult res;
	struct timeval tv;
	//CUdeviceptr part_root_C_dev;
	//CUdeviceptr part_part_C_dev;
	int thread_num_x, thread_num_y, block_num_x, block_num_y;
	struct timeval tv_fconv_kernel_start, tv_fconv_kernel_end;
	float time_fconv_kernel;
	struct timeval tv_fconv_memcpyHtoD_start, tv_fconv_memcpyHtoD_end;
	float time_fconv_memcpyHtoD;
	struct timeval tv_fconv_memcpyDtoH_start, tv_fconv_memcpyDtoH_end;
	float time_fconv_memcpyDtoH;
	struct timeval tv_fconv_texture_start, tv_fconv_texture_end;
	float time_fconv_texture;
#ifdef PRINT_INFO
	float time_fconv_memalloc;
#endif
	struct timeval tv_fconv_others_start, tv_fconv_others_end;
	float time_fconv_others;

	if(pt->calc_flag == PART) part_error_array_num = pt->error_array_num;

	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_others_start, NULL);
	}
	/* set CUDA context to this CPU thread */
	res = cuCtxSetCurrent(ctx[pt->pid]);
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSetCurrent(ctx[%d]) failed: res = %s\n", pt->pid, cuda_response_to_string(res));
		exit(1);
	}

	/* define CUDA block shape */
	int max_threads_num = 0;
	res = cuDeviceGetAttribute(&max_threads_num, CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, dev[pt->pid]);
	if(res != CUDA_SUCCESS){
		printf("\ncuDeviceGetAttribute() failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}


	/* calculate max size of each block dimension */
	NR_MAXTHREADS_X[pt->pid] = (int)sqrt((FLOAT)max_threads_num); //(int)sqrt((FLOAT)max_threads_num/pt->len);
	NR_MAXTHREADS_Y[pt->pid] = (int)sqrt((FLOAT)max_threads_num); //(int)sqrt((FLOAT)max_threads_num/pt->len);
	if(NR_MAXTHREADS_X[pt->pid] < 1) NR_MAXTHREADS_X[0]++;
	if(NR_MAXTHREADS_Y[pt->pid] < 1) NR_MAXTHREADS_Y[0]++;


	thread_num_x = (pt->max_width < NR_MAXTHREADS_X[pt->pid]) ? pt->max_width : NR_MAXTHREADS_X[pt->pid];
	thread_num_y = (pt->max_height < NR_MAXTHREADS_Y[pt->pid]) ? pt->max_height : NR_MAXTHREADS_Y[pt->pid];


	block_num_x = pt->max_width / thread_num_x;
	block_num_y = pt->max_height / thread_num_y;
	if(pt->max_width % thread_num_x != 0) block_num_x++;
	if(pt->max_height % thread_num_y != 0) block_num_y++;


	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_others_end, NULL);
		tvsub(&tv_fconv_others_end, &tv_fconv_others_start, &tv);
		time_fconv_others += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}


	/* upload data to GPU memory */
	if(pt->pid == 0){
		gettimeofday(&tv_memcpy_start, NULL);
	}
	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_memcpyHtoD_start, NULL);
	}
	/* upload resized source images to GPU */
	res = cuMemcpyHtoD(featp2_dev[pt->pid], pt->featp2[0], pt->SUM_SIZE_feat);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(featp2) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	/* upload resize image sizes to GPU */
	res = cuMemcpyHtoD(A_SIZE_dev[pt->pid], pt->A_SIZE, pt->L_MAX*3*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(new_PADsize) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	/* upload filter to GPU */
	res = cuMemcpyHtoD(B_dev[pt->pid], pt->filter[pt->start],  pt->SUM_SIZE_B);
	if(res != CUDA_SUCCESS){
		printf("cuMemcpyHtoD(B_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	/* upload error condition to GPU */
	res = cuMemcpyHtoD(fconvs_error_array_dev[pt->pid], pt->error_array, pt->error_array_num*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(part_error_array_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_memcpyHtoD_end, NULL);
		tvsub(&tv_fconv_memcpyHtoD_end, &tv_fconv_memcpyHtoD_start, &tv);
		time_fconv_memcpyHtoD += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}


	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_texture_start, NULL);
	}
	/* get handle to a texture memory on GPU */
	CUtexref featp2_texref, B_texref;
	if(sizeof(FLOAT) == sizeof(float)) // if configured to use single precision
	{
		res = cuModuleGetTexRef(&featp2_texref, module[pt->pid], "A");
		if(res != CUDA_SUCCESS) {
			printf("cuModuleGetTexRef(featp2) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetTexRef(&B_texref, module[pt->pid], "B");
		if(res != CUDA_SUCCESS) {
			printf("cuModuleGetTexRef(B) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
			exit(1);
		}
	}
	else                        // if configured to use double precision
	{
		res = cuModuleGetTexRef(&featp2_texref, module[pt->pid], "A_double");
		if(res != CUDA_SUCCESS) {
			printf("cuModuleGetTexRef(featp2) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuModuleGetTexRef(&B_texref, module[pt->pid], "B_double");
		if(res != CUDA_SUCCESS) {
			printf("cuModuleGetTexRef(B) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
			exit(1);
		}
	}

	/* bind to texture memory on GPU */
	res = cuTexRefSetAddress(NULL, featp2_texref, featp2_dev[pt->pid], pt->SUM_SIZE_feat);
	if (res != CUDA_SUCCESS) {
		printf("cuTexRefSetAddress(featp2_dev) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
		exit(1);
	}


	res = cuTexRefSetAddress(NULL, B_texref, B_dev[pt->pid],  pt->SUM_SIZE_B);
	if (res != CUDA_SUCCESS) {
		printf("cuTexRefSetAddress(B_dev) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
		exit(1);
	}

	/* texture memory configuration */
	res = cuTexRefSetFlags(featp2_texref, CU_TRSF_NORMALIZED_COORDINATES);
	if (res != CUDA_SUCCESS) {
		printf("cuTexRefSetFlags(featp2_texref) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
		exit(1);
	}

	res = cuTexRefSetFlags(B_texref, CU_TRSF_NORMALIZED_COORDINATES);
	if (res != CUDA_SUCCESS) {
		printf("cuTexRefSetFlags(B_texref) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
		exit(1);
	}

	if(sizeof(FLOAT) == sizeof(float)) // if configured to use single precision
	{
		res = cuTexRefSetFormat(featp2_texref, CU_AD_FORMAT_FLOAT, 1);
		if (res != CUDA_SUCCESS) {
			printf("cuTexRefSetFormat(featp2_texref) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuTexRefSetFormat(B_texref, CU_AD_FORMAT_FLOAT, 1);
		if (res != CUDA_SUCCESS) {
			printf("cuTexRefSetFormat(B_texref) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
			exit(1);
		}
	}
	else                          // if configured to use double precision
	{
		res = cuTexRefSetFormat(featp2_texref, CU_AD_FORMAT_UNSIGNED_INT32, 2);
		if (res != CUDA_SUCCESS) {
			printf("cuTexRefSetFormat(featp2_texref) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
			exit(1);
		}

		res = cuTexRefSetFormat(B_texref, CU_AD_FORMAT_UNSIGNED_INT32, 2);
		if (res != CUDA_SUCCESS) {
			printf("cuTexRefSetFormat(B_texref) failed: res = %d\n->%s\n", res, getCudaDrvErrorString(res));
			exit(1);
		}
	}
	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_texture_end, NULL);
		tvsub(&tv_fconv_texture_end, &tv_fconv_texture_start, &tv);
		time_fconv_texture += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}

	if(pt->pid == 0){
		gettimeofday(&tv_memcpy_end, NULL);
		tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
		time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}


	/* allocate output region on GPU memory and upload date to GPU*/

	if(pt->pid == 0){
		gettimeofday(&tv_memcpy_start, NULL);
	}
	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_memcpyHtoD_start, NULL);
	}

	res = cuMemcpyHtoD(fconvs_C_dev[pt->pid], pt->dst_C, pt->SUM_SIZE_C);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(part_C_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(B_dims_dev[pt->pid], pt->B_dimension, 3*pt->len*sizeof(int));
	if(res != CUDA_SUCCESS){
		printf("cuMemcpyHtoD(B_dims) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_memcpyHtoD_end, NULL);
		tvsub(&tv_fconv_memcpyHtoD_end, &tv_fconv_memcpyHtoD_start, &tv);
		time_fconv_memcpyHtoD += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}
	if(pt->pid == 0){
		gettimeofday(&tv_memcpy_end, NULL);
		tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
		time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}

	/* launch kernel
	   grid shape : block_num_x * block_num_y * L_MAX,
	   block shape : thread_num_x * thread_num_y * len
	*/
	/* dealing with 1 feature(A) by 1 z_dimension of grid */
	/* dealing with 1 model(B) by 1 z_dimension of block */

	void *kernel_args[] = {
		// &featp2_dev,                 // kernel_args[0]
		// &B_dev,                      // kernel_args[1]
		&fconvs_C_dev[pt->pid],         // kernel_args[2]
		&A_SIZE_dev[pt->pid],           // kernel_args[3]
		&B_dims_dev[pt->pid],           // kernel_args[4]
		(void *)&(pt->len),             // kernel_args[5]
		(void *)&(pt->interval),        // kernel_args[6]
		(void *)&(pt->L_MAX),           // kernel_args[7]
		&fconvs_error_array_dev[pt->pid], // kernel_args[8]
		(void *)&(pt->error_array_num), // kernel_args[9]
		(void *)&(pt->pid),             // kernel_args[10]
		(void *)&(device_num)           // kernel_args[11]
	};

#ifdef PRINT_INFO
	if(pt->calc_flag == PART){
		printf("block_num_x = %d\n",block_num_x);
		printf("block_num_y = %d\n",block_num_y);
		printf("block_num_z = %d\n",pt->L_MAX*pt->len);
		printf("thread_num_x = %d\n",thread_num_x);
		printf("thread_num_y = %d\n",thread_num_y);
		printf("thread_num_z = 1\n");
	}
#endif

	int sharedMemBytes = 0;

	int blockDimX = thread_num_x / device_num;
	if(thread_num_x % device_num != 0) blockDimX++;


	if(pt->pid == 0){
		gettimeofday(&tv_kernel_start, NULL);
	}
	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_kernel_start, NULL);
	}

	switch(pt->calc_flag) {
	case ROOT:
		res = cuLaunchKernel(
			func_process_root[pt->pid], // call function
			block_num_x,                // gridDimX
			block_num_y,                // gridDimY
			(pt->L_MAX)*(pt->len),      // gridDimZ
			blockDimX,                  // blockDimX
			thread_num_y,               // blockDimY
			1,                          // blockDimZ
			sharedMemBytes,             // sharedMemBytes
			NULL,                       // hStream
			kernel_args,                // kernelParams
			NULL                        // extra
			);
		if(res != CUDA_SUCCESS){
			printf("cuLaunchKernel(root) failed: res = %s\n", cuda_response_to_string(res));
			exit(1);
		}
		break;

	case PART:
		res = cuLaunchKernel(
			func_process_part[pt->pid], // call function
			block_num_x,                // gridDimX
			block_num_y,                // gridDimY
			(pt->L_MAX)*(pt->len),      // gridDimZ
			blockDimX,                  // blockDimX
			thread_num_y,               // blockDimY
			1,                          // blockDimZ
			sharedMemBytes,             // sharedMemBytes
			NULL,                       // hStream
			kernel_args,                // kernelParams
			NULL                        // extra
			);
		if(res != CUDA_SUCCESS){
			printf("cuLaunchKernel(part) failed: res = %s\n", cuda_response_to_string(res));
			exit(1);
		}
		break;

	default:
		printf("NOT DEFINED value: calc_flag = %d\n", pt->calc_flag);
		exit(1);
		break;
	}

	/* synchronize GPU threads */
	res = cuCtxSynchronize();
	if(res != CUDA_SUCCESS){
		printf("pid = %d, calc_flag = %d\n",pt->pid, pt->calc_flag);
		printf("cuCtxSynchronize(process) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_kernel_end, NULL);
		tvsub(&tv_fconv_kernel_end, &tv_fconv_kernel_start, &tv);
		time_fconv_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}

	if(pt->pid == 0){
		gettimeofday(&tv_kernel_end, NULL);
		tvsub(&tv_kernel_end, &tv_kernel_start, &tv);
		time_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}


	/* download C from GPU */
	if(pt->pid == 0){
		gettimeofday(&tv_memcpy_start, NULL);
	}
	if(pt->pid == 0 && pt->calc_flag == PART){
		gettimeofday(&tv_fconv_memcpyDtoH_start, NULL);
	}

	int C_dims0 = 0;
	int C_dims1 = 0;
	int C_x = 0;
	size_t x_size = 0;
	int error_flag = 0;
	unsigned long long int pointer_C = (unsigned long long int)pt->dst_C;
	unsigned long long int root_pointer_dev = (unsigned long long int)fconvs_C_dev[pt->pid];
	unsigned long long int part_pointer_dev = (unsigned long long int)fconvs_C_dev[pt->pid];


	switch(pt->calc_flag) {
	case ROOT:
		for(int lev = pt->interval; lev < pt->L_MAX; lev++){

			/* loop condition */
			for(int k = 0; k < pt->error_array_num; k++) {
				if(pt->error_array[k] == lev)
					error_flag = 1;
			}

			for(int ii = 0; ii < pt->len; ii++) {

				if(error_flag == 1) {
					error_flag = 0;
					break;
				}

				C_dims0 = pt->A_SIZE[lev*3] - pt->B_dimension[ii*3] + 1;
				C_dims1 = pt->A_SIZE[lev*3+1] - pt->B_dimension[ii*3+1] + 1;

				if(C_dims0 < 1 || C_dims1 < 1) continue;

				C_x = C_dims1 / device_num;

				if(C_dims1 % device_num != 0){
					C_x++;
				}


				if(pt->pid*C_x + C_x > C_dims1)
					x_size = (C_dims1-pt->pid*C_x)*C_dims0*sizeof(FLOAT);
				else
					x_size = C_x*C_dims0*sizeof(FLOAT);

				if(pt->pid*C_x < C_dims1){

					res = cuMemcpyDtoH((void *)(pointer_C+(unsigned long long int)(pt->pid*C_x*C_dims0*sizeof(FLOAT))), (CUdeviceptr)(root_pointer_dev+(unsigned long long int)(pt->pid*C_x*C_dims0*sizeof(FLOAT))), x_size);
					if(res != CUDA_SUCCESS) {
						printf("cuMemcpyDtoH(dst_C root) failed: res = %s\n", cuda_response_to_string(res));
						exit(1);
					}

				}


				pointer_C += (unsigned long long int)(C_dims0 * C_dims1 * sizeof(FLOAT));
				root_pointer_dev += (unsigned long long int)(C_dims0 * C_dims1 * sizeof(FLOAT));

			}

		}

		if(pt->pid == 0){
			gettimeofday(&tv_memcpy_end, NULL);
			tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
			time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
		}

		break;

	case PART:

		for(int lev = 0; lev < (pt->L_MAX-pt->interval); lev++) {

			for(int k = 0; k < part_error_array_num; k++) {
				if(pt->error_array[k] == lev)
					error_flag = 1;
			}

			for(int ii = 0; ii < pt->len; ii++) {

				if(error_flag == 1) {
					error_flag = 0;
					break;
				}

				C_dims0 = pt->A_SIZE[lev*3] - pt->B_dimension[ii*3] + 1;
				C_dims1 = pt->A_SIZE[lev*3+1] - pt->B_dimension[ii*3+1] + 1;

				if(C_dims0 < 1 || C_dims1 < 1) continue;

				C_x = C_dims1 / device_num;

				if(C_dims1 % device_num != 0) {
					C_x++;
				}

				if(pt->pid*C_x + C_x > C_dims1)
					x_size = (C_dims1-pt->pid*C_x)*C_dims0*sizeof(FLOAT);
				else
					x_size = C_x*C_dims0*sizeof(FLOAT);

				if(pt->pid*C_x < C_dims1){

					res = cuMemcpyDtoH((void *)(pointer_C+(unsigned long long int)(pt->pid*C_x*C_dims0*sizeof(FLOAT))), (CUdeviceptr)(part_pointer_dev+(unsigned long long int)(pt->pid*C_x*C_dims0*sizeof(FLOAT))), x_size);
					if(res != CUDA_SUCCESS) {
						printf("cuMemcpyDtoH(dst_C root) failed: res = %s\n", cuda_response_to_string(res));
						exit(1);
					}

				}

				pointer_C += (unsigned long long int)(C_dims0 * C_dims1 * sizeof(FLOAT));
				part_pointer_dev += (unsigned long long int)(C_dims0 * C_dims1 * sizeof(FLOAT));


			}

		}

		if(pt->pid == 0 && pt->calc_flag == PART){
			gettimeofday(&tv_fconv_memcpyDtoH_end, NULL);
			tvsub(&tv_fconv_memcpyDtoH_end, &tv_fconv_memcpyDtoH_start, &tv);
			time_fconv_memcpyDtoH += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
		}

		if(pt->pid == 0){
			gettimeofday(&tv_memcpy_end, NULL);
			tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
			time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
		}

		break;

	default:
		printf("NOT DEFINED value: calc_flag = %d\n", pt->calc_flag);
		exit(1);
		break;

	}

#ifdef PRINT_INFO
	if(pt->pid == 0 && pt->calc_flag == PART){
		printf("fconv_kernel : %f[ms]\n",time_fconv_kernel);
		printf("fconv_texture : %f[ms]\n",time_fconv_texture);
		printf("fconv_others : %f[ms]\n",time_fconv_others);
		printf("fconv_memalloc : %f[ms]\n",time_fconv_memalloc);
		printf("fconv_memfree : %f[ms]\n",time_fconv_memfree);
		printf("fconv_memcpyHtoD : %f[ms]\n",time_fconv_memcpyHtoD);
		printf("fconv_memcpyDtoH : %f[ms]\n",time_fconv_memcpyDtoH);
		printf("fconv_thread_time : %f[ms]\n",time_fconv_kernel+time_fconv_texture+time_fconv_others+time_fconv_memcpyHtoD+time_fconv_memcpyDtoH);
	}
#endif


	/* end of thread */
	CUT_THREADEND;
}

FLOAT ***fconvsMT_GPU(
  FLOAT **featp2,
  size_t SUM_SIZE_feat,
  FLOAT **filter,
  int *sym_info,
  int start,
  int end,
  int *A_SIZE,
  int **B_SIZE,
  int **M_size_array,
  int L_MAX,
  int interval,
  int *FSIZE,
  int padx,
  int pady,
  int max_X,
  int max_Y,
  int calc_flag
                      )
{

	struct timeval tv_fconv_start, tv_fconv_end;
	float time_fconv;
	struct timeval tv_fconv_others_start, tv_fconv_others_end;
	float time_fconv_others;

	if(calc_flag == PART){
#ifdef PRINT_INFO
		printf("*******fconv PART print*******\n");
#endif
		gettimeofday(&tv_fconv_others_start, NULL);
	}

	fconvs_partition *p = (fconvs_partition *)malloc(device_num*sizeof(fconvs_partition));

	for(int i = 0; i < device_num; i++){
		p[i].max_height = 0;
		p[i].max_width = 0;
	}

	start=start-1;
	end=end-1;

	const int len=end-start+1;
	FLOAT ***Output = (FLOAT ***)malloc(L_MAX*sizeof(FLOAT **));  // make FLOAT* Output[L_MAX][len]

	struct timeval tv;

	thread_data **td = (thread_data **)malloc(L_MAX*sizeof(thread_data *));  // make thread_data td[L_MAX][len]
	thread_data *dst_td = (thread_data *)calloc(L_MAX*len, sizeof(thread_data));
	unsigned long long int ptr_td = (unsigned long long int)dst_td;
	for(int i=0; i<L_MAX; i++) {
		td[i] = (thread_data *)ptr_td;
		ptr_td += (unsigned long long int)(len*sizeof(thread_data));
	}


	CUresult res;

	int *B_dimension = (int*)malloc(3*len*sizeof(int));

	size_t SUM_SIZE_B = 0;

	SUM_SIZE_C = 0;

	/* array in order to apply loop condition to kernel */
	int error_array_num = 0;

	int *error_array;

	/**********************************************************************/
	/* prepare output region */

	/* allocate output region in lump */
	FLOAT **dst_output;
	dst_output = (FLOAT **)malloc(L_MAX*len*sizeof(FLOAT *));
	if(dst_output == NULL) {
		printf("allocate dst_output failed\n");
		exit(1);
	}

	memset(dst_output, 0, L_MAX*len*sizeof(FLOAT *));  // zero clear

	/* distribution to Output[L_MAX - interval]*/
	unsigned long long int ptr_output = (unsigned long long int)dst_output;
	for(int i=0; i<L_MAX; i++) {
		Output[i] = (FLOAT **)ptr_output;
		ptr_output += (unsigned long long int)(len*sizeof(FLOAT *));
	}


	/* prepare output region */
	/**********************************************************************/



	/* prepare for launch kernel */
	for(int ii=0;ii<len;ii++)  // filter's loop(B's loop)
	{
		/* store B dimendion in B_dimension */
		B_dimension[ii*3] = B_SIZE[ii][0];
		B_dimension[ii*3 + 1] = B_SIZE[ii][1];
		B_dimension[ii*3 + 2] = 31;


		SUM_SIZE_B += B_dimension[ii*3]*B_dimension[ii*3 + 1]*B_dimension[ii*3 + 2]*sizeof(FLOAT);

	}  //for(len)


	for(int level=interval; level<L_MAX; level++) {

		int L = level - interval;
		/**************************************************************************/
		/* loop conditon */
		//int level = ii + interval;

		if( (FSIZE[level*2]+2*pady < max_Y) || (FSIZE[level*2+1]+2*padx < max_X) ){
			error_array_num++;
			continue;
		}
		/* loop conditon */
		/**************************************************************************/

		for(int jj=0; jj<len; jj++) {

			/* compute size of output */

			int height, width;
			switch(calc_flag) {
			case ROOT:
				height = A_SIZE[level*3] - B_SIZE[jj][0] + 1;
				width = A_SIZE[level*3+1] - B_SIZE[jj][1] + 1;
				break;
			case PART:
				height = A_SIZE[L*3] - B_SIZE[jj][0] + 1;
				width = A_SIZE[L*3+1] - B_SIZE[jj][1] + 1;
				break;
			default:
				printf("NOT DEFINED value: calc_flag = %d\n", calc_flag);
				exit(1);
				break;
			}


			/* search max height and max width */
			for(int i = 0; i < device_num; i++){
				p[i].max_height = (p[i].max_height < height) ? height : p[i].max_height;
				p[i].max_width = (p[i].max_width < width) ? width : p[i].max_width;
			}


			if (height < 1 || width < 1)
			{
				printf("Invalid input: B should be smaller than A\n");
				printf("height %d, width %d\n", height, width);
				exit(0);
			}

			switch(calc_flag){
			case ROOT:
				td[level][jj].C_dims[0]=height;
				td[level][jj].C_dims[1]=width;

				SUM_SIZE_C += td[level][jj].C_dims[0]*td[level][jj].C_dims[1]*sizeof(FLOAT);

				M_size_array[level][jj*2]=height;
				M_size_array[level][jj*2+1]=width;
				break;

			case PART:
				td[L][jj].C_dims[0]=height;
				td[L][jj].C_dims[1]=width;

				SUM_SIZE_C += td[L][jj].C_dims[0]*td[L][jj].C_dims[1]*sizeof(FLOAT);

				M_size_array[L][jj*2]=height;
				M_size_array[L][jj*2+1]=width;
				break;

			default:
				printf("NOT DEFINED value: calc_flag = %d\n", calc_flag);
				exit(1);
				break;
			}


		}
	}

	/* save loop condition */
	res = cuMemHostAlloc((void **)&(error_array), error_array_num*sizeof(int), CU_MEMHOSTALLOC_DEVICEMAP);
	if(res != CUDA_SUCCESS) {
		printf("cuMemHostAlloc(error_array) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	int hh=0;

	if(calc_flag == PART){
		part_error_array = (int *)malloc(error_array_num*sizeof(int));
	}

	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if( (FSIZE[level*2]+2*pady < max_Y) || (FSIZE[level*2+1]+2*padx < max_X) ){ /* if this evaluation formula is TRUE, the level will not be calculated */

			switch(calc_flag){

			case ROOT:
				error_array[hh] = level;
				break;
			case PART:
				part_error_array[hh] = L;
				error_array[hh] = L;
				break;

			default:
				printf("NOT DEFINED value: calc_flag = %d\n", calc_flag);
				exit(1);
				break;
			}


			hh++;
			if(hh > error_array_num) {
				printf("beyond error_array_num!\n");
				exit(1);
			}
		}
	}


	/* allocate output region on CPU memory */
	res = cuMemHostAlloc((void **)&(dst_C), SUM_SIZE_C, CU_MEMHOSTALLOC_DEVICEMAP);
	if(res != CUDA_SUCCESS){
		printf("cuMemHostAlloc(dst_C) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	memset(dst_C, 0, SUM_SIZE_C); //zero clear

	/* distribution */
	unsigned long long int pointer = (unsigned long long int)dst_C;
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;
		for(int jj=0; jj<len; jj++) {
			switch(calc_flag) {
			case ROOT:
				td[level][jj].C = (FLOAT *)pointer;
				pointer += (unsigned long long int)(td[level][jj].C_dims[0]*td[level][jj].C_dims[1]*sizeof(FLOAT));
				break;

			case PART:
				td[L][jj].C = (FLOAT *)pointer;
				pointer += (unsigned long long int)(td[L][jj].C_dims[0]*td[L][jj].C_dims[1]*sizeof(FLOAT));
				break;

			default:
				printf("NOT DEFINED value: calc_flag = %d\n", calc_flag);
				exit(1);
				break;

			}


		}
	}

	if(calc_flag == PART){
		gettimeofday(&tv_fconv_others_end, NULL);
		tvsub(&tv_fconv_others_end, &tv_fconv_others_start, &tv);
		time_fconv_others += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}

	/* start threads */
	if(calc_flag == PART){
		gettimeofday(&tv_fconv_start, NULL);
	}

	CUTThread* threads = (CUTThread *)malloc(sizeof(CUTThread) * device_num);
	for(int i = 0; i < device_num; i++){
		p[i].featp2 = featp2;
		p[i].SUM_SIZE_feat = SUM_SIZE_feat;
		p[i].A_SIZE = A_SIZE;
		p[i].SUM_SIZE_B = SUM_SIZE_B;
		p[i].SUM_SIZE_C = SUM_SIZE_C;
		p[i].dst_C = dst_C;
		p[i].filter = filter;
		p[i].len = len;
		p[i].calc_flag = calc_flag;
		p[i].error_array_num = error_array_num;
		p[i].start = start;
		p[i].error_array = error_array;
		p[i].B_dimension = B_dimension;
		p[i].interval = interval;
		p[i].L_MAX = L_MAX;
		p[i].pid = i;
		threads[i] = cutStartThread((CUT_THREADROUTINE)fconvs_thread_func, (void *)&p[i]);

	}

	cutWaitForThreads(threads, device_num);
	free(threads);


	if(calc_flag == PART){
		gettimeofday(&tv_fconv_end, NULL);
		tvsub(&tv_fconv_end, &tv_fconv_start, &tv);
		time_fconv += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}



	if(calc_flag == PART){
		gettimeofday(&tv_fconv_others_start, NULL);
	}

	/* free CPU memory */
	res = cuMemFreeHost((void *)(error_array));
	if(res != CUDA_SUCCESS) {
		printf("cuMemFreeHost(error_array) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	/* close handle and get output */
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;
		/**************************************************************************/
		/* loop condition */
		if( (FSIZE[level*2]+2*pady < max_Y) || (FSIZE[level*2+1]+2*padx < max_X) )
		{
			continue;
		}
		/* loop conditon */
		/**************************************************************************/
		for(int jj=0; jj<len; jj++) {
			switch(calc_flag){

			case ROOT:
				Output[level][jj] = td[level][jj].C;
				break;

			case PART:
				Output[L][jj] = td[L][jj].C;
				break;

			default:
				printf("NOT DEFINED value: calc_flag = %d\n", calc_flag);
				exit(1);
				break;
			}

		}
	}

	s_free(B_dimension);
	s_free(td[0]);
	s_free(td);
	free(p);

	if(calc_flag == PART){
		gettimeofday(&tv_fconv_others_end, NULL);
		tvsub(&tv_fconv_others_end, &tv_fconv_others_start, &tv);
		time_fconv_others += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
#ifdef PRINT_INFO
		printf("fconv_others_time = %f[ms]\n",time_fconv_others);
		printf("******************************\n");
#endif
	}

	return(Output);

}
