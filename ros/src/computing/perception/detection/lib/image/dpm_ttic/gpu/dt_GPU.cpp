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
#include <cstdlib>

#include <cuda_util.hpp>

#include "multithreading.h"
#include "for_use_GPU.h"
#include "dt_GPU.hpp"
#include "fconvsMT.hpp"

struct dt_partition {
	int NoP;
	int NoC;
	int interval;
	int L_MAX;
	int sum_size_SQ;
	int max_numpart;
	int max_dim0;
	int max_dim1;
	FLOAT *dst_M;
	int *dst_tmpIx;
	int *dst_tmpIy;
	int ***PIDX_array;
	int **size_array;
	const int *numpart;
	int *FSIZE;
	int padx;
	int pady;
	int max_X;
	int max_Y;
	FLOAT *def;
	int tmp_array_size;
	int *dst_PIDX;
	int *dst_DID_4;
	int pid;
};

//CUdeviceptr *part_error_array_dev;
//CUdeviceptr *part_C_dev;

static CUT_THREADPROC dt_thread_func(void *p)
{
	dt_partition *pt = (dt_partition *)p;
	struct timeval tv;
	CUresult res;
	int thread_num_x=0, thread_num_y=0;
	int block_num_x=0, block_num_y=0;

	res = cuCtxSetCurrent(ctx[pt->pid]);
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSetCurrent(ctx[%d]) failed: res = %s\n", pt->pid, cuda_response_to_string(res));
		exit(1);
	}

	/* allocate GPU memory */

	//printf("part_error_array_num = %d\n",part_error_array_num);


	if(pt->pid == 0){
		gettimeofday(&tv_memcpy_start, NULL);
	}

	res = cuMemcpyHtoD(part_C_dev[pt->pid], dst_C, SUM_SIZE_C);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(part_C_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(part_error_array_dev[pt->pid], part_error_array, part_error_array_num*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(part_error_array_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(pm_size_array_dev[pt->pid], &pt->size_array[0][0], pt->NoP*2*pt->L_MAX*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(pm_size_array_dev) falied: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(def_array_dev[pt->pid], pt->def, sum_size_def_array);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(def_array_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(numpart_dev[pt->pid], pt->numpart, pt->NoC*sizeof(int));
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(cuMemcpyHtoD(numpart_dev) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(PIDX_array_dev[pt->pid], pt->dst_PIDX, pt->tmp_array_size);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(PIDX_array) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuMemcpyHtoD(DID_4_array_dev[pt->pid], pt->dst_DID_4, pt->tmp_array_size);
	if(res != CUDA_SUCCESS) {
		printf("cuMemcpyHtoD(DID_4__array) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}


	if(pt->pid == 0){
		gettimeofday(&tv_memcpy_end, NULL);
		tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
		time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}

	int sharedMemBytes = 0;

	/* get max thread num per block */
	int max_threads_num = 0;
	res = cuDeviceGetAttribute(&max_threads_num, CU_DEVICE_ATTRIBUTE_MAX_THREADS_PER_BLOCK, dev[pt->pid]);
	if(res != CUDA_SUCCESS){
		printf("\ncuDeviceGetAttribute() failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	/* prepare for launch inverse_Q */
	void* kernel_args_inverse[] = {
		&part_C_dev[pt->pid],
		&pm_size_array_dev[pt->pid],
		&part_error_array_dev[pt->pid],
		&part_error_array_num,
		(void*)&(pt->NoP),
		&PIDX_array_dev[pt->pid],
		&numpart_dev[pt->pid],
		(void*)&(pt->NoC),
		(void*)&(pt->max_numpart),
		(void*)&(pt->interval),
		(void*)&(pt->L_MAX),
		(void*)&(pt->pid),
		(void*)&(device_num)
	};

	/* define CUDA block shape */
	int upper_limit_th_num_x = max_threads_num/(pt->max_numpart*pt->NoC);
	int upper_limit_th_num_y = max_threads_num/upper_limit_th_num_x;
	if(upper_limit_th_num_x < 1) upper_limit_th_num_x++;
	if(upper_limit_th_num_y < 1) upper_limit_th_num_y++;

	thread_num_x = (pt->max_dim0*pt->max_dim1 < upper_limit_th_num_x) ? (pt->max_dim0*pt->max_dim1) : upper_limit_th_num_x;
	thread_num_y = (pt->max_numpart < upper_limit_th_num_y) ? pt->max_numpart : upper_limit_th_num_y;

	block_num_x = (pt->max_dim0*pt->max_dim1) / thread_num_x;
	block_num_y = (pt->max_numpart) / thread_num_y;
	if((pt->max_dim0*pt->max_dim1) % thread_num_x != 0) block_num_x++;
	if(pt->max_numpart % thread_num_y != 0) block_num_y++;


	int blockDimY = thread_num_y / device_num;
	if(thread_num_y%device_num != 0){
		blockDimY++;
	}

	/* launch iverse_Q */
	if(pt->pid == 0){
		gettimeofday(&tv_kernel_start, NULL);
	}
	res = cuLaunchKernel(
		func_inverse_Q[pt->pid],      // call function
		block_num_x,         // gridDimX
		block_num_y,         // gridDimY
		pt->L_MAX-pt->interval,      // gridDimZ
		thread_num_x,        // blockDimX
		blockDimY,        // blockDimY
		pt->NoC,                 // blockDimZ
		sharedMemBytes,      // sharedMemBytes
		NULL,                // hStream
		kernel_args_inverse, // kernelParams
		NULL                 // extra
		);
	if(res != CUDA_SUCCESS) {
		printf("block_num_x %d, block_num_y %d, thread_num_x %d, thread_num_y %d\n", block_num_x, block_num_y, thread_num_x, thread_num_y);
		printf("cuLaunchKernel(inverse_Q) failed : res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuCtxSynchronize();
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSynchronize(inverse_Q) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	if(pt->pid == 0){
		gettimeofday(&tv_kernel_end, NULL);
		tvsub(&tv_kernel_end, &tv_kernel_start, &tv);
		time_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}


	/* prepare for launch dt1d_x */
	void* kernel_args_x[] = {
		&part_C_dev[pt->pid],                  // FLOAT *src_start
		&tmpM_dev[pt->pid],                    // FLOTA *dst
		&tmpIy_dev[pt->pid],                   // int *ptr
		&DID_4_array_dev[pt->pid],             // int *DID_4_array,
		&def_array_dev[pt->pid],               // FLOAT *def_array,
		&pm_size_array_dev[pt->pid],           // int *size_array
		(void*)&(pt->NoP),                  // int NoP
		&PIDX_array_dev[pt->pid],              // int *PIDX_array
		&part_error_array_dev[pt->pid],        // int *error_array
		(void*)&(part_error_array_num), // int error_array_num
		&numpart_dev[pt->pid],                 // int *numpart
		(void*)&(pt->NoC),                  // int NoC
		(void*)&(pt->max_numpart),          // int max_numpart
		(void*)&(pt->interval),             // int interval
		(void*)&(pt->L_MAX),                 // int L_MAX
		(void*)&(pt->pid),                   // int pid
		(void*)&(device_num)                 // int device_num
	};


	max_threads_num = 64/pt->NoC;
	if(max_threads_num < 1) max_threads_num++;

	thread_num_x = (pt->max_dim1 < max_threads_num) ? pt->max_dim1 : max_threads_num;
	thread_num_y = (pt->max_numpart < max_threads_num) ? pt->max_numpart : max_threads_num;

	block_num_x = pt->max_dim1 / thread_num_x;
	block_num_y = pt->max_numpart / thread_num_y;
	if(pt->max_dim1 % thread_num_x != 0) block_num_x++;
	if(pt->max_numpart % thread_num_y != 0) block_num_y++;

	blockDimY = thread_num_y / device_num;
	if(thread_num_y%device_num != 0){
		blockDimY++;
	}

	/* launch dt1d_x */
	if(pt->pid == 0){
		gettimeofday(&tv_kernel_start, NULL);
	}

	res = cuLaunchKernel(
		func_dt1d_x[pt->pid],    // call function
		block_num_x,    // gridDimX
		block_num_y,    // gridDimY
		pt->L_MAX-pt->interval, // gridDimZ
		thread_num_x,   // blockDimX
		blockDimY,   // blockDimY
		pt->NoC,            // blockDimZ
		sharedMemBytes, // sharedMemBytes
		NULL,           // hStream
		kernel_args_x,  // kernelParams
		NULL            // extra
		);
	if(res != CUDA_SUCCESS) {

		printf("block_num_x %d, block_num_y %d, thread_num_x %d, thread_num_y %d\n", block_num_x, block_num_y, thread_num_x, thread_num_y);

		printf("cuLaunchKernel(dt1d_x) failed : res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	res = cuCtxSynchronize();
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSynchronize(dt1d_x) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	if(pt->pid == 0){
		gettimeofday(&tv_kernel_end, NULL);
		tvsub(&tv_kernel_end, &tv_kernel_start, &tv);
		time_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}


	/* prepare for launch dt1d_y */
	void* kernel_args_y[] = {
		&tmpM_dev[pt->pid],                    // FLOAT *src_start
		&M_dev[pt->pid],                       // FLOAT *dst_start
		&tmpIx_dev[pt->pid],                   // int *ptr_start
		&DID_4_array_dev[pt->pid],             // int *DID_4_array,
		&def_array_dev[pt->pid],               // FLOAT *def_array,
		(void*)&(pt->NoP),                  // int NoP
		&pm_size_array_dev[pt->pid],           // int *size_array
		&numpart_dev[pt->pid],                 // int *numpart,
		&PIDX_array_dev[pt->pid],              // int *PIDX_array,
		(void*)&(pt->NoC),                  // int NoC
		(void*)&(pt->max_numpart),          // int max_numpart
		(void*)&(pt->interval),             // int interval
		(void*)&(pt->L_MAX),                // int L_MAX
		&part_error_array_dev[pt->pid],        // int *error_array
		(void*)&(part_error_array_num), // int error_array_num
		(void*)&(pt->pid),                   // int pid
		(void*)&(device_num)                 // int device_num
	};


	thread_num_x = (pt->max_dim0 < max_threads_num) ? pt->max_dim0 : max_threads_num;
	thread_num_y = (pt->max_numpart < max_threads_num) ? pt->max_numpart : max_threads_num;

	block_num_x = pt->max_dim0 / thread_num_x;
	block_num_y = pt->max_numpart / thread_num_y;
	if(pt->max_dim0 % thread_num_x != 0) block_num_x++;
	if(pt->max_numpart % thread_num_y != 0) block_num_y++;

	blockDimY = thread_num_y / device_num;
	if(thread_num_y%device_num != 0){
		blockDimY++;
	}

	/* prepare for launch dt1d_y */
	if(pt->pid == 0){
		gettimeofday(&tv_kernel_start, NULL);
	}

	res = cuLaunchKernel(
		func_dt1d_y[pt->pid],    // call functions
		block_num_x,    // gridDimX
		block_num_y,    // gridDimY
		pt->L_MAX-pt->interval, // gridDimZ
		thread_num_x,   // blockDimX
		blockDimY,   // blockDimY
		pt->NoC,            // blockDimZ
		sharedMemBytes, // sharedMemBytes
		NULL,           // hStream
		kernel_args_y,  // kernelParams
		NULL            // extra
		);
	if(res != CUDA_SUCCESS) {
		printf("cuLaunchKernel(dt1d_y failed : res = %s\n", cuda_response_to_string(res));
		exit(1);
	}


	res = cuCtxSynchronize();
	if(res != CUDA_SUCCESS) {
		printf("cuCtxSynchronize(dt1d_y) failed: res = %s\n", cuda_response_to_string(res));
		exit(1);
	}

	if(pt->pid == 0){
		gettimeofday(&tv_kernel_end, NULL);
		tvsub(&tv_kernel_end, &tv_kernel_start, &tv);
		time_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
	}



	/* downloads datas from GPU */

	/* downloads M from GPU */

	int sum_part_size = 0;
	int sum_pointer_size = 0;
	int sum_move_size = 0;
	int part_size = 0;
	int pointer_size = 0;
	int part_y = 0;
	int move_size = 0;
	int start_kk = 0;
	int end_kk = 0;
	int part_end_kk = 0;
	unsigned long long int pointer_dst_M = (unsigned long long int)pt->dst_M;
	unsigned long long int pointer_M_dev = (unsigned long long int)M_dev[pt->pid];

	for(int L=0; L<(pt->L_MAX-pt->interval); L++) {

		/**************************************************************************/
		/* loop condition */
		if( (pt->FSIZE[(L+pt->interval)*2]+2*pt->pady < pt->max_Y) || (pt->FSIZE[(L+pt->interval)*2+1]+2*pt->padx < pt->max_X) )
		{
			continue;
		}
		/* loop conditon */
		/**************************************************************************/


		for(int jj=0; jj<pt->NoC; jj++) {

			part_y = pt->numpart[jj] / device_num;
			if(pt->numpart[jj]%device_num != 0){
				part_y++;
			}


			start_kk = part_y * pt->pid;
			end_kk = part_y * (pt->pid + 1);

			if(end_kk > pt->numpart[jj]){
				end_kk = pt->numpart[jj];
			}

			if(pt->pid > 0){
				part_end_kk = part_y * pt->pid;
			}

			for(int kk=0; kk<pt->numpart[jj]; kk++) {

				int PIDX = pt->PIDX_array[L][jj][kk];
				int dims0 = pt->size_array[L][PIDX*2];
				int dims1 = pt->size_array[L][PIDX*2+1];
				if(start_kk <= kk && kk < end_kk){
					part_size += dims0 * dims1;
				}
				//if(pt->pid > 0 && part_start_kk <= kk && kk < part_end_kk){
				if(pt->pid > 0 && 0 <= kk && kk < part_end_kk){
					pointer_size += dims0 * dims1;
				}
				move_size += dims0 * dims1;
			}

			sum_part_size += part_size;
			sum_pointer_size += pointer_size;
			sum_move_size += move_size;

			// error pt->pid == 2 && L == 24 && jj == 1

			if(pt->pid*part_y < pt->numpart[jj]){

				if(pt->pid == 0){
					gettimeofday(&tv_memcpy_start, NULL);
				}


				res = cuMemcpyDtoH((void *)(pointer_dst_M+(unsigned long long int)(pointer_size*sizeof(FLOAT))), (CUdeviceptr)(pointer_M_dev+(unsigned long long int)(pointer_size*sizeof(FLOAT))), part_size*sizeof(FLOAT));
				if(res != CUDA_SUCCESS) {
					printf("error pid = %d\n",pt->pid);
					printf("cuMemcpyDtoH(dst_M) failed: res = %s\n", cuda_response_to_string(res));
					exit(1);
				}

				if(pt->pid == 0){
					gettimeofday(&tv_memcpy_end, NULL);
					tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
					time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
				}

			}

			pointer_dst_M += (unsigned long long int)(move_size * sizeof(FLOAT));
			pointer_M_dev += (unsigned long long int)(move_size * sizeof(FLOAT));

			part_size = 0;
			pointer_size = 0;
			move_size = 0;

		}

	}


	/* downloads tmpIx from GPU */

	sum_part_size = 0;
	sum_pointer_size = 0;
	part_size = 0;
	pointer_size = 0;
	part_y = 0;
	move_size = 0;
	start_kk = 0;
	end_kk = 0;
	part_end_kk = 0;
	unsigned long long int pointer_dst_tmpIx = (unsigned long long int)pt->dst_tmpIx;
	unsigned long long int pointer_tmpIx_dev = (unsigned long long int)tmpIx_dev[pt->pid];


	for(int L=0; L<(pt->L_MAX-pt->interval); L++) {

		/**************************************************************************/
		/* loop condition */
		if( (pt->FSIZE[(L+pt->interval)*2]+2*pt->pady < pt->max_Y) || (pt->FSIZE[(L+pt->interval)*2+1]+2*pt->padx < pt->max_X) )
		{
			continue;
		}
		/* loop conditon */
		/**************************************************************************/


		for(int jj=0; jj<pt->NoC; jj++) {

			part_y = pt->numpart[jj] / device_num;
			if(pt->numpart[jj]%device_num != 0){
				part_y++;
			}

			start_kk = part_y * pt->pid;
			end_kk = part_y * (pt->pid + 1);

			if(end_kk > pt->numpart[jj]){
				end_kk = pt->numpart[jj];
			}

			if(pt->pid > 0){
				part_end_kk = part_y * pt->pid;
			}

			for(int kk=0; kk<pt->numpart[jj]; kk++) {

				int PIDX = pt->PIDX_array[L][jj][kk];
				int dims0 = pt->size_array[L][PIDX*2];
				int dims1 = pt->size_array[L][PIDX*2+1];
				if(start_kk <= kk && kk < end_kk){
					part_size += dims0 * dims1;
				}
				if(pt->pid > 0){
					if(0 <= kk && kk < part_end_kk){
						pointer_size += dims0 * dims1;
					}
				}
				move_size += dims0 * dims1;
			}

			sum_part_size += part_size;
			sum_pointer_size += pointer_size;


			if(pt->pid*part_y < pt->numpart[jj]){

				if(pt->pid == 0){
					gettimeofday(&tv_memcpy_start, NULL);
				}


				res = cuMemcpyDtoH((void *)(pointer_dst_tmpIx+(unsigned long long int)(pointer_size*sizeof(int))), (CUdeviceptr)(pointer_tmpIx_dev+(unsigned long long int)(pointer_size*sizeof(int))), part_size*sizeof(int));
				if(res != CUDA_SUCCESS) {
					printf("error pid = %d\n",pt->pid);
					printf("cuMemcpyDtoH(tmpIx) failed: res = %s\n", cuda_response_to_string(res));
					exit(1);
				}

				if(pt->pid == 0){
					gettimeofday(&tv_memcpy_end, NULL);
					tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
					time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
				}

			}

			pointer_dst_tmpIx += (unsigned long long int)(move_size * sizeof(int));
			pointer_tmpIx_dev += (unsigned long long int)(move_size * sizeof(int));

			part_size = 0;
			pointer_size = 0;
			move_size = 0;

		}

	}


	/* downloads tmpIy from GPU */

	sum_part_size = 0;
	sum_pointer_size = 0;
	part_size = 0;
	pointer_size = 0;
	part_y = 0;
	move_size = 0;
	start_kk = 0;
	end_kk = 0;
	part_end_kk = 0;
	unsigned long long int pointer_dst_tmpIy = (unsigned long long int)pt->dst_tmpIy;
	unsigned long long int pointer_tmpIy_dev = (unsigned long long int)tmpIy_dev[pt->pid];


	for(int L=0; L<(pt->L_MAX-pt->interval); L++) {

		/**************************************************************************/
		/* loop condition */
		if( (pt->FSIZE[(L+pt->interval)*2]+2*pt->pady < pt->max_Y) || (pt->FSIZE[(L+pt->interval)*2+1]+2*pt->padx < pt->max_X) )
		{
			continue;
		}
		/* loop conditon */
		/**************************************************************************/


		for(int jj=0; jj<pt->NoC; jj++) {

			part_y = pt->numpart[jj] / device_num;
			if(pt->numpart[jj]%device_num != 0){
				part_y++;
			}

			start_kk = part_y * pt->pid;
			end_kk = part_y * (pt->pid + 1);

			if(end_kk > pt->numpart[jj]){
				end_kk = pt->numpart[jj];
			}

			if(pt->pid > 0){
				part_end_kk = part_y * pt->pid;
			}

			for(int kk=0; kk<pt->numpart[jj]; kk++) {

				int PIDX = pt->PIDX_array[L][jj][kk];
				int dims0 = pt->size_array[L][PIDX*2];
				int dims1 = pt->size_array[L][PIDX*2+1];
				if(start_kk <= kk && kk < end_kk){
					part_size += dims0 * dims1;
				}
				if(pt->pid > 0){
					if(0 <= kk && kk < part_end_kk){
						pointer_size += dims0 * dims1;
					}
				}
				move_size += dims0 * dims1;
			}

			sum_part_size += part_size;
			sum_pointer_size += pointer_size;

			if(pt->pid*part_y < pt->numpart[jj]){

				if(pt->pid == 0){
					gettimeofday(&tv_memcpy_start, NULL);
				}

				res = cuMemcpyDtoH((void *)(pointer_dst_tmpIy+(unsigned long long int)(pointer_size*sizeof(int))), (CUdeviceptr)(pointer_tmpIy_dev+(unsigned long long int)(pointer_size*sizeof(int))), part_size*sizeof(int));
				if(res != CUDA_SUCCESS) {
					printf("error pid = %d\n",pt->pid);
					printf("cuMemcpyDtoH(tmpIy) failed: res = %s\n", cuda_response_to_string(res));
					exit(1);
				}

				if(pt->pid == 0){
					gettimeofday(&tv_memcpy_end, NULL);
					tvsub(&tv_memcpy_end, &tv_memcpy_start, &tv);
					time_memcpy += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
				}

			}

			pointer_dst_tmpIy += (unsigned long long int)(move_size * sizeof(int));
			pointer_tmpIy_dev += (unsigned long long int)(move_size * sizeof(int));

			part_size = 0;
			pointer_size = 0;
			move_size = 0;

		}

	}


	/* end of thread */
	CUT_THREADEND;
}

FLOAT ****dt_GPU(
  int ****Ix_array,
  int ****Iy_array,
  int ***PIDX_array,
  int **size_array,
  int NoP,
  const int *numpart,
  int NoC,
  int interval,
  int L_MAX,
  int *FSIZE,
  int padx,
  int pady,
  int max_X,
  int max_Y,
  FLOAT *def,
  int tmp_array_size,
  int *dst_PIDX,
  int *dst_DID_4
               )
{
	dt_partition *p = (dt_partition *)malloc(device_num*sizeof(dt_partition));

	int max_dim0 = 0, max_dim1 = 0;
	/* prepare for parallel execution */
	int sum_size_SQ = 0;
	int sum_numpart = 0;

	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if( (FSIZE[level*2]+2*pady < max_Y) || (FSIZE[level*2+1]+2*padx < max_X) )
			continue;

		for(int jj=0; jj<NoC; jj++) {
			for(int kk=0; kk<numpart[jj]; kk++) {
				int PIDX = PIDX_array[L][jj][kk];
				int dims0 = size_array[L][PIDX*2];
				int dims1 = size_array[L][PIDX*2+1];

				sum_size_SQ += dims0*dims1;

				/* search max values */
				max_dim0 = (max_dim0 < dims0) ? dims0 : max_dim0;
				max_dim1 = (max_dim1 < dims1) ? dims1 : max_dim1;
			}
			sum_numpart += numpart[jj];
		}
	}

	/* allocate region each array in a lump */
	FLOAT ****M_array = (FLOAT ****)malloc((L_MAX-interval)*sizeof(FLOAT***));
	FLOAT ***sub_sub_dst_M = (FLOAT ***)malloc(NoC*(L_MAX-interval)*sizeof(FLOAT**));
	FLOAT **sub_dst_M = (FLOAT **)malloc(sum_numpart*sizeof(FLOAT*));
	FLOAT *dst_M =  (FLOAT *)malloc(sum_size_SQ*sizeof(FLOAT));

	FLOAT ****tmpM_array = (FLOAT ****)malloc((L_MAX-interval)*sizeof(FLOAT***));
	FLOAT ***sub_sub_dst_tmpM = (FLOAT ***)malloc(NoC*(L_MAX-interval)*sizeof(FLOAT**));
	FLOAT **sub_dst_tmpM = (FLOAT **)malloc(sum_numpart*sizeof(FLOAT*));
	FLOAT *dst_tmpM = (FLOAT *)malloc(sum_size_SQ*sizeof(FLOAT));

	int ****tmpIx_array = (int ****)malloc((L_MAX-interval)*sizeof(int***));
	int ***sub_sub_dst_tmpIx = (int ***)malloc(NoC*(L_MAX-interval)*sizeof(int**));
	int **sub_dst_tmpIx = (int **)malloc(sum_numpart*sizeof(int*));
	int *dst_tmpIx = (int *)malloc(sum_size_SQ*sizeof(int));

	int ****tmpIy_array = (int ****)malloc((L_MAX-interval)*sizeof(int***));
	int ***sub_sub_dst_tmpIy = (int ***)malloc(NoC*(L_MAX-interval)*sizeof(int**));
	int **sub_dst_tmpIy = (int **)malloc(sum_numpart*sizeof(int*));
	int *dst_tmpIy = (int *)malloc(sum_size_SQ*sizeof(int));

	/* distribute allocated region */
	unsigned long long int pointer_M = (unsigned long long int)sub_sub_dst_M;
	unsigned long long int pointer_tmpM = (unsigned long long int)sub_sub_dst_tmpM;
	unsigned long long int pointer_tmpIx = (unsigned long long int)sub_sub_dst_tmpIx;
	unsigned long long int pointer_tmpIy = (unsigned long long int)sub_sub_dst_tmpIy;
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if( (FSIZE[level*2]+2*pady < max_Y) || (FSIZE[level*2+1]+2*padx < max_X) )
		{
			continue;
		}

		M_array[L] = (FLOAT ***)pointer_M;
		pointer_M += (unsigned long long int)(NoC*sizeof(FLOAT**));

		tmpM_array[L] = (FLOAT ***)pointer_tmpM;
		pointer_tmpM += (unsigned long long int)(NoC*sizeof(FLOAT**));

		tmpIx_array[L] = (int ***)pointer_tmpIx;
		pointer_tmpIx += (unsigned long long int)(NoC*sizeof(int**));

		tmpIy_array[L] = (int ***)pointer_tmpIy;
		pointer_tmpIy += (unsigned long long int)(NoC*sizeof(int**));
	}

	pointer_M = (unsigned long long int)sub_dst_M;
	pointer_tmpM = (unsigned long long int)sub_dst_tmpM;
	pointer_tmpIx = (unsigned long long int)sub_dst_tmpIx;
	pointer_tmpIy = (unsigned long long int)sub_dst_tmpIy;
	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;
		if( (FSIZE[level*2]+2*pady < max_Y) || (FSIZE[level*2+1]+2*padx < max_X) )
		{
			continue;
		}

		for(int jj=0; jj<NoC; jj++) {
			int numpart_jj = numpart[jj];

			M_array[L][jj] = (FLOAT **)pointer_M;
			pointer_M += (unsigned long long int)(numpart_jj*sizeof(FLOAT*));

			tmpM_array[L][jj] = (FLOAT **)pointer_tmpM;
			pointer_tmpM += (unsigned long long int)(numpart_jj*sizeof(FLOAT*));

			tmpIx_array[L][jj] = (int **)pointer_tmpIx;
			pointer_tmpIx += (unsigned long long int)(numpart_jj*sizeof(int*));

			tmpIy_array[L][jj] = (int **)pointer_tmpIy;
			pointer_tmpIy += (unsigned long long int)(numpart_jj*sizeof(int*));
		}
	}

	pointer_M = (unsigned long long int)dst_M;
	pointer_tmpM = (unsigned long long int)dst_tmpM;
	pointer_tmpIx = (unsigned long long int)dst_tmpIx;
	pointer_tmpIy = (unsigned long long int)dst_tmpIy;

	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		if( (FSIZE[level*2]+2*pady < max_Y) || (FSIZE[level*2+1]+2*padx < max_X) )
		{
			continue;
		}

		for(int jj=0; jj<NoC; jj++) {
			for(int kk=0; kk<numpart[jj]; kk++) {

				int PIDX = PIDX_array[L][jj][kk];
				int dims0 = size_array[L][PIDX*2];
				int dims1 = size_array[L][PIDX*2+1];

				M_array[L][jj][kk] = (FLOAT *)pointer_M;
				pointer_M += (unsigned long long int)(dims0*dims1*sizeof(FLOAT));

				tmpM_array[L][jj][kk] = (FLOAT *)pointer_tmpM;
				pointer_tmpM += (unsigned long long int)(dims0*dims1*sizeof(FLOAT));

				tmpIx_array[L][jj][kk] = (int *)pointer_tmpIx;
				pointer_tmpIx += (unsigned long long int)(dims0*dims1*sizeof(int));

				tmpIy_array[L][jj][kk] = (int *)pointer_tmpIy;
				pointer_tmpIy += (unsigned long long int)(dims0*dims1*sizeof(int));

			}
		}
	}

	/* start threads */
	CUTThread* threads = (CUTThread *)malloc(sizeof(CUTThread) * device_num);

	for(int i = 0; i < device_num; i++){
		p[i].NoP = NoP;
		p[i].NoC = NoC;
		p[i].interval = interval;
		p[i].L_MAX = L_MAX;
		p[i].sum_size_SQ = sum_size_SQ;
		p[i].max_numpart = max_numpart;
		p[i].max_dim0 = max_dim0;
		p[i].max_dim1 = max_dim1;
		p[i].dst_M = dst_M;
		p[i].dst_tmpIx = dst_tmpIx;
		p[i].dst_tmpIy = dst_tmpIy;
		p[i].PIDX_array = PIDX_array;
		p[i].size_array = size_array;
		p[i].numpart = numpart;
		p[i].FSIZE = FSIZE;
		p[i].padx = padx;
		p[i].pady = pady;
		p[i].max_X = max_X;
		p[i].max_Y = max_Y;
		p[i].def = def;
		p[i].tmp_array_size = tmp_array_size;
		p[i].dst_PIDX = dst_PIDX;
		p[i].dst_DID_4 = dst_DID_4;
		p[i].pid = i;
		threads[i] = cutStartThread((CUT_THREADROUTINE)dt_thread_func, (void *)&p[i]);
	}

	cutWaitForThreads(threads, device_num);

	free(threads);
	free(p);

	for(int level=interval; level<L_MAX; level++) {
		int L = level - interval;

		/* loop condition */
		if( (FSIZE[level*2]+2*pady < max_Y) || (FSIZE[level*2+1]+2*padx < max_X) )
		{
			continue;
		}

		for(int jj=0; jj<NoC; jj++) {
			for(int kk=0; kk<numpart[jj]; kk++) {

				int *IX_P = Ix_array[L][jj][kk];
				int *IY_P = Iy_array[L][jj][kk];
				int *tmpIx_P=tmpIx_array[L][jj][kk];

				int PIDX = PIDX_array[L][jj][kk];
				int dims0 = size_array[L][PIDX*2];
				int dims1 = size_array[L][PIDX*2+1];

				for (int x = 0; x < dims1; x++)
				{
					for (int y = 0; y < dims0; y++)
					{
						*(IX_P++) = *tmpIx_P;
						*(IY_P++) = tmpIy_array[L][jj][kk][(*tmpIx_P)*(dims0)+y];
						tmpIx_P++;       // increment address
					}
				}
			}
		}
	}

	/* free CPU memory */
	free(part_error_array);
	free(dst_tmpM);
	free(sub_dst_tmpM);
	free(sub_sub_dst_tmpM);
	free(tmpM_array);

	free(dst_tmpIx);
	free(sub_dst_tmpIx);
	free(sub_sub_dst_tmpIx);
	free(tmpIx_array);

	free(dst_tmpIy);
	free(sub_dst_tmpIy);
	free(sub_sub_dst_tmpIy);
	free(tmpIy_array);

	return(M_array);
}
