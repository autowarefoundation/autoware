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

#ifndef _CUDA_H
#define _CUDA_H
#include <cuda.h>
#endif

#include "MODEL_info.h"

#ifndef _SWITCH_FLOAT_H
#define _SWITCH_FLOAT_H
#include "switch_float.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* define variables for using GPU */

extern CUdevice *dev;
extern CUcontext *ctx;
extern CUfunction *func_process_root, *func_process_part, *func_dt1d_x, *func_dt1d_y, *func_calc_a_score, *func_inverse_Q;
extern CUmodule *module;
extern int *NR_MAXTHREADS_X, *NR_MAXTHREADS_Y;
extern CUdeviceptr *featp2_dev;
extern CUdeviceptr *A_SIZE_dev;
extern CUdeviceptr *B_dev;
extern CUdeviceptr *B_dims_dev;
extern CUdeviceptr *fconvs_error_array_dev;
extern CUdeviceptr *fconvs_C_dev;
extern CUdeviceptr *part_C_dev;
extern CUdeviceptr *part_error_array_dev;
extern CUdeviceptr *M_dev;
extern CUdeviceptr *tmpM_dev;
extern CUdeviceptr *tmpIx_dev;
extern CUdeviceptr *tmpIy_dev;
extern CUdeviceptr *pm_size_array_dev;
extern CUdeviceptr *PIDX_array_dev;
extern CUdeviceptr *def_array_dev;
extern int sum_size_def_array;
extern CUdeviceptr *DID_4_array_dev;
extern CUdeviceptr *numpart_dev;
extern int max_numpart;
extern int device_num;

/* definition of calc_flag */
#define ROOT 0
#define PART 1

/* switch define sentence  which use original source or GPU function */
//#define ORIGINAL
#ifdef __cplusplus
}
#endif
