/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
