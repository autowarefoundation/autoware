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

#ifndef _SWITCH_FLOAT_H
#define _SWITCH_FLOAT_H
#include "switch_float.h"
#endif

#ifndef _MODEL_INFO
#define _MODEL_INFO
//struct for model component information
struct Model_info {

    //basic information
    //from xxxcomp.csv
    int numcomponent;           //number of component
    int sbin;                   //cell size
    int interval;               //interval (for hierachical detection)
    int max_X;
    int max_Y;
    //from calculation
    int padx;                   //pad information
    int pady;
    int max_scale;
    //image size information
    int IM_WIDTH;
    int IM_HEIGHT;

//per root
    int *ridx;                  //root index information
    int *oidx;                  //offsetindex information
    FLOAT *offw;                //offset weight
    int *rsize;                 //root size
    int *numpart;               //number of part filter per component

//per part
    int **pidx;                 //part index information
    int **didx;                 //define index of part
    int **psize;

//defs
    FLOAT *def;                 //defs
    int *anchor;                //anchor

//least_square info
    FLOAT **x1;
    FLOAT **y1;
    FLOAT **x2;
    FLOAT **y2;

    bool ini;                   //flag for initialization
    FLOAT ratio;                //ratio of zooming image

};

//struct for root_filter_information
struct Rootfilters {
    int NoR;                    //number of root filter
    int **root_size;            //size of root filter
    FLOAT **rootfilter;         //weight of root filter
    int *rootsym;               //symmetric information
};

//struct for part_filter_information
struct Partfilters {
    int NoP;                    //number of part filter
    int **part_size;            //size of part filter
    FLOAT **partfilter;         //weight of root filter
    int *part_partner;          //symmetric-partner information
    int *part_sym;              //symmetric information of part filter
};


//model information
struct MODEL {
    Model_info *MI;
    Rootfilters *RF;
    Partfilters *PF;
};
#endif



#ifdef __cplusplus
extern "C" {
#endif


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
extern int part_error_array_num;
extern CUdeviceptr *pm_size_array_dev;
extern CUdeviceptr *PIDX_array_dev;
extern CUdeviceptr *def_array_dev;
extern int sum_size_def_array;
extern CUdeviceptr *DID_4_array_dev;
extern CUdeviceptr *numpart_dev;
extern int max_numpart;
extern int device_num;
extern int *part_error_array;
extern size_t SUM_SIZE_C;
extern FLOAT *dst_C;

/* functions for using GPU and to calculate on GPU */
extern void init_cuda(void);
extern void init_cuda_with_cubin(const char *cubin_path);

extern void clean_cuda(void);

/* function for GPU execution correspond to fconvsMT */
extern
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
    );

/* definition of calc_flag */
#define ROOT 0
#define PART 1

extern
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
    );



/* switch define sentence  which use original source or GPU function */
//#define ORIGINAL

//#define SEPARETE_MEM



#ifdef __cplusplus
}
#endif
