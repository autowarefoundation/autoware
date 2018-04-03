#ifndef GPU_COMMON_H_
#define GPU_COMMON_H_

#include <cuda.h>
#include <cuda_runtime.h>

#define CUDAH __forceinline__ __host__ __device__
#define BLOCK_SIZE_X 1024
#define BLOCK_SIZE_X2 512
#define BLOCK_SIZE_X3 256

#define BLOCK_X 16
#define BLOCK_Y 16
#define BLOCK_Z 4

#define SHARED_MEM_SIZE 3072
#endif

//  This is the temploary patch for CUDA9 build problem 
#if ( __CUDACC_VER_MAJOR__ >=9 )
#undef  __CUDACC_VER__
#define __CUDACC_VER__ 90000 
#endif