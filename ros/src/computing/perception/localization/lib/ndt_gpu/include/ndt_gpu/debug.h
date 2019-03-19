#ifndef GDEBUG_H_
#define GDEBUG_H_

#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

inline void gassert(cudaError_t err_code, const char *file, int line)
{
	if (err_code != cudaSuccess) {
		fprintf(stderr, "Error: %s %s %d\n", cudaGetErrorString(err_code), file, line);
		cudaDeviceReset();
		exit(EXIT_FAILURE);
	}
}

#define checkCudaErrors(err_code) gassert(err_code, __FILE__, __LINE__)

#define timeDiff(start, end) ((end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec)

#endif
