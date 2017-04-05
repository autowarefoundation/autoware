#ifndef _CUDA_CHECK_H_
#define _CUDA_CHECK_H_

#include <cstdio>
#include <cstdlib>
#include <cuda.h>
#include "drvapi_error_string.h"

/* error handling macro */
#define CUDA_CHECK(res, fmt, ...)							\
do {											\
	if ((res) != CUDA_SUCCESS) {							\
		printf("[%s:%s:%d] Failed: %s\n" fmt "\n",				\
		       __FILE__, __func__, __LINE__, getCudaDrvErrorString((res)),	\
		       ##__VA_ARGS__);							\
		std::exit(1);								\
	}										\
} while(0)

#endif // _CUDA_CHECK_H_
