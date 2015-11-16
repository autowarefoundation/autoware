#ifndef _CUDA_CHECK_H_
#define _CUDA_CHECK_H_

#include <cuda.h>
#include "drvapi_error_string.h"

/* error handling macro */
#define CUDA_CHECK(res, text)									\
do {												\
	if ((res) != CUDA_SUCCESS) {								\
		printf("[%s:%s:%d] %s failed: res = %d\n->%s\n",				\
		       __FILE__, __func__, __LINE__,						\
		       (text), (res), getCudaDrvErrorString((res)));				\
		exit(1);									\
	}											\
} while(0)


#endif // _CUDA_CHECK_H_
