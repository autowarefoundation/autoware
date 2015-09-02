#ifndef _CUDA_CHECK_H_
#define _CUDA_CHECK_H_

#include <cuda.h>
#include "drvapi_error_string.h"

/* error handling macro */
#define CUDA_CHECK(res, text)                \
  if ((res) != CUDA_SUCCESS) {                  \
    printf("%s failed: res = %d\n->%s\n", (text), (res), getCudaDrvErrorString((res))); \
  exit(1);                                      \
  }

#endif // _CUDA_CHECK_H_
