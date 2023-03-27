/*
 * MIT License
 *
 * Copyright (c) 2021 Yifu Zhang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LAPJV_H_
#define LAPJV_H_

#define LARGE 1000000

#if !defined TRUE
#define TRUE 1
#endif
#if !defined FALSE
#define FALSE 0
#endif

#define NEW(x, t, n)                                               \
  if ((x = reinterpret_cast<t *>(malloc(sizeof(t) * (n)))) == 0) { \
    return -1;                                                     \
  }
#define FREE(x) \
  if (x != 0) { \
    free(x);    \
    x = 0;      \
  }
#define SWAP_INDICES(a, b) \
  {                        \
    int_t _temp_index = a; \
    a = b;                 \
    b = _temp_index;       \
  }

#if 0
#include <assert.h>
#define ASSERT(cond) assert(cond)
#define PRINTF(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define PRINT_COST_ARRAY(a, n)         \
  while (1) {                          \
    printf(#a " = [");                 \
    if ((n) > 0) {                     \
      printf("%f", (a)[0]);            \
      for (uint_t j = 1; j < n; j++) { \
        printf(", %f", (a)[j]);        \
      }                                \
    }                                  \
    printf("]\n");                     \
    break;                             \
  }
#define PRINT_INDEX_ARRAY(a, n)        \
  while (1) {                          \
    printf(#a " = [");                 \
    if ((n) > 0) {                     \
      printf("%d", (a)[0]);            \
      for (uint_t j = 1; j < n; j++) { \
        printf(", %d", (a)[j]);        \
      }                                \
    }                                  \
    printf("]\n");                     \
    break;                             \
  }
#else
#define ASSERT(cond)
#define PRINTF(fmt, ...)
#define PRINT_COST_ARRAY(a, n)
#define PRINT_INDEX_ARRAY(a, n)
#endif

typedef signed int int_t;
typedef unsigned int uint_t;
typedef double cost_t;
typedef char boolean;
typedef enum fp_t { FP_1 = 1, FP_2 = 2, FP_DYNAMIC = 3 } fp_t;

extern int_t lapjv_internal(const uint_t n, cost_t * cost[], int_t * x, int_t * y);

#endif  // LAPJV_H_
