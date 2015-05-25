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

/* typedef to switch float and double */
#define USE_FLOAT_AS_DECIMAL

#ifdef USE_FLOAT_AS_DECIMAL
typedef float FLOAT;
#ifdef FROM_GPU
#define sqrt sqrtf
#endif
#else
typedef double FLOAT;
#endif

#ifndef TVSUB
#define TVSUB

/* for measurement */
#include <sys/time.h>
/* tvsub: ret = x - y. */
static inline void tvsub
(
 struct timeval *x,
 struct timeval *y,
 struct timeval *ret
 )
{
  ret->tv_sec = x->tv_sec - y->tv_sec;
  ret->tv_usec = x->tv_usec - y->tv_usec;
  if (ret->tv_usec < 0) {
    ret->tv_sec--;
    ret->tv_usec += 1000000;
  }
}
/* for measurement */
#endif

extern struct timeval tv_memcpy_start, tv_memcpy_end;
extern float time_memcpy;
extern struct timeval tv_kernel_start, tv_kernel_end;
extern float time_kernel;
