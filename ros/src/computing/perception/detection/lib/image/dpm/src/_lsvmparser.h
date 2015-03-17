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

#ifndef LSVM_PARSER
#define LSVM_PARSER

#include "_lsvm_types.h"

#define MODEL    1
#define P        2
#define COMP     3
#define SCORE    4
#define RFILTER  100
#define PFILTERs 101
#define PFILTER  200
#define SIZEX    150
#define SIZEY    151
#define WEIGHTS  152
#define TAGV     300
#define Vx       350
#define Vy       351
#define TAGD     400
#define Dx       451
#define Dy       452
#define Dxx      453
#define Dyy      454
#define BTAG     500

#define STEP_END 1000

#define EMODEL    (STEP_END + MODEL)
#define EP        (STEP_END + P)
#define ECOMP     (STEP_END + COMP)
#define ESCORE    (STEP_END + SCORE)
#define ERFILTER  (STEP_END + RFILTER)
#define EPFILTERs (STEP_END + PFILTERs)
#define EPFILTER  (STEP_END + PFILTER)
#define ESIZEX    (STEP_END + SIZEX)
#define ESIZEY    (STEP_END + SIZEY)
#define EWEIGHTS  (STEP_END + WEIGHTS)
#define ETAGV     (STEP_END + TAGV)
#define EVx       (STEP_END + Vx)
#define EVy       (STEP_END + Vy)
#define ETAGD     (STEP_END + TAGD)
#define EDx       (STEP_END + Dx)
#define EDy       (STEP_END + Dy)
#define EDxx      (STEP_END + Dxx)
#define EDyy      (STEP_END + Dyy)
#define EBTAG     (STEP_END + BTAG)

//extern "C" {
    int LSVMparser(const char * filename, CvLSVMFilterObject *** model, int *last, int *max,
                   int **comp, float **b, int *count, float * score);
#ifdef __cplusplus
extern "C"
#endif
    int loadModel(

              const char *modelPath,

              CvLSVMFilterObject ***filters,
              int *kFilters,
              int *kComponents,
              int **kPartFilters,
              float **b,
              float *scoreThreshold);
//};
#endif
