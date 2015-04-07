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

#ifndef _LSVM_TBBVERSION_H
#define _LSVM_TBBVERSION_H

#include "_lsvm_matching.h"

/*
// Computation score function using TBB tasks
//
// API
// int tbbTasksThresholdFunctionalScore(const CvLSVMFilterObject **filters, const int n,
                                        const CvLSVMFeaturePyramid *H, const float b,
                                        const int maxXBorder, const int maxYBorder,
                                        const float scoreThreshold,
                                        int *kLevels, int **procLevels,
                                        const int threadsNum,
                                        float **score, CvPoint ***points,
                                        int *kPoints,
                                        CvPoint ****partsDisplacement);
// INPUT
// filters           - the set of filters (the first element is root filter,
                       the other - part filters)
// n                 - the number of part filters
// H                 - feature pyramid
// b                 - linear term of the score function
// maxXBorder        - the largest root filter size (X-direction)
// maxYBorder        - the largest root filter size (Y-direction)
// scoreThreshold    - score threshold
// kLevels           - array that contains number of levels processed
                       by each thread
// procLevels        - array that contains lists of levels processed
                       by each thread
// threadsNum        - the number of created threads
// OUTPUT
// score             - score function values that exceed threshold
// points            - the set of root filter positions (in the block space)
// kPoints           - number of root filter positions
// partsDisplacement - displacement of part filters (in the block space)
// RESULT
//
*/
int tbbTasksThresholdFunctionalScore(const CvLSVMFilterObject **filters, const int n,
                                     const CvLSVMFeaturePyramid *H, const float b,
                                     const int maxXBorder, const int maxYBorder,
                                     const float scoreThreshold,
                                     int *kLevels, int **procLevels,
                                     const int threadsNum,
                                     float **score, CvPoint ***points,
                                     int *kPoints,
                                     CvPoint ****partsDisplacement);

#endif
