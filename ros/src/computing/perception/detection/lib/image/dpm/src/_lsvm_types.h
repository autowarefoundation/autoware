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

#ifndef SVM_TYPE
#define SVM_TYPE

#include "float.h"

//#define FFT_CONV

#define PI    CV_PI

#define EPS 0.000001

#define F_MAX FLT_MAX
#define F_MIN -FLT_MAX

// The number of elements in bin
// The number of sectors in gradient histogram building
#define NUM_SECTOR 9// ORIGINAL 9

// The number of levels in image resize procedure
// We need Lambda levels to resize image twice
#define LAMBDA 10 // ORIGINAL 10

// Block size. Used in feature pyramid building procedure
#define SIDE_LENGTH 8 //ORIGINAL 8

#define VAL_OF_TRUNCATE 0.2f

//////////////////////////////////////////////////////////////
// main data structures                                     //
//////////////////////////////////////////////////////////////

// DataType: STRUCT featureMap
// FEATURE MAP DESCRIPTION
//   Rectangular map (sizeX x sizeY),
//   every cell stores feature vector (dimension = numFeatures)
// map             - matrix of feature vectors
//                   to set and get feature vectors (i,j)
//                   used formula map[(j * sizeX + i) * p + k], where
//                   k - component of feature vector in cell (i, j)
typedef struct{
    int sizeX;
    int sizeY;
    int numFeatures;
    float *map;
} CvLSVMFeatureMap;

// DataType: STRUCT featurePyramid
//
// numLevels    - number of levels in the feature pyramid
// pyramid      - array of pointers to feature map at different levels
typedef struct{
    int numLevels;
    CvLSVMFeatureMap **pyramid;
} CvLSVMFeaturePyramid;

// DataType: STRUCT filterDisposition
// The structure stores preliminary results in optimization process
// with objective function D
//
// x            - array with X coordinates of optimization problems solutions
// y            - array with Y coordinates of optimization problems solutions
// score        - array with optimal objective values
typedef struct{
    float *score;
    int *x;
    int *y;
} CvLSVMFilterDisposition;

// DataType: STRUCT fftImage
// The structure stores FFT image
//
// numFeatures  - number of channels
// x            - array of FFT images for 2d signals
// n            - number of rows
// m            - number of collums
typedef struct{
    int numFeatures;
    int dimX;
    int dimY;
    float **channels;
} CvLSVMFftImage;

#endif
