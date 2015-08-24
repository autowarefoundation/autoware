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

#ifndef _LSVM_FFT_H_
#define _LSVM_FFT_H_

#include "_lsvm_types.h"
#include "_lsvm_error.h"
#include <math.h>

/*
// 1-dimensional FFT
//
// API
// int fft(float *x_in, float *x_out, int n, int shift);
// INPUT
// x_in              - input signal
// n                 - number of elements for searching Fourier image
// shift             - shift between input elements
// OUTPUT
// x_out             - output signal (contains 2n elements in order
                       Re(x_in[0]), Im(x_in[0]), Re(x_in[1]), Im(x_in[1]) and etc.)
// RESULT
// Error status
*/
int fft(float *x_in, float *x_out, int n, int shift);

/*
// Inverse 1-dimensional FFT
//
// API
// int fftInverse(float *x_in, float *x_out, int n, int shift);
// INPUT
// x_in              - Fourier image of 1d input signal(contains 2n elements
                       in order Re(x_in[0]), Im(x_in[0]),
                       Re(x_in[1]), Im(x_in[1]) and etc.)
// n                 - number of elements for searching counter FFT image
// shift             - shift between input elements
// OUTPUT
// x_in              - input signal (contains n elements)
// RESULT
// Error status
*/
int fftInverse(float *x_in, float *x_out, int n, int shift);

/*
// 2-dimensional FFT
//
// API
// int fft2d(float *x_in, float *x_out, int numRows, int numColls);
// INPUT
// x_in              - input signal (matrix, launched by rows)
// numRows           - number of rows
// numColls          - number of collumns
// OUTPUT
// x_out             - output signal (contains (2 * numRows * numColls) elements
                       in order Re(x_in[0][0]), Im(x_in[0][0]),
                       Re(x_in[0][1]), Im(x_in[0][1]) and etc.)
// RESULT
// Error status
*/
int fft2d(float *x_in, float *x_out, int numRows, int numColls);

/*
// Inverse 2-dimensional FFT
//
// API
// int fftInverse2d(float *x_in, float *x_out, int numRows, int numColls);
// INPUT
// x_in              - Fourier image of matrix (contains (2 * numRows * numColls)
                       elements in order Re(x_in[0][0]), Im(x_in[0][0]),
                       Re(x_in[0][1]), Im(x_in[0][1]) and etc.)
// numRows           - number of rows
// numColls          - number of collumns
// OUTPUT
// x_out             - initial signal (matrix, launched by rows)
// RESULT
// Error status
*/
int fftInverse2d(float *x_in, float *x_out, int numRows, int numColls);

#endif
