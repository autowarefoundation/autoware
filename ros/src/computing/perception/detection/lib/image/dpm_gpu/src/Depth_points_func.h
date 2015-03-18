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

#ifndef _Depth_points_func_
#define _Depth_points_func_

// custom ros version
/* #include "MODEL_info.h"		//File information */
/* #include "Laser_info.h" */
/* #include "Common.h" */

#define X_THETA 0.0
#define Y_THETA 0.0
#define Z_THETA 0.0
#define X_VECTOR -0.265 //width
#define Y_VECTOR 0.16 //height
#define Z_VECTOR 0 //depth
#define FX_CAMERA_PARAM 585
#define FY_CAMERA_PARAM 585
#define OX_CAMERA_PARAM 350.790854
#define OY_CAMERA_PARAM 240.620026
#define READ_POINT_BUFFER_SIZE 126
#define SCAN_POINT_NUM 721
/*
typedef struct {
    double x[SCAN_POINT_NUM];
    double y[SCAN_POINT_NUM];
    double z[SCAN_POINT_NUM];
}Three_dimensional_vector;
*/
typedef struct {
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> z;
}Three_dimensional_vector;
/*
typedef struct {
    double x[SCAN_POINT_NUM];
    double y[SCAN_POINT_NUM];
}Two_dimensional_vector;
*/
typedef struct {
    std::vector<float> x;
    std::vector<float> y;
}Two_dimensional_vector;

extern void trans_depth_points_to_image_points(Three_dimensional_vector* depth_points, Two_dimensional_vector* image_points, std::vector<float> *distance);
extern double get_processing_time(struct timespec start, struct timespec end);
extern void write_file_processing_time(struct timespec start, struct timespec end);
extern void init_depth_points(Three_dimensional_vector* depth_points);
extern void print_all_point(double *x_src, double *y_src, double *z_src, double *x_dst, double *y_dst, double *z_dst, double *u, double *v);

#endif
