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
