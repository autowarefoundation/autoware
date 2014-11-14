/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "cxcore.h"
//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>

#include <math.h>
#include "Depth_points_func.h"
#include "ros/ros.h"
#include <libxml/xmlreader.h>
#include <image_transport/image_transport.h>//C++ library
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/array.hpp>

#if 1 // AXE
#define XSTR(x) #x
#define STR(x) XSTR(x)
#endif

//parameter
int window_lrf_width = 1240;
int window_lrf_height = 960;
int scale = 300;
int line_division_num = 10;
double judge_margin = 20.0;
int pat_row =7 ;
int pat_col = 11;
int pat_size = pat_row * pat_col;
double chess_size = 20.0;
double paper_width_margin = 30.0;
cv::Mat intrinsic_opencv2to1;
cv::Mat distortion_opencv2to1;


char WINDOW_NAME_LRF[] = "LRF";
char WINDOW_NAME_CAMERA[] = "camera";
int g_lrf_judge = 0; //checkerboardの判定
int g_camera_judge = 0; //checkerboardの判定
IplImage *image_camera; //メモリリークを防ぐため
int g_calibrate_judge = 0; //キャリブレーションの実行判定
int g_sensers_fusion_judge = 0;
int g_save_param_judge = 0;
int g_on_mouse_calibrate_button = 0; //change color
int g_on_mouse_save_param_button = 0;
double g_lrf_z;
Three_dimensional_vector g_lrf_depth_points;

CvMat *translation_global2lrf = cvCreateMat (1, 3, CV_32FC1);
CvMat *translation_global2camera = cvCreateMat (1, 3, CV_32FC1);
CvMat *translation_global2camera_const = cvCreateMat (1, 3, CV_32FC1);
CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
CvMat *rotation_matrix = cvCreateMat (3, 3, CV_32FC1);
CvMat *rotation_matrix_temp = cvCreateMat (3, 3, CV_32FC1);
CvMat *intrinsic = cvCreateMat (3, 3, CV_32FC1);
CvMat *rotation = cvCreateMat (1, 3, CV_32FC1);
CvMat *distortion = cvCreateMat (1, 4, CV_32FC1);

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

/* コールバック関数 */
void vector_x_bar(int val)
{
    cvmSet(translation_global2camera, 0, 0, cvmGet(translation_global2camera_const, 0, 0) + val - 1000);
}
void vector_y_bar(int val)
{
    cvmSet(translation_global2camera, 0, 1, cvmGet(translation_global2camera_const, 0, 1) + val - 1000);
}
void vector_z_bar(int val)
{
    cvmSet(translation_global2camera, 0, 2, cvmGet(translation_global2camera_const, 0, 2) + val - 1000);
}
void rad_x_bar(int val)
{
    static int previous_val_x = 180;
    CvMat rotation_matrix_x;
    float init_rotation_matrix_x[9] ={
        1, 0, 0,
        0, (float)cos((val-previous_val_x)/180.0*M_PI), (float)-sin((val-previous_val_x)/180.0*M_PI),
        0, (float)sin((val-previous_val_x)/180.0*M_PI), (float)cos((val-previous_val_x)/180.0*M_PI)};

    cvInitMatHeader(&rotation_matrix_x, 3, 3, CV_32FC1, init_rotation_matrix_x, CV_AUTOSTEP);
    cvCopy(rotation_matrix, rotation_matrix_temp);
    cvmMul(rotation_matrix_temp, &rotation_matrix_x, rotation_matrix);
    previous_val_x = val;
}
void rad_y_bar(int val)
{
    static int previous_val_y = 180;
    CvMat rotation_matrix_y;
    float init_rotation_matrix_y[9] ={
        (float)cos((val-previous_val_y)/180.0*M_PI), 0, (float)sin((val-previous_val_y)/180.0*M_PI),
        0, 1, 0,
        (float)-sin((val-previous_val_y)/180.0*M_PI), 0, (float)cos((val-previous_val_y)/180.0*M_PI)};

    cvInitMatHeader(&rotation_matrix_y, 3, 3, CV_32FC1, init_rotation_matrix_y, CV_AUTOSTEP);
    cvCopy(rotation_matrix, rotation_matrix_temp);
    cvmMul(rotation_matrix_temp, &rotation_matrix_y, rotation_matrix);
    previous_val_y = val;
}
void rad_z_bar(int val)
{
    static int previous_val_z = 180;
    CvMat rotation_matrix_z;
    float init_rotation_matrix_z[9] ={
        (float)cos((val-previous_val_z)/180.0*M_PI), (float)-sin((val-previous_val_z)/180.0*M_PI), 0,
        (float)sin((val-previous_val_z)/180.0*M_PI), (float)cos((val-previous_val_z)/180.0*M_PI), 0,
        0, 0, 1};

    cvInitMatHeader(&rotation_matrix_z, 3, 3, CV_32FC1, init_rotation_matrix_z);
    cvCopy(rotation_matrix, rotation_matrix_temp);
    cvmMul(rotation_matrix_temp, &rotation_matrix_z, rotation_matrix);
}


void on_mouse (int event, int x, int y, int flags, void *param = NULL)
{
    if (50 < x && x < 320 && 150< y && y < 220) {
        g_on_mouse_calibrate_button = 1;
        g_on_mouse_save_param_button = 0;
        switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            g_calibrate_judge = 1;
            break;

        default:
            break;
        }
    } else if ( 50 < x && x < 320 && 200< y && y < 270) {
        g_on_mouse_save_param_button = 1;
        g_on_mouse_calibrate_button = 0;
        switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            g_save_param_judge = 1;
            break;

        default:
            break;
        }
    } else {
        g_on_mouse_calibrate_button = 0;
        g_on_mouse_save_param_button = 0;
    }
}

void trans_depth_points_to_image_points(Three_dimensional_vector* depth_points, Two_dimensional_vector* image_points)
{
    // std::vector<float> x_dst;
    // std::vector<float> y_dst;
    // std::vector<float> z_dst;
    // Three_dimensional_vector __global_depth_points;

    // int i;

    // x_dst.resize(depth_points->x.size());
    // y_dst.resize(depth_points->y.size());
    // z_dst.resize(depth_points->z.size());
    // __global_depth_points.x.resize(depth_points->x.size());
    // __global_depth_points.y.resize(depth_points->y.size());
    // __global_depth_points.z.resize(depth_points->z.size());

    // for(i = 0; i < depth_points->x.size(); i++) {
    //     //global座標系に変換
    //     __global_depth_points.y[i] = (-1.0 * depth_points->x[i]) - (cvmGet(translation_global2lrf, 0, 0) /1000);
    //     __global_depth_points.x[i] = depth_points->y[i] - (cvmGet(translation_global2lrf, 0, 1)/1000);
    //     __global_depth_points.z[i] = -1.0 * (depth_points->z[i] - (cvmGet(translation_global2lrf, 0, 2)/1000));
    //     //回転行列
    //     x_dst[i] = cvmGet(rotation_matrix, 0, 0) * __global_depth_points.x[i] + cvmGet(rotation_matrix, 1, 0) * __global_depth_points.y[i] + cvmGet(rotation_matrix, 2, 0) * __global_depth_points.z[i];
    //     y_dst[i] = cvmGet(rotation_matrix, 0, 1) * __global_depth_points.x[i] + cvmGet(rotation_matrix, 1, 1) * __global_depth_points.y[i] + cvmGet(rotation_matrix, 2, 1) * __global_depth_points.z[i];
    //     z_dst[i] = cvmGet(rotation_matrix, 0, 2) * __global_depth_points.x[i] + cvmGet(rotation_matrix, 1, 2) * __global_depth_points.y[i] + cvmGet(rotation_matrix, 2, 2) * __global_depth_points.z[i];
    //     //並進ベクトル
    //     x_dst[i] = x_dst[i] + (cvmGet(translation_global2camera, 0, 0)/1000);
    //     y_dst[i] = y_dst[i] + (cvmGet(translation_global2camera, 0, 1)/1000);
    //     z_dst[i] = z_dst[i] + (cvmGet(translation_global2camera, 0, 2)/1000);

    //     /*
    //      * 投影変換
    //      */
    //     if (z_dst[i] > 0.0) {
    //         image_points->x[i] = x_dst[i] * cvmGet(intrinsic, 0, 0) / z_dst[i] + cvmGet(intrinsic, 0, 2);
    //         image_points->y[i] = y_dst[i] * cvmGet(intrinsic, 1, 1) / z_dst[i] + cvmGet(intrinsic, 1, 2);
    //     } else {
    //         image_points->x[i] = -1;
    //         image_points->y[i] = -1;
    //     }
    // }
    float global_x;
    float global_y;
    float global_z;
    float camera_x;
    float camera_y;
    float camera_z;
    int i;

#if 1 // AXE
    for(i = 0; i < (int)depth_points->x.size(); i++) {
#else
    for(i = 0; i < depth_points->x.size(); i++) {
#endif
        //global座標系に変換
        global_y = (-1.0 * depth_points->x[i]) - (cvmGet(translation_global2lrf, 0, 0) /1000);
        global_x = depth_points->y[i] - (cvmGet(translation_global2lrf, 0, 1)/1000);
        global_z = -1.0 * (depth_points->z[i] - (cvmGet(translation_global2lrf, 0, 2)/1000));
        //回転行列
        camera_x = cvmGet(rotation_matrix, 0, 0) * global_x + cvmGet(rotation_matrix, 1, 0) * global_y + cvmGet(rotation_matrix, 2, 0) * global_z;
        camera_y = cvmGet(rotation_matrix, 0, 1) * global_x + cvmGet(rotation_matrix, 1, 1) * global_y + cvmGet(rotation_matrix, 2, 1) * global_z;
        camera_z = cvmGet(rotation_matrix, 0, 2) * global_x + cvmGet(rotation_matrix, 1, 2) * global_y + cvmGet(rotation_matrix, 2, 2) * global_z;
        //並進ベクトル
        camera_x = camera_x + (cvmGet(translation_global2camera, 0, 0)/1000);
        camera_y = camera_y + (cvmGet(translation_global2camera, 0, 1)/1000);
        camera_z = camera_z + (cvmGet(translation_global2camera, 0, 2)/1000);

        /*
         * 投影変換
         */
        if (camera_z > 0.0) {
            image_points->x[i] = camera_x * cvmGet(intrinsic, 0, 0) / camera_z + cvmGet(intrinsic, 0, 2);
            image_points->y[i] = camera_y * cvmGet(intrinsic, 1, 1) / camera_z + cvmGet(intrinsic, 1, 2);
        } else {
            image_points->x[i] = -1;
            image_points->y[i] = -1;
        }
    }
}

void plot_depth_points(Two_dimensional_vector* image_points)
{
    CvSeq *points;
    CvPoint pt;
    CvMemStorage *storage = cvCreateMemStorage (0);
#if 1 // AXE
    int i;
#else
    int i, j;
#endif

    //画像に点群データをプロット
    points = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), storage);
#if 1 // AXE
    for (i = 0; i < (int)image_points->x.size(); i++) {
#else
    for (i = 0; i < image_points->x.size(); i++) {
#endif
        if(0 > image_points->x[i] || image_points->x[i] > 639) {
            continue;
        }
        if(0 > image_points->y[i] || image_points->y[i] > 479) {
            continue;
        }
        pt.x = image_points->x[i];
        pt.y = image_points->y[i];

        cvSeqPush (points, &pt);
        cvCircle(image_camera, pt, 2, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
    }

    cvClearSeq(points);
    cvReleaseMemStorage(&storage);
}

void imageCallback(const sensor_msgs::Image& image_raw) {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
    IplImage temp = cv_image->image;
    image_camera = &temp;

    int i, j;
    int corner_count, found;
    int p_count;
    CvSize pattern_size = cvSize (pat_col, pat_row);
    CvPoint3D32f objects[pat_size];
    CvPoint2D32f *corners = (CvPoint2D32f *) cvAlloc (sizeof (CvPoint2D32f) * pat_size);
    CvMat object_points;
    CvMat image_points;
    CvMat point_counts;
    IplImage* src_gray;
    if (g_sensers_fusion_judge != 1) {
        // (2)3次元空間座標の設定
        for (i = 0; i < pat_row; i++) {
            for (j = 0; j < pat_col; j++) {
                objects[i * pat_col + j].x = i * chess_size;
                objects[i * pat_col + j].y = j * chess_size;
                objects[i * pat_col + j].z = 0.0;
            }
        }
        cvInitMatHeader(&object_points, pat_size, 3, CV_32FC1, objects);

        // (3)チェスボード（キャリブレーションパターン）のコーナー検出
        found = cvFindChessboardCorners (image_camera, pattern_size, &corners[0], &corner_count);
        if (found) {g_camera_judge = 1;}
        else {g_camera_judge = 0;}
        // (4)コーナー位置をサブピクセル精度に修正，描画
        src_gray = cvCreateImage (cvGetSize (image_camera), IPL_DEPTH_8U, 1);
        cvCvtColor (image_camera, src_gray, CV_BGR2GRAY);
        cvFindCornerSubPix (src_gray, &corners[0], corner_count,
                            cvSize (3, 3), cvSize (-1, -1), cvTermCriteria (CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
        cvDrawChessboardCorners (image_camera, pattern_size, &corners[0], corner_count, found);
        p_count = corner_count;
        cvInitMatHeader (&image_points, pat_size, 1, CV_32FC2, corners);
        cvInitMatHeader (&point_counts, 1, 1, CV_32SC1, &p_count);
        cvReleaseImage (&src_gray);
    }

    if (g_calibrate_judge == 1) {
        g_sensers_fusion_judge = 1;
        g_calibrate_judge = 0;
        // (6)外部パラメータの推定
        CvMat sub_image_points, sub_object_points;

        *intrinsic = intrinsic_opencv2to1;
        *distortion = distortion_opencv2to1;

        cvGetRows(&image_points, &sub_image_points, 0, pat_size);
        cvGetRows(&object_points, &sub_object_points, 0, pat_size);
        cvFindExtrinsicCameraParams2 (&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation_global2camera);
        //回転ベクトル→回転行列
        cvRodrigues2(rotation, rotation_matrix);

        //lrfへの並進ベクトル
        cvmSet(translation_global2lrf, 0, 0, -1.0 * chess_size * (pat_col - 1) / 2);
        cvmSet(translation_global2lrf, 0, 1, -1.0 * chess_size * (pat_row - 1) / 2);
        cvmSet(translation_global2lrf, 0, 2, g_lrf_z * 1000);

        printf("--transration_global2camera vector--\n");
        for (i = 0; i < translation_global2camera->rows; i++) {
            for (j = 0; j < translation_global2camera->cols; j++) {
                cvmSet(translation_global2camera_const, i, j, cvmGet (translation_global2camera, i, j));
                printf ("% lf\t", cvmGet (translation_global2camera, i, j));
            }
            printf ("\n");
        }
        printf("--transration_global2lrf vector--\n");
        for (i = 0; i < translation_global2lrf->rows; i++) {
            for (j = 0; j < translation_global2lrf->cols; j++) {
                printf ("% lf\t", cvmGet (translation_global2lrf, i, j));
            }
            printf ("\n");
        }

//        cvmSub(translation_global2camera, translation_global2lrf, translation);
        //回転行列を表示
        printf("--rotation matrix--\n");
        for (i = 0; i < rotation_matrix->rows; i++) {
            for (j = 0; j < rotation_matrix->cols; j++) {
                cvmSet(rotation_matrix_temp, i, j, cvmGet (rotation_matrix, i, j));
                printf ("% lf\t", cvmGet (rotation_matrix, i, j));
            }
            printf ("\n");
        }
        printf("\n");

    }
    else if (g_sensers_fusion_judge == 1) {
        Two_dimensional_vector lrf_image_points;
        lrf_image_points.x.resize(g_lrf_depth_points.x.size());
        lrf_image_points.y.resize(g_lrf_depth_points.x.size());
        trans_depth_points_to_image_points(&g_lrf_depth_points, &lrf_image_points);
        plot_depth_points(&lrf_image_points);

    }
    if (g_save_param_judge == 1) {
        // (7)yamlファイルへの書き出し
#if 1 // AXE
        ros::NodeHandle n;
        std::string camera_yaml;
	n.param<std::string>("/camera_lidar2d/camera_yaml", camera_yaml, STR(CAMERA_YAML));
        cv::FileStorage fs(camera_yaml.c_str(), cv::FileStorage::WRITE);
#else
        cv::FileStorage fs("camera.yaml", cv::FileStorage::WRITE);
#endif
        fs << "intrinsic" << intrinsic;
        fs << "rotation" << rotation;
        fs << "rotation_matrix" << rotation_matrix;
        fs << "translation_global2lrf" << translation_global2lrf;
        fs << "translation_global2camera" << translation_global2camera;
        fs << "distortion" << distortion;
        fs.release();
        printf("saved\n");
        g_sensers_fusion_judge = 1;
        g_save_param_judge = 0;
    }
    cvShowImage (WINDOW_NAME_CAMERA, image_camera);
    cvWaitKey (2);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
#if 1 // AXE
#else
    CvSeq *points;
#endif
    IplImage *image_lrf;
    CvPoint pt, center_pt, line_start, line_end;


    image_lrf=cvCreateImage(cvSize(window_lrf_width, window_lrf_height), IPL_DEPTH_8U, 3);
//    ROS_INFO("angle_min[%f]\nangle_max:[%f]\nangle_increment:[%f]\ntime_increment:[%f]\nscan_time:[%f]\nrange_min:[%f]\nrange_max:[%f]\n", msg->angle_min * 180 / 3.141592, msg->angle_max * 180 / 3.141592, msg->angle_increment * 180 / 3.141592, msg->time_increment, msg->scan_time, msg->range_min, msg->range_max);
    int i;


    g_lrf_depth_points.x.resize(scan->ranges.size());
    g_lrf_depth_points.y.resize(scan->ranges.size());
    g_lrf_depth_points.z.resize(scan->ranges.size());

#if 1 // AXE
    for(i = 0; i < (int)scan->ranges.size(); i++) {
#else
    for(i = 0; i < scan->ranges.size(); i++) {
#endif
        g_lrf_depth_points.x[i] = scan->ranges[i] * sin(scan->angle_min + scan->angle_increment * i);
        g_lrf_depth_points.y[i] = 0;
        g_lrf_depth_points.z[i] = scan->ranges[i] * cos(scan->angle_min + scan->angle_increment * i);
    }
    /* 横ライン */
    for (i = 0; i < line_division_num; i++) {
        line_start.x = 0;
        line_start.y = window_lrf_height / line_division_num * (i + 1);
        line_end.x = window_lrf_width;
        line_end.y = window_lrf_height / line_division_num * (i + 1);

        cvLine(image_lrf, line_start, line_end, CV_RGB (120, 120, 120), 1, 8, 0);
    }
    /* 縦ライン */
    line_start.x = window_lrf_width / 2;
    line_start.y = 0;
    line_end.x = window_lrf_width /2;
    line_end.y = window_lrf_height;
    cvLine(image_lrf, line_start, line_end, CV_RGB (120, 120, 120), 1, 8, 0);


    /* scanデータのプロット */

    //中央point
    center_pt.x = g_lrf_depth_points.x[scan->ranges.size() / 2] * scale + (window_lrf_width / 2);
    center_pt.y = g_lrf_depth_points.z[scan->ranges.size() / 2] * scale;
    //中央pointの横ライン
    line_start.x = window_lrf_width / 2 - ((chess_size / 1000) * (pat_col + 1) + (paper_width_margin /1000)) * scale / 2;
    line_start.y = center_pt.y;
    line_end.x = window_lrf_width /2 + ((chess_size / 1000) * (pat_col + 1) + (paper_width_margin /1000)) * scale / 2;
    line_end.y = center_pt.y;
    cvLine(image_lrf, line_start, line_end, CV_RGB (255, 255, 0), 1, 8, 0);
    //checkerboard領域の縦ライン
    line_start.x = window_lrf_width / 2 - ((chess_size / 1000) * (pat_col + 1) + (paper_width_margin /1000)) * scale / 2;
    line_start.y = 0;
    line_end.x = window_lrf_width /2 - ((chess_size / 1000) * (pat_col + 1) + (paper_width_margin /1000)) * scale / 2;
    line_end.y = window_lrf_height;
    cvLine(image_lrf, line_start, line_end, CV_RGB (255, 255, 0), 1, 8, 0);

    line_start.x = window_lrf_width / 2 + ((chess_size / 1000) * (pat_col + 1) + (paper_width_margin /1000)) * scale / 2;
    line_start.y = 0;
    line_end.x = window_lrf_width /2 + ((chess_size / 1000) * (pat_col + 1) + (paper_width_margin /1000)) * scale / 2;
    line_end.y = window_lrf_height;
    cvLine(image_lrf, line_start, line_end, CV_RGB (255, 255, 0), 1, 8, 0);

    //scanデータのプロット
    g_lrf_judge = 1;
#if 1 // AXE
    for (i = 0; i < (int)scan->ranges.size(); i++) {
#else
    for (i = 0; i < scan->ranges.size(); i++) {
#endif
        if(0 > g_lrf_depth_points.x[i] * scale + (window_lrf_width / 2) || g_lrf_depth_points.x[i] * scale + (window_lrf_width / 2) > window_lrf_width) {
            continue;
        }
        if(0 > g_lrf_depth_points.z[i] * scale || g_lrf_depth_points.z[i] * scale > window_lrf_height) {
            continue;
        }
        pt.x = g_lrf_depth_points.x[i] * scale + (window_lrf_width / 2);
        pt.y = g_lrf_depth_points.z[i] * scale;

        //checkerboard領域であれば赤点
        if(center_pt.x - ((chess_size / 1000) * (pat_col + 1) + (paper_width_margin /1000)) * scale / 2 <= pt.x && pt.x <= center_pt.x + ((chess_size / 1000) * (pat_col + 1) + (paper_width_margin /1000)) * scale / 2) {
            if(g_lrf_depth_points.z[scan->ranges.size() / 2] - (judge_margin/1000) <= g_lrf_depth_points.z[i] && g_lrf_depth_points.z[i] <= g_lrf_depth_points.z[scan->ranges.size() / 2] + (judge_margin/1000)) {
                cvCircle(image_lrf, pt, 1, CV_RGB (255, 0, 0), CV_FILLED, 8, 0);
            } else {
                cvCircle(image_lrf, pt, 1, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
                g_lrf_judge = 0; //全部赤じゃなかった場合
            }
        } else {
            cvCircle(image_lrf, pt, 1, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
        }
    }

    //中央pointのplot
    cvCircle(image_lrf, center_pt, 6, CV_RGB (255, 255, 0), CV_FILLED, 8, 0);
    //距離の表示
    CvFont dfont;
    float hscale      = 1.0f;
    float vscale      = 1.0f;
    float italicscale = 0.0f;
    int  thickness    = 1;
    char text[255];
    g_lrf_z = g_lrf_depth_points.z[scan->ranges.size() / 2];
    sprintf(text, "%.3fm", g_lrf_depth_points.z[scan->ranges.size() / 2]);
    cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
    cvPutText(image_lrf, text, cvPoint(50, 50), &dfont, CV_RGB(255, 255, 255));

    thickness = 4;
    cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
    if(g_lrf_judge == 1) {
        sprintf(text, "LRF OK");
        cvPutText(image_lrf, text, cvPoint(50, 100), &dfont, CV_RGB(255, 0, 0));
    }
    if(g_camera_judge == 1) {
        sprintf(text, "CAMERA OK");
        cvPutText(image_lrf, text, cvPoint(50, 150), &dfont, CV_RGB(255, 0, 0));
    }
    //クリックされたらキャリブレーションを実行
    sprintf(text, "CALIBRATE(click)");
    cvSetMouseCallback (WINDOW_NAME_LRF, on_mouse);
    if (g_on_mouse_calibrate_button == 1) {
        cvPutText(image_lrf, text, cvPoint(50, 200), &dfont, CV_RGB(250, 250, 250));
    } else {
        cvPutText(image_lrf, text, cvPoint(50, 200), &dfont, CV_RGB(150, 150, 150));
    }
    if(g_sensers_fusion_judge == 1) {
        sprintf(text, "SAVE(click)");
        if(g_on_mouse_save_param_button == 1) {
            cvPutText(image_lrf, text, cvPoint(50, 250), &dfont, CV_RGB(250, 250, 250));
        } else {
            cvPutText(image_lrf, text, cvPoint(50, 250), &dfont, CV_RGB(150, 150, 150));
        }
    }

    cvShowImage(WINDOW_NAME_LRF,image_lrf);
    cvWaitKey(2);
    cvReleaseImage (&image_lrf);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
#if 1 // AXE
    ros::init(argc, argv, "camera_lidar2d_offline_calib");
    ros::NodeHandle n;

    /* xmlからパラメータ設定 */
    std::string param_yaml;
    n.param<std::string>("/camera_lidar2d/param_yaml", param_yaml, STR(PARAM_YAML));

    // ファイルの種類は，内容から決定
    cv::FileStorage fs(param_yaml.c_str(), cv::FileStorage::READ);
#else
    /* xmlからパラメータ設定 */
    char path[128];
    strcpy(path, getenv("HOME"));
    strcat(path, "/catkin_ws/src/calibration_of_camera_and_lrf/param.yaml");

    // ファイルの種類は，内容から決定
    cv::FileStorage fs(path, cv::FileStorage::READ);
#endif
    cv::FileNode tm = fs["checkerboard"];
    CV_Assert(tm.type() == cv::FileNode::MAP && tm.size() == 4);
    pat_row = static_cast<int>(tm["pat_row"]);
    pat_col = static_cast<int>(tm["pat_col"]);
    chess_size =static_cast<double>(tm["chess_size"]);
    paper_width_margin = static_cast<double>(tm["paper_width_margin"]);
    pat_size = pat_col * pat_row;
    tm = fs["window_lrf"];
    CV_Assert(tm.type() == cv::FileNode::MAP && tm.size() == 4);
    window_lrf_width = static_cast<int>(tm["width"]);
    window_lrf_height = static_cast<int>(tm["height"]);
    line_division_num = static_cast<int>(tm["line_division_num"]);
    judge_margin = static_cast<double>(tm["judge_margin"]);
    fs["intrinsic_matrix"] >> intrinsic_opencv2to1;
    fs["distrotion_matrix"] >> distortion_opencv2to1;

    cvNamedWindow(WINDOW_NAME_LRF, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(WINDOW_NAME_CAMERA, CV_WINDOW_AUTOSIZE);
    int default_vector_trackbar = 1000;
    int default_rad_trackbar = 180;
    cvCreateTrackbar ("vector_X", WINDOW_NAME_CAMERA, &default_vector_trackbar, 2000, vector_x_bar);
    cvCreateTrackbar ("vector_Y", WINDOW_NAME_CAMERA, &default_vector_trackbar, 2000, vector_y_bar);
    cvCreateTrackbar ("vector_Z", WINDOW_NAME_CAMERA, &default_vector_trackbar, 2000, vector_z_bar);
    cvCreateTrackbar ("rad_X", WINDOW_NAME_CAMERA, &default_rad_trackbar, 360, rad_x_bar);
    cvCreateTrackbar ("rad_Y", WINDOW_NAME_CAMERA, &default_rad_trackbar, 360, rad_y_bar);
    cvCreateTrackbar ("rad_Z", WINDOW_NAME_CAMERA, &default_rad_trackbar, 360, rad_z_bar);

#if 1 // AXE
#else
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "calibration_of_camera_and_LRF");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;
#endif

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber scan_subscriber = n.subscribe("scan", 1, scanCallback);
  ros::Subscriber image_subscriber = n.subscribe("image_raw", 1, imageCallback);


//  image_transport::ImageTransport it(n);
//  image_transport::Subscriber sub = it.subscribe("imageraw", 1, chatterCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%
  cvDestroyWindow(WINDOW_NAME_LRF);
  cvDestroyWindow(WINDOW_NAME_CAMERA);
  cvReleaseMat (&distortion);
  cvReleaseMat (&intrinsic);
  cvReleaseMat (&rotation);
  cvReleaseMat (&translation_global2lrf);
  cvReleaseMat (&translation_global2camera);
  cvReleaseMat (&translation);
  cvReleaseMat (&rotation_matrix);
  cvReleaseMat (&rotation_matrix_temp);

  return 0;
}
// %EndTag(FULLTEXT)%
