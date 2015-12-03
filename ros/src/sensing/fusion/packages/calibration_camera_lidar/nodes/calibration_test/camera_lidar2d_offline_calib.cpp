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

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <time.h>

#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "ros/ros.h"
#include <libxml/xmlreader.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CompressedImage.h>
#include <boost/array.hpp>
#include "chessboard.h"
#include "scan_window.h"
#include "trans.h"
#include "common_2d_calib.h"
#include "image_window.h"
#include "data_struct.h"

#define XSTR(x) #x
#define STR(x) XSTR(x)

#define WINDOW_NAME_SCAN "LRF"
#define WINDOW_NAME_IMAGE "CAMERA"

static int window_scan_width = 1240;
static int window_scan_height = 960;
static int scale = 300;
static int line_division_num = 10;
static double judge_margin = 20.0;
static int pat_row =7 ;
static int pat_col = 11;
static int pat_size = pat_row * pat_col;
static double chess_size = 20.0;
static double paper_width_margin = 30.0;

static bool g_chess_scan_flag = false;
static bool g_chess_camera_flag = false;
static IplImage *image_camera;
static bool g_calib_flag = false;
static bool g_fusion_flag = false;
static bool g_save_flag = false;
static bool g_on_calib_flag = false;
static bool g_on_save_flag = false;
static double g_scan_z;
static Three_dimensional_vector g_3d_scan;

static void on_mouse(int event, int x, int y, int flags, void *param = NULL)
{
    if (50 < x && x < 320 && 150< y && y < 220) { // calib
        g_on_calib_flag = true;
        g_on_save_flag = false;
        switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            g_calib_flag = true;
            break;

        default:
            break;
        }
    } else if ( 50 < x && x < 320 && 200< y && y < 270) { //save
        g_on_save_flag = true;
        g_on_calib_flag = false;
        switch(event) {
        case CV_EVENT_LBUTTONDOWN:
            g_save_flag = true;
            break;

        default:
            break;
        }
    } else {
        g_on_calib_flag = false;
        g_on_save_flag = false;
    }
}

static void print_param(CvMat *matrix)
{
    for (int i = 0; i < matrix->rows; i++) {
        for (int j = 0; j < matrix->cols; j++) {
                printf ("% lf\t", cvmGet (matrix, i, j));
            }
        printf ("\n");
    }
}

static void imageCallback(const sensor_msgs::Image& image_raw) {
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_raw, sensor_msgs::image_encodings::BGR8);
    IplImage temp = cv_image->image;
    image_camera = &temp;

    CvMat image_points;
    CvMat object_points;
    CvPoint3D32f objects[pat_row*pat_col];

    if (!g_fusion_flag) {
        int corner_count;
        CvPoint2D32f *corners = (CvPoint2D32f *) cvAlloc (sizeof (CvPoint2D32f) * pat_size);
        if(findChessboard(image_camera, pat_col, pat_row, chess_size, &object_points, &corner_count, corners, &objects[0])){
            g_chess_camera_flag = true;
            drawChessboard(image_camera, pat_col, pat_row, corner_count, corners, &image_points);
        } else {
            g_chess_camera_flag = false;
        }
    }
    if (g_calib_flag) {
        g_fusion_flag = true;
        g_calib_flag = false;

        /* Get external parameter */
        CvMat sub_image_points, sub_object_points;
        cvGetRows(&image_points, &sub_image_points, 0, pat_size);
        cvGetRows(&object_points, &sub_object_points, 0, pat_size);
        cvFindExtrinsicCameraParams2 (&sub_object_points, &sub_image_points, m_intrinsic, m_dist, v_rotation, v_g2c);
        //Chnage vector to matrix
        cvRodrigues2(v_rotation, m_rotation);

        //Get global to scan parameter
        cvmSet(v_g2l, 0, 0, chess_size * (pat_col - 1) / 2);
        cvmSet(v_g2l, 0, 1, chess_size * (pat_row - 1) / 2);
        cvmSet(v_g2l, 0, 2, g_scan_z * 1000);
        set_rotation(m_rotation);
        set_g2c(v_g2c);
    }

    else if (g_fusion_flag) {
        Two_dimensional_vector scan_image;
        scan_image.x.resize(g_3d_scan.x.size());
        scan_image.y.resize(g_3d_scan.x.size());
        scan_image = trans_scan2scan_image(&g_3d_scan, v_g2l, v_g2c, m_rotation, m_intrinsic);
        plot_scan_image(image_camera, &scan_image);
    }

    if (g_save_flag) {
        // Write parameter
        ros::NodeHandle n;
        std::string camera_yaml;
        n.param<std::string>("/camera_lidar2d/camera_yaml", camera_yaml, STR(CAMERA_YAML));
        cv::FileStorage fs(camera_yaml.c_str(), cv::FileStorage::WRITE);
        if(!fs.isOpened()){
            fprintf(stderr, "%s : cannot open file\n", camera_yaml.c_str());
            exit(EXIT_FAILURE);
        }

        CvMat *affine_l2g = cvCreateMat(4, 4, CV_64FC1);
        CvMat *affine_g2c = cvCreateMat(4, 4, CV_64FC1);
        CvMat *affine_l2c = cvCreateMat(4, 4, CV_64FC1);
        cvSetZero(affine_l2g);
        cvSetZero(affine_g2c);
        cvSetZero(affine_l2c);

        cvmSet(affine_l2g, 0, 1, 1.0);
        cvmSet(affine_l2g, 1, 0, -1.0);
        cvmSet(affine_l2g, 2, 2, -1.0);
        cvmSet(affine_l2g, 0, 3, cvmGet(v_g2l, 0, 1));
        cvmSet(affine_l2g, 1, 3, cvmGet(v_g2l, 0, 0));
        cvmSet(affine_l2g, 2, 3, cvmGet(v_g2l, 0, 2));
        cvmSet(affine_l2g, 3, 3, 1);

        cvmSet(affine_g2c, 0, 0, cvmGet(m_rotation, 0, 0));
        cvmSet(affine_g2c, 0, 1, cvmGet(m_rotation, 0, 1));
        cvmSet(affine_g2c, 0, 2, cvmGet(m_rotation, 0, 2));
        cvmSet(affine_g2c, 1, 0, cvmGet(m_rotation, 1, 0));
        cvmSet(affine_g2c, 1, 1, cvmGet(m_rotation, 1, 1));
        cvmSet(affine_g2c, 1, 2, cvmGet(m_rotation, 1, 2));
        cvmSet(affine_g2c, 2, 0, cvmGet(m_rotation, 2, 0));
        cvmSet(affine_g2c, 2, 1, cvmGet(m_rotation, 2, 1));
        cvmSet(affine_g2c, 2, 2, cvmGet(m_rotation, 2, 2));
        cvmSet(affine_g2c, 0, 3, cvmGet(v_g2c, 0, 0));
        cvmSet(affine_g2c, 1, 3, cvmGet(v_g2c, 0, 1));
        cvmSet(affine_g2c, 2, 3, cvmGet(v_g2c, 0, 2));
        cvmSet(affine_g2c, 3, 3, 1);

        cvmMul(affine_g2c, affine_l2g, affine_l2c);

        printf("--transration v_g2l\n");
        print_param(v_g2l);
        printf("--transration v_g2c\n");
        print_param(v_g2c);
        printf("--transration m_rotation\n");
        print_param(m_rotation);
        printf("--transration l2g\n");
        print_param(affine_l2g);
        printf("--transration g2c\n");
        print_param(affine_g2c);
        printf("--transration l2c\n");
        print_param(affine_l2c);
        printf("--intrinsic\n");
        print_param(m_intrinsic);

        cv::Size image_size(image_raw.width, image_raw.height);

        fs << "CameraExtrinsicMat" << affine_l2c;
        fs << "CameraMat" << m_intrinsic;
        fs << "DistCoeff" << m_dist;
        fs << "ImageSize" << image_size;
        fs << "ReprojectionError" << 0;

        cvReleaseMat (&affine_g2c);
        cvReleaseMat (&affine_l2g);
        cvReleaseMat (&affine_l2c);

        fs.release();
        printf("saved\n");
        g_fusion_flag = true;
        g_save_flag = false;
    }
    cvShowImage (WINDOW_NAME_IMAGE, image_camera);
    cvWaitKey (2);
}

static void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    IplImage *image_lrf;
    CvPoint pt, center_pt;

    image_lrf = cvCreateImage(cvSize(window_scan_width, window_scan_height), IPL_DEPTH_8U, 3);
    cvSetZero(image_lrf);

    /* Convert to 3d-axis */
    g_3d_scan.x.resize(scan_msg->ranges.size());
    g_3d_scan.y.resize(scan_msg->ranges.size());
    g_3d_scan.z.resize(scan_msg->ranges.size());
    for(int i = 0; i < (int)scan_msg->ranges.size(); i++) {
        g_3d_scan.x[i] = scan_msg->ranges[i] * sin(scan_msg->angle_min + scan_msg->angle_increment * i);
        g_3d_scan.y[i] = 0;
        g_3d_scan.z[i] = scan_msg->ranges[i] * cos(scan_msg->angle_min + scan_msg->angle_increment * i);
    }

    /* Plot window line */
    plot_vertical_line(image_lrf, line_division_num, window_scan_width, window_scan_height);
    plot_horizontal_line(image_lrf, window_scan_width, window_scan_height);

    // Plot chessboard line
    center_pt = get_center_pt(window_scan_width, &g_3d_scan, scale);
    plot_center_pt_line(image_lrf, center_pt, chess_size, pat_col, paper_width_margin, window_scan_width, window_scan_height, scale);

    /* Plot scan points on image */
    g_chess_scan_flag = true;
    g_scan_z = g_3d_scan.z[scan_msg->ranges.size()/2];
    for (int i = 0; i < (int)scan_msg->ranges.size(); i++) {
        if(0 > g_3d_scan.x[i]*scale+(window_scan_width/2) || g_3d_scan.x[i]*scale+(window_scan_width/2) > window_scan_width) {
            continue;
        }
        if(0 > g_3d_scan.z[i]*scale || g_3d_scan.z[i]*scale > window_scan_height) {
            continue;
        }
        // scan point on image
        pt.x = g_3d_scan.x[i]*scale + (window_scan_width/2);
        pt.y = g_3d_scan.z[i]*scale;

        // Plot point
        int half_paper_col = ((chess_size/1000)*(pat_col+1)+(paper_width_margin/1000))*scale/2;
        if(center_pt.x - half_paper_col <= pt.x && pt.x <= center_pt.x + half_paper_col) {
            // Judge point in chessboard
            if(g_scan_z - (judge_margin/1000) <= g_3d_scan.z[i]
               && g_3d_scan.z[i] <= g_scan_z + (judge_margin/1000)) {
                // red point
                cvCircle(image_lrf, pt, 1, CV_RGB (255, 0, 0), CV_FILLED, 8, 0);
            } else {
                // green point
                cvCircle(image_lrf, pt, 1, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
                g_chess_scan_flag = false;
            }
        } else {
            //green point
            cvCircle(image_lrf, pt, 1, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
        }
    }

    /* Plot center point */
    // yellow point
    cvCircle(image_lrf, center_pt, 6, CV_RGB (255, 255, 0), CV_FILLED, 8, 0);

    /* Plot info */
    // Plot center point's distance
    {
        char text[32];
        sprintf(text, "%.3fm", g_scan_z);
        plot_string(image_lrf, text, 1, 50, 50, CV_RGB(255, 0, 0));
    }
    // Plot String
    if(g_chess_scan_flag) {
        plot_string(image_lrf, "LRF OK", 4, 50, 100, CV_RGB(255, 0, 0));
    }
    if(g_chess_camera_flag) {
        plot_string(image_lrf, "CAMERA OK", 4, 50, 150, CV_RGB(255, 0, 0));
    }
    cvSetMouseCallback (WINDOW_NAME_SCAN, on_mouse);
    plot_string_on_buttun(image_lrf, "CALIBRATE(click)", 4, 50, 200, g_on_calib_flag);
    if(g_fusion_flag) {
        plot_string_on_buttun(image_lrf, "SAVE(click)", 4, 50, 250, g_on_save_flag);
    }

    cvShowImage(WINDOW_NAME_SCAN,image_lrf);
    cvWaitKey(2);
    cvReleaseImage (&image_lrf);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_lidar2d_offline_calib");
    ros::NodeHandle n;

    /* Read parameter xml */
    std::string param_yaml;
    n.param<std::string>("/camera_lidar2d/param_yaml", param_yaml, STR(PARAM_YAML));

    cv::FileStorage fs(param_yaml.c_str(), cv::FileStorage::READ);
    if(!fs.isOpened()){
        fprintf(stderr, "%s : cannot open file\n", param_yaml.c_str());
        exit(EXIT_FAILURE);
    }

    cv::FileNode tm = fs["checkerboard"];
    cv::Mat m_intrinsic_opencv2to1;
    cv::Mat m_dist_opencv2to1;
    CV_Assert(tm.type() == cv::FileNode::MAP && tm.size() == 4);
    pat_row = static_cast<int>(tm["pat_row"]);
    pat_col = static_cast<int>(tm["pat_col"]);
    chess_size =static_cast<double>(tm["chess_size"]);
    paper_width_margin = static_cast<double>(tm["paper_width_margin"]);
    pat_size = pat_col * pat_row;
    tm = fs["window_lrf"];
    CV_Assert(tm.type() == cv::FileNode::MAP && tm.size() == 4);
    window_scan_width = static_cast<int>(tm["width"]);
    window_scan_height = static_cast<int>(tm["height"]);
    line_division_num = static_cast<int>(tm["line_division_num"]);
    judge_margin = static_cast<double>(tm["judge_margin"]);
    fs["intrinsic_matrix"] >> m_intrinsic_opencv2to1;
    fs["distrotion_matrix"] >> m_dist_opencv2to1;
    *m_intrinsic = m_intrinsic_opencv2to1;
    *m_dist = m_dist_opencv2to1;

    /* Create windows */
    cvNamedWindow(WINDOW_NAME_SCAN, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(WINDOW_NAME_IMAGE, CV_WINDOW_AUTOSIZE);

    /* Create bar in window*/
    int default_vector_trackbar = 1000;
    int default_rad_trackbar = 180;
    cvCreateTrackbar ("vector_X", WINDOW_NAME_IMAGE, &default_vector_trackbar, 2000, vector_x_bar);
    cvCreateTrackbar ("vector_Y", WINDOW_NAME_IMAGE, &default_vector_trackbar, 2000, vector_y_bar);
    cvCreateTrackbar ("vector_Z", WINDOW_NAME_IMAGE, &default_vector_trackbar, 2000, vector_z_bar);
    cvCreateTrackbar ("rad_X", WINDOW_NAME_IMAGE, &default_rad_trackbar, 360, rad_x_bar);
    cvCreateTrackbar ("rad_Y", WINDOW_NAME_IMAGE, &default_rad_trackbar, 360, rad_y_bar);
    cvCreateTrackbar ("rad_Z", WINDOW_NAME_IMAGE, &default_rad_trackbar, 360, rad_z_bar);

    ros::Subscriber scan_sub = n.subscribe("scan", 1, scanCallback);
    ros::Subscriber image_sub = n.subscribe("image_raw", 1, imageCallback);

    ros::spin();

    cvDestroyWindow(WINDOW_NAME_SCAN);
    cvDestroyWindow(WINDOW_NAME_IMAGE);
    release_common();
    release_image_window();

    return 0;
}
