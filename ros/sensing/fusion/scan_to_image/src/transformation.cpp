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
#include "ros/ros.h"
#include "std_msgs/String.h"

//yukky
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv/cxcore.h"
#ifdef _DEBUG
//Debugモードの場合
#pragma comment(lib,"cv200d.lib")
#pragma comment(lib,"cxcore200d.lib")
#pragma comment(lib,"cvaux200d.lib")
#pragma comment(lib,"highgui200d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"cv200.lib")
#pragma comment(lib,"cxcore200.lib")
#pragma comment(lib,"cvaux200.lib")
#pragma comment(lib,"highgui200.lib")
#endif
//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#include "Depth_points_func.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "scan_to_image/PointedImage.h"
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

struct sched_attr {
    uint32_t size;
    uint32_t sched_policy;
    uint64_t sched_flags;

    /* SCHED_NORMAL, SCHED_BATCH */
    int32_t sched_nice;
    /* SCHED_FIFO, SCHED_RR */
    uint32_t sched_priority;
    /* SCHED_DEADLINE */
    uint64_t sched_runtime;
    uint64_t sched_deadline;
    uint64_t sched_period;
};
#ifdef __x86_64__
#define __NR_sched_setattr              314
#define __NR_sched_getattr              315
#endif

#ifdef __i386__
#define __NR_sched_setattr              351
#define __NR_sched_getattr              352
#endif

#ifdef __arm__
#define __NR_sched_setattr              380
#define __NR_sched_getattr              381
#endif

#ifndef SCHED_DEADLINE
#define SCHED_DEADLINE          6
#endif

#ifndef SCHED_FLAG_RESET_ON_FORK
#define SCHED_FLAG_RESET_ON_FORK        0x01
#endif




IplImage *IMAGE_PATH;
char WINDOW_NAME[] = "CAR_TRACK";
ros::Publisher pointed_image;
cv::Mat translation_global2lrf, translation_global2camera, rotation, intrinsic;
double x_theta, y_theta, z_theta, x_vector, y_vector, z_vector, fx_camera_param, fy_camera_param, ox_camera_param, oy_camera_param;
int manual_mode;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void auto_trans_depth_points_to_image_points(Three_dimensional_vector* depth_points, Two_dimensional_vector* image_points, std::vector<float> *distance)
{
    struct timespec start, end;
    std::vector<float> x_dst;
    std::vector<float> y_dst;
    std::vector<float> z_dst;
    int i;

    clock_gettime(CLOCK_REALTIME, &start);
    x_dst.resize(depth_points->x.size());
    y_dst.resize(depth_points->y.size());
    z_dst.resize(depth_points->z.size());

    for(i = 0; i < depth_points->x.size(); i++) {
        //global座標系に変換
        double temp =  -1.0 * depth_points->y[i];
        depth_points->y[i] = depth_points->x[i] - (translation_global2lrf.at<float>(0,0)/1000);
        depth_points->x[i] = temp - (translation_global2lrf.at<float>(0,1)/1000);
        depth_points->z[i] = -1.0 * (depth_points->z[i] - (translation_global2lrf.at<float>(0,2)/1000));
        //回転行列
        x_dst[i] = rotation.at<float>(0,0) * depth_points->x[i] + rotation.at<float>(1,0) * depth_points->y[i] + rotation.at<float>(2,0) * depth_points->z[i];
        y_dst[i] = rotation.at<float>(0,1) * depth_points->x[i] + rotation.at<float>(1,1) * depth_points->y[i] + rotation.at<float>(2,1) * depth_points->z[i];
        z_dst[i] = rotation.at<float>(0,2) * depth_points->x[i] + rotation.at<float>(1,2) * depth_points->y[i] + rotation.at<float>(2,2) * depth_points->z[i];
        //並進ベクトル
        x_dst[i] = x_dst[i] + (translation_global2camera.at<float>(0,0)/1000);
        y_dst[i] = y_dst[i] + (translation_global2camera.at<float>(0,1)/1000);
        z_dst[i] = z_dst[i] + (translation_global2camera.at<float>(0,2)/1000);

        /*
         * ユークリッド距離算出（カメラと障害物）
         */
        (*distance).at(i) = sqrt(x_dst[i] * x_dst[i] + y_dst[i] * y_dst[i] + z_dst[i] * z_dst[i]);
        /*
         * 投影変換
         */
        if (z_dst[i] >= 0.0) {
            image_points->x[i] = x_dst[i] * intrinsic.at<float>(0,0) / z_dst[i] + intrinsic.at<float>(0,2);
            image_points->y[i] = y_dst[i] * intrinsic.at<float>(1,1) / z_dst[i] + intrinsic.at<float>(1,2);
        } else {
            image_points->x[i] = -1;
            image_points->y[i] = -1;
        }
    }

    clock_gettime(CLOCK_REALTIME, &end);
    printf("time:%f msec\n", get_processing_time(start, end));
}

void manual_trans_depth_points_to_image_points(Three_dimensional_vector* depth_points, Two_dimensional_vector* image_points, std::vector<float> *distance)
{
    struct timespec start, end;
    std::vector<float> x_dst;
    std::vector<float> y_dst;
    std::vector<float> z_dst;
    int i;

    clock_gettime(CLOCK_REALTIME, &start);
    x_dst.resize(depth_points->x.size());
    y_dst.resize(depth_points->y.size());
    z_dst.resize(depth_points->z.size());

    for(i = 0; i < depth_points->x.size(); i++) {

        //X軸回転
        y_dst[i] = depth_points->y[i] * cos(x_theta / 180.0 * M_PI) - depth_points->z[i] * sin(x_theta / 180.0 * M_PI);
        z_dst[i] = depth_points->y[i] * sin(x_theta / 180.0 * M_PI) + depth_points->z[i] * cos(x_theta / 180.0 * M_PI);

        //Y軸回転
        x_dst[i] = depth_points->x[i] * cos(y_theta / 180.0 * M_PI) + z_dst[i] * sin(y_theta / 180.0 * M_PI);
        z_dst[i] = -1.0 * depth_points->x[i] * sin(y_theta / 180.0 * M_PI) + z_dst[i] * cos(y_theta / 180.0 * M_PI);

        //Z軸回転
        x_dst[i] = x_dst[i] * cos(z_theta / 180.0 * M_PI) - y_dst[i] * sin(z_theta / 180.0 * M_PI);
        y_dst[i] = x_dst[i] * sin(z_theta / 180.0 * M_PI) + y_dst[i] * cos(z_theta / 180.0 * M_PI);
        //平行移動
         x_dst[i] = x_dst[i] + x_vector;
         y_dst[i] = y_dst[i] + y_vector;
         z_dst[i] = z_dst[i] + z_vector;

        /*
         * ユークリッド距離算出（カメラと障害物）
         */
        (*distance).at(i) = sqrt(x_dst[i] * x_dst[i] + y_dst[i] * y_dst[i] + z_dst[i] * z_dst[i]);
        /*
         * 投影変換
         */
        if (z_dst[i] >= 0.0) {
            image_points->x[i] = x_dst[i] * fx_camera_param / z_dst[i] + ox_camera_param;
            image_points->y[i] = y_dst[i] * fy_camera_param / z_dst[i] + oy_camera_param;
        } else {
            image_points->x[i] = -1;
            image_points->y[i] = -1;
        }
    }

    clock_gettime(CLOCK_REALTIME, &end);
    printf("time:%f msec\n", get_processing_time(start, end));
}

// %Tag(CALLBACK)%
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    Three_dimensional_vector depth_points;
    std::vector<float> distance;
    Two_dimensional_vector image_points;
    int i;
//    ROS_INFO("angle_min[%f]\nangle_max:[%f]\nangle_increment:[%f]\ntime_increment:[%f]\nscan_time:[%f]\nrange_min:[%f]\nrange_max:[%f]\n", msg->angle_min * 180 / 3.141592, msg->angle_max * 180 / 3.141592, msg->angle_increment * 180 / 3.141592, msg->time_increment, msg->scan_time, msg->range_min, msg->range_max);

    distance.resize(msg->ranges.size());
    depth_points.x.resize(msg->ranges.size());
    depth_points.y.resize(msg->ranges.size());
    depth_points.z.resize(msg->ranges.size());
    image_points.x.resize(msg->ranges.size());
    image_points.y.resize(msg->ranges.size());

    for(i = 0; i < msg->ranges.size(); i++) {
        depth_points.x[i] = -1.0 * msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
        depth_points.y[i] = 0;
        depth_points.z[i] = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
    }
    if(manual_mode == 1) {
        manual_trans_depth_points_to_image_points(&depth_points, &image_points, &distance);
    } else {
        printf("auto\n");

        auto_trans_depth_points_to_image_points(&depth_points, &image_points, &distance);
    }

    scan_to_image::PointedImage pointed_image_msg;
    pointed_image_msg.scan = *msg;
    pointed_image_msg.image_point_x = image_points.x;
    pointed_image_msg.image_point_y = image_points.y;
    pointed_image_msg.distance = distance;
    pointed_image.publish(pointed_image_msg);

}


double get_processing_time(struct timespec start, struct timespec end)
{
    long sec, nsec;
    sec =  end.tv_sec - start.tv_sec;
    nsec =  end.tv_nsec - start.tv_nsec;
    if(nsec < 0) {
        sec--;
        nsec += 1000000000L;
    }
    return (double)sec * 100 + (double)nsec/1000000;
}

void write_file_processing_time(struct timespec start, struct timespec end)
{
    FILE *fp;
    fp = fopen("./result.txt", "a+");
    if(fp == NULL){
        printf("%s : cannot open file\n", "result.txt");
        exit(EXIT_FAILURE);
    }
    fprintf(fp,"%lf\n", get_processing_time(start, end));
    fclose(fp);
}


// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{

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
    /* xmlからパラメータ設定 */
    char path[128];
#if 1 // AXE
    ros::init(argc, argv, "point_to_image");
    ros::NodeHandle n;

    std::string camera_yaml;
    n.param<std::string>("/scan_to_image/camera_yaml", camera_yaml, "camera.yaml");
    cv::FileStorage fs_auto_file(camera_yaml.c_str(), cv::FileStorage::READ);
#else
    strcpy(path, getenv("HOME"));
    strcat(path, "/catkin_ws/src/sensors_fusion/camera.yaml");
    // ファイルの種類は，内容から決定
    cv::FileStorage fs_auto_file(path, cv::FileStorage::READ);
#endif
    fs_auto_file["translation_global2lrf"] >> translation_global2lrf;
    fs_auto_file["translation_global2camera"] >> translation_global2camera;
    fs_auto_file["rotation_matrix"] >> rotation;
    fs_auto_file["intrinsic"] >> intrinsic;
    fs_auto_file.release();

#if 1 // AXE
    std::string manual_yaml;
    n.param<std::string>("/scan_to_image/manual_yaml", manual_yaml, "manual.yaml");
    cv::FileStorage fs_manual_file(manual_yaml.c_str(), cv::FileStorage::READ);
#else
    strcpy(path, getenv("HOME"));
    strcat(path, "/catkin_ws/src/sensors_fusion/manual.yaml");
    // ファイルの種類は，内容から決定
    cv::FileStorage fs_manual_file(path, cv::FileStorage::READ);
#endif
    fs_manual_file["manual_mode"] >> manual_mode;
    fs_manual_file["x_theta"] >> x_theta;
    fs_manual_file["y_theta"] >> y_theta;
    fs_manual_file["z_theta"] >> z_theta;
    fs_manual_file["x_vector"] >> x_vector;
    fs_manual_file["y_vector"] >> y_vector;
    fs_manual_file["z_vector"] >> z_vector;
    fs_manual_file["fx_camera_param"] >> fx_camera_param;
    fs_manual_file["fy_camera_param"] >> fy_camera_param;
    fs_manual_file["ox_camera_param"] >> ox_camera_param;
    fs_manual_file["oy_camera_param"] >> oy_camera_param;
    fs_manual_file.release();
    if(manual_mode == 1) {
        printf("manual_mode\n");
    }
    else {
        printf("auto_mode\n");
    }


#if 1 // AXE
#else
    ros::init(argc, argv, "point_to_image");

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

  ros::Subscriber sub = n.subscribe("scan", 1, chatterCallback);
  pointed_image = n.advertise<scan_to_image::PointedImage>("pointed_image", 10);

// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%

  cvDestroyWindow("CAR_TRACKING");
  cvReleaseImage(&IMAGE_PATH);

  return 0;
}
// %EndTag(FULLTEXT)%
