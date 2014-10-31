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
#include "sensors_fusion/TransformedPointData.h"
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
//end yukky there are not hedarfile
IplImage *IMAGE_PATH;
char WINDOW_NAME[] = "CAR_TRACK";
ros::Publisher transformed_point_data;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

// %Tag(CALLBACK)%
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    Three_dimensional_vector depth_points;
    std::vector<float> distance;
    Two_dimensional_vector image_points;
//    ROS_INFO("angle_min[%f]\nangle_max:[%f]\nangle_increment:[%f]\ntime_increment:[%f]\nscan_time:[%f]\nrange_min:[%f]\nrange_max:[%f]\n", msg->angle_min * 180 / 3.141592, msg->angle_max * 180 / 3.141592, msg->angle_increment * 180 / 3.141592, msg->time_increment, msg->scan_time, msg->range_min, msg->range_max);

    int i;
    distance.resize(msg->ranges.size());
    depth_points.x.resize(msg->ranges.size());
    depth_points.y.resize(msg->ranges.size());
    depth_points.z.resize(msg->ranges.size());
    image_points.x.resize(msg->ranges.size());
    image_points.y.resize(msg->ranges.size());

    for(i = 0; i < msg->ranges.size(); i++) {
        depth_points.x[i] = -1 * msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
        depth_points.y[i] = 0;
        depth_points.z[i] = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
    }
    trans_depth_points_to_image_points(&depth_points, &image_points, &distance);

/*
    for(i = 0; i < msg->ranges.size(); i++) {
        if((msg->angle_min + msg->angle_increment * i) > (msg->range_max)) {
            printf("scan num:%d\n", i);
            break;
        }

        depth_points.x[i] = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i);
        depth_points.y[i] = 0;
        depth_points.z[i] = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i);
    }
    trans_depth_points_to_image_points(&depth_points, &image_points, distance);
*/
    sensors_fusion::TransformedPointData transformed_point_data_msg;
    transformed_point_data_msg.scan = *msg;
    transformed_point_data_msg.image_point_x = image_points.x;
    transformed_point_data_msg.image_point_y = image_points.y;
    transformed_point_data_msg.distance = distance;
    transformed_point_data.publish(transformed_point_data_msg);

}

void trans_depth_points_to_image_points(Three_dimensional_vector* depth_points, Two_dimensional_vector* image_points, std::vector<float> *distance)
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
        y_dst[i] = depth_points->y[i] * cos(X_THETA / 180.0 * M_PI) - depth_points->z[i] * sin(X_THETA / 180.0 * M_PI);
        z_dst[i] = depth_points->y[i] * sin(X_THETA / 180.0 * M_PI) + depth_points->z[i] * cos(X_THETA / 180.0 * M_PI);

        //Y軸回転
        x_dst[i] = depth_points->x[i] * cos(Y_THETA / 180.0 * M_PI) + z_dst[i] * sin(Y_THETA / 180.0 * M_PI);
        z_dst[i] = -1.0 * depth_points->x[i] * sin(Y_THETA / 180.0 * M_PI) + z_dst[i] * cos(Y_THETA / 180.0 * M_PI);

        //Z軸回転
        x_dst[i] = x_dst[i] * cos(Z_THETA / 180.0 * M_PI) - y_dst[i] * sin(Z_THETA / 180.0 * M_PI);
        y_dst[i] = x_dst[i] * sin(Z_THETA / 180.0 * M_PI) + y_dst[i] * cos(Z_THETA / 180.0 * M_PI);

        //平行移動
        x_dst[i] = x_dst[i] + X_VECTOR;
        y_dst[i] = y_dst[i] + Y_VECTOR;
        z_dst[i] = z_dst[i] + Z_VECTOR;

        /*
         * ユークリッド距離算出（カメラと障害物）
         */
        (*distance).at(i) = sqrt(x_dst[i] * x_dst[i] + y_dst[i] * y_dst[i] + z_dst[i] * z_dst[i]);
        /*
         * 投影変換
         */
        if (z_dst[i] > 0.0) {
            image_points->x[i] = x_dst[i] * FX_CAMERA_PARAM / z_dst[i] + OX_CAMERA_PARAM;
            image_points->y[i] = y_dst[i] * FY_CAMERA_PARAM / z_dst[i] + OY_CAMERA_PARAM;
        } else {
            image_points->x[i] = -1;
            image_points->y[i] = -1;
        }
    }

    clock_gettime(CLOCK_REALTIME, &end);
    printf("time:%f msec\n", get_processing_time(start, end));
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


void print_all_point(double *x_src, double *y_src, double *z_src, double *x_dst, double *y_dst, double *z_dst, double *u, double *v) {
    int i;
    for(i = 0; i < SCAN_POINT_NUM; i++) {
        printf("変換前:%f, %f, %f\n変換後:%f, %f, %f\nカメラ上の点:%f, %f\n", x_src[i], y_src[i], z_src[i], x_dst[i], y_dst[i], z_dst[i], u[i], v[i]);
    }
}


// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
//    IMAGE_PATH = cvLoadImage("/home/yukky/workspace/cartrack/CAR_TRACKING/Test_Images/Daytime_Image_PNG/5.png", CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
//    cvNamedWindow("CAR_TRACKING", CV_WINDOW_AUTOSIZE);
//    cvShowImage("CAR_TRACKING", IMAGE_PATH);

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
  ros::init(argc, argv, "transformation");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

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
  transformed_point_data = n.advertise<sensors_fusion::TransformedPointData>("transformed_point_data", 10);

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
