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

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <scan2image/ScanImage.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include "scan2image.h"

#if 1 // AXE
#define XSTR(x) #x
#define STR(x) XSTR(x)
#endif
#define _MANUAL 0

ros::Publisher transformed_point_data;
cv::Mat translation_global2lrf, translation_global2camera, rotation, intrinsic; //for auto mode
double x_theta, y_theta, z_theta, x_vector, y_vector, z_vector, fx_camera_param, fy_camera_param, ox_camera_param, oy_camera_param; // for manual mode
int manual_mode;

void auto_trans_depth_points_to_image_points(Scan_points_dataset* scan_points_dataset, Image_points_dataset* image_points_dataset)
{
    float global_x;
    float global_y;
    float global_z;
    float camera_x;
    float camera_y;
    float camera_z;
    int i;

    for(i = 0; i < (int)scan_points_dataset->scan_points.x.size(); i++) {

        /*
         * Coordinate transformation. Change from laser range finder coordinate to global coordinate
         */
        global_x = (-1.0 * scan_points_dataset->scan_points.y.at(i)) - (translation_global2lrf.at<float>(0,1)/10);
        global_y = scan_points_dataset->scan_points.x.at(i) - (translation_global2lrf.at<float>(0,0)/10);
        global_z = -1.0 * (scan_points_dataset->scan_points.z.at(i) - (translation_global2lrf.at<float>(0,2)/10));

        /*
         * Coordinate transformation. Change from global coordinate to camera coordinate
         */
        /* Rotation matrix */
        camera_x = rotation.at<float>(0,0) * global_x + rotation.at<float>(1,0) * global_y + rotation.at<float>(2,0) * global_z;
        camera_y = rotation.at<float>(0,1) * global_x + rotation.at<float>(1,1) * global_y + rotation.at<float>(2,1) * global_z;
        camera_z = rotation.at<float>(0,2) * global_x + rotation.at<float>(1,2) * global_y + rotation.at<float>(2,2) * global_z;
        /* Transration vector */
        camera_x = camera_x + (translation_global2camera.at<float>(0,0)/10);
        camera_y = camera_y + (translation_global2camera.at<float>(0,1)/10);
        camera_z = camera_z + (translation_global2camera.at<float>(0,2)/10);

        if (camera_z >= 0.0) {
            /*
             * Projection transformation. Change from camera coordinate to image coordinate
             */
            image_points_dataset->image_points.x.push_back(camera_x * intrinsic.at<float>(0,0) / camera_z + intrinsic.at<float>(0,2));
            image_points_dataset->image_points.y.push_back(camera_y * intrinsic.at<float>(1,1) / camera_z + intrinsic.at<float>(1,2));

            /*
             * Calculate euclidean distance from the camera to objects
             */
            image_points_dataset->distance.push_back(sqrt(camera_x * camera_x + camera_y * camera_y + camera_z * camera_z)); //unit of length is centimeter

            /*
             * Copy to intensity
             */
            if(!(scan_points_dataset->intensity.empty())){
                image_points_dataset->intensity.push_back(scan_points_dataset->intensity.at(i));
            }
        }
    }
}

void manual_trans_depth_points_to_image_points(Scan_points_dataset* scan_points_dataset, Image_points_dataset* image_points_dataset)
{
    float camera_x;
    float camera_y;
    float camera_z;
    float temp;
    int i;

    for(i = 0; i < (int)scan_points_dataset->scan_points.x.size(); i++) {

        /*
         * Coordinate transformation. Change from laser range finder coordinate to camera coordinate
         */
        /* X-axis rotation */
        camera_y = scan_points_dataset->scan_points.y.at(i) * cos(x_theta / 180.0 * M_PI) - scan_points_dataset->scan_points.z.at(i) * sin(x_theta / 180.0 * M_PI);
        camera_z = scan_points_dataset->scan_points.y.at(i) * sin(x_theta / 180.0 * M_PI) + scan_points_dataset->scan_points.z.at(i) * cos(x_theta / 180.0 * M_PI);
        /* Y-axis rotation */
        camera_x = scan_points_dataset->scan_points.x.at(i) * cos(y_theta / 180.0 * M_PI) + camera_z * sin(y_theta / 180.0 * M_PI);
        camera_z = -1.0 * scan_points_dataset->scan_points.x.at(i) * sin(y_theta / 180.0 * M_PI) + camera_z * cos(y_theta / 180.0 * M_PI);
        /* Z-axis rotation */
        temp = camera_x;
        camera_x = camera_x * cos(z_theta / 180.0 * M_PI) - camera_y * sin(z_theta / 180.0 * M_PI);
        camera_y = temp * sin(z_theta / 180.0 * M_PI) + camera_y * cos(z_theta / 180.0 * M_PI);
        /* Transration vector */
        camera_x = camera_x + x_vector;
        camera_y = camera_y + y_vector;
        camera_z = camera_z + z_vector;

        if (camera_z >= 0.0) {
            /*
             * Projection transformation. Change from camera coordinate to image coordinate
             */
            image_points_dataset->image_points.x.push_back((int)(camera_x * intrinsic.at<float>(0,0) / camera_z + intrinsic.at<float>(0,2)));
            image_points_dataset->image_points.y.push_back((int)(camera_y * intrinsic.at<float>(1,1) / camera_z + intrinsic.at<float>(1,2)));

            /*
             * Calculate euclidean distance from the camera to objects
             */
            image_points_dataset->distance.push_back(sqrt(camera_x * camera_x + camera_y * camera_y + camera_z * camera_z)); //unit of length is centimeter

            /*
             * Copy to intensity
             */
            if(!(scan_points_dataset->intensity.empty())){
                image_points_dataset->intensity.push_back(scan_points_dataset->intensity.at(i));
            }
        }
    }
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    Scan_points_dataset scan_points_dataset;
    Image_points_dataset image_points_dataset;
    int i;

//    ROS_INFO("angle_min[%f]\nangle_max:[%f]\nangle_increment:[%f]\ntime_increment:[%f]\nscan_time:[%f]\nrange_min:[%f]\nrange_max:[%f]\n", msg->angle_min * 180 / 3.141592, msg->angle_max * 180 / 3.141592, msg->angle_increment * 180 / 3.141592, msg->time_increment, msg->scan_time, msg->range_min, msg->range_max);

    /*
     * Initialize
     */
    scan_points_dataset.scan_points.x.resize(msg->ranges.size());
    scan_points_dataset.scan_points.y.resize(msg->ranges.size());
    scan_points_dataset.scan_points.z.resize(msg->ranges.size());
    scan_points_dataset.intensity.resize(msg->intensities.size());
    image_points_dataset.image_points.x.clear();
    image_points_dataset.image_points.y.clear();
    image_points_dataset.distance.clear();
    image_points_dataset.intensity.clear();

    /*
     * Change to three dimentional coordinate. And copy intensity
     */
    for(i = 0; i < (int)msg->ranges.size(); i++) {
        scan_points_dataset.scan_points.x.at(i) = -1.0 * msg->ranges.at(i) * sin(msg->angle_min + msg->angle_increment * i) * 100; //unit of length is centimeter
        scan_points_dataset.scan_points.y.at(i) = 0; //unit of length is centimeter
        scan_points_dataset.scan_points.z.at(i) = msg->ranges.at(i) * cos(msg->angle_min + msg->angle_increment * i) * 100; //unit of length is centimeter
        if(!(msg->intensities.empty())){
            scan_points_dataset.intensity.at(i) = msg->intensities.at(i);
        }
    }

    /*
     * Change from laser range finder coordinate to image coordinate
     */
#if _MANUAL
    if(manual_mode == 1) {
        manual_trans_depth_points_to_image_points(&scan_points_dataset, &image_points_dataset);
    } else {
        auto_trans_depth_points_to_image_points(&scan_points_dataset, &image_points_dataset);
    }
#else
    auto_trans_depth_points_to_image_points(&scan_points_dataset, &image_points_dataset);
#endif

    /*
     * Judge out of image frame. And Determine max_y and min_y
     */
    Scan_image scan_image = {{}, {}, NO_DATA, NO_DATA};
    for (i = 0; i < (int)image_points_dataset.image_points.x.size(); i++) {
        /* Judge NaN */
        if(isnan(image_points_dataset.image_points.x.at(i)) == 1 || isnan(image_points_dataset.image_points.y.at(i)) == 1) {
            std::cout <<"Not a Number is i:" << i << std::endl;
            continue;
        }
        /* Judge out of X-axis image */
        if(0 > (int)image_points_dataset.image_points.x.at(i) || (int)image_points_dataset.image_points.x.at(i) > IMAGE_WIDTH - 1) {
            continue;
        }
        /* Judge out of Y-axis image */
        if(0 > (int)image_points_dataset.image_points.y.at(i) || (int)image_points_dataset.image_points.y.at(i) > IMAGE_HEIGHT - 1) {
            continue;
        }

        scan_image.distance[(int)image_points_dataset.image_points.x.at(i)][(int)image_points_dataset.image_points.y.at(i)] = image_points_dataset.distance.at(i);
        if(!msg->intensities.empty()){
            scan_image.intensity[(int)image_points_dataset.image_points.x.at(i)][(int)image_points_dataset.image_points.y.at(i)] = image_points_dataset.intensity.at(i);
        }

        if ((scan_image.max_y < (int)image_points_dataset.image_points.y.at(i)) || (scan_image.max_y == NO_DATA)) {
            scan_image.max_y = (int)image_points_dataset.image_points.y.at(i);
        } else if ((scan_image.min_y > (int)image_points_dataset.image_points.y.at(i)) || (scan_image.min_y == NO_DATA)) {
            scan_image.min_y = (int)image_points_dataset.image_points.y.at(i);
        }
    }

    /*
     * Create message(Topic)
     */
    scan2image::ScanImage scan_image_msg;
    scan_image_msg.header = msg->header;
    scan_image_msg.distance.assign(scan_image.distance[0], scan_image.distance[0] + IMAGE_WIDTH * IMAGE_HEIGHT);
    scan_image_msg.intensity.assign(scan_image.intensity[0], scan_image.intensity[0] + IMAGE_WIDTH * IMAGE_HEIGHT);
    scan_image_msg.max_y = scan_image.max_y;
    scan_image_msg.min_y = scan_image.min_y;

    /*
     * Publish message(Topic)
     */
    transformed_point_data.publish(scan_image_msg);
}

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

    ros::init(argc, argv, "scan2image");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;


    /*
     * Read parameters from yaml file
     */
    std::string camera_yaml;
    n.param<std::string>("/scan2image/camera_yaml", camera_yaml, STR(CAMERA_YAML));
    cv::FileStorage fs_auto_file(camera_yaml.c_str(), cv::FileStorage::READ);
    if(!fs_auto_file.isOpened()){
        fprintf(stderr, "%s : cannot open file\n", camera_yaml.c_str());
        exit(EXIT_FAILURE);
    }
    fs_auto_file["translation_global2lrf"] >> translation_global2lrf;
    fs_auto_file["translation_global2camera"] >> translation_global2camera;
    fs_auto_file["rotation_matrix"] >> rotation;
    fs_auto_file["intrinsic"] >> intrinsic;
    fs_auto_file.release();

#if _MANUAL
    std::string manual_yaml;
    n.param<std::string>("/scan2image/manual_yaml", manual_yaml, STR(MANUAL_YAML));
    cv::FileStorage fs_manual_file(manual_yaml.c_str(), cv::FileStorage::READ);
    if(!fs_manual_file.isOpened()){
        fprintf(stderr, "%s : cannot open file\n", manual_yaml.c_str());
        exit(EXIT_FAILURE);
    }
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
        std::cout << "manual mode" << std::endl;
    }
    else {
        std::cout << "auto mode" << std::endl;
    }
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

  ros::Subscriber sub = n.subscribe("scan", 1, scanCallback);
  transformed_point_data = n.advertise<scan2image::ScanImage>("scan_image", 1);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
