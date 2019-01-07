/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

//openCV library
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "autoware_msgs/ScanImage.h"
#include <sensor_msgs/CameraInfo.h>

static char window_name[] = "SCAN_IMAGE_VIEWER";
//for imageCallback
static cv_bridge::CvImagePtr cv_image;
static IplImage image;
static autoware_msgs::ScanImage scan_image;
static bool exist_image = false;
static bool exist_scan = false;
static cv::Mat colormap;
static cv::Size imageSize;

static void show()
{
    if(!exist_image || !exist_scan){
        return;
    }

    IplImage* image_view = cvCreateImage(cvGetSize(&image), image.depth, image.nChannels);
    cvCopy(&image, image_view);

    float min_d, max_d;
    min_d = max_d = scan_image.distance.at(0);
    for(int i = 1; i < imageSize.width * imageSize.height; i++){
        float di = scan_image.distance.at(i);
        max_d = di > max_d ? di : max_d;
        min_d = di < min_d ? di : min_d;
    }

    float wid_d = max_d - min_d;

    /*
     * Plot depth points on an image
     */
    CvPoint pt;
    for(int i = 0; i < (int)scan_image.distance.size(); i++) {
        int height = (int)(i % imageSize.height);
        int width = (int)(i / imageSize.height);
        if(scan_image.distance.at(i) != 0.0) {
            pt.x = width;
            pt.y = height;
			int colorid= wid_d ? ( (scan_image.distance.at(i) - min_d) * 255 / wid_d ) : 128;
			cv::Vec3b color=colormap.at<cv::Vec3b>(colorid);
			int g = color[1];
			int b = color[2];
			int r = color[0];
            cvCircle(image_view, pt, 2, CV_RGB (r, g, b), CV_FILLED, 8, 0);
        }
    }
    /*
     * Show image
     */
    cvShowImage(window_name, image_view);
    cvWaitKey(2);
    cvReleaseImage(&image_view);
}

static void scan_image_callback(const autoware_msgs::ScanImage& scan_image_msg)
{
    scan_image = scan_image_msg;
    exist_scan = true;
    show();
}

static void image_callback(const sensor_msgs::Image& image_msg)
{
    imageSize.height = image_msg.height;
    imageSize.width = image_msg.width;
    cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    image = cv_image->image;
    exist_image = true;
    show();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_image_viewer");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");
    std::string image_topic_name;

    if (private_nh.getParam("image_raw_topic", image_topic_name)) {
        ROS_INFO("Setting image topic to %s", image_topic_name.c_str());
    } else {
        ROS_INFO("No image topic received, defaulting to image_raw, you can use _image_raw_topic:=YOUR_NODE");
        image_topic_name = "/image_raw";
    }

    ros::Subscriber scan_image_sub = n.subscribe("/scan_image", 1, scan_image_callback);
    ros::Subscriber image_sub = n.subscribe(image_topic_name, 1, image_callback);

    cv::Mat grayscale(256,1,CV_8UC1);
    for(int i = 0; i < 256; i++) {
        grayscale.at<uchar>(i)=i;
    }
    cv::applyColorMap(grayscale, colormap, cv::COLORMAP_JET);
    cvNamedWindow(window_name, 2);

    ros::spin();

    cvDestroyWindow(window_name);
    return 0;
}
