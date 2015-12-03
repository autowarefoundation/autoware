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
#include "scan2image/ScanImage.h"
#include <sensor_msgs/CameraInfo.h>

static char window_name[] = "SCAN_IMAGE_VIEWER";
//for imageCallback
static cv_bridge::CvImagePtr cv_image;
static IplImage image;
static scan2image::ScanImage scan_image;
static bool exist_image = false;
static bool exist_scan = false;
static cv::Mat colormap;
static cv::Size imageSize;
static bool isIntrinsic = false;

static void show()
{
    if(!exist_image || !exist_scan || !isIntrinsic){
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

static void scan_image_callback(const scan2image::ScanImage& scan_image_msg)
{
    scan_image = scan_image_msg;
    exist_scan = true;
    show();
}

static void image_callback(const sensor_msgs::Image& image_msg)
{
    cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    image = cv_image->image;
    exist_image = true;
    show();
}

static void intrinsic_callback(const sensor_msgs::CameraInfo& msg)
{
    printf("intrinsic\n");

	imageSize.height = msg.height;
	imageSize.width = msg.width;

    isIntrinsic = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_image_viewer");
    ros::NodeHandle n;

    ros::Subscriber scan_image_sub = n.subscribe("/scan_image", 1, scan_image_callback);
    ros::Subscriber image_sub = n.subscribe("/image_raw", 1, image_callback);
    ros::Subscriber intrinsic_sub = n.subscribe("camera/camera_info", 1, intrinsic_callback);

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
