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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include "points2image/PointsImage.h"

#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 640

static bool existImage = false;
static bool existPoints = false;
static sensor_msgs::Image image_msg;
static points2image::PointsImageConstPtr points_msg;
static cv::Mat colormap;

static const char window_name[] = "vscan_image_viewer";

static void show(void)
{
	if(!existImage || !existPoints){
		return;
	}
	const auto& encoding = sensor_msgs::image_encodings::BGR8;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, encoding);
	IplImage frame = cv_image->image;

	cv::Mat matImage(&frame, false);

	int w = matImage.size().width;
	int h = matImage.size().height;
	int n = w * h;

	float min_d, max_d;
	min_d = max_d = points_msg->distance[0];
	for(int i=1; i<n; i++){
		float di = points_msg->distance[i];
		max_d = di > max_d ? di : max_d;
		min_d = di < min_d ? di : min_d;
	}
	float wid_d = max_d - min_d;

	for(int y=0; y<h; y++){
		for(int x=0; x<w; x++){
			int j = y * w + x;
			float distance = points_msg->distance[j];
			if(distance == 0){
				continue;
			}
			int colorid= wid_d ? ( (distance - min_d) * 255 / wid_d ) : 128;
			cv::Vec3b color=colormap.at<cv::Vec3b>(colorid);
			int g = color[1];
			int b = color[2];
			int r = color[0];
			cvRectangle(&frame, cvPoint(x, y), cvPoint(x+1, y+1), CV_RGB(r, g, b));
		}
	}

	if (cvGetWindowHandle(window_name) != NULL) // Guard not to write destroyed window by using close button on the window
	{
		cvShowImage(window_name, &frame);
		cvWaitKey(2);
	}
}

static void image_cb(const sensor_msgs::Image& msg)
{
	image_msg = msg;
	existImage = true;
	show();
}

static void points_cb(const points2image::PointsImageConstPtr& msg)
{
	points_msg = msg;
	existPoints = true;
	show();
}

int main(int argc, char **argv)
{
	/* create resizable window */
	cvNamedWindow(window_name, CV_WINDOW_NORMAL);
	cvStartWindowThread();

	ros::init(argc, argv, "vscan_image_viewer");
	ros::NodeHandle n;
	ros::Subscriber sub_image = n.subscribe("image_raw", 1, image_cb);
	ros::Subscriber sub_points = n.subscribe("vscan_image", 1, points_cb);

	cv::Mat grayscale(256,1,CV_8UC1);
	for(int i=0;i<256;i++)
	{
		grayscale.at<uchar>(i)=i;
	}
	cv::applyColorMap(grayscale,colormap,cv::COLORMAP_JET);

	ros::spin();

	cvDestroyWindow(window_name);

	return 0;
}
