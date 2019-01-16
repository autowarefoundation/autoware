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

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include "autoware_msgs/PointsImage.h"

#define IMAGE_WIDTH 800
#define IMAGE_HEIGHT 640

static bool existImage = false;
static bool existPoints = false;
static sensor_msgs::Image image_msg;
static autoware_msgs::PointsImageConstPtr points_msg;
static cv::Mat colormap;

static const char window_name_base[] = "points_image_viewer";
static std::string window_name;

static void show(void)
{
	if(!existImage || !existPoints){
		return;
	}
	const auto& encoding = sensor_msgs::image_encodings::BGR8;
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, encoding);
	IplImage frame = cv_image->image;

	cv::Mat matImage = cv::cvarrToMat(&frame);//(&frame, false);

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

	if (cvGetWindowHandle(window_name.c_str()) != NULL) // Guard not to write destroyed window by using close button on the window
	{
		cvShowImage(window_name.c_str(), &frame);
		cvWaitKey(2);
	}
}

static void image_cb(const sensor_msgs::Image& msg)
{
	image_msg = msg;
	existImage = true;
	show();
}

static void points_cb(const autoware_msgs::PointsImageConstPtr& msg)
{
	points_msg = msg;
	existPoints = true;
	show();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "points_image_viewer");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");

	std::string points_topic;
	if (!private_nh.getParam("points_topic", points_topic)) {
		points_topic = "points_image";
	}

	std::string image_topic;
	if (private_nh.getParam("image_raw_topic", image_topic)) {
		ROS_INFO("Setting image topic to %s", image_topic.c_str());
	} else {
		ROS_INFO("No image topic received, defaulting to image_raw, you can use _image_raw_topic:=YOUR_NODE");
		image_topic = "/image_raw";
	}

	std::string name_space_str = ros::this_node::getNamespace();
	window_name = std::string(window_name_base);
	if (name_space_str != "/") {
		window_name += " (" + name_space_str + ")";
	}

	/* create resizable window */
	cvNamedWindow(window_name.c_str(), CV_WINDOW_NORMAL);
	cvStartWindowThread();

	ros::Subscriber sub_image = n.subscribe(image_topic, 1, image_cb);
	ros::Subscriber sub_points = n.subscribe(points_topic, 1, points_cb);

	cv::Mat grayscale(256,1,CV_8UC1);
	for(int i=0;i<256;i++)
	{
		grayscale.at<uchar>(i)=i;
	}
	cv::applyColorMap(grayscale,colormap,cv::COLORMAP_JET);

	ros::spin();

	cvDestroyWindow(window_name.c_str());
	return 0;
}
