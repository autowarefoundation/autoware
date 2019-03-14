/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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
//
// Created by amc on 2017-11-15.
//
 */
#include <string>
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#define _NODE_NAME_ "image_rectifier"

class ROSImageRectifierApp

{
	ros::Subscriber     subscriber_image_raw_;
	ros::Subscriber     subscriber_intrinsics_;

	ros::Publisher      publisher_image_rectified_;

	cv::Size            image_size_;
	cv::Mat             camera_instrinsics_;
	cv::Mat             distortion_coefficients_;


	void ImageCallback(const sensor_msgs::Image& in_image_sensor)
	{
		//Receive Image, convert it to OpenCV Mat
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
		cv::Mat tmp_image = cv_image->image;
		cv::Mat image;
		if (camera_instrinsics_.empty())
		{
			ROS_INFO("[%s] Make sure camera_info is being published in the specified topic", _NODE_NAME_);
			image = tmp_image;
		}
		else
		{
			cv::undistort(tmp_image, image, camera_instrinsics_, distortion_coefficients_);
		}

		cv_bridge::CvImage out_msg;
		out_msg.header   = in_image_sensor.header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image    = image; // Your cv::Mat

		publisher_image_rectified_.publish(out_msg.toImageMsg());

	}

	void IntrinsicsCallback(const sensor_msgs::CameraInfo& in_message)
	{
		image_size_.height = in_message.height;
		image_size_.width = in_message.width;

		camera_instrinsics_ = cv::Mat(3,3, CV_64F);
		for (int row=0; row<3; row++) {
			for (int col=0; col<3; col++) {
				camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
			}
		}

		distortion_coefficients_ = cv::Mat(1,5,CV_64F);
		for (int col=0; col<5; col++) {
			distortion_coefficients_.at<double>(col) = in_message.D[col];
		}
	}

public:
	void Run()
	{
		ros::NodeHandle node_handle("~");//to receive args

		std::string image_raw_topic_str, camera_info_topic_str, image_rectified_str = "/image_rectified";
		std::string name_space_str = ros::this_node::getNamespace();

		node_handle.param<std::string>("image_src", image_raw_topic_str, "/image_raw");

		node_handle.param<std::string>("camera_info_src", camera_info_topic_str, "/camera_info");

		if (name_space_str != "/") {
			if (name_space_str.substr(0, 2) == "//") {
				/* if name space obtained by ros::this::node::getNamespace()
				   starts with "//", delete one of them */
				name_space_str.erase(name_space_str.begin());
			}
			image_raw_topic_str = name_space_str + image_raw_topic_str;
			image_rectified_str = name_space_str + image_rectified_str;
			camera_info_topic_str = name_space_str + camera_info_topic_str;
		}

		ROS_INFO("[%s] image_src: %s", _NODE_NAME_, image_raw_topic_str.c_str());
		ROS_INFO("[%s] camera_info_src: %s", _NODE_NAME_, camera_info_topic_str.c_str());

		ROS_INFO("[%s] Subscribing to... %s", _NODE_NAME_, image_raw_topic_str.c_str());
		subscriber_image_raw_ = node_handle.subscribe(image_raw_topic_str, 1, &ROSImageRectifierApp::ImageCallback, this);

		ROS_INFO("[%s] Subscribing to... %s", _NODE_NAME_, camera_info_topic_str.c_str());
		subscriber_intrinsics_ = node_handle.subscribe(camera_info_topic_str, 1, &ROSImageRectifierApp::IntrinsicsCallback, this);

		publisher_image_rectified_ = node_handle.advertise<sensor_msgs::Image>(image_rectified_str, 1);
		ROS_INFO("[%s] Publishing Rectified image in %s", _NODE_NAME_, image_rectified_str.c_str());

		ROS_INFO("[%s] Ready. Waiting for data...", _NODE_NAME_);
		ros::spin();
		ROS_INFO("[%s] END rect", _NODE_NAME_);
	}

	~ROSImageRectifierApp()
	{
	}

	ROSImageRectifierApp()
	{
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, _NODE_NAME_);

	ROSImageRectifierApp app;

	app.Run();

	return 0;
}
