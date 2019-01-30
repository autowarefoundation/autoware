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

#define _NODE_NAME_ "image_rotator"

class ROSImageRotatorApp

{
	ros::Subscriber     subscriber_image_raw_;
	ros::Subscriber     subscriber_camera_info_;

	ros::Publisher      publisher_image_rotated_;

	int                 rotation_angle_;
	bool                mirror_vertical_;
	bool                mirror_horizontal_;


	void ImageCallback(const sensor_msgs::Image& in_image_sensor)
	{
		//Receive Image, convert it to OpenCV Mat
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
		cv::Mat image = cv_image->image;

		if(mirror_vertical_)
			{cv::flip(image, image, 0);}
		if(mirror_horizontal_)
			{cv::flip(image, image, 1);}

		switch(rotation_angle_)
		{
			case 0:
				break;
			case 90:
				transpose(image, image);
				flip(image, image, 1);
				break;
			case 180:
				flip(image, image, -1);
				break;
			case 270:
				transpose(image, image);
				flip(image, image, 0);
				break;
			default:
				cv::Mat rotation_matrix = cv::getRotationMatrix2D( cv::Point(image.cols/2, image.rows/2), (rotation_angle_ ), 1 );
				cv::warpAffine( image, image, rotation_matrix, image.size() );
		}

		cv_bridge::CvImage out_msg;
		out_msg.header   = in_image_sensor.header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image    = image; // Your cv::Mat

		publisher_image_rotated_.publish(out_msg.toImageMsg());

	}

public:
	void Run()
	{
		ros::NodeHandle node_handle("~");//to receive args

		std::string image_raw_topic_str, image_rotated_str = "/image_rotated";
		std::string name_space_str = ros::this_node::getNamespace();

		node_handle.param<std::string>("image_src", image_raw_topic_str, "/image_raw");
		node_handle.param("rotation_angle", rotation_angle_, 0);
		ROS_INFO("[%s] rotation_angle: %d", _NODE_NAME_, rotation_angle_);

		node_handle.param("mirror_vertical", mirror_vertical_, false);
		ROS_INFO("[%s] mirror_vertical: %d", _NODE_NAME_, mirror_vertical_);

		node_handle.param("mirror_horizontal", mirror_horizontal_, false);
		ROS_INFO("[%s] mirror_horizontal: %d", _NODE_NAME_, mirror_horizontal_);

		if (name_space_str != "/") {
			if (name_space_str.substr(0, 2) == "//") {
				/* if name space obtained by ros::this::node::getNamespace()
				   starts with "//", delete one of them */
				name_space_str.erase(name_space_str.begin());
			}
			image_raw_topic_str = name_space_str + image_raw_topic_str;
			image_rotated_str = name_space_str + image_rotated_str;
		}

		ROS_INFO("[%s] image_src: %s", _NODE_NAME_, image_raw_topic_str.c_str());

		ROS_INFO("[%s] Subscribing to... %s", _NODE_NAME_, image_raw_topic_str.c_str());
		subscriber_image_raw_ = node_handle.subscribe(image_raw_topic_str, 1, &ROSImageRotatorApp::ImageCallback, this);

		publisher_image_rotated_ = node_handle.advertise<sensor_msgs::Image>(image_rotated_str, 1);
		ROS_INFO("[%s] Publishing Rotated image in %s", _NODE_NAME_, image_rotated_str.c_str());

		ROS_INFO("[%s] Ready. Waiting for data...", _NODE_NAME_);
		ros::spin();
		ROS_INFO("[%s] END rot", _NODE_NAME_);
	}

	~ROSImageRotatorApp()
	{
	}

	ROSImageRotatorApp()
	{
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, _NODE_NAME_);

	ROSImageRotatorApp app;

	app.Run();

	return 0;
}
