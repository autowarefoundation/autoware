/*
 * camera_mirror.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: ne0
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

class RosCameraMirrorApp

{
	ros::Subscriber 	subscriber_image_raw_;
	ros::Subscriber 	subscriber_camera_info_;

	ros::Publisher 		publisher_image_rect_;
	ros::Publisher 		publisher_image_flip_;

	cv::Mat intrisic_mat_;
	cv::Mat distortion_coeff;


	void ImageCallback(const sensor_msgs::Image& in_image_sensor)
	{
		//Receive Image, convert it to OpenCV Mat
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
		cv::Mat image = cv_image->image;

		cv::flip(image, image, 1);

		cv_bridge::CvImage out_msg;
		out_msg.header   = in_image_sensor.header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image    = image; // Your cv::Mat

		publisher_image_flip_.publish(out_msg.toImageMsg());

	}

	void InfoCallback(const sensor_msgs::CameraInfo& in_info)
	{

	}

public:
	void Run()
	{
		ros::NodeHandle private_node_handle("~");//to receive args

		std::string image_raw_topic_str, camera_info_topic_str,
				image_rect_str = "/image_rect",
				image_flip_str = "/image_flip";

		private_node_handle.param<std::string>("image_src", image_raw_topic_str, "/image_raw");
		private_node_handle.param<std::string>("info_src", camera_info_topic_str, "/camera_info");

		std::string name_space_str = ros::this_node::getNamespace();

		if (name_space_str != "/") {
			if (name_space_str.substr(0, 2) == "//") {
				/* if name space obtained by ros::this::node::getNamespace()
				   starts with "//", delete one of them */
				name_space_str.erase(name_space_str.begin());
			}
			image_raw_topic_str = name_space_str + image_raw_topic_str;
			camera_info_topic_str = name_space_str + camera_info_topic_str;
			image_rect_str = name_space_str + image_rect_str;
			image_flip_str = name_space_str + image_flip_str;
		}

		ROS_INFO("[camera_mirror_node]info_src: %s", camera_info_topic_str.c_str());
		ROS_INFO("[camera_mirror_node]image_src: %s", image_raw_topic_str.c_str());


		ROS_INFO("[camera_mirror_node]Subscribing to... %s", image_raw_topic_str.c_str());
		subscriber_image_raw_ = private_node_handle.subscribe(image_raw_topic_str, 1, &RosCameraMirrorApp::ImageCallback, this);

		ROS_INFO("[camera_mirror_node]Subscribing to... %s", camera_info_topic_str.c_str());
		subscriber_camera_info_ = private_node_handle.subscribe(camera_info_topic_str, 1, &RosCameraMirrorApp::InfoCallback, this);

		publisher_image_rect_ = private_node_handle.advertise<sensor_msgs::Image>(image_rect_str, 1);
		ROS_INFO("[camera_mirror_node]Publishing Rectified image in %s", image_rect_str.c_str());

		publisher_image_flip_ = private_node_handle.advertise<sensor_msgs::Image>(image_flip_str, 1);
		ROS_INFO("[camera_mirror_node]Publishing Flipped image in %s", image_flip_str.c_str());

		ROS_INFO("[camera_mirror_node]Ready. Waiting for data...");
		ros::spin();
		ROS_INFO("[camera_mirror_node]END rect");
	}

	~RosCameraMirrorApp()
	{

	}

	RosCameraMirrorApp()
	{
		intrisic_mat_ = cv::Mat::zeros(3,3, CV_32F);
		distortion_coeff = cv::Mat::zeros(1,5, CV_32F);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "camera_mirror_node");

	RosCameraMirrorApp app;

	app.Run();

	return 0;
}
