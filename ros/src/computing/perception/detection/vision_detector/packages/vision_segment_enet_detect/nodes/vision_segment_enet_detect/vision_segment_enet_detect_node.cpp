/*
 * image_segmenter_enet_node.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: ne0
 */

#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "vision_segment_enet_detect.h"

class RosENetSegmenterApp
{
	ros::Subscriber subscriber_image_raw_;
	ros::NodeHandle node_handle_;

	ros::Publisher publisher_image_segmented_;
	ros::Publisher publisher_image_segmented_blended_;

	ENetSegmenter* enet_segmenter_;

	void image_callback(const sensor_msgs::Image& in_image_sensor)
	{
		//Receive Image, convert it to OpenCV Mat
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
		cv::Mat image = cv_image->image;

		cv::Mat segmented_mat;
		cv::Mat blended_mat;

		enet_segmenter_->Predict(image, segmented_mat);

		if (!segmented_mat.empty())
		{
			addWeighted( image, 0.4, segmented_mat, 0.6, 0.0, blended_mat);

			cv_bridge::CvImage segmented_rosimage;
			segmented_rosimage.header = in_image_sensor.header; segmented_rosimage.encoding = "bgr8";
			segmented_rosimage.image = segmented_mat;

			cv_bridge::CvImage blended_rosimage;
			blended_rosimage.header = in_image_sensor.header; blended_rosimage.encoding = "bgr8";
			blended_rosimage.image = blended_mat;

			publisher_image_segmented_.publish(segmented_rosimage.toImageMsg());
			publisher_image_segmented_blended_.publish(blended_rosimage.toImageMsg());
		}
	}

public:
	void Run()
	{
		ros::NodeHandle private_node_handle("~");//to receive args

		std::string image_raw_topic_str;
		if (private_node_handle.getParam("image_src", image_raw_topic_str))
		{
			ROS_INFO("Setting image node to %s", image_raw_topic_str.c_str());
		}
		else
		{
			ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_src:=YOUR_TOPIC");
			image_raw_topic_str = "/image_raw";
		}

		std::string network_definition_file;
		std::string pretrained_model_file;
		std::string lookuptable_file;
		if (private_node_handle.getParam("network_definition_file", network_definition_file))
		{
			ROS_INFO("Network Definition File: %s", network_definition_file.c_str());
		}
		else
		{
			ROS_INFO("No Network Definition File was received. Finishing execution.");
			return;
		}
		if (private_node_handle.getParam("pretrained_model_file", pretrained_model_file))
		{
			ROS_INFO("Pretrained Model File: %s", pretrained_model_file.c_str());
		}
		else
		{
			ROS_INFO("No Pretrained Model File was received. Finishing execution.");
			return;
		}
		if (private_node_handle.getParam("lookuptable_file", lookuptable_file))
		{
			ROS_INFO("lookuptable File: %s", lookuptable_file.c_str());
		}
		else
		{
			ROS_INFO("No lookuptable File was received. Finishing execution.");
			return;
		}

		enet_segmenter_ = new ENetSegmenter(network_definition_file,
												pretrained_model_file,
												lookuptable_file);

		if (NULL == enet_segmenter_)
		{
			ROS_INFO("Error while creating ENetSegmenter Object");
			return;
		}
		ROS_INFO("ENetSegmenter initialized.");

		ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &RosENetSegmenterApp::image_callback, this);

		publisher_image_segmented_ = node_handle_.advertise<sensor_msgs::Image>("/image_segmented", 1);
		ROS_INFO("Publishing /image_segmented");
		publisher_image_segmented_blended_ = node_handle_.advertise<sensor_msgs::Image>("/image_segmented_blended", 1);
		ROS_INFO("Publishing /image_segmented_blended");

		ROS_INFO("Waiting for data...");
		ros::spin();
		ROS_INFO("END ENetSegmenter");
	}

	~RosENetSegmenterApp()
	{
		if (NULL != enet_segmenter_)
			delete enet_segmenter_;
	}

	RosENetSegmenterApp()
	{
		enet_segmenter_ = NULL;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_segmenter_enet");

	RosENetSegmenterApp app;

	app.Run();

	return 0;
}
