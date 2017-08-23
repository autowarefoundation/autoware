/*
 * enet_image_segmenter_node.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: ne0
 */

#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include "enet_image_segmenter.hpp"

class RosENetSegmenterApp
{
	ros::Subscriber subscriber_image_raw_;
	ros::NodeHandle node_handle_;

	cv::Scalar pixel_mean_;

	//Caffe based Object Detection ConvNet
	ENetSegmenter* enet_segmenter_;

	//If GPU is enabled, stores the GPU Device to use
	unsigned int gpu_device_id_;

	//Sets whether or not use GPU acceleration
	bool use_gpu_;

	//vector of indices of the classes to search for
	std::vector<unsigned int> detect_classes_;

	void image_callback(const sensor_msgs::Image& in_image_sensor)
	{
		//Receive Image, convert it to OpenCV Mat
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
		cv::Mat image = cv_image->image;

		cv::Mat segmented_image;

		enet_segmenter_->Predict(image, segmented_image);

		if (!segmented_image.empty())
		{
		cv::imshow("Segmentation window", segmented_image);
		cv::waitKey(10);
		}
	}


public:
	void Run()
	{
		//ROS STUFF
		ros::NodeHandle private_node_handle("~");//to receive args

		//RECEIVE IMAGE TOPIC NAME
		std::string image_raw_topic_str;
		if (private_node_handle.getParam("image_raw_node", image_raw_topic_str))
		{
			ROS_INFO("Setting image node to %s", image_raw_topic_str.c_str());
		}
		else
		{
			ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_raw_node:=YOUR_TOPIC");
			image_raw_topic_str = "/image_raw";
		}

		//RECEIVE CONVNET FILENAMES
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

		if (private_node_handle.getParam("use_gpu", use_gpu_))
		{
			ROS_INFO("GPU Mode: %d", use_gpu_);
		}
		int gpu_id;
		if (private_node_handle.getParam("gpu_device_id", gpu_id ))
		{
			ROS_INFO("GPU Device ID: %d", gpu_id);
			gpu_device_id_ = (unsigned int) gpu_id;
		}

		//SSD STUFF
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
		use_gpu_ 		= false;
		gpu_device_id_ 	= 0;
		pixel_mean_		= cv::Scalar(102.9801, 115.9465, 122.7717);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ssd_unc");

	RosENetSegmenterApp app;

	app.Run();

	return 0;
}

