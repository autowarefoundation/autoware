/*
 * Copyright 2016-2019 Autoware Foundation. All rights reserved.
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
#include <ros/ros.h>
#include "ros/package.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <thread>
#include <signal.h>
#include<unistd.h>

#include "VectaCam.h"

class ROSVectaCam
{
public:
	void Run()
	{
		std::string config_file_path;
		std::string camera_ip;

		ros::NodeHandle private_node_handle("~");

		if (private_node_handle.getParam("configfile", config_file_path))
		{
			ROS_INFO("Setting config file path to %s", config_file_path.c_str());
		}
		else
		{
			ROS_INFO("No config file received. Terminating...");
			config_file_path = ros::package::getPath(ros::this_node::getName())+"/initialization_params.txt";
			//return;
		}
		if (private_node_handle.getParam("camera_ip", camera_ip))
		{
			ROS_INFO("Setting camera IP to %s", camera_ip.c_str());
		}
		else
		{
			ROS_INFO("No IP, defaulting to %s, you can use _img_obj_node:=YOUR_TOPIC", VECTACAM_CAMERA_IP);
			camera_ip = VECTACAM_CAMERA_IP;
		}

		for (unsigned int i=0; i< VECTACAM_NUM_CAMERAS; i++)
		{
			std::string current_topic = "camera" + std::to_string(i) + "/image_raw";
			publishers_cameras_[i] = node_handle_.advertise<sensor_msgs::Image>(current_topic, 1);
		}

		VectaCam vectacamera(camera_ip, VECTACAM_CONFIG_PORT, VECTACAM_DATA_PORT, config_file_path);
		std::thread *capture_thread= new std::thread(&VectaCam::StartCamera, &vectacamera);

		cv::Mat image;
		std::vector<cv::Mat> camera_images(VECTACAM_NUM_CAMERAS);
		unsigned long int counter = 0;
		ros::Rate loop_rate(7); // Hz
		ros::Publisher full_publisher = node_handle_.advertise<sensor_msgs::Image>("image_raw", 1);
		while(ros::ok())
		{
			vectacamera.GetImage(image);
			if(!image.empty())
			{
				cv::flip(image, image, 0);
				_publish_image(image, full_publisher, counter, VECTACAM_NUM_CAMERAS);
				for (unsigned int i=0; i< VECTACAM_NUM_CAMERAS; i++)
				{
					camera_images[i]= image(cv::Rect(i*image.cols/VECTACAM_NUM_CAMERAS, 0, image.cols/VECTACAM_NUM_CAMERAS,image.rows));
					_publish_image(camera_images[i], publishers_cameras_[i], counter, i);
				}

				counter++;
				if (counter<=2)
					std::cout << "Image received" << std::endl;
			}
			else
			{
				std::cout << "No image received" << std::endl;
			}
			loop_rate.sleep();
		}
		vectacamera.StopCamera();
		capture_thread->join();
		delete capture_thread;
	}
private:
	ros::Publisher 		publishers_cameras_[VECTACAM_NUM_CAMERAS];
	ros::NodeHandle 	node_handle_;

	void _publish_image(cv::Mat &in_image, ros::Publisher &in_publisher, unsigned long int in_counter, size_t camera_id)
	{
		sensor_msgs::ImagePtr msg;
		std_msgs::Header header;
		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", in_image).toImageMsg();
		msg->header.frame_id = "camera" + std::to_string(camera_id);
		msg->header.stamp.sec = ros::Time::now().sec;
		msg->header.stamp.nsec = ros::Time::now().nsec;
		msg->header.seq = in_counter;

		in_publisher.publish(msg);
	}
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "tier_fusion");

	ROSVectaCam app;

	app.Run();

	return 0;
}
