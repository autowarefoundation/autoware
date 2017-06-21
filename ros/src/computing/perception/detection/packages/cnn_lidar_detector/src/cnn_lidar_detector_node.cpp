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
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#if (CV_MAJOR_VERSION == 3)
	#include <opencv2/imgcodecs.hpp>
#else
	#include <opencv2/contrib/contrib.hpp>
#endif

#include <cmath>

#include "cnn_lidar_detector.hpp"

class RosLidarDetectorApp
{
	ros::Subscriber subscriber_image_raw_;
	ros::Publisher publisher_pointcloud_class_;
	ros::Publisher publisher_boxes_;
	ros::Publisher publisher_depth_image_;
	ros::Publisher publisher_intensity_image_;
	ros::NodeHandle node_handle_;

	//Caffe based Object Detection ConvNet
	CnnLidarDetector* lidar_detector_;

	//Sets whether or not use GPU acceleration
	bool use_gpu_;

	unsigned int gpu_device_id_;

	const double pi_ = 3.14159265;

	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
		//get cloud and project to image

		double horizontal_res = 0.32 * pi_ /180.;
		double vertical_res = 0.4*pi_/180.;

		unsigned int image_width = 2.8/horizontal_res;
		unsigned int image_height = 0.63 / vertical_res;

		cv::Mat projected_cloud_depth(image_height, image_width, CV_64F, 0.);//depth image
		cv::Mat projected_cloud_height(image_height, image_width, CV_64F, 0.);//height image
		cv::Mat projected_cloud_intensity(image_height, image_width, CV_64F, 0.);//height image

		std::vector<cv::Point2d> image_points;
		std::vector<pcl::PointXYZI> cloud_points;

		image_points.resize(current_sensor_cloud_ptr->points.size());
		cloud_points.resize(current_sensor_cloud_ptr->points.size());

		for(unsigned int i=0; i<current_sensor_cloud_ptr->points.size(); i++)
		{
			pcl::PointXYZI point = current_sensor_cloud_ptr->points[i];
			if (point.x > 5. && point.x <70.)
			{
				double theta = atan2(point.y, point.x);
				double length = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
				double angle = asin(point.z/length);
				double depth = sqrt(point.x*point.x + point.y*point.y);
				unsigned int image_x = floor((theta/horizontal_res) + 250.5);
				unsigned int image_y = floor((angle/vertical_res) + 71.);

				if ( (image_x >= 0) && (image_x < image_width) &&
					 (image_y >= 0) && (image_y < image_height)) //check the projection is inside the image
				{
					projected_cloud_depth.at<double>(image_y, image_x) = depth;
					projected_cloud_height.at<double>(image_y, image_x) = point.z;
					projected_cloud_intensity.at<double>(image_y, image_x) = point.intensity;
					//store correspondence between cloud and image coords
					image_points[i] = cv::Point2d(image_x, image_y);
					cloud_points[i] = point;
				}
			}
		}

		cv::flip(projected_cloud_depth, projected_cloud_depth,-1);

		cv::Mat grayscale_cloud;
		double min;
		double max;
		cv::minMaxIdx(projected_cloud_depth, &min, &max);
		projected_cloud_depth.convertTo(grayscale_cloud,CV_8UC1, 255 / (max-min), -min);
		
		cv::Mat rainbow_cloud;
		//Apply the colormap:
		applyColorMap(grayscale_cloud, rainbow_cloud, cv::COLORMAP_JET);
		
		//move back to ROS
		sensor_msgs::ImagePtr depth_image_msg;
		depth_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rainbow_cloud).toImageMsg();
		depth_image_msg->header.seq = current_sensor_cloud_ptr->header.seq;
		depth_image_msg->header.frame_id = current_sensor_cloud_ptr->header.frame_id;
		depth_image_msg->header.stamp = ros::Time(current_sensor_cloud_ptr->header.stamp);
		publisher_depth_image_.publish(depth_image_msg);

		///////////////////intensity
		cv::flip(projected_cloud_intensity, projected_cloud_intensity,-1);
		cv::minMaxIdx(projected_cloud_intensity, &min, &max);
		projected_cloud_intensity.convertTo(grayscale_cloud,CV_8UC1, 255 / (max-min), -min);
		applyColorMap(grayscale_cloud, rainbow_cloud, cv::COLORMAP_JET);

		sensor_msgs::ImagePtr intensity_image_msg;
		intensity_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rainbow_cloud).toImageMsg();
		intensity_image_msg->header.seq = current_sensor_cloud_ptr->header.seq;
		intensity_image_msg->header.frame_id = current_sensor_cloud_ptr->header.frame_id;
		intensity_image_msg->header.stamp = ros::Time(current_sensor_cloud_ptr->header.stamp);
		publisher_intensity_image_.publish(intensity_image_msg);
	}//end cloud_callback



public:
	void Run()
	{
		//ROS STUFF
		ros::NodeHandle private_node_handle("~");//to receive args

		//RECEIVE TOPIC NAME
		std::string cloud_topic_str;
		if (private_node_handle.getParam("points_node", cloud_topic_str))
		{
			ROS_INFO("Setting input cloud node to %s", cloud_topic_str.c_str());
		}
		else
		{
			ROS_INFO("No cloud node received, defaulting to /points_raw, you can use _points_raw_node:=YOUR_TOPIC");
			cloud_topic_str = "/points_raw";
		}

		//RECEIVE CONVNET FILENAMES
		/*std::string network_definition_file;
		std::string pretrained_model_file;
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
		}*/

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

		/*lidar_detector_ = new CnnLidarDetector(network_definition_file, pretrained_model_file, use_gpu_, gpu_device_id_);

		if (NULL == lidar_detector_)
		{
			ROS_INFO("Error while creating LidarDetector Object");
			return;
		}
		ROS_INFO("CNN Lidar Detector initialized.");*/

		publisher_pointcloud_class_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/points_class",1);
		publisher_boxes_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes",1);
		publisher_depth_image_= node_handle_.advertise<sensor_msgs::Image>("/image_depth",1);
		publisher_intensity_image_= node_handle_.advertise<sensor_msgs::Image>("/image_intensity",1);

		ROS_INFO("Subscribing to... %s", cloud_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(cloud_topic_str, 1, &RosLidarDetectorApp::cloud_callback, this);


		ros::spin();
		ROS_INFO("END Ssd");

	}

	~RosLidarDetectorApp()
	{
		if (NULL != lidar_detector_)
			delete lidar_detector_;
	}

	RosLidarDetectorApp()
	{
		lidar_detector_ 	= NULL;
		use_gpu_ 		= false;
		gpu_device_id_ 	= 0;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cnn_lidar_detector");

	RosLidarDetectorApp app;

	app.Run();

	return 0;
}
