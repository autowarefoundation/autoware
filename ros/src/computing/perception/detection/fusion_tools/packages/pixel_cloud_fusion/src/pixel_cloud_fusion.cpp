/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 *
 * pixel_cloud_fusion.cpp
 *
 *  Created on: May, 19th, 2018
 */


#include "pixel_cloud_fusion/pixel_cloud_fusion.h"

pcl::PointXYZ
ROSPixelCloudFusionApp::TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform)
{
	tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
	tf::Vector3 tf_point_t = in_transform * tf_point;
	return pcl::PointXYZ(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

void ROSPixelCloudFusionApp::ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg)
{
	if (!camera_info_ok_)
	{
		ROS_INFO("[%s] Waiting for Intrinsics to be available.", __APP_NAME__);
		return;
	}
	if (processing_)
		return;

	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, "bgr8");
	cv::Mat in_image = cv_image->image;

	cv::Mat undistorted_image;
	cv::undistort(in_image, current_frame_, camera_instrinsics_, distortion_coefficients_);

	image_frame_id_ = in_image_msg->header.frame_id;
	image_size_.height = current_frame_.rows;
	image_size_.width = current_frame_.cols;
}

void ROSPixelCloudFusionApp::CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg)
{
	if (current_frame_.empty() || image_frame_id_ == "")
	{
		ROS_INFO("[%s] Waiting for Image frame to be available.", __APP_NAME__);
		return;
	}
	if (!camera_lidar_tf_ok_)
	{
		camera_lidar_tf_ = FindTransform(image_frame_id_,
		                                 in_cloud_msg->header.frame_id);
	}
	if (!camera_info_ok_ || !camera_lidar_tf_ok_)
	{
		ROS_INFO("[%s] Waiting for Camera-Lidar TF and Intrinsics to be available.", __APP_NAME__);
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*in_cloud_msg, *in_cloud);
	std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;

	std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
	for (size_t i = 0; i < in_cloud->points.size(); i++)
	{
		cam_cloud[i] = TransformPoint(in_cloud->points[i], camera_lidar_tf_);
		int u = int(cam_cloud[i].x * fx_ / cam_cloud[i].z + cx_);
		int v = int(cam_cloud[i].y * fy_ / cam_cloud[i].z + cy_);
		if ((u >= 0) && (u < image_size_.width)
			&& (v >= 0) && (v < image_size_.height)
			&& cam_cloud[i].z > 0
				)
		{
			projection_map.insert(std::pair<cv::Point, pcl::PointXYZ>(cv::Point(u, v), in_cloud->points[i]));
		}
	}

	out_cloud->points.clear();

#pragma omp for
	for (int row = 0; row < image_size_.height; row++)
	{
		for (int col = 0; col < image_size_.width; col++)
		{
			std::unordered_map<cv::Point, pcl::PointXYZ>::const_iterator iterator_3d_2d;
			pcl::PointXYZ corresponding_3d_point;
			pcl::PointXYZRGB colored_3d_point;
			iterator_3d_2d = projection_map.find(cv::Point(col, row));
			if (iterator_3d_2d != projection_map.end())
			{
				corresponding_3d_point = iterator_3d_2d->second;
				cv::Vec3b rgb_pixel = current_frame_.at<cv::Vec3b>(row, col);
				colored_3d_point.x = corresponding_3d_point.x;
				colored_3d_point.y = corresponding_3d_point.y;
				colored_3d_point.z = corresponding_3d_point.z;
				colored_3d_point.r = rgb_pixel[2];
				colored_3d_point.g = rgb_pixel[1];
				colored_3d_point.b = rgb_pixel[0];
				out_cloud->points.push_back(colored_3d_point);
			}
		}
	}
	// Publish PC
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::toROSMsg(*out_cloud, cloud_msg);
	cloud_msg.header = in_cloud_msg->header;
	publisher_fused_cloud_.publish(cloud_msg);
}

void ROSPixelCloudFusionApp::IntrinsicsCallback(const sensor_msgs::CameraInfo &in_message)
{
	image_size_.height = in_message.height;
	image_size_.width = in_message.width;

	camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
		}
	}

	distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		distortion_coefficients_.at<double>(col) = in_message.D[col];
	}

	fx_ = static_cast<float>(in_message.P[0]);
	fy_ = static_cast<float>(in_message.P[5]);
	cx_ = static_cast<float>(in_message.P[2]);
	cy_ = static_cast<float>(in_message.P[6]);

	intrinsics_subscriber_.shutdown();
	camera_info_ok_ = true;
	ROS_INFO("[%s] CameraIntrinsics obtained.", __APP_NAME__);
}

tf::StampedTransform
ROSPixelCloudFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
	tf::StampedTransform transform;

	camera_lidar_tf_ok_ = false;
	try
	{
		transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
		camera_lidar_tf_ok_ = true;
		ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
	}

	return transform;
}

void ROSPixelCloudFusionApp::InitializeROSIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_src, image_src, camera_info_src, fused_topic_str = "/points_fused";
	std::string name_space_str = ros::this_node::getNamespace();

	ROS_INFO("[%s] This node requires: Registered TF(Lidar-Camera), CameraInfo, Image, and PointCloud.", __APP_NAME__);
	in_private_handle.param<std::string>("points_src", points_src, "/points_raw");
	ROS_INFO("[%s] points_src: %s", __APP_NAME__, points_src.c_str());

	in_private_handle.param<std::string>("image_src", image_src, "/image_rectified");
	ROS_INFO("[%s] image_src: %s", __APP_NAME__, image_src.c_str());

	in_private_handle.param<std::string>("camera_info_src", camera_info_src, "/camera_info");
	ROS_INFO("[%s] camera_info_src: %s", __APP_NAME__, camera_info_src.c_str());

	if (name_space_str != "/")
	{
		if (name_space_str.substr(0, 2) == "//")
		{
			name_space_str.erase(name_space_str.begin());
		}
		image_src = name_space_str + image_src;
		fused_topic_str = name_space_str + fused_topic_str;
		camera_info_src = name_space_str + camera_info_src;
	}

	//generate subscribers and sychronizers
	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, camera_info_src.c_str());
	intrinsics_subscriber_ = in_private_handle.subscribe(camera_info_src,
	                                                     1,
	                                                     &ROSPixelCloudFusionApp::IntrinsicsCallback, this);

	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, image_src.c_str());
	cloud_subscriber_ = in_private_handle.subscribe(image_src,
	                                                1,
	                                                &ROSPixelCloudFusionApp::ImageCallback, this);
	ROS_INFO("[%s] Subscribing to... %s", __APP_NAME__, points_src.c_str());
	image_subscriber_ = in_private_handle.subscribe(points_src,
	                                                1,
	                                                &ROSPixelCloudFusionApp::CloudCallback, this);

	publisher_fused_cloud_ = node_handle_.advertise<sensor_msgs::PointCloud2>(fused_topic_str, 1);
	ROS_INFO("[%s] Publishing fused pointcloud in %s", __APP_NAME__, fused_topic_str.c_str());

}


void ROSPixelCloudFusionApp::Run()
{
	ros::NodeHandle private_node_handle("~");
	tf::TransformListener transform_listener;

	transform_listener_ = &transform_listener;

	InitializeROSIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END", __APP_NAME__);
}

ROSPixelCloudFusionApp::ROSPixelCloudFusionApp()
{
	camera_lidar_tf_ok_ = false;
	camera_info_ok_ = false;
	processing_ = false;
	image_frame_id_ = "";
}