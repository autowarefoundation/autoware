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
 * pixel_cloud_fusion.h
 *
 *  Created on: May, 19th, 2018
 */

#ifndef PROJECT_PIXEL_CLOUD_FUSION_H
#define PROJECT_PIXEL_CLOUD_FUSION_H

#define __APP_NAME__ "pixel_cloud_fusion"

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Eigen>

namespace std {
	template <>
	class hash< cv::Point >{
	public :
		size_t operator()(const cv::Point &pixel_cloud ) const
		{
			return hash<std::string>()( std::to_string(pixel_cloud.x) + "|" + std::to_string(pixel_cloud.y) );
		}
	};
};

class ROSPixelCloudFusionApp
{
	ros::NodeHandle                     node_handle_;
	ros::Publisher                      publisher_fused_cloud_;
	ros::Subscriber                     intrinsics_subscriber_;

	tf::TransformListener*              transform_listener_;
	tf::StampedTransform                camera_lidar_tf_;

	cv::Size                            image_size_;
	cv::Mat                             camera_instrinsics_;
	cv::Mat                             distortion_coefficients_;
	cv::Mat                             current_frame_;

	std::string 						image_frame_id_;

	bool                                processing_;
	bool                                camera_info_ok_;
	bool                                camera_lidar_tf_ok_;

	float                               fx_, fy_, cx_, cy_;
	pcl::PointCloud<pcl::PointXYZRGB>   colored_cloud_;

	typedef
	message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> SyncPolicyT;

	ros::Subscriber                     cloud_subscriber_;
	ros::Subscriber                     image_subscriber_;
	message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

	pcl::PointXYZ TransformPoint(const pcl::PointXYZ &in_point, const tf::StampedTransform &in_transform);

	void ImageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg);

	void CloudCallback(const sensor_msgs::PointCloud2::ConstPtr &in_cloud_msg);

	/*!
	 * Obtains Transformation between two transforms registered in the TF Tree
	 * @param in_target_frame
	 * @param in_source_frame
	 * @return the found transformation in the tree
	 */
	tf::StampedTransform
	FindTransform(const std::string &in_target_frame, const std::string &in_source_frame);

	void IntrinsicsCallback(const sensor_msgs::CameraInfo& in_message);

	/*!
	 * Reads the config params from the command line
	 * @param in_private_handle
	 */
	void InitializeROSIo(ros::NodeHandle &in_private_handle);

public:
	void Run();
	ROSPixelCloudFusionApp();
};

#endif //PROJECT_PIXEL_CLOUD_FUSION_H
