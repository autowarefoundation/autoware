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
 * multi_lidar_calibrator.h
 *
 *  Created on: Feb 27, 2018
 */

#ifndef PROJECT_MULTI_LIDAR_CALIBRATOR_H
#define PROJECT_MULTI_LIDAR_CALIBRATOR_H


#include <string>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <tf/tf.h>

#define __APP_NAME__ "multi_lidar_calibrator"

class ROSMultiLidarCalibratorApp

{
	ros::NodeHandle                     node_handle_;
	ros::Publisher                      calibrated_cloud_publisher_;

	ros::Subscriber                     initialpose_subscriber_;

	double                              voxel_size_;
	double                              ndt_epsilon_;
	double                              ndt_step_size_;
	double                              ndt_resolution_;

	double                              initial_x_;
	double                              initial_y_;
	double                              initial_z_;
	double                              initial_roll_;
	double                              initial_pitch_;
	double                              initial_yaw_;

	int                                 ndt_iterations_;

	//tf::Quaternion                      initialpose_quaternion_;
	//tf::Vector3                         initialpose_position_;

	std::string                         parent_frame_;
	std::string                         child_frame_;

	Eigen::Matrix4f                     current_guess_;

	typedef
	message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
			sensor_msgs::PointCloud2>   SyncPolicyT;

	typedef pcl::PointXYZI              PointT;

	message_filters::Subscriber<sensor_msgs::PointCloud2>   *cloud_parent_subscriber_, *cloud_child_subscriber_;
	message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

	/*!
	 * Receives 2 synchronized point cloud messages.
	 * @param[in] in_parent_cloud_msg Message containing pointcloud classified as ground.
	 * @param[in] in_child_cloud_msg Message containing pointcloud classified as obstacle.
	 */
	void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_parent_cloud_msg,
	                    const sensor_msgs::PointCloud2::ConstPtr& in_child_cloud_msg);

	//void InitialPoseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr in_initialpose);

	/*!
	 * Obtains parameters from the command line, initializes subscribers and publishers.
	 * @param in_private_handle ROS private handle to get parameters for this node.
	 */
	void InitializeROSIo(ros::NodeHandle& in_private_handle);

	/*!
	 * Applies a Voxel Grid filter to the point cloud
	 * @param in_cloud_ptr point cloud to downsample
	 * @param out_cloud_ptr downsampled point cloud
	 * @param in_leaf_size voxel side size
	 */
	void DownsampleCloud(pcl::PointCloud<PointT>::ConstPtr in_cloud_ptr, pcl::PointCloud<PointT>::Ptr out_cloud_ptr, double in_leaf_size);

	/*!
	 * Publishes a PointCloud in the specified publisher
	 * @param in_publisher Publisher to use
	 * @param in_cloud_to_publish_ptr Cloud to Publish
	 */
	void PublishCloud(const ros::Publisher& in_publisher, pcl::PointCloud<PointT>::ConstPtr in_cloud_to_publish_ptr);

public:
	void Run();

	ROSMultiLidarCalibratorApp();
};

#endif //PROJECT_MULTI_LIDAR_CALIBRATOR_H
