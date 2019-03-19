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
 * ndt_matching_monitor.cpp
 *
 *  Created on: March 20, 2018
 */

#ifndef PROJECT_NDT_MATCHING_MONITOR_H
#define PROJECT_NDT_MATCHING_MONITOR_H

#include <iostream>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <jsk_rviz_plugins/OverlayText.h>

#include <autoware_msgs/NDTStat.h>

#define __APP_NAME__ "ROSNDTMatchingMonitor"

#define NDT_THRESHOLD_ITERATION_WARN 10
#define NDT_THRESHOLD_ITERATION_STOP 32
#define NDT_THRESHOLD_SCORE_MAX_DELTA 14.
#define NDT_MIN_STABLE_SAMPLES 30
#define NDT_TIME_TO_FATAL_PREDICTIONS 2.

class ROSNDTMatchingMonitor
{
	enum ndt_status{
		NDT_NOT_INITIALIZED,
		NDT_OK,
		NDT_WARNING,
		NDT_ERROR,
		NDT_FATAL
	};
	const std::vector<std::string> ndt_status_names = {"NDT_NOT_INITIALIZED", "NDT_OK", "NDT_WARNING", "NDT_ERROR", "NDT_FATAL"};
	ros::Publisher          initialpose_pub_;
	ros::Publisher          overlay_info_text_pub_;
	ros::Publisher			ndt_status_pub_;
	ros::Time               last_prediction_time_;

	double                  last_score_;
	double                  current_score_;
	double                  score_delta_threshold_;
	double                  score_delta_;
	int                     iteration_threshold_warning_;
	int                     iteration_threshold_stop_;
	double                  fatal_time_threshold_;
	ndt_status              ndt_status_;

	int                     iteration_count_;
	bool                    gnss_pose_available_;
	bool                    initialized_;
	int                     min_stable_samples_;
	unsigned int            stable_samples_;
	unsigned int            prediction_samples_;
	std::string             gnss_text_;

	jsk_rviz_plugins::OverlayText ndt_normal_text_;
	jsk_rviz_plugins::OverlayText ndt_warn_text_;
	jsk_rviz_plugins::OverlayText ndt_error_text_;
	jsk_rviz_plugins::OverlayText ndt_fatal_text_;
	jsk_rviz_plugins::OverlayText ndt_not_ready_text_;

	geometry_msgs::PoseWithCovarianceStamped initialpose_;
	geometry_msgs::PoseWithCovarianceStamped prev_initialpose_;
	geometry_msgs::PoseWithCovarianceStamped gnss_pose_;
	geometry_msgs::PoseWithCovarianceStamped prev_gnss_pose_;
	std::deque< geometry_msgs::PoseWithCovarianceStamped > ndt_pose_array_;

	/*!
	 * Callback for NDT statistics
	 * @param input message published by ndt
	 */
	void ndt_stat_callback(const autoware_msgs::NDTStat::ConstPtr& input);

	/*!
	 * Callback for transformation result from NDT
	 * @param input message published by ndt containing the pose of the ego-vehicle
	 */
	void ndt_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& input);

	/*!
	 * If available GNSS will be used to try to reset
	 * @param input message containing the pose as given by the GNSS device
	 */
	void gnss_callback(const geometry_msgs::PoseStamped::ConstPtr& input);

	/*!
	 * Calculates the possible next location based on the current and previous positions
	 * @param prev_pose previous position
	 * @param current_pose current position
	 * @return the estimated position
	 */
	geometry_msgs::PoseWithCovarianceStamped predict_next_pose(geometry_msgs::PoseWithCovarianceStamped prev_pose,
	                                             geometry_msgs::PoseWithCovarianceStamped current_pose);

	/*!
	 * In case of Fatal status a halt will be generated, the only way to recover would be using a reinitialization
	 * @param input the pose as given by rviz
	 */
	void initialpose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& input);
public:
	void Run();
	ROSNDTMatchingMonitor();
};

#endif //PROJECT_NDT_MATCHING_MONITOR_H
