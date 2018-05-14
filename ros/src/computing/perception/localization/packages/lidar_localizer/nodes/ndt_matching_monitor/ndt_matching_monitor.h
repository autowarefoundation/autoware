/*
 *  Copyright (c) 2018, Nagoya University
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

#include <autoware_msgs/ndt_stat.h>

#define __APP_NAME__ "RosNdtMatchingMonitor"

#define NDT_THRESHOLD_ITERATION_WARN 10
#define NDT_THRESHOLD_ITERATION_STOP 32
#define NDT_THRESHOLD_SCORE_MAX_DELTA 14.
#define NDT_MIN_STABLE_SAMPLES 30
#define NDT_TIME_TO_FATAL_PREDICTIONS 2.

class RosNdtMatchingMonitor
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
	void ndt_stat_callback(const autoware_msgs::ndt_stat::ConstPtr& input);

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
	RosNdtMatchingMonitor();
};

#endif //PROJECT_NDT_MATCHING_MONITOR_H
