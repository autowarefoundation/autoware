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
 */

#include <iostream>
#include <vector>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <autoware_msgs/VehicleStatus.h>

#include "amathutils_lib/kalman_filter.hpp"
#include "amathutils_lib/kalman_filter_delayed_measurement.hpp"

class EKFLocalizer
{
public:
  EKFLocalizer();
  ~EKFLocalizer();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_pose_, pub_pose_cov_, pub_twist_, pub_debug_, pub_measured_pose_, pub_yaw_bias_;
  ros::Subscriber sub_initialpose_, sub_pose_, sub_vehicle_status_, sub_twist_, sub_pose_with_cov_;
  ros::Timer timer_control_, timer_tf_;
  tf2_ros::TransformBroadcaster tf_br_;

  KalmanFilterDelayedMeasurement ekf_;

  /* parameters */
  bool show_debug_info_;
  double ekf_rate_;                 // EKF predict rate
  double ekf_dt_;                   // = 1 / ekf_rate_
  double tf_rate_;                  // tf publish rate
  bool enable_yaw_bias_estimation_; // for LiDAR mount error. if true, publish /estimate_yaw_bias
  std::string pose_frame_id_;

  int dim_x_;             // dimension of EKF state
  int extend_state_step_; // for time delay compensation
  int dim_x_ex_;          // dimension of extended EKF state (dim_x_ * extended_state_step)

  /* Pose */
  double pose_additional_delay_;         // compensated pose delay time = (pose.header.stamp - now) + additional_delay [s]
  double pose_measure_uncertainty_time_; // added for measurement covariance
  double pose_rate_;                     // pose rate [s], used for covariance calculation
  double pose_gate_dist_;                // pose measurement is ignored if the maharanobis distance is larger than this value.
  double pose_stddev_x_;
  double pose_stddev_y_;
  double pose_stddev_yaw_;
  bool use_pose_with_covariance_;

  /* twist */
  double twist_additional_delay_; // compensated delay time = (twist.header.stamp - now) + additional_delay [s]
  double twist_rate_;             // rate [s], used for covariance calculation
  double twist_gate_dist_;        // measurement is ignored if the maharanobis distance is larger than this value.
  double twist_stddev_vx_;        // standard deviation for linear vx
  double twist_stddev_wz_;        // standard deviation for angular wx

  /* process noise variance for discrete model */
  double cov_proc_yaw_d_;      // discrete yaw process noise
  double cov_proc_yaw_bias_d_; // discrete yaw bias process noise
  double cov_proc_vx_d_;
  double cov_proc_wz_d_;

  enum IDX
  {
    X = 0,
    Y = 1,
    YAW = 2,
    YAWB = 3,
    VX = 4,
    WZ = 5,
  };

  /* for model prediction */
  std::shared_ptr<geometry_msgs::TwistStamped> current_twist_ptr_;
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;
  geometry_msgs::PoseStamped current_ekf_pose_;
  geometry_msgs::TwistStamped current_ekf_twist_;
  boost::array<double, 36ul> current_pose_covariance_;

  void timerCallback(const ros::TimerEvent &e);
  void timerTFCallback(const ros::TimerEvent &e);
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void callbackPoseWithCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg);

  void initEKF();
  void predictKinematicsModel();
  void measurementUpdatePose(const geometry_msgs::PoseStamped &pose);
  void measurementUpdateTwist(const geometry_msgs::TwistStamped &twist);
  bool mahalanobisGate(const double &dist_max, const Eigen::MatrixXd &x, const Eigen::MatrixXd &obj_x, const Eigen::MatrixXd &cov);
  bool getTransformFromTF(std::string parent_frame, std::string child_frame, geometry_msgs::TransformStamped &transform);
  double normalizeYaw(const double &yaw);

  void setCurrentResult();
  void publishEstimatedPose();

  // debug
  void showCurrentX();
};
