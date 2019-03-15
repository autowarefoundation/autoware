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
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include <autoware_msgs/VehicleStatus.h>
#include <tf/transform_broadcaster.h>

#include "kalman_filter/kalman_filter.h"
#include "kalman_filter/kalman_filter_delayed_measurement.h"

class KalmanFilterNode
{
public:
  KalmanFilterNode();
  ~KalmanFilterNode();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_pose_, pub_pose_cov_, pub_twist_, pub_debug_, pub_ndt_pose_, pub_yaw_bias_;
  ros::Subscriber sub_initialpose_, sub_ndt_pose_, sub_vehicle_status_, sub_twist_;
  ros::Timer timer_control_, timer_tf_;
  tf::TransformBroadcaster tf_br_;

  KalmanFilterDelayedMeasurement kf_;

  /* parameters */
  bool show_debug_info_;
  double kf_rate_;                  // kalman filter predict rate
  double kf_dt_;                    // = 1 / kf_rate_
  double tf_rate_;                  // tf publish rate
  bool enable_yaw_bias_estimation_; // for LiDAR mount error. if true, publish /estimate_yaw_bias
  std::string pose_frame_id_;

  int dim_x_;             // dimension of kalman state
  int extend_state_step_; // for time delay compensation
  int dim_x_ex_;          // dimension of extended kalman state (dim_x_ * extended_state_step)

  /* NDT */
  double ndt_additional_delay_;         // compensated ndt delay time = (ndt.header.stamp - now) + additional_delay [s]
  double ndt_measure_uncertainty_time_; // added for measurement covariance
  double ndt_rate_;                     // ndt rate [s], used for covariance calculation
  double ndt_gate_dist_;                // ndt measurement is ignored if the maharanobis distance is larger than this value.
  double ndt_stddev_x_;
  double ndt_stddev_y_;
  double ndt_stddev_yaw_;

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
  std::shared_ptr<geometry_msgs::PoseStamped> current_ndt_pose_ptr_;
  std::shared_ptr<sensor_msgs::Imu> current_imu_ptr_;
  geometry_msgs::PoseStamped current_kf_pose_;
  geometry_msgs::TwistStamped current_kf_twist_;

  void timerCallback(const ros::TimerEvent &e);
  void timerTFCallback(const ros::TimerEvent &e);
  void callbackNDTPose(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void callbackTwist(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped &msg);

  void initKalmanFilter();
  void predictKinematicsModel();
  void measurementUpdateNDTPose(const geometry_msgs::PoseStamped &ndt_pose);
  void measurementUpdateIMU(const sensor_msgs::Imu &msg);
  void measurementUpdateTwist(const geometry_msgs::TwistStamped &twist);
  bool mahalanobisGate(const double &dist_max, const Eigen::MatrixXd &x, const Eigen::MatrixXd &obj_x, const Eigen::MatrixXd &cov);

  int getDelayStep(const double &delay_time, const int &extend_state_step, const double &kf_dt);
  void setCurrentResult();
  void publishEstimatedPose();
};
