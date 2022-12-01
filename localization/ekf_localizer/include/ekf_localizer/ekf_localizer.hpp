// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EKF_LOCALIZER__EKF_LOCALIZER_HPP_
#define EKF_LOCALIZER__EKF_LOCALIZER_HPP_

#include "ekf_localizer/hyper_parameters.hpp"
#include "ekf_localizer/warning.hpp"

#include <kalman_filter/kalman_filter.hpp>
#include <kalman_filter/time_delay_kalman_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tier4_debug_msgs/msg/float64_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

struct PoseInfo
{
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose;
  int counter;
  int smoothing_steps;
};

struct TwistInfo
{
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist;
  int counter;
  int smoothing_steps;
};

class Simple1DFilter
{
public:
  Simple1DFilter()
  {
    initialized_ = false;
    x_ = 0;
    dev_ = 1e9;
    proc_dev_x_c_ = 0.0;
    return;
  };
  void init(const double init_obs, const double obs_dev, const rclcpp::Time time)
  {
    x_ = init_obs;
    dev_ = obs_dev;
    latest_time_ = time;
    initialized_ = true;
    return;
  };
  void update(const double obs, const double obs_dev, const rclcpp::Time time)
  {
    if (!initialized_) {
      init(obs, obs_dev, time);
      return;
    }

    // Prediction step (current stddev_)
    double dt = (time - latest_time_).seconds();
    double proc_dev_x_d = proc_dev_x_c_ * dt * dt;
    dev_ = dev_ + proc_dev_x_d;

    // Update step
    double kalman_gain = dev_ / (dev_ + obs_dev);
    x_ = x_ + kalman_gain * (obs - x_);
    dev_ = (1 - kalman_gain) * dev_;

    latest_time_ = time;
    return;
  };
  void set_proc_dev(const double proc_dev) { proc_dev_x_c_ = proc_dev; }
  double get_x() { return x_; }

private:
  bool initialized_;
  double x_;
  double dev_;
  double proc_dev_x_c_;
  rclcpp::Time latest_time_;
};

class EKFLocalizer : public rclcpp::Node
{
public:
  EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  const Warning warning_;

  //!< @brief ekf estimated pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  //!< @brief estimated ekf pose with covariance publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  //!< @brief estimated ekf odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  //!< @brief ekf estimated twist publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  //!< @brief ekf estimated twist with covariance publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
  //!< @brief debug info publisher
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>::SharedPtr pub_debug_;
  //!< @brief debug measurement pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_measured_pose_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr pub_yaw_bias_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_biased_pose_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_biased_pose_cov_;
  //!< @brief initial pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  //!< @brief measurement pose with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;
  //!< @brief measurement twist with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_;
  //!< @brief time for ekf calculation callback
  rclcpp::TimerBase::SharedPtr timer_control_;
  //!< @brief last predict time
  std::shared_ptr<const rclcpp::Time> last_predict_time_;
  //!< @brief trigger_node service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  //!< @brief timer to send transform
  rclcpp::TimerBase::SharedPtr timer_tf_;
  //!< @brief tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  //!< @brief  extended kalman filter instance.
  TimeDelayKalmanFilter ekf_;
  Simple1DFilter z_filter_;
  Simple1DFilter roll_filter_;
  Simple1DFilter pitch_filter_;

  const HyperParameters params_;

  double ekf_rate_;
  double ekf_dt_;

  /* parameters */

  int dim_x_;     //!< @brief  dimension of EKF state
  int dim_x_ex_;  //!< @brief  dimension of extended EKF state (dim_x_ * extended_state_step)

  /* process noise variance for discrete model */
  double proc_cov_yaw_d_;       //!< @brief  discrete yaw process noise
  double proc_cov_yaw_bias_d_;  //!< @brief  discrete yaw bias process noise
  double proc_cov_vx_d_;        //!< @brief  discrete process noise in d_vx=0
  double proc_cov_wz_d_;        //!< @brief  discrete process noise in d_wz=0

  bool is_activated_;

  /* for model prediction */
  std::queue<TwistInfo> current_twist_info_queue_;    //!< @brief current measured pose
  std::queue<PoseInfo> current_pose_info_queue_;      //!< @brief current measured pose
  geometry_msgs::msg::PoseStamped current_ekf_pose_;  //!< @brief current estimated pose
  geometry_msgs::msg::PoseStamped
    current_biased_ekf_pose_;  //!< @brief current estimated pose without yaw bias correction
  geometry_msgs::msg::TwistStamped current_ekf_twist_;  //!< @brief current estimated twist
  std::array<double, 36ul> current_pose_covariance_;
  std::array<double, 36ul> current_twist_covariance_;

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timerCallback();

  /**
   * @brief publish tf for tf_rate [Hz]
   */
  void timerTFCallback();

  /**
   * @brief set poseWithCovariance measurement
   */
  void callbackPoseWithCovariance(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set twistWithCovariance measurement
   */
  void callbackTwistWithCovariance(geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set initial_pose to current EKF pose
   */
  void callbackInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief initialization of EKF
   */
  void initEKF();

  /**
   * @brief update predict frequency
   */
  void updatePredictFrequency();

  /**
   * @brief compute EKF prediction
   */
  void predictKinematicsModel();

  /**
   * @brief compute EKF update with pose measurement
   * @param pose measurement value
   */
  void measurementUpdatePose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  /**
   * @brief compute EKF update with pose measurement
   * @param twist measurement value
   */
  void measurementUpdateTwist(const geometry_msgs::msg::TwistWithCovarianceStamped & twist);

  /**
   * @brief get transform from frame_id
   */
  bool getTransformFromTF(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief set current EKF estimation result to current_ekf_pose_ & current_ekf_twist_
   */
  void setCurrentResult();

  /**
   * @brief publish current EKF estimation result
   */
  void publishEstimateResult();

  /**
   * @brief for debug
   */
  void showCurrentX();

  /**
   * @brief update simple1DFilter
   */
  void updateSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  /**
   * @brief initialize simple1DFilter
   */
  void initSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  /**
   * @brief trigger node
   */
  void serviceTriggerNode(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;

  friend class EKFLocalizerTestSuite;  // for test code
};
#endif  // EKF_LOCALIZER__EKF_LOCALIZER_HPP_
