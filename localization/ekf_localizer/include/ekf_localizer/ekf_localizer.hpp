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
    stddev_ = 1e9;
    proc_stddev_x_c_ = 0.0;
    return;
  };
  void init(const double init_obs, const double obs_stddev, const rclcpp::Time time)
  {
    x_ = init_obs;
    stddev_ = obs_stddev;
    latest_time_ = time;
    initialized_ = true;
    return;
  };
  void update(const double obs, const double obs_stddev, const rclcpp::Time time)
  {
    if (!initialized_) {
      init(obs, obs_stddev, time);
      return;
    }

    // Prediction step (current stddev_)
    double dt = (time - latest_time_).seconds();
    double proc_stddev_x_d = proc_stddev_x_c_ * dt;
    stddev_ = std::sqrt(stddev_ * stddev_ + proc_stddev_x_d * proc_stddev_x_d);

    // Update step
    double kalman_gain = stddev_ * stddev_ / (stddev_ * stddev_ + obs_stddev * obs_stddev);
    x_ = x_ + kalman_gain * (obs - x_);
    stddev_ = std::sqrt(1 - kalman_gain) * stddev_;

    latest_time_ = time;
    return;
  };
  void set_proc_stddev(const double proc_stddev) { proc_stddev_x_c_ = proc_stddev; }
  double get_x() { return x_; }

private:
  bool initialized_;
  double x_;
  double stddev_;
  double proc_stddev_x_c_;
  rclcpp::Time latest_time_;
};

class EKFLocalizer : public rclcpp::Node
{
public:
  EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_no_yawbias_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pub_pose_cov_no_yawbias_;
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

  //!< @brief timer to send transform
  rclcpp::TimerBase::SharedPtr timer_tf_;
  //!< @brief tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  //!< @brief  extended kalman filter instance.
  TimeDelayKalmanFilter ekf_;
  Simple1DFilter z_filter_;
  Simple1DFilter roll_filter_;
  Simple1DFilter pitch_filter_;

  /* parameters */
  bool show_debug_info_;
  double ekf_rate_;                  //!< @brief  EKF predict rate
  double ekf_dt_;                    //!< @brief  = 1 / ekf_rate_
  double tf_rate_;                   //!< @brief  tf publish rate
  bool enable_yaw_bias_estimation_;  //!< @brief for LiDAR mount error.
                                     //!< if true,publish /estimate_yaw_bias
  std::string pose_frame_id_;

  int dim_x_;              //!< @brief  dimension of EKF state
  int extend_state_step_;  //!< @brief  for time delay compensation
  int dim_x_ex_;  //!< @brief  dimension of extended EKF state (dim_x_ * extended_state_step)

  /* Pose */
  double pose_additional_delay_;          //!< @brief  compensated pose delay time =
                                          //!< (pose.header.stamp - now) + additional_delay [s]
  double pose_measure_uncertainty_time_;  //!< @brief  added for measurement covariance
  double pose_rate_;  //!< @brief  pose rate [s], used for covariance calculation
  //!< @brief  the mahalanobis distance threshold to ignore pose measurement
  double pose_gate_dist_;

  /* twist */
  double twist_additional_delay_;  //!< @brief  compensated delay = (twist.header.stamp - now)
                                   //!< + additional_delay [s]
  double twist_rate_;              //!< @brief  rate [s], used for covariance calculation
  //!< @brief  measurement is ignored if the mahalanobis distance is larger than this value.
  double twist_gate_dist_;

  /* process noise standard deviation */
  double proc_stddev_yaw_c_;       //!< @brief  yaw process noise
  double proc_stddev_yaw_bias_c_;  //!< @brief  yaw bias process noise
  double proc_stddev_vx_c_;        //!< @brief  vx process noise
  double proc_stddev_wz_c_;        //!< @brief  wz process noise

  /* process noise variance for discrete model */
  double proc_cov_yaw_d_;       //!< @brief  discrete yaw process noise
  double proc_cov_yaw_bias_d_;  //!< @brief  discrete yaw bias process noise
  double proc_cov_vx_d_;        //!< @brief  discrete process noise in d_vx=0
  double proc_cov_wz_d_;        //!< @brief  discrete process noise in d_wz=0

  enum IDX {
    X = 0,
    Y = 1,
    YAW = 2,
    YAWB = 3,
    VX = 4,
    WZ = 5,
  };

  /* for model prediction */
  std::queue<TwistInfo> current_twist_info_queue_;    //!< @brief current measured pose
  std::queue<PoseInfo> current_pose_info_queue_;      //!< @brief current measured pose
  geometry_msgs::msg::PoseStamped current_ekf_pose_;  //!< @brief current estimated pose
  geometry_msgs::msg::PoseStamped
    current_ekf_pose_no_yawbias_;  //!< @brief current estimated pose w/o yaw bias
  geometry_msgs::msg::TwistStamped current_ekf_twist_;  //!< @brief current estimated twist
  std::array<double, 36ul> current_pose_covariance_;
  std::array<double, 36ul> current_twist_covariance_;

  int pose_smoothing_steps_;
  int twist_smoothing_steps_;

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
   * @brief check whether a measurement value falls within the mahalanobis distance threshold
   * @param dist_max mahalanobis distance threshold
   * @param estimated current estimated state
   * @param measured measured state
   * @param estimated_cov current estimation covariance
   * @return whether it falls within the mahalanobis distance threshold
   */
  bool mahalanobisGate(
    const double & dist_max, const Eigen::MatrixXd & estimated, const Eigen::MatrixXd & measured,
    const Eigen::MatrixXd & estimated_cov) const;

  /**
   * @brief get transform from frame_id
   */
  bool getTransformFromTF(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief normalize yaw angle
   * @param yaw yaw angle
   * @return normalized yaw
   */
  double normalizeYaw(const double & yaw) const;

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

  void updateSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch_;

  friend class EKFLocalizerTestSuite;  // for test code
};
#endif  // EKF_LOCALIZER__EKF_LOCALIZER_HPP_
