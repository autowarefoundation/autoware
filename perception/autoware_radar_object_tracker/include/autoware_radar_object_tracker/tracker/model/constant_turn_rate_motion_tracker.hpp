// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE_RADAR_OBJECT_TRACKER__TRACKER__MODEL__CONSTANT_TURN_RATE_MOTION_TRACKER_HPP_
#define AUTOWARE_RADAR_OBJECT_TRACKER__TRACKER__MODEL__CONSTANT_TURN_RATE_MOTION_TRACKER_HPP_

#include "autoware/kalman_filter/kalman_filter.hpp"
#include "autoware_radar_object_tracker/tracker/model/tracker_base.hpp"

#include <string>
namespace autoware::radar_object_tracker
{
using Label = autoware_perception_msgs::msg::ObjectClassification;
using autoware::kalman_filter::KalmanFilter;
class ConstantTurnRateMotionTracker : public Tracker  // means constant turn rate motion tracker
{
private:
  autoware_perception_msgs::msg::DetectedObject object_;
  rclcpp::Logger logger_;

private:
  KalmanFilter ekf_;
  rclcpp::Time last_update_time_;
  enum IDX { X = 0, Y = 1, YAW = 2, VX = 3, WZ = 4 };

  struct EkfParams
  {
    // dimension
    char dim_x = 5;
    // system noise
    double q_cov_x;
    double q_cov_y;
    double q_cov_yaw;
    double q_cov_vx;
    double q_cov_wz;
    // measurement noise
    double r_cov_x;
    double r_cov_y;
    double r_cov_yaw;
    double r_cov_vx;
    // initial state covariance
    double p0_cov_x;
    double p0_cov_y;
    double p0_cov_yaw;
    double p0_cov_vx;
    double p0_cov_wz;
  };
  static EkfParams ekf_params_;

  // limitation
  static double max_vx_;
  // rough tracking parameters
  float z_;

  // lpf parameter
  static double filter_tau_;  // time constant of 1st order low pass filter
  static double filter_dt_;   // sampling time of 1st order low pass filter

  // static flags
  static bool is_initialized_;
  static bool trust_yaw_input_;
  static bool trust_twist_input_;
  static bool use_polar_coordinate_in_measurement_noise_;
  static bool assume_zero_yaw_rate_;

private:
  struct BoundingBox
  {
    double length;
    double width;
    double height;
  };
  struct Cylinder
  {
    double width;
    double height;
  };
  BoundingBox bounding_box_;
  Cylinder cylinder_;

public:
  ConstantTurnRateMotionTracker(
    const rclcpp::Time & time, const autoware_perception_msgs::msg::DetectedObject & object,
    const std::string & tracker_param_file, const std::uint8_t & label);

  static void loadDefaultModelParameters(const std::string & path);
  bool predict(const rclcpp::Time & time) override;
  bool predict(const double dt, KalmanFilter & ekf) const;
  bool measure(
    const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform) override;
  bool measureWithPose(
    const autoware_perception_msgs::msg::DetectedObject & object,
    const geometry_msgs::msg::Transform & self_transform);
  bool measureWithShape(const autoware_perception_msgs::msg::DetectedObject & object);
  bool getTrackedObject(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObject & object) const override;
  virtual ~ConstantTurnRateMotionTracker() {}
};
}  // namespace autoware::radar_object_tracker
#endif  // AUTOWARE_RADAR_OBJECT_TRACKER__TRACKER__MODEL__CONSTANT_TURN_RATE_MOTION_TRACKER_HPP_
