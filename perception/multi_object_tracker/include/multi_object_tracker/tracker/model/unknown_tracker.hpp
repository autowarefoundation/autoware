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
//
//
// v1.0 Yukihiro Saito
//

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__UNKNOWN_TRACKER_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__UNKNOWN_TRACKER_HPP_

#include "tracker_base.hpp"

#include <kalman_filter/kalman_filter.hpp>

class UnknownTracker : public Tracker
{
private:
  autoware_auto_perception_msgs::msg::DetectedObject object_;
  rclcpp::Logger logger_;

private:
  KalmanFilter ekf_;
  rclcpp::Time last_update_time_;
  enum IDX {
    X = 0,
    Y = 1,
    VX = 2,
    VY = 3,
  };
  struct EkfParams
  {
    char dim_x = 4;
    float q_cov_x;
    float q_cov_y;
    float q_cov_vx;
    float q_cov_vy;
    float p0_cov_vx;
    float p0_cov_vy;
    float r_cov_x;
    float r_cov_y;
    float p0_cov_x;
    float p0_cov_y;
  } ekf_params_;
  float max_vx_, max_vy_;
  float z_;

public:
  UnknownTracker(
    const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object);

  bool predict(const rclcpp::Time & time) override;
  bool predict(const double dt, KalmanFilter & ekf) const;
  bool measure(
    const autoware_auto_perception_msgs::msg::DetectedObject & object,
    const rclcpp::Time & time) override;
  bool measureWithPose(const autoware_auto_perception_msgs::msg::DetectedObject & object);
  bool measureWithShape(const autoware_auto_perception_msgs::msg::DetectedObject & object);
  bool getTrackedObject(
    const rclcpp::Time & time,
    autoware_auto_perception_msgs::msg::TrackedObject & object) const override;
  virtual ~UnknownTracker() {}
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__UNKNOWN_TRACKER_HPP_
