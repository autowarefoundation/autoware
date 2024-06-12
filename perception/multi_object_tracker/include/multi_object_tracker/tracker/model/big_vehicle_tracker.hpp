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
// Author: v1.0 Yukihiro Saito
//

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__BIG_VEHICLE_TRACKER_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__BIG_VEHICLE_TRACKER_HPP_

#include "multi_object_tracker/tracker/model/tracker_base.hpp"
#include "multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"

#include <kalman_filter/kalman_filter.hpp>

class BigVehicleTracker : public Tracker
{
private:
  autoware_perception_msgs::msg::DetectedObject object_;
  rclcpp::Logger logger_;

private:
  struct EkfParams
  {
    double r_cov_x;
    double r_cov_y;
    double r_cov_yaw;
    double r_cov_vel;
  } ekf_params_;
  double velocity_deviation_threshold_;

  double z_;

  struct BoundingBox
  {
    double length;
    double width;
    double height;
  };
  BoundingBox bounding_box_;
  Eigen::Vector2d tracking_offset_;

private:
  BicycleMotionModel motion_model_;
  const char DIM = motion_model_.DIM;
  using IDX = BicycleMotionModel::IDX;

public:
  BigVehicleTracker(
    const rclcpp::Time & time, const autoware_perception_msgs::msg::DetectedObject & object,
    const geometry_msgs::msg::Transform & self_transform, const size_t channel_size,
    const uint & channel_index);

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform) override;
  autoware_perception_msgs::msg::DetectedObject getUpdatingObject(
    const autoware_perception_msgs::msg::DetectedObject & object,
    const geometry_msgs::msg::Transform & self_transform);
  bool measureWithPose(const autoware_perception_msgs::msg::DetectedObject & object);
  bool measureWithShape(const autoware_perception_msgs::msg::DetectedObject & object);
  bool getTrackedObject(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObject & object) const override;
  virtual ~BigVehicleTracker() {}
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__BIG_VEHICLE_TRACKER_HPP_
