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

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__BICYCLE_TRACKER_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__BICYCLE_TRACKER_HPP_

#include "kalman_filter/kalman_filter.hpp"
#include "multi_object_tracker/tracker/model/tracker_base.hpp"
#include "multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"
#include "multi_object_tracker/tracker/object_model/object_model.hpp"

class BicycleTracker : public Tracker
{
private:
  autoware_perception_msgs::msg::DetectedObject object_;
  rclcpp::Logger logger_;

  object_model::ObjectModel object_model_ = object_model::bicycle;

  double z_;

  struct BoundingBox
  {
    double length;
    double width;
    double height;
  };
  BoundingBox bounding_box_;

  BicycleMotionModel motion_model_;
  using IDX = BicycleMotionModel::IDX;

public:
  BicycleTracker(
    const rclcpp::Time & time, const autoware_perception_msgs::msg::DetectedObject & object,
    const geometry_msgs::msg::Transform & self_transform, const size_t channel_size,
    const uint & channel_index);

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform) override;
  bool measureWithPose(const autoware_perception_msgs::msg::DetectedObject & object);
  bool measureWithShape(const autoware_perception_msgs::msg::DetectedObject & object);
  bool getTrackedObject(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObject & object) const override;

private:
  autoware_perception_msgs::msg::DetectedObject getUpdatingObject(
    const autoware_perception_msgs::msg::DetectedObject & object,
    const geometry_msgs::msg::Transform & self_transform) const;
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__BICYCLE_TRACKER_HPP_
