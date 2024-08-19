// Copyright 2022 TIER IV, Inc.
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
// Author: v1.0 Yutaka Shimizu
//

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__PASS_THROUGH_TRACKER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__PASS_THROUGH_TRACKER_HPP_

#include "autoware/kalman_filter/kalman_filter.hpp"
#include "tracker_base.hpp"

namespace autoware::multi_object_tracker
{

class PassThroughTracker : public Tracker
{
private:
  autoware_perception_msgs::msg::DetectedObject object_;
  autoware_perception_msgs::msg::DetectedObject prev_observed_object_;
  rclcpp::Logger logger_;
  rclcpp::Time last_update_time_;

public:
  PassThroughTracker(
    const rclcpp::Time & time, const autoware_perception_msgs::msg::DetectedObject & object,
    const geometry_msgs::msg::Transform & self_transform, const size_t channel_size,
    const uint & channel_index);
  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform) override;
  bool getTrackedObject(
    const rclcpp::Time & time,
    autoware_perception_msgs::msg::TrackedObject & object) const override;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__PASS_THROUGH_TRACKER_HPP_
