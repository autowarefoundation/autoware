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

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__PEDESTRIAN_AND_BICYCLE_TRACKER_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__PEDESTRIAN_AND_BICYCLE_TRACKER_HPP_

#include "multi_object_tracker/tracker/model/bicycle_tracker.hpp"
#include "multi_object_tracker/tracker/model/pedestrian_tracker.hpp"
#include "multi_object_tracker/tracker/model/tracker_base.hpp"

#include <kalman_filter/kalman_filter.hpp>

class PedestrianAndBicycleTracker : public Tracker
{
private:
  PedestrianTracker pedestrian_tracker_;
  BicycleTracker bicycle_tracker_;

public:
  PedestrianAndBicycleTracker(
    const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object);

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const autoware_auto_perception_msgs::msg::DetectedObject & object,
    const rclcpp::Time & time) override;
  bool getTrackedObject(
    const rclcpp::Time & time,
    autoware_auto_perception_msgs::msg::TrackedObject & object) const override;
  virtual ~PedestrianAndBicycleTracker() {}
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__PEDESTRIAN_AND_BICYCLE_TRACKER_HPP_
