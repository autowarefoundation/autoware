// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE_RADAR_OBJECT_TRACKER__UTILS__RADAR_OBJECT_TRACKER_UTILS_HPP_
#define AUTOWARE_RADAR_OBJECT_TRACKER__UTILS__RADAR_OBJECT_TRACKER_UTILS_HPP_

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/tracked_object.hpp"
#include <geometry_msgs/msg/transform.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>

#include <string>
#include <utility>

namespace autoware::radar_object_tracker::utils
{

boost::optional<geometry_msgs::msg::Transform> getTransformAnonymous(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time);

bool isDuplicated(
  const std::pair<double, lanelet::ConstLanelet> & target_lanelet,
  const lanelet::ConstLanelets & lanelets);

bool checkCloseLaneletCondition(
  const std::pair<double, lanelet::Lanelet> & lanelet,
  const autoware_perception_msgs::msg::TrackedObject & object, const double max_distance_from_lane,
  const double max_angle_diff_from_lane);

lanelet::ConstLanelets getClosestValidLanelets(
  const autoware_perception_msgs::msg::TrackedObject & object,
  const lanelet::LaneletMapPtr & lanelet_map_ptr, const double max_distance_from_lane,
  const double max_angle_diff_from_lane);

bool hasValidVelocityDirectionToLanelet(
  const autoware_perception_msgs::msg::TrackedObject & object,
  const lanelet::ConstLanelets & lanelets, const double max_lateral_velocity);

}  // namespace autoware::radar_object_tracker::utils

#endif  // AUTOWARE_RADAR_OBJECT_TRACKER__UTILS__RADAR_OBJECT_TRACKER_UTILS_HPP_
