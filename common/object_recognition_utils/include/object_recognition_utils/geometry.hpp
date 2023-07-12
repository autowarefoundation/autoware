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

#ifndef OBJECT_RECOGNITION_UTILS__GEOMETRY_HPP_
#define OBJECT_RECOGNITION_UTILS__GEOMETRY_HPP_

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace object_recognition_utils
{
template <class T>
geometry_msgs::msg::Pose getPose([[maybe_unused]] const T & p)
{
  static_assert(sizeof(T) == 0, "Only specializations of getPose can be used.");
  throw std::logic_error("Only specializations of getPose can be used.");
}

template <>
inline geometry_msgs::msg::Pose getPose(const geometry_msgs::msg::Pose & p)
{
  return p;
}

template <>
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_perception_msgs::msg::DetectedObject & obj)
{
  return obj.kinematics.pose_with_covariance.pose;
}

template <>
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_perception_msgs::msg::TrackedObject & obj)
{
  return obj.kinematics.pose_with_covariance.pose;
}

template <>
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_perception_msgs::msg::PredictedObject & obj)
{
  return obj.kinematics.initial_pose_with_covariance.pose;
}
}  // namespace object_recognition_utils

#endif  // OBJECT_RECOGNITION_UTILS__GEOMETRY_HPP_
