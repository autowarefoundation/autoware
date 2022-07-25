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

#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#define EIGEN_MPL2_ONLY
#include "multi_object_tracker/tracker/model/pass_through_tracker.hpp"
#include "multi_object_tracker/utils/utils.hpp"
#include "perception_utils/perception_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

PassThroughTracker::PassThroughTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object)
: Tracker(time, object.classification),
  logger_(rclcpp::get_logger("PassThroughTracker")),
  last_update_time_(time)
{
  object_ = object;
  prev_observed_object_ = object;
}

bool PassThroughTracker::predict(const rclcpp::Time & time)
{
  if (0.5 /*500msec*/ < std::fabs((time - last_update_time_).seconds())) {
    RCLCPP_WARN(
      logger_, "There is a large gap between predicted time and measurement time. (%f)",
      (time - last_update_time_).seconds());
  }

  return true;
}

bool PassThroughTracker::measure(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time)
{
  prev_observed_object_ = object_;
  object_ = object;

  // Update Velocity if the observed object does not have twist information
  const double dt = (time - last_update_time_).seconds();
  if (!object_.kinematics.has_twist && dt > 1e-6) {
    const double dx = object_.kinematics.pose_with_covariance.pose.position.x -
                      prev_observed_object_.kinematics.pose_with_covariance.pose.position.x;
    const double dy = object_.kinematics.pose_with_covariance.pose.position.y -
                      prev_observed_object_.kinematics.pose_with_covariance.pose.position.y;
    object_.kinematics.twist_with_covariance.twist.linear.x = std::hypot(dx, dy) / dt;
  }
  last_update_time_ = time;

  return true;
}

bool PassThroughTracker::getTrackedObject(
  const rclcpp::Time & time, autoware_auto_perception_msgs::msg::TrackedObject & object) const
{
  object = perception_utils::toTrackedObject(object_);
  object.object_id = getUUID();
  object.classification = getClassification();
  object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_X] = 0.0;
  object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::X_Y] = 0.0;
  object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_X] = 0.0;
  object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y] = 0.0;
  object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::Z_Z] = 0.0;
  object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = 0.0;
  object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = 0.0;
  object.kinematics.pose_with_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW] = 0.0;

  // twist covariance
  object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X] = 0.0;
  object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::Y_Y] = 0.0;
  object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::Z_Z] = 0.0;
  object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::ROLL_ROLL] = 0.0;
  object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::PITCH_PITCH] = 0.0;
  object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW] = 0.0;

  const double dt = (time - last_update_time_).seconds();
  if (0.5 /*500msec*/ < dt) {
    RCLCPP_WARN(
      logger_, "There is a large gap between last updated time and current time. (%f)",
      (time - last_update_time_).seconds());
  }

  return true;
}
