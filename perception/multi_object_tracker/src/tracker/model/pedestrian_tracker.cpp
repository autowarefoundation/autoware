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
#define EIGEN_MPL2_ONLY

#include "multi_object_tracker/tracker/model/pedestrian_tracker.hpp"

#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/math/normalization.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"
#include "multi_object_tracker/utils/utils.hpp"
#include "object_recognition_utils/object_recognition_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

using Label = autoware_perception_msgs::msg::ObjectClassification;

PedestrianTracker::PedestrianTracker(
  const rclcpp::Time & time, const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/, const size_t channel_size,
  const uint & channel_index)
: Tracker(time, object.classification, channel_size),
  logger_(rclcpp::get_logger("PedestrianTracker")),
  z_(object.kinematics.pose_with_covariance.pose.position.z)
{
  object_ = object;

  // initialize existence probability
  initializeExistenceProbabilities(channel_index, object.existence_probability);

  // OBJECT SHAPE MODEL
  bounding_box_ = {
    object_model_.init_size.length, object_model_.init_size.width,
    object_model_.init_size.height};                                             // default value
  cylinder_ = {object_model_.init_size.length, object_model_.init_size.height};  // default value
  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_ = {object.shape.dimensions.x, object.shape.dimensions.z};
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    // do not update polygon shape
  }
  // set maximum and minimum size
  bounding_box_.length = std::clamp(
    bounding_box_.length, object_model_.size_limit.length_min, object_model_.size_limit.length_max);
  bounding_box_.width = std::clamp(
    bounding_box_.width, object_model_.size_limit.width_min, object_model_.size_limit.width_max);
  bounding_box_.height = std::clamp(
    bounding_box_.height, object_model_.size_limit.height_min, object_model_.size_limit.height_max);
  cylinder_.width = std::clamp(
    cylinder_.width, object_model_.size_limit.length_min, object_model_.size_limit.length_max);
  cylinder_.height = std::clamp(
    cylinder_.height, object_model_.size_limit.height_min, object_model_.size_limit.height_max);

  // Set motion model parameters
  {
    const double q_stddev_x = object_model_.process_noise.vel_long;
    const double q_stddev_y = object_model_.process_noise.vel_lat;
    const double q_stddev_yaw = object_model_.process_noise.yaw_rate;
    const double q_stddev_vx = object_model_.process_noise.acc_long;
    const double q_stddev_wz = object_model_.process_noise.acc_turn;
    motion_model_.setMotionParams(q_stddev_x, q_stddev_y, q_stddev_yaw, q_stddev_vx, q_stddev_wz);
  }

  // Set motion limits
  {
    const double max_vel = object_model_.process_limit.vel_long_max;
    const double max_turn_rate = object_model_.process_limit.yaw_rate_max;
    motion_model_.setMotionLimits(max_vel, max_turn_rate);  // maximum velocity and slip angle
  }

  // Set initial state
  {
    using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;
    const double yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);

    auto pose_cov = object.kinematics.pose_with_covariance.covariance;
    if (!object.kinematics.has_position_covariance) {
      // initial state covariance
      const auto & p0_cov_x = object_model_.initial_covariance.pos_x;
      const auto & p0_cov_y = object_model_.initial_covariance.pos_y;
      const auto & p0_cov_yaw = object_model_.initial_covariance.yaw;

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double sin_2yaw = std::sin(2.0 * yaw);
      pose_cov[XYZRPY_COV_IDX::X_X] = p0_cov_x * cos_yaw * cos_yaw + p0_cov_y * sin_yaw * sin_yaw;
      pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5 * (p0_cov_x - p0_cov_y) * sin_2yaw;
      pose_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_x * sin_yaw * sin_yaw + p0_cov_y * cos_yaw * cos_yaw;
      pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];
      pose_cov[XYZRPY_COV_IDX::YAW_YAW] = p0_cov_yaw;
    }

    double vel = 0.0;
    double wz = 0.0;
    if (object.kinematics.has_twist) {
      vel = object.kinematics.twist_with_covariance.twist.linear.x;
      wz = object.kinematics.twist_with_covariance.twist.angular.z;
    }

    double vel_cov = object_model_.initial_covariance.vel_long;
    double wz_cov = object_model_.initial_covariance.yaw_rate;
    if (object.kinematics.has_twist_covariance) {
      vel_cov = object.kinematics.twist_with_covariance.covariance[XYZRPY_COV_IDX::X_X];
      wz_cov = object.kinematics.twist_with_covariance.covariance[XYZRPY_COV_IDX::YAW_YAW];
    }

    // initialize motion model
    motion_model_.initialize(time, x, y, yaw, pose_cov, vel, vel_cov, wz, wz_cov);
  }
}

bool PedestrianTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

autoware_perception_msgs::msg::DetectedObject PedestrianTracker::getUpdatingObject(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/) const
{
  autoware_perception_msgs::msg::DetectedObject updating_object = object;

  // UNCERTAINTY MODEL
  if (!object.kinematics.has_position_covariance) {
    // measurement noise covariance
    auto r_cov_x = object_model_.measurement_covariance.pos_x;
    auto r_cov_y = object_model_.measurement_covariance.pos_y;

    // yaw angle fix
    const double pose_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    const bool is_yaw_available =
      object.kinematics.orientation_availability !=
      autoware_perception_msgs::msg::DetectedObjectKinematics::UNAVAILABLE;

    // fill covariance matrix
    using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    auto & pose_cov = updating_object.kinematics.pose_with_covariance.covariance;
    const double cos_yaw = std::cos(pose_yaw);
    const double sin_yaw = std::sin(pose_yaw);
    const double sin_2yaw = std::sin(2.0 * pose_yaw);
    pose_cov[XYZRPY_COV_IDX::X_X] =
      r_cov_x * cos_yaw * cos_yaw + r_cov_y * sin_yaw * sin_yaw;           // x - x
    pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5 * (r_cov_x - r_cov_y) * sin_2yaw;  // x - y
    pose_cov[XYZRPY_COV_IDX::Y_Y] =
      r_cov_x * sin_yaw * sin_yaw + r_cov_y * cos_yaw * cos_yaw;                   // y - y
    pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];                 // y - x
    pose_cov[XYZRPY_COV_IDX::X_YAW] = 0.0;                                         // x - yaw
    pose_cov[XYZRPY_COV_IDX::Y_YAW] = 0.0;                                         // y - yaw
    pose_cov[XYZRPY_COV_IDX::YAW_X] = 0.0;                                         // yaw - x
    pose_cov[XYZRPY_COV_IDX::YAW_Y] = 0.0;                                         // yaw - y
    pose_cov[XYZRPY_COV_IDX::YAW_YAW] = object_model_.measurement_covariance.yaw;  // yaw - yaw
    if (!is_yaw_available) {
      pose_cov[XYZRPY_COV_IDX::YAW_YAW] *= 1e3;  // yaw is not available, multiply large value
    }
    auto & twist_cov = updating_object.kinematics.twist_with_covariance.covariance;
    twist_cov[XYZRPY_COV_IDX::X_X] = object_model_.measurement_covariance.vel_long;  // vel - vel
  }

  return updating_object;
}

bool PedestrianTracker::measureWithPose(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  // update motion model
  bool is_updated = false;
  {
    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;

    is_updated =
      motion_model_.updateStatePose(x, y, object.kinematics.pose_with_covariance.covariance);
    motion_model_.limitStates();
  }

  // position z
  constexpr double gain = 0.1;
  z_ = (1.0 - gain) * z_ + gain * object.kinematics.pose_with_covariance.pose.position.z;

  return is_updated;
}

bool PedestrianTracker::measureWithShape(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  constexpr double gain = 0.1;
  constexpr double gain_inv = 1.0 - gain;

  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    // check bound box size abnormality
    constexpr double size_max = 30.0;  // [m]
    constexpr double size_min = 0.1;   // [m]
    bool is_size_valid =
      (object.shape.dimensions.x <= size_max && object.shape.dimensions.y <= size_max &&
       object.shape.dimensions.z <= size_max && object.shape.dimensions.x >= size_min &&
       object.shape.dimensions.y >= size_min && object.shape.dimensions.z >= size_min);
    if (!is_size_valid) {
      return false;
    }
    // update bounding box size
    bounding_box_.length = gain_inv * bounding_box_.length + gain * object.shape.dimensions.x;
    bounding_box_.width = gain_inv * bounding_box_.width + gain * object.shape.dimensions.y;
    bounding_box_.height = gain_inv * bounding_box_.height + gain * object.shape.dimensions.z;

  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    // check cylinder size abnormality
    constexpr double size_max = 30.0;  // [m]
    constexpr double size_min = 0.1;   // [m]
    bool is_size_valid =
      (object.shape.dimensions.x <= size_max && object.shape.dimensions.z <= size_max &&
       object.shape.dimensions.x >= size_min && object.shape.dimensions.z >= size_min);
    if (!is_size_valid) {
      return false;
    }
    // update cylinder size
    cylinder_.width = gain_inv * cylinder_.width + gain * object.shape.dimensions.x;
    cylinder_.height = gain_inv * cylinder_.height + gain * object.shape.dimensions.z;

  } else {
    // do not update polygon shape
    return false;
  }

  // set maximum and minimum size
  bounding_box_.length = std::clamp(
    bounding_box_.length, object_model_.size_limit.length_min, object_model_.size_limit.length_max);
  bounding_box_.width = std::clamp(
    bounding_box_.width, object_model_.size_limit.width_min, object_model_.size_limit.width_max);
  bounding_box_.height = std::clamp(
    bounding_box_.height, object_model_.size_limit.height_min, object_model_.size_limit.height_max);
  cylinder_.width = std::clamp(
    cylinder_.width, object_model_.size_limit.length_min, object_model_.size_limit.length_max);
  cylinder_.height = std::clamp(
    cylinder_.height, object_model_.size_limit.height_min, object_model_.size_limit.height_max);

  return true;
}

bool PedestrianTracker::measure(
  const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  // keep the latest input object
  object_ = object;

  const auto & current_classification = getClassification();
  if (object_recognition_utils::getHighestProbLabel(object.classification) == Label::UNKNOWN) {
    setClassification(current_classification);
  }

  // check time gap
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "PedestrianTracker::measure There is a large gap between predicted time and measurement "
      "time. (%f)",
      dt);
  }

  // update object
  const autoware_perception_msgs::msg::DetectedObject updating_object =
    getUpdatingObject(object, self_transform);
  measureWithPose(updating_object);
  measureWithShape(updating_object);

  (void)self_transform;  // currently do not use self vehicle position
  return true;
}

bool PedestrianTracker::getTrackedObject(
  const rclcpp::Time & time, autoware_perception_msgs::msg::TrackedObject & object) const
{
  object = object_recognition_utils::toTrackedObject(object_);
  object.object_id = getUUID();
  object.classification = getClassification();

  auto & pose_with_cov = object.kinematics.pose_with_covariance;
  auto & twist_with_cov = object.kinematics.twist_with_covariance;

  // predict from motion model
  if (!motion_model_.getPredictedState(
        time, pose_with_cov.pose, pose_with_cov.covariance, twist_with_cov.twist,
        twist_with_cov.covariance)) {
    RCLCPP_WARN(logger_, "PedestrianTracker::getTrackedObject: Failed to get predicted state.");
    return false;
  }

  // position
  pose_with_cov.pose.position.z = z_;

  // set shape
  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    object.shape.dimensions.x = bounding_box_.length;
    object.shape.dimensions.y = bounding_box_.width;
    object.shape.dimensions.z = bounding_box_.height;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    object.shape.dimensions.x = cylinder_.width;
    object.shape.dimensions.y = cylinder_.width;
    object.shape.dimensions.z = cylinder_.height;
  } else if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    const auto origin_yaw = tf2::getYaw(object_.kinematics.pose_with_covariance.pose.orientation);
    const auto ekf_pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
    object.shape.footprint =
      autoware::universe_utils::rotatePolygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);
  }

  return true;
}
