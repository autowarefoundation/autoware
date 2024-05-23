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

#include "multi_object_tracker/tracker/model/pedestrian_tracker.hpp"

#include "multi_object_tracker/utils/utils.hpp"

#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <bits/stdc++.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include "object_recognition_utils/object_recognition_utils.hpp"

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

PedestrianTracker::PedestrianTracker(
  const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/, const size_t channel_size,
  const uint & channel_index)
: Tracker(time, object.classification, channel_size),
  logger_(rclcpp::get_logger("PedestrianTracker")),
  z_(object.kinematics.pose_with_covariance.pose.position.z)
{
  object_ = object;

  // initialize existence probability
  initializeExistenceProbabilities(channel_index, object.existence_probability);

  // Initialize parameters
  // measurement noise covariance
  float r_stddev_x = 0.4;                                  // [m]
  float r_stddev_y = 0.4;                                  // [m]
  float r_stddev_yaw = tier4_autoware_utils::deg2rad(30);  // [rad]
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);
  ekf_params_.r_cov_yaw = std::pow(r_stddev_yaw, 2.0);

  // OBJECT SHAPE MODEL
  bounding_box_ = {0.5, 0.5, 1.7};
  cylinder_ = {0.3, 1.7};
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_ = {object.shape.dimensions.x, object.shape.dimensions.z};
  }
  // set minimum size
  bounding_box_.length = std::max(bounding_box_.length, 0.3);
  bounding_box_.width = std::max(bounding_box_.width, 0.3);
  bounding_box_.height = std::max(bounding_box_.height, 0.3);
  cylinder_.width = std::max(cylinder_.width, 0.3);
  cylinder_.height = std::max(cylinder_.height, 0.3);

  // Set motion model parameters
  {
    constexpr double q_stddev_x = 0.5;                                  // [m/s]
    constexpr double q_stddev_y = 0.5;                                  // [m/s]
    constexpr double q_stddev_yaw = tier4_autoware_utils::deg2rad(20);  // [rad/s]
    constexpr double q_stddev_vx = 9.8 * 0.3;                           // [m/(s*s)]
    constexpr double q_stddev_wz = tier4_autoware_utils::deg2rad(30);   // [rad/(s*s)]
    motion_model_.setMotionParams(q_stddev_x, q_stddev_y, q_stddev_yaw, q_stddev_vx, q_stddev_wz);
  }

  // Set motion limits
  motion_model_.setMotionLimits(
    tier4_autoware_utils::kmph2mps(100), /* [m/s] maximum velocity */
    30.0                                 /* [deg/s] maximum turn rate */
  );

  // Set initial state
  {
    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;
    const double yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    auto pose_cov = object.kinematics.pose_with_covariance.covariance;
    double vel = 0.0;
    double wz = 0.0;
    double vel_cov;
    double wz_cov;

    if (object.kinematics.has_twist) {
      vel = object.kinematics.twist_with_covariance.twist.linear.x;
      wz = object.kinematics.twist_with_covariance.twist.angular.z;
    }

    if (!object.kinematics.has_position_covariance) {
      // initial state covariance
      constexpr double p0_stddev_x = 2.0;  // in object coordinate [m]
      constexpr double p0_stddev_y = 2.0;  // in object coordinate [m]
      constexpr double p0_stddev_yaw =
        tier4_autoware_utils::deg2rad(1000);  // in map coordinate [rad]
      constexpr double p0_cov_x = std::pow(p0_stddev_x, 2.0);
      constexpr double p0_cov_y = std::pow(p0_stddev_y, 2.0);
      constexpr double p0_cov_yaw = std::pow(p0_stddev_yaw, 2.0);

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double sin_2yaw = std::sin(2.0 * yaw);
      pose_cov[utils::MSG_COV_IDX::X_X] =
        p0_cov_x * cos_yaw * cos_yaw + p0_cov_y * sin_yaw * sin_yaw;
      pose_cov[utils::MSG_COV_IDX::X_Y] = 0.5 * (p0_cov_x - p0_cov_y) * sin_2yaw;
      pose_cov[utils::MSG_COV_IDX::Y_Y] =
        p0_cov_x * sin_yaw * sin_yaw + p0_cov_y * cos_yaw * cos_yaw;
      pose_cov[utils::MSG_COV_IDX::Y_X] = pose_cov[utils::MSG_COV_IDX::X_Y];
      pose_cov[utils::MSG_COV_IDX::YAW_YAW] = p0_cov_yaw;
    }

    if (!object.kinematics.has_twist_covariance) {
      constexpr double p0_stddev_vel =
        tier4_autoware_utils::kmph2mps(120);  // in object coordinate [m/s]
      constexpr double p0_stddev_wz =
        tier4_autoware_utils::deg2rad(360);  // in object coordinate [rad/s]
      vel_cov = std::pow(p0_stddev_vel, 2.0);
      wz_cov = std::pow(p0_stddev_wz, 2.0);
    } else {
      vel_cov = object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::X_X];
      wz_cov = object.kinematics.twist_with_covariance.covariance[utils::MSG_COV_IDX::YAW_YAW];
    }

    // initialize motion model
    motion_model_.initialize(time, x, y, yaw, pose_cov, vel, vel_cov, wz, wz_cov);
  }
}

bool PedestrianTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

autoware_auto_perception_msgs::msg::DetectedObject PedestrianTracker::getUpdatingObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/)
{
  autoware_auto_perception_msgs::msg::DetectedObject updating_object = object;

  // UNCERTAINTY MODEL
  if (!object.kinematics.has_position_covariance) {
    const double & r_cov_x = ekf_params_.r_cov_x;
    const double & r_cov_y = ekf_params_.r_cov_y;
    auto & pose_cov = updating_object.kinematics.pose_with_covariance.covariance;
    const double pose_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    const double cos_yaw = std::cos(pose_yaw);
    const double sin_yaw = std::sin(pose_yaw);
    const double sin_2yaw = std::sin(2.0f * pose_yaw);
    pose_cov[utils::MSG_COV_IDX::X_X] =
      r_cov_x * cos_yaw * cos_yaw + r_cov_y * sin_yaw * sin_yaw;                // x - x
    pose_cov[utils::MSG_COV_IDX::X_Y] = 0.5f * (r_cov_x - r_cov_y) * sin_2yaw;  // x - y
    pose_cov[utils::MSG_COV_IDX::Y_Y] =
      r_cov_x * sin_yaw * sin_yaw + r_cov_y * cos_yaw * cos_yaw;            // y - y
    pose_cov[utils::MSG_COV_IDX::Y_X] = pose_cov[utils::MSG_COV_IDX::X_Y];  // y - x
  }
  return updating_object;
}

bool PedestrianTracker::measureWithPose(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
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
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  constexpr double gain = 0.1;
  constexpr double gain_inv = 1.0 - gain;

  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_.length = gain_inv * bounding_box_.length + gain * object.shape.dimensions.x;
    bounding_box_.width = gain_inv * bounding_box_.width + gain * object.shape.dimensions.y;
    bounding_box_.height = gain_inv * bounding_box_.height + gain * object.shape.dimensions.z;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    cylinder_.width = gain_inv * cylinder_.width + gain * object.shape.dimensions.x;
    cylinder_.height = gain_inv * cylinder_.height + gain * object.shape.dimensions.z;
  } else {
    return false;
  }

  // set minimum size
  bounding_box_.length = std::max(bounding_box_.length, 0.3);
  bounding_box_.width = std::max(bounding_box_.width, 0.3);
  bounding_box_.height = std::max(bounding_box_.height, 0.3);
  cylinder_.width = std::max(cylinder_.width, 0.3);
  cylinder_.height = std::max(cylinder_.height, 0.3);

  return true;
}

bool PedestrianTracker::measure(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
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
  const autoware_auto_perception_msgs::msg::DetectedObject updating_object =
    getUpdatingObject(object, self_transform);
  measureWithPose(updating_object);
  measureWithShape(updating_object);

  (void)self_transform;  // currently do not use self vehicle position
  return true;
}

bool PedestrianTracker::getTrackedObject(
  const rclcpp::Time & time, autoware_auto_perception_msgs::msg::TrackedObject & object) const
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
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX) {
    object.shape.dimensions.x = bounding_box_.length;
    object.shape.dimensions.y = bounding_box_.width;
    object.shape.dimensions.z = bounding_box_.height;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::CYLINDER) {
    object.shape.dimensions.x = cylinder_.width;
    object.shape.dimensions.y = cylinder_.width;
    object.shape.dimensions.z = cylinder_.height;
  } else if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    const auto origin_yaw = tf2::getYaw(object_.kinematics.pose_with_covariance.pose.orientation);
    const auto ekf_pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
    object.shape.footprint =
      tier4_autoware_utils::rotatePolygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);
  }

  return true;
}
