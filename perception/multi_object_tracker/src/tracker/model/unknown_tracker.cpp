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
#define EIGEN_MPL2_ONLY

#include "multi_object_tracker/tracker/model/unknown_tracker.hpp"

#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware/universe_utils/math/normalization.hpp"
#include "autoware/universe_utils/math/unit_conversion.hpp"
#include "autoware/universe_utils/ros/msg_covariance.hpp"
#include "multi_object_tracker/utils/utils.hpp"

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
#include "object_recognition_utils/object_recognition_utils.hpp"

UnknownTracker::UnknownTracker(
  const rclcpp::Time & time, const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/, const size_t channel_size,
  const uint & channel_index)
: Tracker(time, object.classification, channel_size),
  logger_(rclcpp::get_logger("UnknownTracker")),
  z_(object.kinematics.pose_with_covariance.pose.position.z)
{
  object_ = object;

  // initialize existence probability
  initializeExistenceProbabilities(channel_index, object.existence_probability);

  // initialize params
  // measurement noise covariance
  constexpr double r_stddev_x = 1.0;  // [m]
  constexpr double r_stddev_y = 1.0;  // [m]
  ekf_params_.r_cov_x = std::pow(r_stddev_x, 2.0);
  ekf_params_.r_cov_y = std::pow(r_stddev_y, 2.0);

  // Set motion model parameters
  {
    constexpr double q_stddev_x = 0.5;         // [m/s]
    constexpr double q_stddev_y = 0.5;         // [m/s]
    constexpr double q_stddev_vx = 9.8 * 0.3;  // [m/(s*s)]
    constexpr double q_stddev_vy = 9.8 * 0.3;  // [m/(s*s)]
    motion_model_.setMotionParams(q_stddev_x, q_stddev_y, q_stddev_vx, q_stddev_vy);
  }

  // Set motion limits
  motion_model_.setMotionLimits(
    autoware::universe_utils::kmph2mps(60), /* [m/s] maximum velocity, x */
    autoware::universe_utils::kmph2mps(60)  /* [m/s] maximum velocity, y */
  );

  // Set initial state
  {
    using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;
    auto pose_cov = object.kinematics.pose_with_covariance.covariance;
    auto twist_cov = object.kinematics.twist_with_covariance.covariance;
    const double yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);

    double vx = 0.0;
    double vy = 0.0;
    if (object.kinematics.has_twist) {
      const double & vel_x = object.kinematics.twist_with_covariance.twist.linear.x;
      const double & vel_y = object.kinematics.twist_with_covariance.twist.linear.y;
      vx = std::cos(yaw) * vel_x - std::sin(yaw) * vel_y;
      vy = std::sin(yaw) * vel_x + std::cos(yaw) * vel_y;
    }

    if (!object.kinematics.has_position_covariance) {
      constexpr double p0_stddev_x = 1.0;  // [m]
      constexpr double p0_stddev_y = 1.0;  // [m]

      const double p0_cov_x = std::pow(p0_stddev_x, 2.0);
      const double p0_cov_y = std::pow(p0_stddev_y, 2.0);

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double sin_2yaw = std::sin(2.0 * yaw);
      pose_cov[XYZRPY_COV_IDX::X_X] = p0_cov_x * cos_yaw * cos_yaw + p0_cov_y * sin_yaw * sin_yaw;
      pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5 * (p0_cov_x - p0_cov_y) * sin_2yaw;
      pose_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_x * sin_yaw * sin_yaw + p0_cov_y * cos_yaw * cos_yaw;
      pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];
    }

    if (!object.kinematics.has_twist_covariance) {
      constexpr double p0_stddev_vx = autoware::universe_utils::kmph2mps(10);  // [m/s]
      constexpr double p0_stddev_vy = autoware::universe_utils::kmph2mps(10);  // [m/s]
      const double p0_cov_vx = std::pow(p0_stddev_vx, 2.0);
      const double p0_cov_vy = std::pow(p0_stddev_vy, 2.0);
      twist_cov[XYZRPY_COV_IDX::X_X] = p0_cov_vx;
      twist_cov[XYZRPY_COV_IDX::X_Y] = 0.0;
      twist_cov[XYZRPY_COV_IDX::Y_X] = 0.0;
      twist_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_vy;
    }

    // rotate twist covariance matrix, since it is in the vehicle coordinate system
    Eigen::MatrixXd twist_cov_rotate(2, 2);
    twist_cov_rotate(0, 0) = twist_cov[XYZRPY_COV_IDX::X_X];
    twist_cov_rotate(0, 1) = twist_cov[XYZRPY_COV_IDX::X_Y];
    twist_cov_rotate(1, 0) = twist_cov[XYZRPY_COV_IDX::Y_X];
    twist_cov_rotate(1, 1) = twist_cov[XYZRPY_COV_IDX::Y_Y];
    Eigen::MatrixXd R_yaw = Eigen::Rotation2Dd(-yaw).toRotationMatrix();
    Eigen::MatrixXd twist_cov_rotated = R_yaw * twist_cov_rotate * R_yaw.transpose();
    twist_cov[XYZRPY_COV_IDX::X_X] = twist_cov_rotated(0, 0);
    twist_cov[XYZRPY_COV_IDX::X_Y] = twist_cov_rotated(0, 1);
    twist_cov[XYZRPY_COV_IDX::Y_X] = twist_cov_rotated(1, 0);
    twist_cov[XYZRPY_COV_IDX::Y_Y] = twist_cov_rotated(1, 1);

    // initialize motion model
    motion_model_.initialize(time, x, y, pose_cov, vx, vy, twist_cov);
  }
}

bool UnknownTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

autoware_perception_msgs::msg::DetectedObject UnknownTracker::getUpdatingObject(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/)
{
  autoware_perception_msgs::msg::DetectedObject updating_object = object;

  // UNCERTAINTY MODEL
  if (!object.kinematics.has_position_covariance) {
    // fill covariance matrix
    using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    const double & r_cov_x = ekf_params_.r_cov_x;
    const double & r_cov_y = ekf_params_.r_cov_y;
    auto & pose_cov = updating_object.kinematics.pose_with_covariance.covariance;
    const double pose_yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    const double cos_yaw = std::cos(pose_yaw);
    const double sin_yaw = std::sin(pose_yaw);
    const double sin_2yaw = std::sin(2.0f * pose_yaw);
    pose_cov[XYZRPY_COV_IDX::X_X] =
      r_cov_x * cos_yaw * cos_yaw + r_cov_y * sin_yaw * sin_yaw;            // x - x
    pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5f * (r_cov_x - r_cov_y) * sin_2yaw;  // x - y
    pose_cov[XYZRPY_COV_IDX::Y_Y] =
      r_cov_x * sin_yaw * sin_yaw + r_cov_y * cos_yaw * cos_yaw;    // y - y
    pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];  // y - x
  }
  return updating_object;
}

bool UnknownTracker::measureWithPose(const autoware_perception_msgs::msg::DetectedObject & object)
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

bool UnknownTracker::measure(
  const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  // keep the latest input object
  object_ = object;

  // check time gap
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "UnknownTracker::measure There is a large gap between predicted time and measurement time. "
      "(%f)",
      dt);
  }

  // update object
  const autoware_perception_msgs::msg::DetectedObject updating_object =
    getUpdatingObject(object, self_transform);
  measureWithPose(updating_object);

  return true;
}

bool UnknownTracker::getTrackedObject(
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
    RCLCPP_WARN(logger_, "UnknownTracker::getTrackedObject: Failed to get predicted state.");
    return false;
  }

  // position
  pose_with_cov.pose.position.z = z_;

  return true;
}
