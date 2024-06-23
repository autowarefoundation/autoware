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

#include "multi_object_tracker/tracker/model/normal_vehicle_tracker.hpp"

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

NormalVehicleTracker::NormalVehicleTracker(
  const rclcpp::Time & time, const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & /*self_transform*/, const size_t channel_size,
  const uint & channel_index)
: Tracker(time, object.classification, channel_size),
  logger_(rclcpp::get_logger("NormalVehicleTracker")),
  z_(object.kinematics.pose_with_covariance.pose.position.z),
  tracking_offset_(Eigen::Vector2d::Zero())
{
  object_ = object;

  // initialize existence probability
  initializeExistenceProbabilities(channel_index, object.existence_probability);

  // velocity deviation threshold
  //   if the predicted velocity is close to the observed velocity,
  //   the observed velocity is used as the measurement.
  velocity_deviation_threshold_ = autoware::universe_utils::kmph2mps(10);  // [m/s]

  // OBJECT SHAPE MODEL
  if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    bounding_box_ = {
      object.shape.dimensions.x, object.shape.dimensions.y, object.shape.dimensions.z};
  } else {
    autoware_perception_msgs::msg::DetectedObject bbox_object;
    if (!utils::convertConvexHullToBoundingBox(object, bbox_object)) {
      RCLCPP_WARN(
        logger_,
        "NormalVehicleTracker::NormalVehicleTracker: Failed to convert convex hull to bounding "
        "box.");
      bounding_box_ = {
        object_model_.init_size.length, object_model_.init_size.width,
        object_model_.init_size.height};  // default value
    } else {
      bounding_box_ = {
        bbox_object.shape.dimensions.x, bbox_object.shape.dimensions.y,
        bbox_object.shape.dimensions.z};
    }
  }
  // set maximum and minimum size
  bounding_box_.length = std::clamp(
    bounding_box_.length, object_model_.size_limit.length_min, object_model_.size_limit.length_max);
  bounding_box_.width = std::clamp(
    bounding_box_.width, object_model_.size_limit.width_min, object_model_.size_limit.width_max);
  bounding_box_.height = std::clamp(
    bounding_box_.height, object_model_.size_limit.height_min, object_model_.size_limit.height_max);

  // Set motion model parameters
  {
    const double q_stddev_acc_long = object_model_.process_noise.acc_long;
    const double q_stddev_acc_lat = object_model_.process_noise.acc_lat;
    const double q_stddev_yaw_rate_min = object_model_.process_noise.yaw_rate_min;
    const double q_stddev_yaw_rate_max = object_model_.process_noise.yaw_rate_max;
    const double q_stddev_slip_rate_min = object_model_.bicycle_state.slip_rate_stddev_min;
    const double q_stddev_slip_rate_max = object_model_.bicycle_state.slip_rate_stddev_max;
    const double q_max_slip_angle = object_model_.bicycle_state.slip_angle_max;
    const double lf_ratio = object_model_.bicycle_state.wheel_pos_ratio_front;
    const double lf_min = object_model_.bicycle_state.wheel_pos_front_min;
    const double lr_ratio = object_model_.bicycle_state.wheel_pos_ratio_rear;
    const double lr_min = object_model_.bicycle_state.wheel_pos_rear_min;
    motion_model_.setMotionParams(
      q_stddev_acc_long, q_stddev_acc_lat, q_stddev_yaw_rate_min, q_stddev_yaw_rate_max,
      q_stddev_slip_rate_min, q_stddev_slip_rate_max, q_max_slip_angle, lf_ratio, lf_min, lr_ratio,
      lr_min);
  }

  // Set motion limits
  {
    const double max_vel = object_model_.process_limit.vel_long_max;
    const double max_slip = object_model_.bicycle_state.slip_angle_max;
    motion_model_.setMotionLimits(max_vel, max_slip);  // maximum velocity and slip angle
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
    double vel_cov = object_model_.initial_covariance.vel_long;
    if (object.kinematics.has_twist) {
      vel = object.kinematics.twist_with_covariance.twist.linear.x;
    }
    if (object.kinematics.has_twist_covariance) {
      vel_cov = object.kinematics.twist_with_covariance.covariance[XYZRPY_COV_IDX::X_X];
    }

    const double slip = 0.0;
    const double slip_cov = object_model_.bicycle_state.init_slip_angle_cov;
    const double & length = bounding_box_.length;

    // initialize motion model
    motion_model_.initialize(time, x, y, yaw, pose_cov, vel, vel_cov, slip, slip_cov, length);
  }
}

bool NormalVehicleTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

autoware_perception_msgs::msg::DetectedObject NormalVehicleTracker::getUpdatingObject(
  const autoware_perception_msgs::msg::DetectedObject & object,
  const geometry_msgs::msg::Transform & self_transform)
{
  autoware_perception_msgs::msg::DetectedObject updating_object = object;

  // OBJECT SHAPE MODEL
  // convert to bounding box if input is convex shape
  autoware_perception_msgs::msg::DetectedObject bbox_object = object;
  if (object.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    if (!utils::convertConvexHullToBoundingBox(object, bbox_object)) {
      RCLCPP_WARN(
        logger_,
        "NormalVehicleTracker::getUpdatingObject: Failed to convert convex hull to bounding box.");
      bbox_object = object;
    }
  }

  // current (predicted) state
  const double tracked_x = motion_model_.getStateElement(IDX::X);
  const double tracked_y = motion_model_.getStateElement(IDX::Y);
  const double tracked_yaw = motion_model_.getStateElement(IDX::YAW);

  // get offset measurement
  const int nearest_corner_index = utils::getNearestCornerOrSurface(
    tracked_x, tracked_y, tracked_yaw, bounding_box_.width, bounding_box_.length, self_transform);
  utils::calcAnchorPointOffset(
    bounding_box_.width, bounding_box_.length, nearest_corner_index, bbox_object, tracked_yaw,
    updating_object, tracking_offset_);

  // UNCERTAINTY MODEL
  if (!object.kinematics.has_position_covariance) {
    // measurement noise covariance
    auto r_cov_x = object_model_.measurement_covariance.pos_x;
    auto r_cov_y = object_model_.measurement_covariance.pos_y;
    const uint8_t label = object_recognition_utils::getHighestProbLabel(object.classification);
    if (utils::isLargeVehicleLabel(label)) {
      // if label is changed, enlarge the measurement noise covariance
      constexpr float r_stddev_x = 2.0;  // [m]
      constexpr float r_stddev_y = 2.0;  // [m]
      r_cov_x = r_stddev_x * r_stddev_x;
      r_cov_y = r_stddev_y * r_stddev_y;
    }

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

bool NormalVehicleTracker::measureWithPose(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  // current (predicted) state
  const double tracked_vel = motion_model_.getStateElement(IDX::VEL);

  // velocity capability is checked only when the object has velocity measurement
  // and the predicted velocity is close to the observed velocity
  bool is_velocity_available = false;
  if (object.kinematics.has_twist) {
    const double & observed_vel = object.kinematics.twist_with_covariance.twist.linear.x;
    if (std::fabs(tracked_vel - observed_vel) < velocity_deviation_threshold_) {
      // Velocity deviation is small
      is_velocity_available = true;
    }
  }

  // update
  bool is_updated = false;
  {
    const double x = object.kinematics.pose_with_covariance.pose.position.x;
    const double y = object.kinematics.pose_with_covariance.pose.position.y;
    const double yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
    const double vel = object.kinematics.twist_with_covariance.twist.linear.x;

    if (is_velocity_available) {
      is_updated = motion_model_.updateStatePoseHeadVel(
        x, y, yaw, object.kinematics.pose_with_covariance.covariance, vel,
        object.kinematics.twist_with_covariance.covariance);
    } else {
      is_updated = motion_model_.updateStatePoseHead(
        x, y, yaw, object.kinematics.pose_with_covariance.covariance);
    }
    motion_model_.limitStates();
  }

  // position z
  constexpr double gain = 0.1;
  z_ = (1.0 - gain) * z_ + gain * object.kinematics.pose_with_covariance.pose.position.z;

  return is_updated;
}

bool NormalVehicleTracker::measureWithShape(
  const autoware_perception_msgs::msg::DetectedObject & object)
{
  if (object.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    // do not update shape if the input is not a bounding box
    return false;
  }

  // check object size abnormality
  constexpr double size_max = 35.0;  // [m]
  constexpr double size_min = 1.0;   // [m]
  bool is_size_valid =
    (object.shape.dimensions.x <= size_max && object.shape.dimensions.y <= size_max &&
     object.shape.dimensions.x >= size_min && object.shape.dimensions.y >= size_min);
  if (!is_size_valid) {
    return false;
  }

  // update object size
  constexpr double gain = 0.5;
  constexpr double gain_inv = 1.0 - gain;
  bounding_box_.length = gain_inv * bounding_box_.length + gain * object.shape.dimensions.x;
  bounding_box_.width = gain_inv * bounding_box_.width + gain * object.shape.dimensions.y;
  bounding_box_.height = gain_inv * bounding_box_.height + gain * object.shape.dimensions.z;

  // set maximum and minimum size
  bounding_box_.length = std::clamp(
    bounding_box_.length, object_model_.size_limit.length_min, object_model_.size_limit.length_max);
  bounding_box_.width = std::clamp(
    bounding_box_.width, object_model_.size_limit.width_min, object_model_.size_limit.width_max);
  bounding_box_.height = std::clamp(
    bounding_box_.height, object_model_.size_limit.height_min, object_model_.size_limit.height_max);

  // update motion model
  motion_model_.updateExtendedState(bounding_box_.length);

  // update offset into object position
  {
    // rotate back the offset vector from object coordinate to global coordinate
    const double yaw = motion_model_.getStateElement(IDX::YAW);
    const double offset_x_global =
      tracking_offset_.x() * std::cos(yaw) - tracking_offset_.y() * std::sin(yaw);
    const double offset_y_global =
      tracking_offset_.x() * std::sin(yaw) + tracking_offset_.y() * std::cos(yaw);
    motion_model_.adjustPosition(-gain * offset_x_global, -gain * offset_y_global);
    // update offset (object coordinate)
    tracking_offset_.x() = gain_inv * tracking_offset_.x();
    tracking_offset_.y() = gain_inv * tracking_offset_.y();
  }

  return true;
}

bool NormalVehicleTracker::measure(
  const autoware_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
  const geometry_msgs::msg::Transform & self_transform)
{
  // keep the latest input object
  object_ = object;

  // update classification
  const auto & current_classification = getClassification();
  if (object_recognition_utils::getHighestProbLabel(object.classification) == Label::UNKNOWN) {
    setClassification(current_classification);
  }

  // check time gap
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "NormalVehicleTracker::measure There is a large gap between predicted time and measurement "
      "time. (%f)",
      dt);
  }

  // update object
  const autoware_perception_msgs::msg::DetectedObject updating_object =
    getUpdatingObject(object, self_transform);
  measureWithPose(updating_object);
  measureWithShape(updating_object);

  return true;
}

bool NormalVehicleTracker::getTrackedObject(
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
    RCLCPP_WARN(logger_, "NormalVehicleTracker::getTrackedObject: Failed to get predicted state.");
    return false;
  }

  // position
  pose_with_cov.pose.position.z = z_;

  // set shape
  object.shape.dimensions.x = bounding_box_.length;
  object.shape.dimensions.y = bounding_box_.width;
  object.shape.dimensions.z = bounding_box_.height;
  const auto origin_yaw = tf2::getYaw(object_.kinematics.pose_with_covariance.pose.orientation);
  const auto ekf_pose_yaw = tf2::getYaw(pose_with_cov.pose.orientation);
  object.shape.footprint =
    autoware::universe_utils::rotatePolygon(object.shape.footprint, origin_yaw - ekf_pose_yaw);

  return true;
}
