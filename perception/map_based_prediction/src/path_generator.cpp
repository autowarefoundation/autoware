// Copyright 2021 Tier IV, Inc.
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

#include "map_based_prediction/path_generator.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <algorithm>

namespace map_based_prediction
{
PathGenerator::PathGenerator(
  const double time_horizon, const double lateral_time_horizon, const double sampling_time_interval,
  const double min_crosswalk_user_velocity)
: time_horizon_(time_horizon),
  lateral_time_horizon_(lateral_time_horizon),
  sampling_time_interval_(sampling_time_interval),
  min_crosswalk_user_velocity_(min_crosswalk_user_velocity)
{
}

PredictedPath PathGenerator::generatePathForNonVehicleObject(const TrackedObject & object)
{
  return generateStraightPath(object);
}

PredictedPath PathGenerator::generatePathToTargetPoint(
  const TrackedObject & object, const Eigen::Vector2d & point) const
{
  PredictedPath predicted_path{};
  const double ep = 0.001;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;

  const Eigen::Vector2d pedestrian_to_entry_point(point.x() - obj_pos.x, point.y() - obj_pos.y);
  const auto velocity = std::max(std::hypot(obj_vel.x, obj_vel.y), min_crosswalk_user_velocity_);
  const auto arrival_time = pedestrian_to_entry_point.norm() / velocity;

  for (double dt = 0.0; dt < arrival_time + ep; dt += sampling_time_interval_) {
    geometry_msgs::msg::Pose world_frame_pose;
    world_frame_pose.position.x =
      obj_pos.x + velocity * pedestrian_to_entry_point.normalized().x() * dt;
    world_frame_pose.position.y =
      obj_pos.y + velocity * pedestrian_to_entry_point.normalized().y() * dt;
    world_frame_pose.position.z = obj_pos.z;
    world_frame_pose.orientation = object.kinematics.pose_with_covariance.pose.orientation;
    predicted_path.path.push_back(world_frame_pose);
    if (predicted_path.path.size() >= predicted_path.path.max_size()) {
      break;
    }
  }

  predicted_path.confidence = 1.0;
  predicted_path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);

  return predicted_path;
}

PredictedPath PathGenerator::generatePathForCrosswalkUser(
  const TrackedObject & object, const CrosswalkEdgePoints & reachable_crosswalk) const
{
  PredictedPath predicted_path{};
  const double ep = 0.001;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;

  const Eigen::Vector2d pedestrian_to_entry_point(
    reachable_crosswalk.front_center_point.x() - obj_pos.x,
    reachable_crosswalk.front_center_point.y() - obj_pos.y);
  const Eigen::Vector2d entry_to_exit_point(
    reachable_crosswalk.back_center_point.x() - reachable_crosswalk.front_center_point.x(),
    reachable_crosswalk.back_center_point.y() - reachable_crosswalk.front_center_point.y());
  const auto velocity = std::max(std::hypot(obj_vel.x, obj_vel.y), min_crosswalk_user_velocity_);
  const auto arrival_time = pedestrian_to_entry_point.norm() / velocity;

  for (double dt = 0.0; dt < time_horizon_ + ep; dt += sampling_time_interval_) {
    geometry_msgs::msg::Pose world_frame_pose;
    if (dt < arrival_time) {
      world_frame_pose.position.x =
        obj_pos.x + velocity * pedestrian_to_entry_point.normalized().x() * dt;
      world_frame_pose.position.y =
        obj_pos.y + velocity * pedestrian_to_entry_point.normalized().y() * dt;
      world_frame_pose.position.z = obj_pos.z;
      world_frame_pose.orientation = object.kinematics.pose_with_covariance.pose.orientation;
      predicted_path.path.push_back(world_frame_pose);
    } else {
      world_frame_pose.position.x =
        reachable_crosswalk.front_center_point.x() +
        velocity * entry_to_exit_point.normalized().x() * (dt - arrival_time);
      world_frame_pose.position.y =
        reachable_crosswalk.front_center_point.y() +
        velocity * entry_to_exit_point.normalized().y() * (dt - arrival_time);
      world_frame_pose.position.z = obj_pos.z;
      world_frame_pose.orientation = object.kinematics.pose_with_covariance.pose.orientation;
      predicted_path.path.push_back(world_frame_pose);
    }
    if (predicted_path.path.size() >= predicted_path.path.max_size()) {
      break;
    }
  }

  // calculate orientation of each point
  if (predicted_path.path.size() >= 2) {
    for (size_t i = 0; i < predicted_path.path.size() - 1; i++) {
      const auto yaw = tier4_autoware_utils::calcAzimuthAngle(
        predicted_path.path.at(i).position, predicted_path.path.at(i + 1).position);
      predicted_path.path.at(i).orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);
    }
    predicted_path.path.back().orientation =
      predicted_path.path.at(predicted_path.path.size() - 2).orientation;
  }

  predicted_path.confidence = 1.0;
  predicted_path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);

  return predicted_path;
}

PredictedPath PathGenerator::generatePathForLowSpeedVehicle(const TrackedObject & object) const
{
  PredictedPath path;
  path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);
  const double ep = 0.001;
  for (double dt = 0.0; dt < time_horizon_ + ep; dt += sampling_time_interval_) {
    path.path.push_back(object.kinematics.pose_with_covariance.pose);
  }

  return path;
}

PredictedPath PathGenerator::generatePathForOffLaneVehicle(const TrackedObject & object)
{
  return generateStraightPath(object);
}

PredictedPath PathGenerator::generatePathForOnLaneVehicle(
  const TrackedObject & object, const PosePath & ref_paths)
{
  if (ref_paths.size() < 2) {
    return generateStraightPath(object);
  }

  return generatePolynomialPath(object, ref_paths);
}

PredictedPath PathGenerator::generateStraightPath(const TrackedObject & object) const
{
  const auto & object_pose = object.kinematics.pose_with_covariance.pose;
  const auto & object_twist = object.kinematics.twist_with_covariance.twist;
  const double ep = 0.001;
  const double duration = time_horizon_ + ep;

  PredictedPath path;
  path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);
  path.path.reserve(static_cast<size_t>((duration) / sampling_time_interval_));
  for (double dt = 0.0; dt < duration; dt += sampling_time_interval_) {
    const auto future_obj_pose = tier4_autoware_utils::calcOffsetPose(
      object_pose, object_twist.linear.x * dt, object_twist.linear.y * dt, 0.0);
    path.path.push_back(future_obj_pose);
  }

  return path;
}

PredictedPath PathGenerator::generatePolynomialPath(
  const TrackedObject & object, const PosePath & ref_path)
{
  // Get current Frenet Point
  const double ref_path_len = motion_utils::calcArcLength(ref_path);
  const auto current_point = getFrenetPoint(object, ref_path);

  // Step1. Set Target Frenet Point
  // Note that we do not set position s,
  // since we don't know the target longitudinal position
  FrenetPoint terminal_point;
  terminal_point.s_vel = current_point.s_vel;
  terminal_point.s_acc = 0.0;
  terminal_point.d = 0.0;
  terminal_point.d_vel = 0.0;
  terminal_point.d_acc = 0.0;

  // Step2. Generate Predicted Path on a Frenet coordinate
  const auto frenet_predicted_path =
    generateFrenetPath(current_point, terminal_point, ref_path_len);

  // Step3. Interpolate Reference Path for converting predicted path coordinate
  const auto interpolated_ref_path = interpolateReferencePath(ref_path, frenet_predicted_path);

  if (frenet_predicted_path.size() < 2 || interpolated_ref_path.size() < 2) {
    return generateStraightPath(object);
  }

  // Step4. Convert predicted trajectory from Frenet to Cartesian coordinate
  return convertToPredictedPath(object, frenet_predicted_path, interpolated_ref_path);
}

FrenetPath PathGenerator::generateFrenetPath(
  const FrenetPoint & current_point, const FrenetPoint & target_point, const double max_length)
{
  FrenetPath path;
  const double duration = time_horizon_;
  const double lateral_duration = lateral_time_horizon_;

  // Compute Lateral and Longitudinal Coefficients to generate the trajectory
  const Eigen::Vector3d lat_coeff =
    calcLatCoefficients(current_point, target_point, lateral_duration);
  const Eigen::Vector2d lon_coeff = calcLonCoefficients(current_point, target_point, duration);

  path.reserve(static_cast<size_t>(duration / sampling_time_interval_));
  for (double t = 0.0; t <= duration; t += sampling_time_interval_) {
    const double current_acc =
      0.0;  // Currently we assume the object is traveling at a constant speed
    const double d_next_ = current_point.d + current_point.d_vel * t +
                           current_acc * 2.0 * std::pow(t, 2) + lat_coeff(0) * std::pow(t, 3) +
                           lat_coeff(1) * std::pow(t, 4) + lat_coeff(2) * std::pow(t, 5);
    // t > lateral_duration: 0.0, else d_next_
    const double d_next = t > lateral_duration ? 0.0 : d_next_;
    const double s_next = current_point.s + current_point.s_vel * t +
                          2.0 * current_acc * std::pow(t, 2) + lon_coeff(0) * std::pow(t, 3) +
                          lon_coeff(1) * std::pow(t, 4);
    if (s_next > max_length) {
      break;
    }

    // We assume the object is traveling at a constant speed along s direction
    FrenetPoint point;
    point.s = std::max(s_next, 0.0);
    point.s_vel = current_point.s_vel;
    point.s_acc = current_point.s_acc;
    point.d = d_next;
    point.d_vel = current_point.d_vel;
    point.d_acc = current_point.d_acc;
    path.push_back(point);
  }

  return path;
}

Eigen::Vector3d PathGenerator::calcLatCoefficients(
  const FrenetPoint & current_point, const FrenetPoint & target_point, const double T)
{
  // Lateral Path Calculation
  // Quintic polynomial for d
  // A = np.array([[T**3, T**4, T**5],
  //               [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
  //               [6 * T, 12 * T ** 2, 20 * T ** 3]])
  // A_inv = np.matrix([[10/(T**3), -4/(T**2), 1/(2*T)],
  //                    [-15/(T**4), 7/(T**3), -1/(T**2)],
  //                    [6/(T**5), -3/(T**4),  1/(2*T**3)]])
  // b = np.matrix([[xe - self.a0 - self.a1 * T - self.a2 * T**2],
  //                [vxe - self.a1 - 2 * self.a2 * T],
  //                [axe - 2 * self.a2]])
  Eigen::Matrix3d A_lat_inv;
  A_lat_inv << 10 / std::pow(T, 3), -4 / std::pow(T, 2), 1 / (2 * T), -15 / std::pow(T, 4),
    7 / std::pow(T, 3), -1 / std::pow(T, 2), 6 / std::pow(T, 5), -3 / std::pow(T, 4),
    1 / (2 * std::pow(T, 3));
  Eigen::Vector3d b_lat;
  b_lat[0] = target_point.d - current_point.d - current_point.d_vel * T;
  b_lat[1] = target_point.d_vel - current_point.d_vel;
  b_lat[2] = target_point.d_acc;

  return A_lat_inv * b_lat;
}

Eigen::Vector2d PathGenerator::calcLonCoefficients(
  const FrenetPoint & current_point, const FrenetPoint & target_point, const double T)
{
  // Longitudinal Path Calculation
  // Quadric polynomial
  // A_inv = np.matrix([[1/(T**2), -1/(3*T)],
  //                         [-1/(2*T**3), 1/(4*T**2)]])
  // b = np.matrix([[vxe - self.a1 - 2 * self.a2 * T],
  //               [axe - 2 * self.a2]])
  Eigen::Matrix2d A_lon_inv;
  A_lon_inv << 1 / std::pow(T, 2), -1 / (3 * T), -1 / (2 * std::pow(T, 3)),
    1 / (4 * std::pow(T, 2));
  Eigen::Vector2d b_lon;
  b_lon[0] = target_point.s_vel - current_point.s_vel;
  b_lon[1] = 0.0;
  return A_lon_inv * b_lon;
}

PosePath PathGenerator::interpolateReferencePath(
  const PosePath & base_path, const FrenetPath & frenet_predicted_path)
{
  PosePath interpolated_path;
  const size_t interpolate_num = frenet_predicted_path.size();
  if (interpolate_num < 2) {
    interpolated_path.emplace_back(base_path.front());
    return interpolated_path;
  }

  std::vector<double> base_path_x(base_path.size());
  std::vector<double> base_path_y(base_path.size());
  std::vector<double> base_path_z(base_path.size());
  std::vector<double> base_path_s(base_path.size(), 0.0);
  for (size_t i = 0; i < base_path.size(); ++i) {
    base_path_x.at(i) = base_path.at(i).position.x;
    base_path_y.at(i) = base_path.at(i).position.y;
    base_path_z.at(i) = base_path.at(i).position.z;
    if (i > 0) {
      base_path_s.at(i) = base_path_s.at(i - 1) + tier4_autoware_utils::calcDistance2d(
                                                    base_path.at(i - 1), base_path.at(i));
    }
  }

  std::vector<double> resampled_s(frenet_predicted_path.size());
  for (size_t i = 0; i < frenet_predicted_path.size(); ++i) {
    resampled_s.at(i) = frenet_predicted_path.at(i).s;
  }
  if (resampled_s.front() > resampled_s.back()) {
    std::reverse(resampled_s.begin(), resampled_s.end());
  }

  // Spline Interpolation
  std::vector<double> spline_ref_path_x =
    interpolation::spline(base_path_s, base_path_x, resampled_s);
  std::vector<double> spline_ref_path_y =
    interpolation::spline(base_path_s, base_path_y, resampled_s);
  std::vector<double> spline_ref_path_z =
    interpolation::spline(base_path_s, base_path_z, resampled_s);

  interpolated_path.resize(interpolate_num);
  for (size_t i = 0; i < interpolate_num - 1; ++i) {
    geometry_msgs::msg::Pose interpolated_pose;
    const auto current_point =
      tier4_autoware_utils::createPoint(spline_ref_path_x.at(i), spline_ref_path_y.at(i), 0.0);
    const auto next_point = tier4_autoware_utils::createPoint(
      spline_ref_path_x.at(i + 1), spline_ref_path_y.at(i + 1), 0.0);
    const double yaw = tier4_autoware_utils::calcAzimuthAngle(current_point, next_point);
    interpolated_pose.position = tier4_autoware_utils::createPoint(
      spline_ref_path_x.at(i), spline_ref_path_y.at(i), spline_ref_path_z.at(i));
    interpolated_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);
    interpolated_path.at(i) = interpolated_pose;
  }
  interpolated_path.back().position = tier4_autoware_utils::createPoint(
    spline_ref_path_x.back(), spline_ref_path_y.back(), spline_ref_path_z.back());
  interpolated_path.back().orientation = interpolated_path.at(interpolate_num - 2).orientation;

  return interpolated_path;
}

PredictedPath PathGenerator::convertToPredictedPath(
  const TrackedObject & object, const FrenetPath & frenet_predicted_path, const PosePath & ref_path)
{
  PredictedPath predicted_path;
  predicted_path.time_step = rclcpp::Duration::from_seconds(sampling_time_interval_);
  predicted_path.path.resize(ref_path.size());
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    // Reference Point from interpolated reference path
    const auto & ref_pose = ref_path.at(i);

    // Frenet Point from frenet predicted path
    const auto & frenet_point = frenet_predicted_path.at(i);

    // Converted Pose
    auto predicted_pose = tier4_autoware_utils::calcOffsetPose(ref_pose, 0.0, frenet_point.d, 0.0);
    predicted_pose.position.z = object.kinematics.pose_with_covariance.pose.position.z;
    if (i == 0) {
      predicted_pose.orientation = object.kinematics.pose_with_covariance.pose.orientation;
    } else {
      const double yaw = tier4_autoware_utils::calcAzimuthAngle(
        predicted_path.path.at(i - 1).position, predicted_pose.position);
      predicted_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);
    }
    predicted_path.path.at(i) = predicted_pose;
  }

  return predicted_path;
}

FrenetPoint PathGenerator::getFrenetPoint(const TrackedObject & object, const PosePath & ref_path)
{
  FrenetPoint frenet_point;
  const auto obj_point = object.kinematics.pose_with_covariance.pose.position;

  const size_t nearest_segment_idx = motion_utils::findNearestSegmentIndex(ref_path, obj_point);
  const double l =
    motion_utils::calcLongitudinalOffsetToSegment(ref_path, nearest_segment_idx, obj_point);
  const float vx = static_cast<float>(object.kinematics.twist_with_covariance.twist.linear.x);
  const float vy = static_cast<float>(object.kinematics.twist_with_covariance.twist.linear.y);
  const float obj_yaw =
    static_cast<float>(tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation));
  const float lane_yaw =
    static_cast<float>(tf2::getYaw(ref_path.at(nearest_segment_idx).orientation));
  const float delta_yaw = obj_yaw - lane_yaw;

  frenet_point.s = motion_utils::calcSignedArcLength(ref_path, 0, nearest_segment_idx) + l;
  frenet_point.d = motion_utils::calcLateralOffset(ref_path, obj_point);
  frenet_point.s_vel = vx * std::cos(delta_yaw) - vy * std::sin(delta_yaw);
  frenet_point.d_vel = vx * std::sin(delta_yaw) + vy * std::cos(delta_yaw);
  frenet_point.s_acc = 0.0;
  frenet_point.d_acc = 0.0;

  return frenet_point;
}
}  // namespace map_based_prediction
