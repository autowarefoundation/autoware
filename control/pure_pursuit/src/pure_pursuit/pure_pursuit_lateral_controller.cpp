// Copyright 2020-2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pure_pursuit/pure_pursuit_lateral_controller.hpp"

#include "pure_pursuit/pure_pursuit_viz.hpp"
#include "pure_pursuit/util/planning_utils.hpp"
#include "pure_pursuit/util/tf_utils.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

#include <algorithm>
#include <memory>
#include <utility>

namespace
{
enum TYPE {
  VEL_LD = 0,
  CURVATURE_LD = 1,
  LATERAL_ERROR_LD = 2,
  TOTAL_LD = 3,
  CURVATURE = 4,
  LATERAL_ERROR = 5,
  VELOCITY = 6,
  SIZE  // this is the number of enum elements
};
}  // namespace

namespace pure_pursuit
{
PurePursuitLateralController::PurePursuitLateralController(rclcpp::Node & node)
: clock_(node.get_clock()),
  logger_(node.get_logger().get_child("lateral_controller")),
  tf_buffer_(clock_),
  tf_listener_(tf_buffer_)
{
  pure_pursuit_ = std::make_unique<PurePursuit>();

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  param_.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio");
  param_.ld_lateral_error_ratio = node.declare_parameter<double>("ld_lateral_error_ratio");
  param_.ld_curvature_ratio = node.declare_parameter<double>("ld_curvature_ratio");
  param_.long_ld_lateral_error_threshold =
    node.declare_parameter<double>("long_ld_lateral_error_threshold");
  param_.min_lookahead_distance = node.declare_parameter<double>("min_lookahead_distance");
  param_.max_lookahead_distance = node.declare_parameter<double>("max_lookahead_distance");
  param_.reverse_min_lookahead_distance =
    node.declare_parameter<double>("reverse_min_lookahead_distance");
  param_.converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad");
  param_.prediction_ds = node.declare_parameter<double>("prediction_ds");
  param_.prediction_distance_length = node.declare_parameter<double>("prediction_distance_length");
  param_.resampling_ds = node.declare_parameter<double>("resampling_ds");
  param_.curvature_calculation_distance =
    node.declare_parameter<double>("curvature_calculation_distance");
  param_.enable_path_smoothing = node.declare_parameter<bool>("enable_path_smoothing");
  param_.path_filter_moving_ave_num = node.declare_parameter<int64_t>("path_filter_moving_ave_num");

  // Debug Publishers
  pub_debug_marker_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 0);
  pub_debug_values_ = node.create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/debug/ld_outputs", rclcpp::QoS{1});

  // Publish predicted trajectory
  pub_predicted_trajectory_ = node.create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/output/predicted_trajectory", 1);
}

double PurePursuitLateralController::calcLookaheadDistance(
  const double lateral_error, const double curvature, const double velocity, const double min_ld,
  const bool is_control_cmd)
{
  const double vel_ld = abs(param_.ld_velocity_ratio * velocity);
  const double curvature_ld = -abs(param_.ld_curvature_ratio * curvature);
  double lateral_error_ld = 0.0;

  if (abs(lateral_error) >= param_.long_ld_lateral_error_threshold) {
    // If lateral error is higher than threshold, we should make ld larger to prevent entering the
    // road with high heading error.
    lateral_error_ld = abs(param_.ld_lateral_error_ratio * lateral_error);
  }

  const double total_ld =
    std::clamp(vel_ld + curvature_ld + lateral_error_ld, min_ld, param_.max_lookahead_distance);

  auto pubDebugValues = [&]() {
    tier4_debug_msgs::msg::Float32MultiArrayStamped debug_msg{};
    debug_msg.data.resize(TYPE::SIZE);
    debug_msg.data.at(TYPE::VEL_LD) = static_cast<float>(vel_ld);
    debug_msg.data.at(TYPE::CURVATURE_LD) = static_cast<float>(curvature_ld);
    debug_msg.data.at(TYPE::LATERAL_ERROR_LD) = static_cast<float>(lateral_error_ld);
    debug_msg.data.at(TYPE::TOTAL_LD) = static_cast<float>(total_ld);
    debug_msg.data.at(TYPE::VELOCITY) = static_cast<float>(velocity);
    debug_msg.data.at(TYPE::CURVATURE) = static_cast<float>(curvature);
    debug_msg.data.at(TYPE::LATERAL_ERROR) = static_cast<float>(lateral_error);
    debug_msg.stamp = clock_->now();
    pub_debug_values_->publish(debug_msg);
  };

  if (is_control_cmd) {
    pubDebugValues();
  }

  return total_ld;
}

TrajectoryPoint PurePursuitLateralController::calcNextPose(
  const double ds, TrajectoryPoint & point, AckermannLateralCommand cmd) const
{
  geometry_msgs::msg::Transform transform;
  transform.translation = tier4_autoware_utils::createTranslation(ds, 0.0, 0.0);
  transform.rotation =
    planning_utils::getQuaternionFromYaw(((tan(cmd.steering_tire_angle) * ds) / param_.wheel_base));
  TrajectoryPoint output_p;

  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::fromMsg(transform, tf_offset);
  tf2::fromMsg(point.pose, tf_pose);
  tf2::toMsg(tf_pose * tf_offset, output_p.pose);
  return output_p;
}

void PurePursuitLateralController::setResampledTrajectory()
{
  // Interpolate with constant interval distance.
  std::vector<double> out_arclength;
  const auto input_tp_array = motion_utils::convertToTrajectoryPointArray(trajectory_);
  const auto traj_length = motion_utils::calcArcLength(input_tp_array);
  for (double s = 0; s < traj_length; s += param_.resampling_ds) {
    out_arclength.push_back(s);
  }
  trajectory_resampled_ =
    std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>(motion_utils::resampleTrajectory(
      motion_utils::convertToTrajectory(input_tp_array), out_arclength));
  trajectory_resampled_->points.back() = trajectory_.points.back();
  trajectory_resampled_->header = trajectory_.header;
  output_tp_array_ = motion_utils::convertToTrajectoryPointArray(*trajectory_resampled_);
}

double PurePursuitLateralController::calcCurvature(const size_t closest_idx)
{
  // Calculate current curvature
  const size_t idx_dist = static_cast<size_t>(
    std::max(static_cast<int>((param_.curvature_calculation_distance) / param_.resampling_ds), 1));

  // Find the points in trajectory to calculate curvature
  size_t next_idx = trajectory_resampled_->points.size() - 1;
  size_t prev_idx = 0;

  if (static_cast<size_t>(closest_idx) >= idx_dist) {
    prev_idx = closest_idx - idx_dist;
  } else {
    // return zero curvature when backward distance is not long enough in the trajectory
    return 0.0;
  }

  if (trajectory_resampled_->points.size() - 1 >= closest_idx + idx_dist) {
    next_idx = closest_idx + idx_dist;
  } else {
    // return zero curvature when forward distance is not long enough in the trajectory
    return 0.0;
  }
  // TODO(k.sugahara): shift the center point of the curvature calculation to allow sufficient
  // distance, because if sufficient distance cannot be obtained in front or behind, the curvature
  // will be zero in the current implementation.

  // Calculate curvature assuming the trajectory points interval is constant
  double current_curvature = 0.0;

  try {
    current_curvature = tier4_autoware_utils::calcCurvature(
      tier4_autoware_utils::getPoint(trajectory_resampled_->points.at(prev_idx)),
      tier4_autoware_utils::getPoint(trajectory_resampled_->points.at(closest_idx)),
      tier4_autoware_utils::getPoint(trajectory_resampled_->points.at(next_idx)));
  } catch (std::exception const & e) {
    // ...code that handles the error...
    RCLCPP_WARN(rclcpp::get_logger("pure_pursuit"), "%s", e.what());
    current_curvature = 0.0;
  }
  return current_curvature;
}

void PurePursuitLateralController::averageFilterTrajectory(
  autoware_auto_planning_msgs::msg::Trajectory & u)
{
  if (static_cast<int>(u.points.size()) <= 2 * param_.path_filter_moving_ave_num) {
    RCLCPP_ERROR(logger_, "Cannot smooth path! Trajectory size is too low!");
    return;
  }

  autoware_auto_planning_msgs::msg::Trajectory filtered_trajectory(u);

  for (int64_t i = 0; i < static_cast<int64_t>(u.points.size()); ++i) {
    TrajectoryPoint tmp{};
    int64_t num_tmp = param_.path_filter_moving_ave_num;
    int64_t count = 0;
    double yaw = 0.0;
    if (i - num_tmp < 0) {
      num_tmp = i;
    }
    if (i + num_tmp > static_cast<int64_t>(u.points.size()) - 1) {
      num_tmp = static_cast<int64_t>(u.points.size()) - i - 1;
    }
    for (int64_t j = -num_tmp; j <= num_tmp; ++j) {
      const auto & p = u.points.at(static_cast<size_t>(i + j));

      tmp.pose.position.x += p.pose.position.x;
      tmp.pose.position.y += p.pose.position.y;
      tmp.pose.position.z += p.pose.position.z;
      tmp.longitudinal_velocity_mps += p.longitudinal_velocity_mps;
      tmp.acceleration_mps2 += p.acceleration_mps2;
      tmp.front_wheel_angle_rad += p.front_wheel_angle_rad;
      tmp.heading_rate_rps += p.heading_rate_rps;
      yaw += tf2::getYaw(p.pose.orientation);
      tmp.lateral_velocity_mps += p.lateral_velocity_mps;
      tmp.rear_wheel_angle_rad += p.rear_wheel_angle_rad;
      ++count;
    }
    auto & p = filtered_trajectory.points.at(static_cast<size_t>(i));

    p.pose.position.x = tmp.pose.position.x / count;
    p.pose.position.y = tmp.pose.position.y / count;
    p.pose.position.z = tmp.pose.position.z / count;
    p.longitudinal_velocity_mps = tmp.longitudinal_velocity_mps / count;
    p.acceleration_mps2 = tmp.acceleration_mps2 / count;
    p.front_wheel_angle_rad = tmp.front_wheel_angle_rad / count;
    p.heading_rate_rps = tmp.heading_rate_rps / count;
    p.lateral_velocity_mps = tmp.lateral_velocity_mps / count;
    p.rear_wheel_angle_rad = tmp.rear_wheel_angle_rad / count;
    p.pose.orientation = pure_pursuit::planning_utils::getQuaternionFromYaw(yaw / count);
  }
  trajectory_resampled_ = std::make_shared<Trajectory>(filtered_trajectory);
}

boost::optional<Trajectory> PurePursuitLateralController::generatePredictedTrajectory()
{
  const auto closest_idx_result =
    motion_utils::findNearestIndex(output_tp_array_, current_odometry_.pose.pose, 3.0, M_PI_4);

  if (!closest_idx_result) {
    return boost::none;
  }

  const double remaining_distance = planning_utils::calcArcLengthFromWayPoint(
    *trajectory_resampled_, *closest_idx_result, trajectory_resampled_->points.size() - 1);

  const auto num_of_iteration = std::max(
    static_cast<int>(std::ceil(
      std::min(remaining_distance, param_.prediction_distance_length) / param_.prediction_ds)),
    1);
  Trajectory predicted_trajectory;

  // Iterative prediction:
  for (int i = 0; i < num_of_iteration; i++) {
    if (i == 0) {
      // For first point, use the odometry for velocity, and use the current_pose for prediction.

      TrajectoryPoint p;
      p.pose = current_odometry_.pose.pose;
      p.longitudinal_velocity_mps = current_odometry_.twist.twist.linear.x;
      predicted_trajectory.points.push_back(p);

      const auto pp_output = calcTargetCurvature(true, predicted_trajectory.points.at(i).pose);
      AckermannLateralCommand tmp_msg;

      if (pp_output) {
        tmp_msg = generateCtrlCmdMsg(pp_output->curvature);
        predicted_trajectory.points.at(i).longitudinal_velocity_mps = pp_output->velocity;
      } else {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "failed to solve pure_pursuit for prediction");
        tmp_msg = generateCtrlCmdMsg(0.0);
      }
      TrajectoryPoint p2;
      p2 = calcNextPose(param_.prediction_ds, predicted_trajectory.points.at(i), tmp_msg);
      predicted_trajectory.points.push_back(p2);

    } else {
      const auto pp_output = calcTargetCurvature(false, predicted_trajectory.points.at(i).pose);
      AckermannLateralCommand tmp_msg;

      if (pp_output) {
        tmp_msg = generateCtrlCmdMsg(pp_output->curvature);
        predicted_trajectory.points.at(i).longitudinal_velocity_mps = pp_output->velocity;
      } else {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "failed to solve pure_pursuit for prediction");
        tmp_msg = generateCtrlCmdMsg(0.0);
      }
      predicted_trajectory.points.push_back(
        calcNextPose(param_.prediction_ds, predicted_trajectory.points.at(i), tmp_msg));
    }
  }

  // for last point
  predicted_trajectory.points.back().longitudinal_velocity_mps = 0.0;
  predicted_trajectory.header.frame_id = trajectory_resampled_->header.frame_id;
  predicted_trajectory.header.stamp = trajectory_resampled_->header.stamp;

  return predicted_trajectory;
}

bool PurePursuitLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

LateralOutput PurePursuitLateralController::run(const InputData & input_data)
{
  current_pose_ = input_data.current_odometry.pose.pose;
  trajectory_ = input_data.current_trajectory;
  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;

  setResampledTrajectory();
  if (param_.enable_path_smoothing) {
    averageFilterTrajectory(*trajectory_resampled_);
  }
  const auto cmd_msg = generateOutputControlCmd();

  LateralOutput output;
  output.control_cmd = cmd_msg;
  output.sync_data.is_steer_converged = calcIsSteerConverged(cmd_msg);

  // calculate predicted trajectory with iterative calculation
  const auto predicted_trajectory = generatePredictedTrajectory();
  if (!predicted_trajectory) {
    RCLCPP_ERROR(logger_, "Failed to generate predicted trajectory.");
  } else {
    pub_predicted_trajectory_->publish(*predicted_trajectory);
  }

  return output;
}

bool PurePursuitLateralController::calcIsSteerConverged(const AckermannLateralCommand & cmd)
{
  return std::abs(cmd.steering_tire_angle - current_steering_.steering_tire_angle) <
         static_cast<float>(param_.converged_steer_rad_);
}

AckermannLateralCommand PurePursuitLateralController::generateOutputControlCmd()
{
  // Generate the control command
  const auto pp_output = calcTargetCurvature(true, current_odometry_.pose.pose);
  AckermannLateralCommand output_cmd;

  if (pp_output) {
    output_cmd = generateCtrlCmdMsg(pp_output->curvature);
    prev_cmd_ = boost::optional<AckermannLateralCommand>(output_cmd);
    publishDebugMarker();
  } else {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "failed to solve pure_pursuit for control command calculation");
    if (prev_cmd_) {
      output_cmd = *prev_cmd_;
    } else {
      output_cmd = generateCtrlCmdMsg(0.0);
    }
  }
  return output_cmd;
}

AckermannLateralCommand PurePursuitLateralController::generateCtrlCmdMsg(
  const double target_curvature)
{
  const double tmp_steering =
    planning_utils::convertCurvatureToSteeringAngle(param_.wheel_base, target_curvature);
  AckermannLateralCommand cmd;
  cmd.stamp = clock_->now();
  cmd.steering_tire_angle = static_cast<float>(
    std::min(std::max(tmp_steering, -param_.max_steering_angle), param_.max_steering_angle));

  // pub_ctrl_cmd_->publish(cmd);
  return cmd;
}

void PurePursuitLateralController::publishDebugMarker() const
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(createNextTargetMarker(debug_data_.next_target));
  marker_array.markers.push_back(
    createTrajectoryCircleMarker(debug_data_.next_target, current_odometry_.pose.pose));
}

boost::optional<PpOutput> PurePursuitLateralController::calcTargetCurvature(
  bool is_control_output, geometry_msgs::msg::Pose pose)
{
  // Ignore invalid trajectory
  if (trajectory_resampled_->points.size() < 3) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "received path size is < 3, ignored");
    return {};
  }

  // Calculate target point for velocity/acceleration

  const auto closest_idx_result =
    motion_utils::findNearestIndex(output_tp_array_, pose, 3.0, M_PI_4);
  if (!closest_idx_result) {
    RCLCPP_ERROR(logger_, "cannot find closest waypoint");
    return {};
  }

  const double target_vel =
    trajectory_resampled_->points.at(*closest_idx_result).longitudinal_velocity_mps;

  // calculate the lateral error

  const double lateral_error =
    motion_utils::calcLateralOffset(trajectory_resampled_->points, pose.position);

  // calculate the current curvature

  const double current_curvature = calcCurvature(*closest_idx_result);

  // Calculate lookahead distance

  const bool is_reverse = (target_vel < 0);
  const double min_lookahead_distance =
    is_reverse ? param_.reverse_min_lookahead_distance : param_.min_lookahead_distance;
  double lookahead_distance = min_lookahead_distance;
  if (is_control_output) {
    lookahead_distance = calcLookaheadDistance(
      lateral_error, current_curvature, current_odometry_.twist.twist.linear.x,
      min_lookahead_distance, is_control_output);
  } else {
    lookahead_distance = calcLookaheadDistance(
      lateral_error, current_curvature, target_vel, min_lookahead_distance, is_control_output);
  }

  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(pose);
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(*trajectory_resampled_));
  pure_pursuit_->setLookaheadDistance(lookahead_distance);

  // Run PurePursuit
  const auto pure_pursuit_result = pure_pursuit_->run();
  if (!pure_pursuit_result.first) {
    return {};
  }

  const auto kappa = pure_pursuit_result.second;

  // Set debug data
  if (is_control_output) {
    debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();
  }
  PpOutput output{};
  output.curvature = kappa;
  if (!is_control_output) {
    output.velocity = current_odometry_.twist.twist.linear.x;
  } else {
    output.velocity = target_vel;
  }

  return output;
}
}  // namespace pure_pursuit
