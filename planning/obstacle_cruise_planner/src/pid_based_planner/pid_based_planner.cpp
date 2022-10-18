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

#include "obstacle_cruise_planner/pid_based_planner/pid_based_planner.hpp"

#include "obstacle_cruise_planner/utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "tier4_planning_msgs/msg/velocity_limit.hpp"

namespace
{
VelocityLimit createVelocityLimitMsg(
  const rclcpp::Time & current_time, const double vel, const double acc, const double max_jerk,
  const double min_jerk)
{
  VelocityLimit msg;
  msg.stamp = current_time;
  msg.sender = "obstacle_cruise_planner";
  msg.use_constraints = true;

  msg.max_velocity = vel;
  if (acc < 0) {
    msg.constraints.min_acceleration = acc;
  }
  msg.constraints.max_jerk = max_jerk;
  msg.constraints.min_jerk = min_jerk;

  return msg;
}

Float32MultiArrayStamped convertDebugValuesToMsg(
  const rclcpp::Time & current_time, const DebugValues & debug_values)
{
  Float32MultiArrayStamped debug_msg{};
  debug_msg.stamp = current_time;
  for (const auto & v : debug_values.getValues()) {
    debug_msg.data.push_back(v);
  }
  return debug_msg;
}
}  // namespace

PIDBasedPlanner::PIDBasedPlanner(
  rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
  const vehicle_info_util::VehicleInfo & vehicle_info, const EgoNearestParam & ego_nearest_param)
: PlannerInterface(node, longitudinal_info, vehicle_info, ego_nearest_param)
{
  min_accel_during_cruise_ =
    node.declare_parameter<double>("pid_based_planner.min_accel_during_cruise");

  // pid controller
  const double kp = node.declare_parameter<double>("pid_based_planner.kp");
  const double ki = node.declare_parameter<double>("pid_based_planner.ki");
  const double kd = node.declare_parameter<double>("pid_based_planner.kd");
  pid_controller_ = std::make_unique<PIDController>(kp, ki, kd);
  output_ratio_during_accel_ =
    node.declare_parameter<double>("pid_based_planner.output_ratio_during_accel");

  // some parameters
  // use_predicted_obstacle_pose_ =
  //   node.declare_parameter<bool>("pid_based_planner.use_predicted_obstacle_pose");

  vel_to_acc_weight_ = node.declare_parameter<double>("pid_based_planner.vel_to_acc_weight");

  min_cruise_target_vel_ =
    node.declare_parameter<double>("pid_based_planner.min_cruise_target_vel");

  // low pass filter
  const double lpf_cruise_gain =
    node.declare_parameter<double>("pid_based_planner.lpf_cruise_gain");
  lpf_cruise_ptr_ = std::make_shared<LowpassFilter1d>(lpf_cruise_gain);

  // publisher
  debug_values_pub_ = node.create_publisher<Float32MultiArrayStamped>("~/debug/values", 1);
}

Trajectory PIDBasedPlanner::generateCruiseTrajectory(
  const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
  DebugData & debug_data)
{
  stop_watch_.tic(__func__);
  debug_values_.resetValues();

  // calc obstacles to cruise
  boost::optional<CruiseObstacleInfo> cruise_obstacle_info;
  calcObstaclesToCruise(planner_data, cruise_obstacle_info);

  // plan cruise
  planCruise(planner_data, vel_limit, cruise_obstacle_info, debug_data);

  // publish debug values
  publishDebugValues(planner_data);

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), is_showing_debug_info_,
    "  %s := %f [ms]", __func__, calculation_time);

  return planner_data.traj;
}

void PIDBasedPlanner::calcObstaclesToCruise(
  const ObstacleCruisePlannerData & planner_data,
  boost::optional<CruiseObstacleInfo> & cruise_obstacle_info)
{
  debug_values_.setValues(DebugValues::TYPE::CURRENT_VELOCITY, planner_data.current_vel);
  debug_values_.setValues(DebugValues::TYPE::CURRENT_ACCELERATION, planner_data.current_acc);

  auto modified_target_obstacles = planner_data.target_obstacles;

  // search highest probability obstacle for cruise
  for (size_t o_idx = 0; o_idx < planner_data.target_obstacles.size(); ++o_idx) {
    const auto & obstacle = planner_data.target_obstacles.at(o_idx);

    if (obstacle.collision_points.empty()) {
      continue;
    }

    // NOTE: from ego's front to obstacle's back
    const double dist_to_obstacle =
      calcDistanceToCollisionPoint(planner_data, obstacle.collision_points.front().point);

    if (!obstacle.has_stopped) {  // cruise
      // calculate distance between ego and obstacle based on RSS
      const double rss_dist = calcRSSDistance(
        planner_data.current_vel, obstacle.velocity, longitudinal_info_.safe_distance_margin);

      // calculate error distance and normalized one
      const double error_dist = dist_to_obstacle - rss_dist;
      if (cruise_obstacle_info) {
        if (error_dist > cruise_obstacle_info->dist_to_cruise) {
          continue;
        }
      }
      const double normalized_dist_to_cruise = error_dist / dist_to_obstacle;
      cruise_obstacle_info =
        CruiseObstacleInfo(obstacle, error_dist, normalized_dist_to_cruise, dist_to_obstacle);

      // update debug values
      debug_values_.setValues(DebugValues::TYPE::CRUISE_CURRENT_OBJECT_VELOCITY, obstacle.velocity);
      debug_values_.setValues(DebugValues::TYPE::CRUISE_CURRENT_OBJECT_DISTANCE, dist_to_obstacle);
      debug_values_.setValues(DebugValues::TYPE::CRUISE_TARGET_OBJECT_DISTANCE, rss_dist);
      debug_values_.setValues(DebugValues::TYPE::CRUISE_ERROR_OBJECT_DISTANCE, error_dist);
    }
  }
}

void PIDBasedPlanner::planCruise(
  const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
  const boost::optional<CruiseObstacleInfo> & cruise_obstacle_info, DebugData & debug_data)
{
  // do cruise
  if (cruise_obstacle_info) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), is_showing_debug_info_,
      "cruise planning");

    vel_limit = doCruise(
      planner_data, cruise_obstacle_info.get(), debug_data.obstacles_to_cruise,
      debug_data.cruise_wall_marker);

    // update debug values
    debug_values_.setValues(DebugValues::TYPE::CRUISE_TARGET_VELOCITY, vel_limit->max_velocity);
    debug_values_.setValues(
      DebugValues::TYPE::CRUISE_TARGET_ACCELERATION, vel_limit->constraints.min_acceleration);
  } else {
    // reset previous target velocity if adaptive cruise is not enabled
    prev_target_vel_ = {};
    lpf_cruise_ptr_->reset();

    // delete marker
    const auto markers =
      motion_utils::createDeletedSlowDownVirtualWallMarker(planner_data.current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data.cruise_wall_marker);
  }
}

VelocityLimit PIDBasedPlanner::doCruise(
  const ObstacleCruisePlannerData & planner_data, const CruiseObstacleInfo & cruise_obstacle_info,
  std::vector<TargetObstacle> & debug_obstacles_to_cruise,
  visualization_msgs::msg::MarkerArray & debug_wall_marker)
{
  const double dist_to_cruise = cruise_obstacle_info.dist_to_cruise;
  const double filtered_normalized_dist_to_cruise = [&]() {
    const double normalized_dist_to_cruise = cruise_obstacle_info.normalized_dist_to_cruise;
    return lpf_cruise_ptr_->filter(normalized_dist_to_cruise);
  }();
  const double dist_to_obstacle = cruise_obstacle_info.dist_to_obstacle;

  const size_t ego_idx = findEgoIndex(planner_data.traj, planner_data.current_pose);

  // calculate target velocity with acceleration limit by PID controller
  const double pid_output_vel = pid_controller_->calc(filtered_normalized_dist_to_cruise);
  [[maybe_unused]] const double prev_vel =
    prev_target_vel_ ? prev_target_vel_.get() : planner_data.current_vel;

  const double additional_vel = [&]() {
    if (filtered_normalized_dist_to_cruise > 0) {
      return pid_output_vel * output_ratio_during_accel_;
    }
    return pid_output_vel;
  }();

  const double positive_target_vel =
    std::max(min_cruise_target_vel_, planner_data.current_vel + additional_vel);

  // calculate target acceleration
  const double target_acc = vel_to_acc_weight_ * additional_vel;
  const double target_acc_with_acc_limit =
    std::clamp(target_acc, min_accel_during_cruise_, longitudinal_info_.max_accel);

  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), is_showing_debug_info_,
    "target_velocity %f", positive_target_vel);

  prev_target_vel_ = positive_target_vel;

  // set target longitudinal motion
  const auto vel_limit = createVelocityLimitMsg(
    planner_data.current_time, positive_target_vel, target_acc_with_acc_limit,
    longitudinal_info_.max_jerk, longitudinal_info_.min_jerk);

  // virtual wall marker for cruise
  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);
  const double dist_to_rss_wall =
    std::min(dist_to_cruise + abs_ego_offset, dist_to_obstacle + abs_ego_offset);

  const auto markers = motion_utils::createSlowDownVirtualWallMarker(
    planner_data.traj.points.at(ego_idx).pose, "obstacle cruise", planner_data.current_time, 0,
    dist_to_rss_wall);
  tier4_autoware_utils::appendMarkerArray(markers, &debug_wall_marker);

  debug_obstacles_to_cruise.push_back(cruise_obstacle_info.obstacle);

  return vel_limit;
}

void PIDBasedPlanner::publishDebugValues(const ObstacleCruisePlannerData & planner_data) const
{
  const auto debug_values_msg = convertDebugValuesToMsg(planner_data.current_time, debug_values_);
  debug_values_pub_->publish(debug_values_msg);
}

void PIDBasedPlanner::updateParam(const std::vector<rclcpp::Parameter> & parameters)
{
  tier4_autoware_utils::updateParam<double>(
    parameters, "pid_based_planner.min_accel_during_cruise", min_accel_during_cruise_);

  // pid controller
  double kp = pid_controller_->getKp();
  double ki = pid_controller_->getKi();
  double kd = pid_controller_->getKd();

  tier4_autoware_utils::updateParam<double>(parameters, "pid_based_planner.kp", kp);
  tier4_autoware_utils::updateParam<double>(parameters, "pid_based_planner.ki", ki);
  tier4_autoware_utils::updateParam<double>(parameters, "pid_based_planner.kd", kd);
  tier4_autoware_utils::updateParam<double>(
    parameters, "pid_based_planner.output_ratio_during_accel", output_ratio_during_accel_);

  // vel_to_acc_weight
  tier4_autoware_utils::updateParam<double>(
    parameters, "pid_based_planner.vel_to_acc_weight", vel_to_acc_weight_);

  // min_cruise_target_vel
  tier4_autoware_utils::updateParam<double>(
    parameters, "pid_based_planner.min_cruise_target_vel", min_cruise_target_vel_);

  pid_controller_->updateParam(kp, ki, kd);
}
