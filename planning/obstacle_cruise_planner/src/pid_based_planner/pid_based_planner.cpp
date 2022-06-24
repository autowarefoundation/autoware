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
StopSpeedExceeded createStopSpeedExceededMsg(
  const rclcpp::Time & current_time, const bool stop_flag)
{
  StopSpeedExceeded msg{};
  msg.stamp = current_time;
  msg.stop_speed_exceeded = stop_flag;
  return msg;
}

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

template <class T>
size_t getIndexWithLongitudinalOffset(
  const T & points, const double longitudinal_offset, boost::optional<size_t> start_idx)
{
  if (points.empty()) {
    throw std::logic_error("points is empty.");
  }

  if (start_idx) {
    if (/*start_idx.get() < 0 || */ points.size() <= start_idx.get()) {
      throw std::out_of_range("start_idx is out of range.");
    }
  } else {
    if (longitudinal_offset > 0) {
      start_idx = 0;
    } else {
      start_idx = points.size() - 1;
    }
  }

  double sum_length = 0.0;
  if (longitudinal_offset > 0) {
    for (size_t i = start_idx.get(); i < points.size() - 1; ++i) {
      const double segment_length =
        tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
      sum_length += segment_length;
      if (sum_length >= longitudinal_offset) {
        const double front_length = segment_length;
        const double back_length = sum_length - longitudinal_offset;
        if (front_length < back_length) {
          return i;
        } else {
          return i + 1;
        }
      }
    }
    return points.size() - 1;
  }

  for (size_t i = start_idx.get(); i > 0; --i) {
    const double segment_length =
      tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
    sum_length += segment_length;
    if (sum_length >= -longitudinal_offset) {
      const double front_length = segment_length;
      const double back_length = sum_length + longitudinal_offset;
      if (front_length < back_length) {
        return i;
      } else {
        return i + 1;
      }
    }
  }
  return 0;
}

double calcMinimumDistanceToStop(const double initial_vel, const double min_acc)
{
  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

tier4_planning_msgs::msg::StopReasonArray makeStopReasonArray(
  const rclcpp::Time & current_time, const geometry_msgs::msg::Pose & stop_pose,
  const boost::optional<PIDBasedPlanner::StopObstacleInfo> & stop_obstacle_info)
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = current_time;

  // create stop factor
  tier4_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose = stop_pose;
  if (stop_obstacle_info) {
    stop_factor.stop_factor_points.emplace_back(stop_obstacle_info->obstacle.collision_point);
  }

  // create stop reason stamped
  tier4_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = tier4_planning_msgs::msg::StopReason::OBSTACLE_STOP;
  stop_reason_msg.stop_factors.emplace_back(stop_factor);

  // create stop reason array
  tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}
}  // namespace

PIDBasedPlanner::PIDBasedPlanner(
  rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
  const vehicle_info_util::VehicleInfo & vehicle_info)
: PlannerInterface(node, longitudinal_info, vehicle_info)
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
  stop_reasons_pub_ =
    node.create_publisher<tier4_planning_msgs::msg::StopReasonArray>("~/output/stop_reasons", 1);
  stop_speed_exceeded_pub_ =
    node.create_publisher<StopSpeedExceeded>("~/output/stop_speed_exceeded", 1);
  debug_values_pub_ = node.create_publisher<Float32MultiArrayStamped>("~/debug/values", 1);
}

Trajectory PIDBasedPlanner::generateTrajectory(
  const ObstacleCruisePlannerData & planner_data, boost::optional<VelocityLimit> & vel_limit,
  DebugData & debug_data)
{
  stop_watch_.tic(__func__);
  debug_values_.resetValues();

  // calc obstacles to cruise and stop
  boost::optional<StopObstacleInfo> stop_obstacle_info;
  boost::optional<CruiseObstacleInfo> cruise_obstacle_info;
  calcObstaclesToCruiseAndStop(planner_data, stop_obstacle_info, cruise_obstacle_info);

  // plan cruise
  planCruise(planner_data, vel_limit, cruise_obstacle_info, debug_data);

  // plan stop
  const auto output_traj = planStop(planner_data, stop_obstacle_info, debug_data);

  // publish debug values
  publishDebugValues(planner_data);

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), is_showing_debug_info_,
    "  %s := %f [ms]", __func__, calculation_time);

  return output_traj;
}

void PIDBasedPlanner::calcObstaclesToCruiseAndStop(
  const ObstacleCruisePlannerData & planner_data,
  boost::optional<StopObstacleInfo> & stop_obstacle_info,
  boost::optional<CruiseObstacleInfo> & cruise_obstacle_info)
{
  debug_values_.setValues(DebugValues::TYPE::CURRENT_VELOCITY, planner_data.current_vel);
  debug_values_.setValues(DebugValues::TYPE::CURRENT_ACCELERATION, planner_data.current_acc);

  auto modified_target_obstacles = planner_data.target_obstacles;

  // search highest probability obstacle for stop and cruise
  for (size_t o_idx = 0; o_idx < planner_data.target_obstacles.size(); ++o_idx) {
    const auto & obstacle = planner_data.target_obstacles.at(o_idx);

    // NOTE: from ego's front to obstacle's back
    const double dist_to_obstacle = calcDistanceToObstacle(planner_data, obstacle);

    const bool is_stop_required = isStopRequired(obstacle, modified_target_obstacles.at(o_idx));
    if (is_stop_required) {  // stop
      // calculate error distance (= distance to stop)
      const double error_dist = dist_to_obstacle - longitudinal_info_.safe_distance_margin;
      if (stop_obstacle_info) {
        if (error_dist > stop_obstacle_info->dist_to_stop) {
          continue;
        }
      }
      stop_obstacle_info = StopObstacleInfo(obstacle, error_dist);

      // update debug values
      debug_values_.setValues(DebugValues::TYPE::STOP_CURRENT_OBJECT_DISTANCE, dist_to_obstacle);
      debug_values_.setValues(DebugValues::TYPE::STOP_CURRENT_OBJECT_VELOCITY, obstacle.velocity);
      debug_values_.setValues(
        DebugValues::TYPE::STOP_TARGET_OBJECT_DISTANCE, longitudinal_info_.safe_distance_margin);
      debug_values_.setValues(
        DebugValues::TYPE::STOP_TARGET_ACCELERATION, longitudinal_info_.limit_min_accel);
      debug_values_.setValues(DebugValues::TYPE::STOP_ERROR_OBJECT_DISTANCE, error_dist);
    } else {  // cruise
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

  // TODO(shimizu) refactor obstacle stop;
  prev_target_obstacles_ = modified_target_obstacles;
}

double PIDBasedPlanner::calcDistanceToObstacle(
  const ObstacleCruisePlannerData & planner_data, const TargetObstacle & obstacle)
{
  const size_t ego_segment_idx =
    findExtendedNearestSegmentIndex(planner_data.traj, planner_data.current_pose);
  const double segment_offset = std::max(
    0.0, tier4_autoware_utils::calcLongitudinalOffsetToSegment(
           planner_data.traj.points, ego_segment_idx, planner_data.current_pose.position));
  const double offset = vehicle_info_.max_longitudinal_offset_m + segment_offset;

  return tier4_autoware_utils::calcSignedArcLength(
           planner_data.traj.points, ego_segment_idx, obstacle.collision_point) -
         offset;
}

Trajectory PIDBasedPlanner::planStop(
  const ObstacleCruisePlannerData & planner_data,
  const boost::optional<StopObstacleInfo> & stop_obstacle_info, DebugData & debug_data)
{
  bool will_collide_with_obstacle = false;

  size_t zero_vel_idx = 0;
  bool zero_vel_found = false;
  if (stop_obstacle_info) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), is_showing_debug_info_,
      "stop planning");

    auto local_stop_obstacle_info = stop_obstacle_info.get();

    // check if the ego will collide with the obstacle with a limit acceleration
    const double feasible_dist_to_stop =
      calcMinimumDistanceToStop(planner_data.current_vel, longitudinal_info_.limit_min_accel);
    if (local_stop_obstacle_info.dist_to_stop < feasible_dist_to_stop) {
      will_collide_with_obstacle = true;
      local_stop_obstacle_info.dist_to_stop = feasible_dist_to_stop;
    }

    // set zero velocity index
    const auto opt_zero_vel_idx = doStop(
      planner_data, local_stop_obstacle_info, debug_data.obstacles_to_stop,
      debug_data.stop_wall_marker);
    if (opt_zero_vel_idx) {
      zero_vel_idx = opt_zero_vel_idx.get();
      zero_vel_found = true;
    }
  }

  // generate output trajectory
  auto output_traj = planner_data.traj;
  if (zero_vel_found) {
    // publish stop reason
    const auto stop_pose = planner_data.traj.points.at(zero_vel_idx).pose;
    const auto stop_reasons_msg =
      makeStopReasonArray(planner_data.current_time, stop_pose, stop_obstacle_info);
    stop_reasons_pub_->publish(stop_reasons_msg);

    // insert zero_velocity
    for (size_t traj_idx = zero_vel_idx; traj_idx < output_traj.points.size(); ++traj_idx) {
      output_traj.points.at(traj_idx).longitudinal_velocity_mps = 0.0;
    }
  }

  // publish stop_speed_exceeded if the ego will collide with the obstacle
  const auto stop_speed_exceeded_msg =
    createStopSpeedExceededMsg(planner_data.current_time, will_collide_with_obstacle);
  stop_speed_exceeded_pub_->publish(stop_speed_exceeded_msg);

  return output_traj;
}

boost::optional<size_t> PIDBasedPlanner::doStop(
  const ObstacleCruisePlannerData & planner_data, const StopObstacleInfo & stop_obstacle_info,
  std::vector<TargetObstacle> & debug_obstacles_to_stop,
  visualization_msgs::msg::MarkerArray & debug_wall_marker) const
{
  // const size_t ego_idx = findExtendedNearestIndex(planner_data.traj, planner_data.current_pose);

  // TODO(murooka) use interpolation to calculate stop point and marker pose
  const auto modified_stop_info = [&]() -> boost::optional<std::pair<size_t, size_t>> {
    // const double dist_to_stop = stop_obstacle_info.dist_to_stop;

    const size_t collision_idx = tier4_autoware_utils::findNearestIndex(
      planner_data.traj.points, stop_obstacle_info.obstacle.collision_point);
    const size_t obstacle_zero_vel_idx = getIndexWithLongitudinalOffset(
      planner_data.traj.points,
      -longitudinal_info_.safe_distance_margin - vehicle_info_.max_longitudinal_offset_m,
      collision_idx);

    // check if there is already stop line between obstacle and zero_vel_idx
    const auto behavior_zero_vel_idx =
      tier4_autoware_utils::searchZeroVelocityIndex(planner_data.traj.points);
    if (behavior_zero_vel_idx) {
      const double zero_vel_diff_length = tier4_autoware_utils::calcSignedArcLength(
        planner_data.traj.points, obstacle_zero_vel_idx, behavior_zero_vel_idx.get());
      std::cerr << zero_vel_diff_length << std::endl;
      if (
        0 < zero_vel_diff_length &&
        zero_vel_diff_length < longitudinal_info_.safe_distance_margin) {
        const size_t modified_obstacle_zero_vel_idx = getIndexWithLongitudinalOffset(
          planner_data.traj.points,
          -min_behavior_stop_margin_ - vehicle_info_.max_longitudinal_offset_m, collision_idx);
        const size_t modified_wall_idx = getIndexWithLongitudinalOffset(
          planner_data.traj.points, -min_behavior_stop_margin_, collision_idx);
        return std::make_pair(modified_obstacle_zero_vel_idx, modified_wall_idx);
      }
    }

    const size_t wall_idx = getIndexWithLongitudinalOffset(
      planner_data.traj.points, -longitudinal_info_.safe_distance_margin, collision_idx);
    return std::make_pair(obstacle_zero_vel_idx, wall_idx);
  }();
  if (!modified_stop_info) {
    return {};
  }
  const size_t modified_zero_vel_idx = modified_stop_info->first;
  const size_t modified_wall_idx = modified_stop_info->second;

  // virtual wall marker for stop
  const auto marker_pose = planner_data.traj.points.at(modified_wall_idx).pose;
  const auto markers = tier4_autoware_utils::createStopVirtualWallMarker(
    marker_pose, "obstacle stop", planner_data.current_time, 0);
  tier4_autoware_utils::appendMarkerArray(markers, &debug_wall_marker);

  debug_obstacles_to_stop.push_back(stop_obstacle_info.obstacle);
  return modified_zero_vel_idx;
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

  const size_t ego_idx = findExtendedNearestIndex(planner_data.traj, planner_data.current_pose);

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
  const double dist_to_rss_wall = std::min(
    dist_to_cruise + vehicle_info_.max_longitudinal_offset_m,
    dist_to_obstacle + vehicle_info_.max_longitudinal_offset_m);
  const size_t wall_idx =
    getIndexWithLongitudinalOffset(planner_data.traj.points, dist_to_rss_wall, ego_idx);

  const auto markers = tier4_autoware_utils::createSlowDownVirtualWallMarker(
    planner_data.traj.points.at(wall_idx).pose, "obstacle cruise", planner_data.current_time, 0);
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
