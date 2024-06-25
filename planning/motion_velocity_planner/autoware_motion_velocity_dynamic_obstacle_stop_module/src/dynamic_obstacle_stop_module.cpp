// Copyright 2023-2024 TIER IV, Inc. All rights reserved.
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

#include "dynamic_obstacle_stop_module.hpp"

#include "collision.hpp"
#include "debug.hpp"
#include "footprint.hpp"
#include "object_filtering.hpp"
#include "object_stop_decision.hpp"
#include "types.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{

void DynamicObstacleStopModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  logger_ = node.get_logger().get_child(ns_);
  clock_ = node.get_clock();
  velocity_factor_interface_.init(autoware::motion_utils::PlanningBehavior::ROUTE_OBSTACLE);

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<autoware::universe_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ = node.create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
    "~/debug/" + ns_ + "/processing_time_ms", 1);

  using autoware::universe_utils::getOrDeclareParameter;
  auto & p = params_;
  p.extra_object_width = getOrDeclareParameter<double>(node, ns_ + ".extra_object_width");
  p.minimum_object_velocity = getOrDeclareParameter<double>(node, ns_ + ".minimum_object_velocity");
  p.stop_distance_buffer = getOrDeclareParameter<double>(node, ns_ + ".stop_distance_buffer");
  p.time_horizon = getOrDeclareParameter<double>(node, ns_ + ".time_horizon");
  p.hysteresis = getOrDeclareParameter<double>(node, ns_ + ".hysteresis");
  p.add_duration_buffer = getOrDeclareParameter<double>(node, ns_ + ".add_stop_duration_buffer");
  p.remove_duration_buffer =
    getOrDeclareParameter<double>(node, ns_ + ".remove_stop_duration_buffer");
  p.minimum_object_distance_from_ego_trajectory =
    getOrDeclareParameter<double>(node, ns_ + ".minimum_object_distance_from_ego_trajectory");
  p.ignore_unavoidable_collisions =
    getOrDeclareParameter<bool>(node, ns_ + ".ignore_unavoidable_collisions");

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  p.ego_lateral_offset =
    std::max(std::abs(vehicle_info.min_lateral_offset_m), vehicle_info.max_lateral_offset_m);
  p.ego_longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
}

void DynamicObstacleStopModule::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;
  auto & p = params_;
  updateParam(parameters, ns_ + ".extra_object_width", p.extra_object_width);
  updateParam(parameters, ns_ + ".minimum_object_velocity", p.minimum_object_velocity);
  updateParam(parameters, ns_ + ".stop_distance_buffer", p.stop_distance_buffer);
  updateParam(parameters, ns_ + ".time_horizon", p.time_horizon);
  updateParam(parameters, ns_ + ".hysteresis", p.hysteresis);
  updateParam(parameters, ns_ + ".add_stop_duration_buffer", p.add_duration_buffer);
  updateParam(parameters, ns_ + ".remove_stop_duration_buffer", p.remove_duration_buffer);
  updateParam(
    parameters, ns_ + ".minimum_object_distance_from_ego_trajectory",
    p.minimum_object_distance_from_ego_trajectory);
  updateParam(parameters, ns_ + ".ignore_unavoidable_collisions", p.ignore_unavoidable_collisions);
}

VelocityPlanningResult DynamicObstacleStopModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  VelocityPlanningResult result;
  debug_data_.reset_data();
  if (ego_trajectory_points.size() < 2) return result;

  autoware::universe_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();
  stopwatch.tic("preprocessing");
  dynamic_obstacle_stop::EgoData ego_data;
  ego_data.pose = planner_data->current_odometry.pose.pose;
  ego_data.trajectory = ego_trajectory_points;
  autoware::motion_utils::removeOverlapPoints(ego_data.trajectory);
  ego_data.first_trajectory_idx =
    autoware::motion_utils::findNearestSegmentIndex(ego_data.trajectory, ego_data.pose.position);
  ego_data.longitudinal_offset_to_first_trajectory_idx =
    autoware::motion_utils::calcLongitudinalOffsetToSegment(
      ego_data.trajectory, ego_data.first_trajectory_idx, ego_data.pose.position);
  const auto min_stop_distance = autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
                                   planner_data->current_odometry.twist.twist.linear.x, 0.0,
                                   planner_data->current_acceleration.accel.accel.linear.x,
                                   planner_data->velocity_smoother_->getMinDecel(),
                                   planner_data->velocity_smoother_->getMaxJerk(),
                                   planner_data->velocity_smoother_->getMinJerk())
                                   .value_or(0.0);
  ego_data.earliest_stop_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    ego_data.trajectory, ego_data.pose.position, min_stop_distance);

  dynamic_obstacle_stop::make_ego_footprint_rtree(ego_data, params_);
  double hysteresis =
    std::find_if(
      object_map_.begin(), object_map_.end(),
      [](const auto & pair) { return pair.second.should_be_avoided(); }) == object_map_.end()
      ? 0.0
      : params_.hysteresis;
  const auto dynamic_obstacles = dynamic_obstacle_stop::filter_predicted_objects(
    planner_data->predicted_objects, ego_data, params_, hysteresis);

  const auto preprocessing_duration_us = stopwatch.toc("preprocessing");

  stopwatch.tic("footprints");
  const auto obstacle_forward_footprints =
    dynamic_obstacle_stop::make_forward_footprints(dynamic_obstacles, params_, hysteresis);
  const auto footprints_duration_us = stopwatch.toc("footprints");
  stopwatch.tic("collisions");
  auto collisions = dynamic_obstacle_stop::find_collisions(
    ego_data, dynamic_obstacles, obstacle_forward_footprints);
  update_object_map(object_map_, collisions, clock_->now(), ego_data.trajectory, params_);
  std::optional<geometry_msgs::msg::Point> earliest_collision =
    dynamic_obstacle_stop::find_earliest_collision(object_map_, ego_data);
  const auto collisions_duration_us = stopwatch.toc("collisions");
  if (earliest_collision) {
    const auto arc_length_diff = autoware::motion_utils::calcSignedArcLength(
      ego_data.trajectory, *earliest_collision, ego_data.pose.position);
    const auto can_stop_before_limit = arc_length_diff < min_stop_distance -
                                                           params_.ego_longitudinal_offset -
                                                           params_.stop_distance_buffer;
    const auto stop_pose = can_stop_before_limit
                             ? autoware::motion_utils::calcLongitudinalOffsetPose(
                                 ego_data.trajectory, *earliest_collision,
                                 -params_.stop_distance_buffer - params_.ego_longitudinal_offset)
                             : ego_data.earliest_stop_pose;
    debug_data_.stop_pose = stop_pose;
    if (stop_pose) {
      result.stop_points.push_back(stop_pose->position);
      const auto stop_pose_reached =
        planner_data->current_odometry.twist.twist.linear.x < 1e-3 &&
        autoware::universe_utils::calcDistance2d(ego_data.pose, *stop_pose) < 1e-3;
      velocity_factor_interface_.set(
        ego_trajectory_points, ego_data.pose, *stop_pose,
        stop_pose_reached ? autoware::motion_utils::VelocityFactor::STOPPED
                          : autoware::motion_utils::VelocityFactor::APPROACHING,
        "dynamic_obstacle_stop");
      result.velocity_factor = velocity_factor_interface_.get();
      create_virtual_walls();
    }
  }

  debug_publisher_->publish(create_debug_marker_array());
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers());

  const auto total_time_us = stopwatch.toc();
  RCLCPP_DEBUG(
    logger_,
    "Total time = %2.2fus\n\tpreprocessing = %2.2fus\n\tfootprints = "
    "%2.2fus\n\tcollisions = %2.2fus\n",
    total_time_us, preprocessing_duration_us, footprints_duration_us, collisions_duration_us);
  debug_data_.ego_footprints = ego_data.trajectory_footprints;
  debug_data_.obstacle_footprints = obstacle_forward_footprints;
  debug_data_.z = ego_data.pose.position.z;
  std::map<std::string, double> processing_times;
  processing_times["preprocessing"] = preprocessing_duration_us / 1000;
  processing_times["footprints"] = footprints_duration_us / 1000;
  processing_times["collisions"] = collisions_duration_us / 1000;
  processing_times["Total"] = total_time_us / 1000;
  processing_diag_publisher_->publish(processing_times);
  tier4_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = clock_->now();
  processing_time_msg.data = processing_times["Total"];
  processing_time_publisher_->publish(processing_time_msg);
  return result;
}

visualization_msgs::msg::MarkerArray DynamicObstacleStopModule::create_debug_marker_array()
{
  const auto z = debug_data_.z;
  visualization_msgs::msg::MarkerArray array;
  std::string ns = "collisions";
  const auto collision_markers =
    dynamic_obstacle_stop::debug::make_collision_markers(object_map_, ns, z, clock_->now());
  dynamic_obstacle_stop::debug::add_markers(
    array, debug_data_.prev_collisions_nb, collision_markers, ns);
  ns = "dynamic_obstacles_footprints";
  const auto obstacle_footprint_markers =
    dynamic_obstacle_stop::debug::make_polygon_markers(debug_data_.obstacle_footprints, ns, z);
  dynamic_obstacle_stop::debug::add_markers(
    array, debug_data_.prev_dynamic_obstacles_nb, obstacle_footprint_markers, ns);
  ns = "ego_footprints";
  const auto ego_footprint_markers =
    dynamic_obstacle_stop::debug::make_polygon_markers(debug_data_.ego_footprints, ns, z);
  dynamic_obstacle_stop::debug::add_markers(
    array, debug_data_.prev_ego_footprints_nb, ego_footprint_markers, ns);
  return array;
}

void DynamicObstacleStopModule::create_virtual_walls()
{
  if (debug_data_.stop_pose) {
    autoware::motion_utils::VirtualWall virtual_wall;
    virtual_wall.text = "dynamic_obstacle_stop";
    virtual_wall.longitudinal_offset = params_.ego_longitudinal_offset;
    virtual_wall.style = autoware::motion_utils::VirtualWallType::stop;
    virtual_wall.pose = *debug_data_.stop_pose;
    virtual_wall_marker_creator.add_virtual_wall(virtual_wall);
  }
}

}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::DynamicObstacleStopModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
