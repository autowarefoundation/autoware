// Copyright 2022-2024 TIER IV, Inc.
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

#include "obstacle_velocity_limiter_module.hpp"

#include "debug.hpp"
#include "map_utils.hpp"
#include "obstacle_velocity_limiter.hpp"
#include "parameters.hpp"
#include "trajectory_preprocessing.hpp"

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry.hpp>

#include <chrono>
#include <map>

namespace autoware::motion_velocity_planner
{
void ObstacleVelocityLimiterModule::init(rclcpp::Node & node, const std::string & module_name)
{
  module_name_ = module_name;
  logger_ = node.get_logger().get_child("obstacle_velocity_limiter");
  clock_ = node.get_clock();
  preprocessing_params_ = obstacle_velocity_limiter::PreprocessingParameters(node);
  projection_params_ = obstacle_velocity_limiter::ProjectionParameters(node);
  obstacle_params_ = obstacle_velocity_limiter::ObstacleParameters(node);
  velocity_params_ = obstacle_velocity_limiter::VelocityParameters(node);
  velocity_factor_interface_.init(autoware::motion_utils::PlanningBehavior::ROUTE_OBSTACLE);

  debug_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/debug_markers", 1);
  virtual_wall_publisher_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/" + ns_ + "/virtual_walls", 1);
  processing_diag_publisher_ = std::make_shared<autoware::universe_utils::ProcessingTimePublisher>(
    &node, "~/debug/" + ns_ + "/processing_time_ms_diag");
  processing_time_publisher_ = node.create_publisher<tier4_debug_msgs::msg::Float64Stamped>(
    "~/debug/" + ns_ + "/processing_time_ms", 1);

  const auto vehicle_info = vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  vehicle_lateral_offset_ = static_cast<double>(vehicle_info.max_lateral_offset_m);
  vehicle_front_offset_ = static_cast<double>(vehicle_info.max_longitudinal_offset_m);
  distance_buffer_ = node.declare_parameter<double>("distance_buffer");

  projection_params_.wheel_base = vehicle_info.wheel_base_m;
  projection_params_.extra_length = vehicle_front_offset_ + distance_buffer_;
}

void ObstacleVelocityLimiterModule::update_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using obstacle_velocity_limiter::ObstacleParameters;
  using obstacle_velocity_limiter::PreprocessingParameters;
  using obstacle_velocity_limiter::ProjectionParameters;
  using obstacle_velocity_limiter::VelocityParameters;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "distance_buffer") {
      distance_buffer_ = static_cast<double>(parameter.as_double());
      projection_params_.extra_length = vehicle_front_offset_ + distance_buffer_;
      // Preprocessing parameters
    } else if (parameter.get_name() == PreprocessingParameters::START_DIST_PARAM) {
      preprocessing_params_.start_distance = static_cast<double>(parameter.as_double());
    } else if (parameter.get_name() == PreprocessingParameters::DOWNSAMPLING_PARAM) {
      preprocessing_params_.updateDownsampleFactor(parameter.as_int());
    } else if (parameter.get_name() == PreprocessingParameters::CALC_STEER_PARAM) {
      preprocessing_params_.calculate_steering_angles = parameter.as_bool();
    } else if (parameter.get_name() == PreprocessingParameters::MAX_LENGTH_PARAM) {
      preprocessing_params_.max_length = static_cast<double>(parameter.as_double());
    } else if (parameter.get_name() == PreprocessingParameters::MAX_DURATION_PARAM) {
      preprocessing_params_.max_duration = static_cast<double>(parameter.as_double());
      // Velocity parameters
    } else if (parameter.get_name() == VelocityParameters::MIN_VEL_PARAM) {
      velocity_params_.min_velocity = static_cast<double>(parameter.as_double());
    } else if (parameter.get_name() == VelocityParameters::MAX_DECEL_PARAM) {
      velocity_params_.max_deceleration = static_cast<double>(parameter.as_double());
      // Obstacle parameters
    } else if (parameter.get_name() == ProjectionParameters::DURATION_PARAM) {
      const auto min_ttc = static_cast<double>(parameter.as_double());
      if (min_ttc > 0.0) projection_params_.duration = min_ttc;
    } else if (parameter.get_name() == ObstacleParameters::DYN_SOURCE_PARAM) {
      obstacle_params_.updateType(logger_, parameter.as_string());
    } else if (parameter.get_name() == ObstacleParameters::OCC_GRID_THRESH_PARAM) {
      obstacle_params_.occupancy_grid_threshold = static_cast<int8_t>(parameter.as_int());
    } else if (parameter.get_name() == ObstacleParameters::BUFFER_PARAM) {
      obstacle_params_.dynamic_obstacles_buffer = static_cast<double>(parameter.as_double());
    } else if (parameter.get_name() == ObstacleParameters::MIN_VEL_PARAM) {
      obstacle_params_.dynamic_obstacles_min_vel = static_cast<double>(parameter.as_double());
    } else if (parameter.get_name() == ObstacleParameters::MAP_TAGS_PARAM) {
      obstacle_params_.static_map_tags = parameter.as_string_array();
    } else if (parameter.get_name() == ObstacleParameters::FILTERING_PARAM) {
      obstacle_params_.filter_envelope = parameter.as_bool();
    } else if (parameter.get_name() == ObstacleParameters::IGNORE_ON_PATH_PARAM) {
      obstacle_params_.ignore_on_path = parameter.as_bool();
    } else if (parameter.get_name() == ObstacleParameters::IGNORE_DIST_PARAM) {
      obstacle_params_.ignore_extra_distance = static_cast<double>(parameter.as_double());
    } else if (parameter.get_name() == ObstacleParameters::RTREE_POINTS_PARAM) {
      obstacle_params_.updateRtreeMinPoints(logger_, static_cast<int>(parameter.as_int()));
    } else if (parameter.get_name() == ObstacleParameters::RTREE_SEGMENTS_PARAM) {
      obstacle_params_.updateRtreeMinSegments(logger_, static_cast<int>(parameter.as_int()));
      // Projection parameters
    } else if (parameter.get_name() == ProjectionParameters::MODEL_PARAM) {
      projection_params_.updateModel(logger_, parameter.as_string());
    } else if (parameter.get_name() == ProjectionParameters::NB_POINTS_PARAM) {
      projection_params_.updateNbPoints(logger_, static_cast<int>(parameter.as_int()));
    } else if (parameter.get_name() == ProjectionParameters::STEER_OFFSET_PARAM) {
      projection_params_.steering_angle_offset = parameter.as_double();
    } else if (parameter.get_name() == ProjectionParameters::DISTANCE_METHOD_PARAM) {
      projection_params_.updateDistanceMethod(logger_, parameter.as_string());
    } else {
      RCLCPP_WARN(logger_, "Unknown parameter %s", parameter.get_name().c_str());
    }
  }
}

VelocityPlanningResult ObstacleVelocityLimiterModule::plan(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  autoware::universe_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();
  VelocityPlanningResult result;
  stopwatch.tic("preprocessing");
  const auto ego_idx = autoware::motion_utils::findNearestIndex(
    ego_trajectory_points, planner_data->current_odometry.pose.pose);
  if (!ego_idx) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, rcutils_duration_value_t(1000),
      "Cannot calculate ego index on the trajectory");
    return result;
  }
  auto original_traj_points = ego_trajectory_points;
  if (preprocessing_params_.calculate_steering_angles)
    obstacle_velocity_limiter::calculateSteeringAngles(
      original_traj_points, projection_params_.wheel_base);
  velocity_params_.current_ego_velocity = planner_data->current_odometry.twist.twist.linear.x;
  const auto start_idx = obstacle_velocity_limiter::calculateStartIndex(
    original_traj_points, *ego_idx, preprocessing_params_.start_distance);
  const auto end_idx = obstacle_velocity_limiter::calculateEndIndex(
    original_traj_points, start_idx, preprocessing_params_.max_length,
    preprocessing_params_.max_duration);
  auto downsampled_traj_points = obstacle_velocity_limiter::downsampleTrajectory(
    original_traj_points, start_idx, end_idx, preprocessing_params_.downsample_factor);
  if (prev_inserted_point_) {
    obstacle_velocity_limiter::add_trajectory_point(downsampled_traj_points, *prev_inserted_point_);
  }
  obstacle_velocity_limiter::ObstacleMasks obstacle_masks;
  const auto preprocessing_us = stopwatch.toc("preprocessing");
  stopwatch.tic("obstacles");
  obstacle_masks.negative_masks = obstacle_velocity_limiter::createPolygonMasks(
    planner_data->predicted_objects, obstacle_params_.dynamic_obstacles_buffer,
    obstacle_params_.dynamic_obstacles_min_vel);
  if (obstacle_params_.ignore_on_path)
    obstacle_masks.negative_masks.push_back(obstacle_velocity_limiter::createTrajectoryFootprint(
      original_traj_points, vehicle_lateral_offset_ + obstacle_params_.ignore_extra_distance));
  const auto projected_linestrings =
    obstacle_velocity_limiter::createProjectedLines(downsampled_traj_points, projection_params_);
  const auto footprint_polygons = obstacle_velocity_limiter::createFootprintPolygons(
    projected_linestrings, vehicle_lateral_offset_);
  obstacle_velocity_limiter::Obstacles obstacles;
  obstacles.lines = obstacle_velocity_limiter::extractStaticObstacles(
    *planner_data->route_handler->getLaneletMapPtr(), obstacle_params_.static_map_tags,
    footprint_polygons);
  if (
    obstacle_params_.dynamic_source != obstacle_velocity_limiter::ObstacleParameters::STATIC_ONLY) {
    if (obstacle_params_.filter_envelope)
      obstacle_masks.positive_mask =
        obstacle_velocity_limiter::createEnvelopePolygon(footprint_polygons);
    obstacle_velocity_limiter::addSensorObstacles(
      obstacles, planner_data->occupancy_grid, planner_data->no_ground_pointcloud, obstacle_masks,
      obstacle_params_);
  }
  const auto obstacles_us = stopwatch.toc("obstacles");
  autoware::motion_utils::VirtualWalls virtual_walls;
  stopwatch.tic("slowdowns");
  result.slowdown_intervals = obstacle_velocity_limiter::calculate_slowdown_intervals(
    downsampled_traj_points,
    obstacle_velocity_limiter::CollisionChecker(
      obstacles, obstacle_params_.rtree_min_points, obstacle_params_.rtree_min_segments),
    projected_linestrings, footprint_polygons, projection_params_, velocity_params_, virtual_walls);
  const auto slowdowns_us = stopwatch.toc("slowdowns");

  for (auto & wall : virtual_walls) {
    wall.longitudinal_offset = vehicle_front_offset_;
    wall.text = ns_;
    wall.style = autoware::motion_utils::VirtualWallType::slowdown;
  }
  virtual_wall_marker_creator.add_virtual_walls(virtual_walls);
  virtual_wall_publisher_->publish(virtual_wall_marker_creator.create_markers(clock_->now()));
  if (!result.slowdown_intervals.empty()) {
    prev_inserted_point_ = result.slowdown_intervals.front().from;
  }

  const auto total_us = stopwatch.toc();
  RCLCPP_DEBUG(
    logger_,
    "Total runtime: %2.2fus\n"
    "\tpreprocessing = %2.0fus\n"
    "\tobstacles = %2.0fus\n"
    "\tslowdowns = %2.0fus\n",
    total_us, preprocessing_us, obstacles_us, slowdowns_us);
  if (debug_publisher_->get_subscription_count() > 0) {
    const auto safe_projected_linestrings =
      obstacle_velocity_limiter::createProjectedLines(downsampled_traj_points, projection_params_);
    const auto safe_footprint_polygons = obstacle_velocity_limiter::createFootprintPolygons(
      safe_projected_linestrings, vehicle_lateral_offset_);
    debug_publisher_->publish(makeDebugMarkers(
      obstacles, projected_linestrings, safe_projected_linestrings, footprint_polygons,
      safe_footprint_polygons, obstacle_masks,
      planner_data->current_odometry.pose.pose.position.z));
  }
  std::map<std::string, double> processing_times;
  processing_times["preprocessing"] = preprocessing_us / 1000;
  processing_times["obstacles"] = obstacles_us / 1000;
  processing_times["slowdowns"] = slowdowns_us / 1000;
  processing_times["Total"] = total_us / 1000;
  processing_diag_publisher_->publish(processing_times);
  tier4_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = clock_->now();
  processing_time_msg.data = processing_times["Total"];
  processing_time_publisher_->publish(processing_time_msg);
  return result;
}
}  // namespace autoware::motion_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::motion_velocity_planner::ObstacleVelocityLimiterModule,
  autoware::motion_velocity_planner::PluginModuleInterface)
