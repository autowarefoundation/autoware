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

#ifndef OBSTACLE_VELOCITY_LIMITER__PARAMETERS_HPP_
#define OBSTACLE_VELOCITY_LIMITER__PARAMETERS_HPP_

#include <obstacle_velocity_limiter/types.hpp>
#include <rclcpp/node.hpp>

#include <tf2/utils.h>

#include <string>
#include <vector>

namespace obstacle_velocity_limiter
{

struct ObstacleParameters
{
  static constexpr auto DYN_SOURCE_PARAM = "obstacles.dynamic_source";
  static constexpr auto OCC_GRID_THRESH_PARAM = "obstacles.occupancy_grid_threshold";
  static constexpr auto BUFFER_PARAM = "obstacles.dynamic_obstacles_buffer";
  static constexpr auto MIN_VEL_PARAM = "obstacles.dynamic_obstacles_min_vel";
  static constexpr auto MAP_TAGS_PARAM = "obstacles.static_map_tags";
  static constexpr auto FILTERING_PARAM = "obstacles.filter_envelope";
  static constexpr auto IGNORE_ON_PATH_PARAM = "obstacles.ignore_obstacles_on_path";
  static constexpr auto IGNORE_DIST_PARAM = "obstacles.ignore_extra_distance";
  static constexpr auto RTREE_SEGMENTS_PARAM = "obstacles.rtree_min_segments";
  static constexpr auto RTREE_POINTS_PARAM = "obstacles.rtree_min_points";

  enum { POINTCLOUD, OCCUPANCYGRID, STATIC_ONLY } dynamic_source = OCCUPANCYGRID;
  int8_t occupancy_grid_threshold{};
  Float dynamic_obstacles_buffer{};
  Float dynamic_obstacles_min_vel{};
  std::vector<std::string> static_map_tags{};
  bool filter_envelope;
  bool ignore_on_path;
  Float ignore_extra_distance;
  size_t rtree_min_points{};
  size_t rtree_min_segments{};

  ObstacleParameters() = default;
  explicit ObstacleParameters(rclcpp::Node & node)
  {
    updateType(node, node.declare_parameter<std::string>(DYN_SOURCE_PARAM));
    occupancy_grid_threshold =
      static_cast<int8_t>(node.declare_parameter<int>(OCC_GRID_THRESH_PARAM));
    dynamic_obstacles_buffer = static_cast<Float>(node.declare_parameter<Float>(BUFFER_PARAM));
    dynamic_obstacles_min_vel = static_cast<Float>(node.declare_parameter<Float>(MIN_VEL_PARAM));
    static_map_tags = node.declare_parameter<std::vector<std::string>>(MAP_TAGS_PARAM);
    filter_envelope = node.declare_parameter<bool>(FILTERING_PARAM);
    ignore_on_path = node.declare_parameter<bool>(IGNORE_ON_PATH_PARAM);
    ignore_extra_distance = static_cast<Float>(node.declare_parameter<Float>(IGNORE_DIST_PARAM));
    updateRtreeMinPoints(node, static_cast<int>(node.declare_parameter<int>(RTREE_POINTS_PARAM)));
    updateRtreeMinSegments(
      node, static_cast<int>(node.declare_parameter<int>(RTREE_SEGMENTS_PARAM)));
  }

  bool updateType(rclcpp::Node & node, const std::string & type)
  {
    if (type == "pointcloud") {
      dynamic_source = POINTCLOUD;
    } else if (type == "occupancy_grid") {
      dynamic_source = OCCUPANCYGRID;
    } else if (type == "static_only") {
      dynamic_source = STATIC_ONLY;
    } else {
      dynamic_source = STATIC_ONLY;
      RCLCPP_WARN(
        node.get_logger(), "Unknown '%s' value: '%s'. Using default 'static_only'.",
        DYN_SOURCE_PARAM, type.c_str());
      return false;
    }
    return true;
  }

  bool updateRtreeMinPoints(rclcpp::Node & node, const int & size)
  {
    if (size < 0) {
      RCLCPP_WARN(
        node.get_logger(), "Min points for the Rtree must be positive. %d was given.", size);
      return false;
    }
    rtree_min_segments = static_cast<size_t>(size);
    return true;
  }

  bool updateRtreeMinSegments(rclcpp::Node & node, const int & size)
  {
    if (size < 0) {
      RCLCPP_WARN(
        node.get_logger(), "Min segments for the Rtree must be positive. %d was given.", size);
      return false;
    }
    rtree_min_segments = static_cast<size_t>(size);
    return true;
  }
};

struct ProjectionParameters
{
  static constexpr auto MODEL_PARAM = "simulation.model";
  static constexpr auto NBPOINTS_PARAM = "simulation.nb_points";
  static constexpr auto STEER_OFFSET_PARAM = "simulation.steering_offset";
  static constexpr auto DISTANCE_METHOD_PARAM = "simulation.distance_method";
  static constexpr auto DURATION_PARAM = "min_ttc";

  enum { PARTICLE, BICYCLE } model = PARTICLE;
  enum { EXACT, APPROXIMATION } distance_method = EXACT;
  double duration{};
  double extra_length{};
  double velocity{};
  double heading{};
  // parameters specific to the bicycle model
  int points_per_projection = 5;
  double wheel_base{};
  double steering_angle{};
  double steering_angle_offset{};

  ProjectionParameters() = default;
  explicit ProjectionParameters(rclcpp::Node & node)
  {
    updateModel(node, node.declare_parameter<std::string>(MODEL_PARAM));
    updateDistanceMethod(node, node.declare_parameter<std::string>(DISTANCE_METHOD_PARAM));
    updateNbPoints(node, node.declare_parameter<int>(NBPOINTS_PARAM));
    steering_angle_offset = node.declare_parameter<double>(STEER_OFFSET_PARAM);
    duration = node.declare_parameter<double>(DURATION_PARAM);
  }

  bool updateModel(rclcpp::Node & node, const std::string & model_str)
  {
    if (model_str == "particle") {
      model = PARTICLE;
    } else if (model_str == "bicycle") {
      model = BICYCLE;
    } else {
      RCLCPP_WARN(
        node.get_logger(), "Unknown projection model: '%s'. Using default PARTICLE model.",
        model_str.c_str());
      return false;
    }
    return true;
  }

  bool updateDistanceMethod(rclcpp::Node & node, const std::string & method_str)
  {
    if (method_str == "exact") {
      distance_method = EXACT;
    } else if (method_str == "approximation") {
      distance_method = APPROXIMATION;
    } else {
      RCLCPP_WARN(
        node.get_logger(), "Unknown distance calculation method: '%s'. Using default EXACT method.",
        method_str.c_str());
      return false;
    }
    return true;
  }

  bool updateNbPoints(rclcpp::Node & node, const int nb_points)
  {
    if (nb_points < 2) {
      RCLCPP_WARN(
        node.get_logger(), "Cannot use less than 2 points per projection. Using value %d instead.",
        points_per_projection);
      return false;
    }
    points_per_projection = nb_points;
    return true;
  }

  void update(const TrajectoryPoint & point)
  {
    velocity = point.longitudinal_velocity_mps;
    heading = tf2::getYaw(point.pose.orientation);
    steering_angle = point.front_wheel_angle_rad;
  }
};

struct VelocityParameters
{
  static constexpr auto MIN_VEL_PARAM = "min_adjusted_velocity";
  static constexpr auto MAX_DECEL_PARAM = "max_deceleration";

  Float min_velocity{};
  Float max_deceleration{};
  Float current_ego_velocity{};

  VelocityParameters() = default;
  explicit VelocityParameters(rclcpp::Node & node)
  {
    min_velocity = static_cast<Float>(node.declare_parameter<double>(MIN_VEL_PARAM));
    max_deceleration = static_cast<Float>(node.declare_parameter<double>(MAX_DECEL_PARAM));
  }
};

struct PreprocessingParameters
{
  static constexpr auto DOWNSAMPLING_PARAM = "trajectory_preprocessing.downsample_factor";
  static constexpr auto START_DIST_PARAM = "trajectory_preprocessing.start_distance";
  static constexpr auto CALC_STEER_PARAM = "trajectory_preprocessing.calculate_steering_angles";
  static constexpr auto MAX_LENGTH_PARAM = "trajectory_preprocessing.max_length";
  static constexpr auto MAX_DURATION_PARAM = "trajectory_preprocessing.max_duration";

  int downsample_factor{};
  Float start_distance{};
  bool calculate_steering_angles{};
  Float max_length{};
  Float max_duration{};

  PreprocessingParameters() = default;
  explicit PreprocessingParameters(rclcpp::Node & node)
  {
    downsample_factor = node.declare_parameter<int>(DOWNSAMPLING_PARAM);
    start_distance = static_cast<Float>(node.declare_parameter<double>(START_DIST_PARAM));
    calculate_steering_angles = node.declare_parameter<bool>(CALC_STEER_PARAM);
    max_length = node.declare_parameter<Float>(MAX_LENGTH_PARAM);
    max_duration = node.declare_parameter<Float>(MAX_DURATION_PARAM);
  }
  bool updateDownsampleFactor(const int new_downsample_factor)
  {
    if (new_downsample_factor > 0) {
      downsample_factor = new_downsample_factor;
      return true;
    }
    return false;
  }
};

}  // namespace obstacle_velocity_limiter
#endif  // OBSTACLE_VELOCITY_LIMITER__PARAMETERS_HPP_
