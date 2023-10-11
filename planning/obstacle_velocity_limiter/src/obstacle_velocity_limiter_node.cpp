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

#include "obstacle_velocity_limiter/obstacle_velocity_limiter_node.hpp"

#include "obstacle_velocity_limiter/debug.hpp"
#include "obstacle_velocity_limiter/forward_projection.hpp"
#include "obstacle_velocity_limiter/map_utils.hpp"
#include "obstacle_velocity_limiter/obstacle_velocity_limiter.hpp"
#include "obstacle_velocity_limiter/parameters.hpp"
#include "obstacle_velocity_limiter/trajectory_preprocessing.hpp"
#include "obstacle_velocity_limiter/types.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <chrono>

namespace obstacle_velocity_limiter
{
ObstacleVelocityLimiterNode::ObstacleVelocityLimiterNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("obstacle_velocity_limiter", node_options),
  preprocessing_params_(*this),
  projection_params_(*this),
  obstacle_params_(*this),
  velocity_params_(*this)
{
  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, [this](const Trajectory::ConstSharedPtr msg) { onTrajectory(msg); });
  sub_occupancy_grid_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid", 1,
    [this](const OccupancyGrid::ConstSharedPtr msg) { occupancy_grid_ptr_ = msg; });
  sub_pointcloud_ = create_subscription<PointCloud>(
    "~/input/obstacle_pointcloud", rclcpp::QoS(1).best_effort(),
    [this](const PointCloud::ConstSharedPtr msg) { pointcloud_ptr_ = msg; });
  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/dynamic_obstacles", 1,
    [this](const PredictedObjects::ConstSharedPtr msg) { dynamic_obstacles_ptr_ = msg; });
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { current_odometry_ptr_ = msg; });
  map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/map", rclcpp::QoS{1}.transient_local(),
    [this](const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg) {
      lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
      static_map_obstacles_ =
        extractStaticObstacles(*lanelet_map_ptr_, obstacle_params_.static_map_tags);
    });

  pub_trajectory_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_debug_markers_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_markers", 1);
  pub_runtime_ =
    create_publisher<tier4_debug_msgs::msg::Float64Stamped>("~/output/runtime_microseconds", 1);

  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  vehicle_lateral_offset_ = static_cast<Float>(vehicle_info.max_lateral_offset_m);
  vehicle_front_offset_ = static_cast<Float>(vehicle_info.max_longitudinal_offset_m);

  projection_params_.wheel_base = vehicle_info.wheel_base_m;
  projection_params_.extra_length = vehicle_front_offset_ + distance_buffer_;

  set_param_res_ =
    add_on_set_parameters_callback([this](const auto & params) { return onParameter(params); });

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

rcl_interfaces::msg::SetParametersResult ObstacleVelocityLimiterNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    if (parameter.get_name() == "distance_buffer") {
      distance_buffer_ = static_cast<Float>(parameter.as_double());
      projection_params_.extra_length = vehicle_front_offset_ + distance_buffer_;
      // Preprocessing parameters
    } else if (parameter.get_name() == PreprocessingParameters::START_DIST_PARAM) {
      preprocessing_params_.start_distance = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == PreprocessingParameters::DOWNSAMPLING_PARAM) {
      if (!preprocessing_params_.updateDownsampleFactor(parameter.as_int())) {
        result.successful = false;
        result.reason = "downsample_factor must be positive";
      }
    } else if (parameter.get_name() == PreprocessingParameters::CALC_STEER_PARAM) {
      preprocessing_params_.calculate_steering_angles = parameter.as_bool();
    } else if (parameter.get_name() == PreprocessingParameters::MAX_LENGTH_PARAM) {
      preprocessing_params_.max_length = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == PreprocessingParameters::MAX_DURATION_PARAM) {
      preprocessing_params_.max_duration = static_cast<Float>(parameter.as_double());
      // Velocity parameters
    } else if (parameter.get_name() == VelocityParameters::MIN_VEL_PARAM) {
      velocity_params_.min_velocity = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == VelocityParameters::MAX_DECEL_PARAM) {
      velocity_params_.max_deceleration = static_cast<Float>(parameter.as_double());
      // Obstacle parameters
    } else if (parameter.get_name() == ProjectionParameters::DURATION_PARAM) {
      const auto min_ttc = static_cast<Float>(parameter.as_double());
      if (min_ttc > 0.0) {
        projection_params_.duration = min_ttc;
      } else {
        result.successful = false;
        result.reason = "duration of forward projection must be positive";
      }
    } else if (parameter.get_name() == ObstacleParameters::DYN_SOURCE_PARAM) {
      if (!obstacle_params_.updateType(*this, parameter.as_string())) {
        result.successful = false;
        result.reason = "dynamic_source value must be 'pointcloud' or 'occupancy_grid'";
      }
    } else if (parameter.get_name() == ObstacleParameters::OCC_GRID_THRESH_PARAM) {
      obstacle_params_.occupancy_grid_threshold = static_cast<int8_t>(parameter.as_int());
    } else if (parameter.get_name() == ObstacleParameters::BUFFER_PARAM) {
      obstacle_params_.dynamic_obstacles_buffer = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == ObstacleParameters::MIN_VEL_PARAM) {
      obstacle_params_.dynamic_obstacles_min_vel = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == ObstacleParameters::MAP_TAGS_PARAM) {
      obstacle_params_.static_map_tags = parameter.as_string_array();
      if (lanelet_map_ptr_)
        static_map_obstacles_ =
          extractStaticObstacles(*lanelet_map_ptr_, obstacle_params_.static_map_tags);
    } else if (parameter.get_name() == ObstacleParameters::FILTERING_PARAM) {
      obstacle_params_.filter_envelope = parameter.as_bool();
    } else if (parameter.get_name() == ObstacleParameters::IGNORE_ON_PATH_PARAM) {
      obstacle_params_.ignore_on_path = parameter.as_bool();
    } else if (parameter.get_name() == ObstacleParameters::IGNORE_DIST_PARAM) {
      obstacle_params_.ignore_extra_distance = static_cast<Float>(parameter.as_double());
    } else if (parameter.get_name() == ObstacleParameters::RTREE_POINTS_PARAM) {
      obstacle_params_.updateRtreeMinPoints(*this, static_cast<int>(parameter.as_int()));
    } else if (parameter.get_name() == ObstacleParameters::RTREE_SEGMENTS_PARAM) {
      obstacle_params_.updateRtreeMinSegments(*this, static_cast<int>(parameter.as_int()));
      // Projection parameters
    } else if (parameter.get_name() == ProjectionParameters::MODEL_PARAM) {
      if (!projection_params_.updateModel(*this, parameter.as_string())) {
        result.successful = false;
        result.reason = "Unknown forward projection model";
      }
    } else if (parameter.get_name() == ProjectionParameters::NB_POINTS_PARAM) {
      if (!projection_params_.updateNbPoints(*this, static_cast<int>(parameter.as_int()))) {
        result.successful = false;
        result.reason = "number of points for projections must be at least 2";
      }
    } else if (parameter.get_name() == ProjectionParameters::STEER_OFFSET_PARAM) {
      projection_params_.steering_angle_offset = parameter.as_double();
    } else if (parameter.get_name() == ProjectionParameters::DISTANCE_METHOD_PARAM) {
      projection_params_.updateDistanceMethod(*this, parameter.as_string());
    } else {
      RCLCPP_WARN(get_logger(), "Unknown parameter %s", parameter.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

void ObstacleVelocityLimiterNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  if (!validInputs()) return;
  const auto t_start = std::chrono::system_clock::now();
  const auto ego_idx =
    motion_utils::findNearestIndex(msg->points, current_odometry_ptr_->pose.pose);
  if (!ego_idx) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), rcutils_duration_value_t(1000),
      "Cannot calculate ego index on the trajectory");
    return;
  }
  auto original_traj = *msg;
  if (preprocessing_params_.calculate_steering_angles)
    calculateSteeringAngles(original_traj, projection_params_.wheel_base);
  velocity_params_.current_ego_velocity = current_odometry_ptr_->twist.twist.linear.x;
  const auto start_idx =
    calculateStartIndex(original_traj, *ego_idx, preprocessing_params_.start_distance);
  const auto end_idx = calculateEndIndex(
    original_traj, start_idx, preprocessing_params_.max_length, preprocessing_params_.max_duration);
  Trajectory downsampled_traj = downsampleTrajectory(
    original_traj, start_idx, end_idx, preprocessing_params_.downsample_factor);
  ObstacleMasks obstacle_masks;
  obstacle_masks.negative_masks = createPolygonMasks(
    *dynamic_obstacles_ptr_, obstacle_params_.dynamic_obstacles_buffer,
    obstacle_params_.dynamic_obstacles_min_vel);
  if (obstacle_params_.ignore_on_path)
    obstacle_masks.negative_masks.push_back(createTrajectoryFootprint(
      *msg, vehicle_lateral_offset_ + obstacle_params_.ignore_extra_distance));
  const auto projected_linestrings = createProjectedLines(downsampled_traj, projection_params_);
  const auto footprint_polygons =
    createFootprintPolygons(projected_linestrings, vehicle_lateral_offset_);
  Obstacles obstacles;
  obstacles.lines = static_map_obstacles_;
  if (obstacle_params_.dynamic_source != ObstacleParameters::STATIC_ONLY) {
    if (obstacle_params_.filter_envelope)
      obstacle_masks.positive_mask = createEnvelopePolygon(footprint_polygons);
    addSensorObstacles(
      obstacles, *occupancy_grid_ptr_, *pointcloud_ptr_, obstacle_masks, transform_listener_,
      original_traj.header.frame_id, obstacle_params_);
  }
  limitVelocity(
    downsampled_traj,
    CollisionChecker(
      obstacles, obstacle_params_.rtree_min_points, obstacle_params_.rtree_min_segments),
    projected_linestrings, footprint_polygons, projection_params_, velocity_params_);
  auto safe_trajectory = copyDownsampledVelocity(
    downsampled_traj, original_traj, start_idx, preprocessing_params_.downsample_factor);
  safe_trajectory.header.stamp = now();

  pub_trajectory_->publish(safe_trajectory);

  const auto t_end = std::chrono::system_clock::now();
  const auto runtime = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
  pub_runtime_->publish(tier4_debug_msgs::msg::Float64Stamped().set__stamp(now()).set__data(
    static_cast<double>(runtime.count())));

  if (pub_debug_markers_->get_subscription_count() > 0) {
    const auto safe_projected_linestrings =
      createProjectedLines(downsampled_traj, projection_params_);
    const auto safe_footprint_polygons =
      createFootprintPolygons(safe_projected_linestrings, vehicle_lateral_offset_);
    pub_debug_markers_->publish(makeDebugMarkers(
      obstacles, projected_linestrings, safe_projected_linestrings, footprint_polygons,
      safe_footprint_polygons, obstacle_masks, occupancy_grid_ptr_->info.origin.position.z));
  }
}

bool ObstacleVelocityLimiterNode::validInputs()
{
  constexpr auto one_sec = rcutils_duration_value_t(1000);
  if (!occupancy_grid_ptr_)
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), one_sec, "Occupancy grid not yet received");
  if (!dynamic_obstacles_ptr_)
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), one_sec, "Dynamic obstacles not yet received");
  if (!current_odometry_ptr_)
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), one_sec, "Current ego velocity not yet received");

  return occupancy_grid_ptr_ && dynamic_obstacles_ptr_ && current_odometry_ptr_;
}
}  // namespace obstacle_velocity_limiter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_velocity_limiter::ObstacleVelocityLimiterNode)
