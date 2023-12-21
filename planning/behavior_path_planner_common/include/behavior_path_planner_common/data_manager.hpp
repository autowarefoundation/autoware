// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_

#include "behavior_path_planner_common/parameters.hpp"
#include "behavior_path_planner_common/turn_signal_decider.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

#include <lanelet2_extension/regulatory_elements/Forward.hpp>
#include <rclcpp/rclcpp/clock.hpp>
#include <rclcpp/rclcpp/time.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/lateral_offset.hpp>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_perception_msgs::msg::TrafficSignal;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using route_handler::RouteHandler;
using tier4_planning_msgs::msg::LateralOffset;
using PlanResult = PathWithLaneId::SharedPtr;
using lanelet::TrafficLight;
using unique_identifier_msgs::msg::UUID;

struct TrafficSignalStamped
{
  builtin_interfaces::msg::Time stamp;
  TrafficSignal signal;
};

struct BoolStamped
{
  explicit BoolStamped(bool in_data) : data(in_data) {}
  bool data{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct ModuleNameStamped
{
  std::string module_name = "NONE";
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct DrivableLanes
{
  lanelet::ConstLanelet right_lane;
  lanelet::ConstLanelet left_lane;
  lanelet::ConstLanelets middle_lanes;
};

// NOTE: To deal with some policies about drivable area generation, currently DrivableAreaInfo is
// quite messy. Needs to be refactored.
struct DrivableAreaInfo
{
  struct Obstacle
  {
    geometry_msgs::msg::Pose pose;
    tier4_autoware_utils::Polygon2d poly;
    bool is_left{true};
  };
  std::vector<DrivableLanes> drivable_lanes{};
  std::vector<Obstacle> obstacles{};  // obstacles to extract from the drivable area
  bool enable_expanding_hatched_road_markings{false};
  bool enable_expanding_intersection_areas{false};

  // temporary only for pull over's freespace planning
  double drivable_margin{0.0};

  // temporary only for side shift
  bool is_already_expanded{false};
};

struct BehaviorModuleOutput
{
  BehaviorModuleOutput() = default;

  // path planed by module
  PathWithLaneId path{};

  // reference path planed by module
  PathWithLaneId reference_path{};

  TurnSignalInfo turn_signal_info{};

  std::optional<PoseWithUuidStamped> modified_goal{};

  // drivable area info to create drivable area
  // NOTE: Drivable area in the path is generated at last from drivable_area_info.
  DrivableAreaInfo drivable_area_info;
};

struct CandidateOutput
{
  CandidateOutput() = default;
  explicit CandidateOutput(PathWithLaneId path) : path_candidate{std::move(path)} {}
  PathWithLaneId path_candidate{};
  double lateral_shift{0.0};
  double start_distance_to_path_change{std::numeric_limits<double>::lowest()};
  double finish_distance_to_path_change{std::numeric_limits<double>::lowest()};
};

struct PlannerData
{
  Odometry::ConstSharedPtr self_odometry{};
  AccelWithCovarianceStamped::ConstSharedPtr self_acceleration{};
  PredictedObjects::ConstSharedPtr dynamic_object{};
  OccupancyGrid::ConstSharedPtr occupancy_grid{};
  OccupancyGrid::ConstSharedPtr costmap{};
  LateralOffset::ConstSharedPtr lateral_offset{};
  OperationModeState::ConstSharedPtr operation_mode{};
  PathWithLaneId::SharedPtr prev_output_path{std::make_shared<PathWithLaneId>()};
  std::optional<PoseWithUuidStamped> prev_modified_goal{};
  std::optional<UUID> prev_route_id{};
  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};
  std::map<int64_t, TrafficSignalStamped> traffic_light_id_map;
  BehaviorPathPlannerParameters parameters{};
  drivable_area_expansion::DrivableAreaExpansionParameters drivable_area_expansion_parameters{};

  mutable std::vector<geometry_msgs::msg::Pose> drivable_area_expansion_prev_path_poses{};
  mutable std::vector<double> drivable_area_expansion_prev_curvatures{};
  mutable TurnSignalDecider turn_signal_decider;

  TurnIndicatorsCommand getTurnSignal(
    const PathWithLaneId & path, const TurnSignalInfo & turn_signal_info,
    TurnSignalDebugData & debug_data)
  {
    const auto & current_pose = self_odometry->pose.pose;
    const auto & current_vel = self_odometry->twist.twist.linear.x;
    return turn_signal_decider.getTurnSignal(
      route_handler, path, turn_signal_info, current_pose, current_vel, parameters, debug_data);
  }

  std::optional<TrafficSignalStamped> getTrafficSignal(const int64_t id) const
  {
    if (traffic_light_id_map.count(id) == 0) {
      return std::nullopt;
    }

    const auto elapsed_time =
      (rclcpp::Clock{RCL_ROS_TIME}.now() - traffic_light_id_map.at(id).stamp).seconds();
    if (elapsed_time > parameters.traffic_light_signal_timeout) {
      return std::nullopt;
    }

    return traffic_light_id_map.at(id);
  }

  template <class T>
  size_t findEgoIndex(const std::vector<T> & points) const
  {
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
  }

  template <class T>
  size_t findEgoSegmentIndex(const std::vector<T> & points) const
  {
    return motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
  }
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_
