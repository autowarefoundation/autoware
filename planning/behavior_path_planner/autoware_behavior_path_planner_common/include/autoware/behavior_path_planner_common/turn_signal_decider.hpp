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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__TURN_SIGNAL_DECIDER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__TURN_SIGNAL_DECIDER_HPP_

#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/behavior_path_planner_common/parameters.hpp>
#include <autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace autoware::behavior_path_planner
{
using autoware::route_handler::RouteHandler;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using tier4_planning_msgs::msg::PathWithLaneId;

const std::map<std::string, uint8_t> g_signal_map = {
  {"left", TurnIndicatorsCommand::ENABLE_LEFT},
  {"right", TurnIndicatorsCommand::ENABLE_RIGHT},
  {"straight", TurnIndicatorsCommand::DISABLE},
  {"none", TurnIndicatorsCommand::DISABLE}};

struct TurnSignalInfo
{
  TurnSignalInfo()
  {
    turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
    hazard_signal.command = HazardLightsCommand::NO_COMMAND;
  }

  TurnSignalInfo(const Pose & start, const Pose & end)
  {
    turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
    hazard_signal.command = HazardLightsCommand::NO_COMMAND;

    desired_start_point = start;
    desired_end_point = end;
    required_start_point = start;
    required_end_point = end;
  }

  // desired turn signal
  TurnIndicatorsCommand turn_signal;
  HazardLightsCommand hazard_signal;

  geometry_msgs::msg::Pose desired_start_point;
  geometry_msgs::msg::Pose desired_end_point;
  geometry_msgs::msg::Pose required_start_point;
  geometry_msgs::msg::Pose required_end_point;
};

struct TurnSignalDebugData
{
  TurnSignalInfo intersection_turn_signal_info;
  TurnSignalInfo behavior_turn_signal_info;
};

class TurnSignalDecider
{
public:
  TurnIndicatorsCommand getTurnSignal(
    const std::shared_ptr<RouteHandler> & route_handler, const PathWithLaneId & path,
    const TurnSignalInfo & turn_signal_info, const Pose & current_pose, const double current_vel,
    const BehaviorPathPlannerParameters & parameters, TurnSignalDebugData & debug_data);

  TurnIndicatorsCommand resolve_turn_signal(
    const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
    const TurnSignalInfo & intersection_signal_info, const TurnSignalInfo & behavior_signal_info,
    const double nearest_dist_threshold, const double nearest_yaw_threshold);

  TurnSignalInfo overwrite_turn_signal(
    const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
    const TurnSignalInfo & original_signal, const TurnSignalInfo & new_signal,
    const double nearest_dist_threshold, const double nearest_yaw_threshold) const;

  TurnSignalInfo use_prior_turn_signal(
    const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
    const TurnSignalInfo & original_signal, const TurnSignalInfo & new_signal,
    const double nearest_dist_threshold, const double nearest_yaw_threshold);

  void setParameters(
    const double base_link2front, const double intersection_search_distance,
    const double intersection_search_time, const double intersection_angle_threshold_deg)
  {
    base_link2front_ = base_link2front;
    intersection_search_distance_ = intersection_search_distance;
    intersection_search_time_ = intersection_search_time;
    intersection_angle_threshold_deg_ = intersection_angle_threshold_deg;
  }

  std::pair<bool, bool> getIntersectionTurnSignalFlag();
  std::pair<Pose, double> getIntersectionPoseAndDistance();

  std::pair<TurnSignalInfo, bool> getBehaviorTurnSignalInfo(
    const ShiftedPath & path, const ShiftLine & shift_line,
    const lanelet::ConstLanelets & current_lanelets,
    const std::shared_ptr<RouteHandler> route_handler,
    const BehaviorPathPlannerParameters & parameters, const Odometry::ConstSharedPtr self_odometry,
    const double current_shift_length, const bool is_driving_forward,
    const bool egos_lane_is_shifted, const bool override_ego_stopped_check = false,
    const bool is_pull_out = false, const bool is_lane_change = false,
    const bool is_pull_over = false) const;

private:
  std::optional<TurnSignalInfo> getIntersectionTurnSignalInfo(
    const PathWithLaneId & path, const Pose & current_pose, const double current_vel,
    const size_t current_seg_idx, const RouteHandler & route_handler,
    const double nearest_dist_threshold, const double nearest_yaw_threshold);

  geometry_msgs::msg::Pose get_required_end_point(const lanelet::ConstLineString3d & centerline);

  bool use_prior_turn_signal(
    const double dist_to_prior_required_start, const double dist_to_prior_required_end,
    const double dist_to_subsequent_required_start, const double dist_to_subsequent_required_end);

  void set_intersection_info(
    const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
    const TurnSignalInfo & intersection_turn_signal_info, const double nearest_dist_threshold,
    const double nearest_yaw_threshold);
  void initialize_intersection_info();

  inline bool isAvoidShift(
    const double start_shift_length, const double end_shift_length, const double threshold) const
  {
    return std::abs(start_shift_length) < threshold && std::abs(end_shift_length) > threshold;
  };

  inline bool isReturnShift(
    const double start_shift_length, const double end_shift_length, const double threshold) const
  {
    return std::abs(start_shift_length) > threshold && std::abs(end_shift_length) < threshold;
  };

  inline bool isLeftMiddleShift(
    const double start_shift_length, const double end_shift_length, const double threshold) const
  {
    return start_shift_length > threshold && end_shift_length > threshold;
  };

  inline bool isRightMiddleShift(
    const double start_shift_length, const double end_shift_length, const double threshold) const
  {
    return start_shift_length < threshold && end_shift_length < threshold;
  };

  inline bool existShiftSideLane(
    const double start_shift_length, const double end_shift_length, const bool no_left_lanes,
    const bool no_right_lanes, const double threshold) const
  {
    const auto relative_shift_length = end_shift_length - start_shift_length;
    if (isAvoidShift(start_shift_length, end_shift_length, threshold)) {
      // Left avoid. But there is no adjacent lane. No need blinker.
      if (relative_shift_length > 0.0 && no_left_lanes) {
        return false;
      }

      // Right avoid. But there is no adjacent lane. No need blinker.
      if (relative_shift_length < 0.0 && no_right_lanes) {
        return false;
      }
    }

    if (isReturnShift(start_shift_length, end_shift_length, threshold)) {
      // Right return. But there is no adjacent lane. No need blinker.
      if (relative_shift_length > 0.0 && no_right_lanes) {
        return false;
      }

      // Left return. But there is no adjacent lane. No need blinker.
      if (relative_shift_length < 0.0 && no_left_lanes) {
        return false;
      }
    }

    if (isLeftMiddleShift(start_shift_length, end_shift_length, threshold)) {
      // Left avoid. But there is no adjacent lane. No need blinker.

      if (relative_shift_length > 0.0 && no_left_lanes) {
        return false;
      }

      // Left return. But there is no adjacent lane. No need blinker.
      if (relative_shift_length < 0.0 && no_left_lanes) {
        return false;
      }
    }

    if (isRightMiddleShift(start_shift_length, end_shift_length, threshold)) {
      // Right avoid. But there is no adjacent lane. No need blinker.

      if (relative_shift_length < 0.0 && no_right_lanes) {
        return false;
      }

      // Left avoid. But there is no adjacent lane. No need blinker.
      if (relative_shift_length > 0.0 && no_right_lanes) {
        return false;
      }
    }
    return true;
  };

  inline bool straddleRoadBound(
    const ShiftedPath & path, const ShiftLine & shift_line, const lanelet::ConstLanelets & lanes,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info) const
  {
    using autoware::universe_utils::pose2transform;
    using autoware::universe_utils::transformVector;
    using boost::geometry::intersects;

    const auto footprint = vehicle_info.createFootprint();
    const auto start_itr = std::next(path.path.points.begin(), shift_line.start_idx);
    const auto end_itr = std::next(path.path.points.begin(), shift_line.end_idx);

    return std::any_of(start_itr, end_itr, [&footprint, &lanes](const auto & point) {
      const auto transform = pose2transform(point.point.pose);
      const auto shifted_vehicle_footprint = transformVector(footprint, transform);

      auto check_for_vehicle_and_bound_intersection =
        [&shifted_vehicle_footprint](const auto & lane) {
          const auto & left_bound = lane.leftBound2d().basicLineString();
          const auto & right_bound = lane.rightBound2d().basicLineString();
          return intersects(left_bound, shifted_vehicle_footprint) ||
                 intersects(right_bound, shifted_vehicle_footprint);
        };

      return std::any_of(lanes.begin(), lanes.end(), check_for_vehicle_and_bound_intersection);
    });
  };

  inline bool isNearEndOfShift(
    const double start_shift_length, const double end_shift_length, const Point & ego_pos,
    const lanelet::ConstLanelets & original_lanes, const double threshold) const
  {
    using boost::geometry::within;
    using lanelet::utils::to2D;
    using lanelet::utils::conversion::toLaneletPoint;

    if (!isReturnShift(start_shift_length, end_shift_length, threshold)) {
      return false;
    }

    return std::any_of(original_lanes.begin(), original_lanes.end(), [&ego_pos](const auto & lane) {
      return within(to2D(toLaneletPoint(ego_pos)), lane.polygon2d().basicPolygon());
    });
  };

  geometry_msgs::msg::Quaternion calc_orientation(const Point & src_point, const Point & dst_point);

  rclcpp::Logger logger_{
    rclcpp::get_logger("behavior_path_planner").get_child("turn_signal_decider")};

  // data
  double base_link2front_{0.0};
  double intersection_search_distance_{0.0};
  double intersection_search_time_{0.0};
  double intersection_angle_threshold_deg_{0.0};
  std::map<lanelet::Id, geometry_msgs::msg::Pose> desired_start_point_map_;
  mutable bool intersection_turn_signal_ = false;
  mutable bool approaching_intersection_turn_signal_ = false;
  mutable double intersection_distance_ = std::numeric_limits<double>::max();
  mutable Pose intersection_pose_point_ = Pose();
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__TURN_SIGNAL_DECIDER_HPP_
