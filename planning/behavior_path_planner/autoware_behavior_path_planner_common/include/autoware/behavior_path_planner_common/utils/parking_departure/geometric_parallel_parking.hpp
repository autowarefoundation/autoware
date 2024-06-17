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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__GEOMETRIC_PARALLEL_PARKING_HPP_  // NOLINT
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__GEOMETRIC_PARALLEL_PARKING_HPP_  // NOLINT

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/parameters.hpp"

#include <autoware/lane_departure_checker/lane_departure_checker.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;

struct ParallelParkingParameters
{
  // common
  double center_line_path_interval{0.0};

  // forward parking
  double after_forward_parking_straight_distance{0.0};
  double forward_parking_velocity{0.0};
  double forward_parking_lane_departure_margin{0.0};
  double forward_parking_path_interval{0.0};
  double forward_parking_max_steer_angle{0.0};

  // backward parking
  double after_backward_parking_straight_distance{0.0};
  double backward_parking_velocity{0.0};
  double backward_parking_lane_departure_margin{0.0};
  double backward_parking_path_interval{0.0};
  double backward_parking_max_steer_angle{0.0};

  // pull_out
  double pull_out_velocity{0.0};
  double pull_out_lane_departure_margin{0.0};
  double pull_out_arc_path_interval{0.0};
  double pull_out_max_steer_angle{0.0};
};

class GeometricParallelParking
{
public:
  bool isParking() const;
  bool planPullOver(
    const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
    const lanelet::ConstLanelets & shoulder_lanes, const bool is_forward,
    const bool left_side_parking);
  bool planPullOut(
    const Pose & start_pose, const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
    const lanelet::ConstLanelets & shoulder_lanes, const bool left_side_start,
    const std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker>
      autoware_lane_departure_checker);
  void setParameters(const ParallelParkingParameters & parameters) { parameters_ = parameters; }
  void setPlannerData(const std::shared_ptr<const PlannerData> & planner_data)
  {
    planner_data_ = planner_data;
  }
  void setTurningRadius(
    const BehaviorPathPlannerParameters & common_params, const double max_steer_angle);

  void incrementPathIndex();

  std::vector<PathWithLaneId> getArcPaths() const { return arc_paths_; }
  std::vector<PathWithLaneId> getPaths() const { return paths_; }
  std::vector<std::pair<double, double>> getPairsTerminalVelocityAndAccel() const
  {
    return pairs_terminal_velocity_and_accel_;
  }
  PathWithLaneId getPathByIdx(size_t const idx) const;
  PathWithLaneId getCurrentPath() const;
  PathWithLaneId getFullPath() const;
  PathWithLaneId getArcPath() const;
  Pose getCr() const { return Cr_; }
  Pose getCl() const { return Cl_; }
  Pose getStartPose() const { return start_pose_; }
  Pose getArcEndPose() const { return arc_end_pose_; }

private:
  std::shared_ptr<const PlannerData> planner_data_{nullptr};
  ParallelParkingParameters parameters_{};

  double R_E_min_{0.0};   // base_link
  double R_Bl_min_{0.0};  // front_left

  std::vector<PathWithLaneId> arc_paths_;
  std::vector<PathWithLaneId> paths_;
  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel_;
  size_t current_path_idx_ = 0;

  void clearPaths();
  std::vector<PathWithLaneId> planOneTrial(
    const Pose & start_pose, const Pose & goal_pose, const double R_E_far,
    const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
    const bool is_forward, const bool left_side_parking, const double end_pose_offset,
    const double lane_departure_margin, const double arc_path_interval,
    const std::shared_ptr<autoware::lane_departure_checker::LaneDepartureChecker>
      autoware_lane_departure_checker);
  PathWithLaneId generateArcPath(
    const Pose & center, const double radius, const double start_yaw, double end_yaw,
    const double arc_path_interval, const bool is_left_turn, const bool is_forward);
  PathPointWithLaneId generateArcPathPoint(
    const Pose & center, const double radius, const double yaw, const bool is_left_turn,
    const bool is_forward);
  std::optional<Pose> calcStartPose(
    const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
    const double start_pose_offset, const double R_E_far, const bool is_forward,
    const bool left_side_parking);
  std::vector<PathWithLaneId> generatePullOverPaths(
    const Pose & start_pose, const Pose & goal_pose, const double R_E_far,
    const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
    const bool is_forward, const bool left_side_parking, const double end_pose_offset,
    const double velocity);
  PathWithLaneId generateStraightPath(
    const Pose & start_pose, const lanelet::ConstLanelets & road_lanes);
  void setVelocityToArcPaths(
    std::vector<PathWithLaneId> & arc_paths, const double velocity, const bool set_stop_end);

  // debug
  Pose Cr_{};
  Pose Cl_{};
  Pose start_pose_{};
  Pose arc_end_pose_{};
};
}  // namespace autoware::behavior_path_planner

// clang-format off
#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PARKING_DEPARTURE__GEOMETRIC_PARALLEL_PARKING_HPP_  // NOLINT
// clang-format on
