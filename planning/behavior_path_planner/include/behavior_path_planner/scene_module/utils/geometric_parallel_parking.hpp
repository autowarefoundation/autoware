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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__GEOMETRIC_PARALLEL_PARKING_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__GEOMETRIC_PARALLEL_PARKING_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/parameters.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;

struct ParallelParkingParameters
{
  double th_arrived_distance;
  double th_stopped_velocity;
  double after_forward_parking_straight_distance;
  double after_backward_parking_straight_distance;
  double forward_parking_velocity;
  double backward_parking_velocity;
  double departing_velocity;
  double backward_parking_lane_departure_margin;
  double forward_parking_lane_departure_margin;
  double departing_lane_departure_margin;
  double arc_path_interval;
  double maximum_deceleration;
  double max_steer_angle;
};

class GeometricParallelParking
{
public:
  bool isParking() const;
  bool planPullOver(
    const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
    const lanelet::ConstLanelets & shoulder_lanes, const bool is_forward);
  bool planPullOut(
    const Pose & start_pose, const Pose & goal_pose, const lanelet::ConstLanelets & road_lanes,
    const lanelet::ConstLanelets & shoulder_lanes);
  void setData(
    const std::shared_ptr<const PlannerData> & planner_data,
    const ParallelParkingParameters & parameters);
  void incrementPathIndex();

  std::vector<PathWithLaneId> getArcPaths() const { return arc_paths_; }
  std::vector<PathWithLaneId> getPaths() const { return paths_; }
  PathWithLaneId getPathByIdx(size_t const idx) const;
  PathWithLaneId getCurrentPath() const;
  PathWithLaneId getFullPath() const;
  PathWithLaneId getArcPath() const;
  Pose getCr() const { return Cr_; }
  Pose getCl() const { return Cl_; }
  Pose getStartPose() const { return start_pose_; }
  Pose getArcEndPose() const { return arc_end_pose_; }

private:
  std::shared_ptr<const PlannerData> planner_data_;
  ParallelParkingParameters parameters_;

  double R_E_min_;   // base_link
  double R_Bl_min_;  // front_left

  std::vector<PathWithLaneId> arc_paths_;
  std::vector<PathWithLaneId> paths_;
  size_t current_path_idx_ = 0;

  void clearPaths();
  bool isEnoughDistanceToStart(const Pose & start_pose) const;
  std::vector<PathWithLaneId> planOneTrial(
    const Pose & start_pose, const Pose & goal_pose, const double R_E_r,
    const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
    bool is_forward, const double end_pose_offset, const double lane_departure_margin);
  PathWithLaneId generateArcPath(
    const Pose & center, const double radius, const double start_yaw, double end_yaw,
    const bool is_left_turn, const bool is_forward);
  PathPointWithLaneId generateArcPathPoint(
    const Pose & center, const double radius, const double yaw, const bool is_left_turn,
    const bool is_forward);
  boost::optional<Pose> calcStartPose(
    const Pose & goal_pose, const double start_pose_offset, const double R_E_r,
    const bool is_forward);
  std::vector<PathWithLaneId> generatePullOverPaths(
    const Pose & start_pose, const Pose & goal_pose, const double R_E_r,
    const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
    const bool is_forward, const double end_pose_offset, const double velocity);
  PathWithLaneId generateStraightPath(const Pose & start_pose);
  void setVelocityToArcPaths(std::vector<PathWithLaneId> & arc_paths, const double velocity);

  // debug
  Pose Cr_;
  Pose Cl_;
  Pose start_pose_;
  Pose arc_end_pose_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__GEOMETRIC_PARALLEL_PARKING_HPP_
