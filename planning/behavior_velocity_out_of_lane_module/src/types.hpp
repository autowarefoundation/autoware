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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::out_of_lane
{
/// @brief parameters for the "out of lane" module
struct PlannerParam
{
  std::string mode;                  // mode used to consider a conflict with an object
  bool skip_if_already_overlapping;  // if true, do not run the module when ego already overlaps
                                     // another lane

  double time_threshold;        // [s](mode="threshold") objects time threshold
  double intervals_ego_buffer;  // [s](mode="intervals") buffer to extend the ego time range
  double intervals_obj_buffer;  // [s](mode="intervals") buffer to extend the objects time range
  double ttc_threshold;  // [s](mode="ttc") threshold on time to collision between ego and an object
  double ego_min_velocity;  // [m/s] minimum velocity of ego used to calculate its ttc or time range

  bool objects_use_predicted_paths;  // whether to use the objects' predicted paths
  double objects_min_vel;            // [m/s] objects lower than this velocity will be ignored
  double objects_min_confidence;     // minimum confidence to consider a predicted path
  double objects_dist_buffer;  // [m] distance buffer used to determine if a collision will occur in
                               // the other lane

  double overlap_extra_length;  // [m] extra length to add around an overlap range
  double overlap_min_dist;      // [m] min distance inside another lane to consider an overlap
  // action to insert in the path if an object causes a conflict at an overlap
  bool skip_if_over_max_decel;  // if true, skip the action if it causes more than the max decel
  double dist_buffer;
  double slow_velocity;
  double slow_dist_threshold;
  double stop_dist_threshold;
  double precision;              // [m] precision when inserting a stop pose in the path
  double min_decision_duration;  // [s] minimum duration needed a decision can be canceled
  // ego dimensions used to create its polygon footprint
  double front_offset;        // [m]  front offset (from vehicle info)
  double rear_offset;         // [m]  rear offset (from vehicle info)
  double right_offset;        // [m]  right offset (from vehicle info)
  double left_offset;         // [m]  left offset (from vehicle info)
  double extra_front_offset;  // [m] extra front distance
  double extra_rear_offset;   // [m] extra rear distance
  double extra_right_offset;  // [m] extra right distance
  double extra_left_offset;   // [m] extra left distance
};

struct EnterExitTimes
{
  double enter_time{};
  double exit_time{};
};
struct RangeTimes
{
  EnterExitTimes ego{};
  EnterExitTimes object{};
};

/// @brief action taken by the "out of lane" module
struct Slowdown
{
  size_t target_path_idx{};             // we want to slowdown before this path index
  double velocity{};                    // desired slow down velocity
  lanelet::ConstLanelet lane_to_avoid;  // we want to slowdown before entering this lane
};
/// @brief slowdown to insert in a path
struct SlowdownToInsert
{
  Slowdown slowdown;
  autoware_auto_planning_msgs::msg::PathWithLaneId::_points_type::value_type point;
};

/// @brief bound of an overlap range (either the first, or last bound)
struct RangeBound
{
  size_t index;
  lanelet::BasicPoint2d point;
  double arc_length;
  double inside_distance;
};

/// @brief representation of an overlap between the ego footprint and some other lane
struct Overlap
{
  double inside_distance = 0.0;  ///!< distance inside the overlap
  double min_arc_length = std::numeric_limits<double>::infinity();
  double max_arc_length = 0.0;
  lanelet::BasicPoint2d min_overlap_point{};  ///!< point with min arc length
  lanelet::BasicPoint2d max_overlap_point{};  ///!< point with max arc length
};

/// @brief range along the path where ego overlaps another lane
struct OverlapRange
{
  lanelet::ConstLanelet lane;
  size_t entering_path_idx{};
  size_t exiting_path_idx{};
  lanelet::BasicPoint2d entering_point;  // pose of the overlapping point closest along the lane
  lanelet::BasicPoint2d exiting_point;   // pose of the overlapping point furthest along the lane
  double inside_distance{};              // [m] how much ego footprint enters the lane
  mutable struct
  {
    std::vector<Overlap> overlaps;
    std::optional<Slowdown> decision;
    RangeTimes times;
    std::optional<autoware_auto_perception_msgs::msg::PredictedObject> object{};
  } debug;
};
using OverlapRanges = std::vector<OverlapRange>;
/// @brief representation of a lane and its current overlap range
struct OtherLane
{
  bool range_is_open = false;
  RangeBound first_range_bound{};
  RangeBound last_range_bound{};
  lanelet::ConstLanelet lanelet;
  lanelet::BasicPolygon2d polygon;

  explicit OtherLane(lanelet::ConstLanelet ll) : lanelet(std::move(ll))
  {
    polygon = lanelet.polygon2d().basicPolygon();
  }

  [[nodiscard]] OverlapRange close_range()
  {
    OverlapRange range;
    range.lane = lanelet;
    range.entering_path_idx = first_range_bound.index;
    range.entering_point = first_range_bound.point;
    range.exiting_path_idx = last_range_bound.index;
    range.exiting_point = last_range_bound.point;
    range.inside_distance =
      std::max(first_range_bound.inside_distance, last_range_bound.inside_distance);
    range_is_open = false;
    last_range_bound = {};
    return range;
  }
};

/// @brief data related to the ego vehicle
struct EgoData
{
  autoware_auto_planning_msgs::msg::PathWithLaneId path{};
  size_t first_path_idx{};
  double velocity{};   // [m/s]
  double max_decel{};  // [m/s²]
  geometry_msgs::msg::Pose pose;
};

/// @brief data needed to make decisions
struct DecisionInputs
{
  OverlapRanges ranges{};
  EgoData ego_data;
  autoware_auto_perception_msgs::msg::PredictedObjects objects{};
  std::shared_ptr<route_handler::RouteHandler> route_handler{};
  lanelet::ConstLanelets lanelets{};
};

/// @brief debug data
struct DebugData
{
  std::vector<lanelet::BasicPolygon2d> footprints;
  std::vector<SlowdownToInsert> slowdowns;
  geometry_msgs::msg::Pose ego_pose;
  OverlapRanges ranges;
  lanelet::BasicPolygon2d current_footprint;
  lanelet::ConstLanelets current_overlapped_lanelets;
  lanelet::ConstLanelets path_lanelets;
  lanelet::ConstLanelets ignored_lanelets;
  lanelet::ConstLanelets other_lanelets;
  autoware_auto_planning_msgs::msg::PathWithLaneId path;
  size_t first_path_idx;

  size_t prev_footprints = 0;
  size_t prev_slowdowns = 0;
  size_t prev_ranges = 0;
  size_t prev_current_overlapped_lanelets = 0;
  size_t prev_ignored_lanelets = 0;
  size_t prev_path_lanelets = 0;
  size_t prev_other_lanelets = 0;
  void reset_data()
  {
    prev_footprints = footprints.size();
    footprints.clear();
    prev_slowdowns = slowdowns.size();
    slowdowns.clear();
    prev_ranges = ranges.size();
    ranges.clear();
    prev_current_overlapped_lanelets = current_overlapped_lanelets.size();
    current_overlapped_lanelets.clear();
    prev_ignored_lanelets = ignored_lanelets.size();
    ignored_lanelets.clear();
    prev_path_lanelets = path_lanelets.size();
    path_lanelets.clear();
    prev_other_lanelets = other_lanelets.size();
    other_lanelets.clear();
  }
};

}  // namespace behavior_velocity_planner::out_of_lane

#endif  // TYPES_HPP_
