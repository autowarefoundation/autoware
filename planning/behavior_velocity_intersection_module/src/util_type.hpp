// Copyright 2022 Tier IV, Inc.
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

#ifndef UTIL_TYPE_HPP_
#define UTIL_TYPE_HPP_

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::util
{

struct DebugData
{
  std::optional<geometry_msgs::msg::Pose> collision_stop_wall_pose = std::nullopt;
  std::optional<geometry_msgs::msg::Pose> occlusion_stop_wall_pose = std::nullopt;
  std::optional<geometry_msgs::msg::Pose> occlusion_first_stop_wall_pose = std::nullopt;
  std::optional<geometry_msgs::msg::Pose> pass_judge_wall_pose = std::nullopt;
  std::optional<std::vector<lanelet::CompoundPolygon3d>> attention_area = std::nullopt;
  std::optional<geometry_msgs::msg::Polygon> intersection_area = std::nullopt;
  std::optional<lanelet::CompoundPolygon3d> ego_lane = std::nullopt;
  std::optional<std::vector<lanelet::CompoundPolygon3d>> adjacent_area = std::nullopt;
  std::optional<geometry_msgs::msg::Polygon> stuck_vehicle_detect_area = std::nullopt;
  std::optional<geometry_msgs::msg::Polygon> candidate_collision_ego_lane_polygon = std::nullopt;
  std::vector<geometry_msgs::msg::Polygon> candidate_collision_object_polygons;
  autoware_auto_perception_msgs::msg::PredictedObjects conflicting_targets;
  autoware_auto_perception_msgs::msg::PredictedObjects stuck_targets;
  std::optional<geometry_msgs::msg::Point> nearest_occlusion_point = std::nullopt;
  std::optional<geometry_msgs::msg::Point> nearest_occlusion_projection_point = std::nullopt;
  std::vector<geometry_msgs::msg::Polygon> occlusion_polygons;
};

struct InterpolatedPathInfo
{
  autoware_auto_planning_msgs::msg::PathWithLaneId path;
  double ds;
  int lane_id;
  std::set<int> associative_lane_ids;
  std::optional<std::pair<size_t, size_t>> lane_id_interval;
};

struct IntersectionLanelets
{
public:
  void update(const bool tl_arrow_solid_on, const InterpolatedPathInfo & interpolated_path_info);
  const lanelet::ConstLanelets & attention() const
  {
    return tl_arrow_solid_on_ ? attention_non_preceding_ : attention_;
  }
  const lanelet::ConstLanelets & conflicting() const { return conflicting_; }
  const lanelet::ConstLanelets & adjacent() const { return adjacent_; }
  const lanelet::ConstLanelets & occlusion_attention() const
  {
    return tl_arrow_solid_on_ ? attention_non_preceding_ : occlusion_attention_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & attention_area() const
  {
    return tl_arrow_solid_on_ ? attention_non_preceding_area_ : attention_area_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_area() const
  {
    return conflicting_area_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & adjacent_area() const { return adjacent_area_; }
  const std::vector<lanelet::CompoundPolygon3d> & occlusion_attention_area() const
  {
    return occlusion_attention_area_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_conflicting_area() const
  {
    return first_conflicting_area_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area() const
  {
    return first_attention_area_;
  }

  lanelet::ConstLanelets attention_;
  lanelet::ConstLanelets attention_non_preceding_;
  lanelet::ConstLanelets conflicting_;
  lanelet::ConstLanelets adjacent_;
  lanelet::ConstLanelets occlusion_attention_;  // for occlusion detection
  std::vector<lanelet::CompoundPolygon3d> attention_area_;
  std::vector<lanelet::CompoundPolygon3d> attention_non_preceding_area_;
  std::vector<lanelet::CompoundPolygon3d> conflicting_area_;
  std::vector<lanelet::CompoundPolygon3d> adjacent_area_;
  std::vector<lanelet::CompoundPolygon3d> occlusion_attention_area_;
  // the first area intersecting with the path
  // even if lane change/re-routing happened on the intersection, these areas area are supposed to
  // be invariant under the 'associative' lanes.
  bool tl_arrow_solid_on_ = false;
  std::optional<lanelet::CompoundPolygon3d> first_conflicting_area_ = std::nullopt;
  std::optional<lanelet::CompoundPolygon3d> first_attention_area_ = std::nullopt;
};

struct DiscretizedLane
{
  int lane_id;
  // discrete fine lines from left to right
  std::vector<lanelet::ConstLineString2d> divisions;
};

struct IntersectionStopLines
{
  // NOTE: for baselink
  size_t closest_idx;
  size_t stuck_stop_line;
  size_t default_stop_line;
  size_t occlusion_peeking_stop_line;
  size_t pass_judge_line;
};

struct PathLanelets
{
  lanelet::ConstLanelets prev;
  // lanelet::ConstLanelet entry2ego; this is included in `all` if exists
  lanelet::ConstLanelet
    ego_or_entry2exit;  // this is `assigned lane` part of the path(not from
                        // ego) if ego is before the intersection, otherwise from ego to exit
  std::optional<lanelet::ConstLanelet> next =
    std::nullopt;  // this is nullopt is the goal is inside intersection
  lanelet::ConstLanelets all;
  lanelet::ConstLanelets
    conflicting_interval_and_remaining;  // the left/right-most interval of path conflicting with
                                         // conflicting lanelets plus the next lane part of the path
};

}  // namespace behavior_velocity_planner::util

#endif  // UTIL_TYPE_HPP_
