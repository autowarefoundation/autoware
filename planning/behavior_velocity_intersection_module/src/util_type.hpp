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

#include <tier4_autoware_utils/geometry/geometry.hpp>

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
  std::optional<geometry_msgs::msg::Pose> collision_stop_wall_pose{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> occlusion_stop_wall_pose{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> occlusion_first_stop_wall_pose{std::nullopt};
  std::optional<geometry_msgs::msg::Pose> pass_judge_wall_pose{std::nullopt};
  std::optional<std::vector<lanelet::CompoundPolygon3d>> attention_area{std::nullopt};
  std::optional<std::vector<lanelet::CompoundPolygon3d>> occlusion_attention_area{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> ego_lane{std::nullopt};
  std::optional<std::vector<lanelet::CompoundPolygon3d>> adjacent_area{std::nullopt};
  std::optional<geometry_msgs::msg::Polygon> stuck_vehicle_detect_area{std::nullopt};
  std::optional<std::vector<lanelet::CompoundPolygon3d>> yield_stuck_detect_area{std::nullopt};
  std::optional<geometry_msgs::msg::Polygon> candidate_collision_ego_lane_polygon{std::nullopt};
  std::vector<geometry_msgs::msg::Polygon> candidate_collision_object_polygons;
  autoware_auto_perception_msgs::msg::PredictedObjects conflicting_targets;
  autoware_auto_perception_msgs::msg::PredictedObjects amber_ignore_targets;
  autoware_auto_perception_msgs::msg::PredictedObjects red_overshoot_ignore_targets;
  autoware_auto_perception_msgs::msg::PredictedObjects stuck_targets;
  autoware_auto_perception_msgs::msg::PredictedObjects yield_stuck_targets;
  std::vector<geometry_msgs::msg::Polygon> occlusion_polygons;
  std::optional<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>>
    nearest_occlusion_projection{std::nullopt};
  autoware_auto_perception_msgs::msg::PredictedObjects blocking_attention_objects;
  std::optional<geometry_msgs::msg::Pose> absence_traffic_light_creep_wall{std::nullopt};
  std::optional<double> static_occlusion_with_traffic_light_timeout{std::nullopt};
};

struct InterpolatedPathInfo
{
  autoware_auto_planning_msgs::msg::PathWithLaneId path;
  double ds{0.0};
  lanelet::Id lane_id{0};
  std::set<lanelet::Id> associative_lane_ids{};
  std::optional<std::pair<size_t, size_t>> lane_id_interval{std::nullopt};
};

struct IntersectionLanelets
{
public:
  void update(
    const bool is_prioritized, const InterpolatedPathInfo & interpolated_path_info,
    const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length);
  const lanelet::ConstLanelets & attention() const
  {
    return is_prioritized_ ? attention_non_preceding_ : attention_;
  }
  const std::vector<std::optional<lanelet::ConstLineString3d>> & attention_stoplines() const
  {
    return is_prioritized_ ? attention_non_preceding_stoplines_ : attention_stoplines_;
  }
  const lanelet::ConstLanelets & conflicting() const { return conflicting_; }
  const lanelet::ConstLanelets & adjacent() const { return adjacent_; }
  const lanelet::ConstLanelets & occlusion_attention() const
  {
    return is_prioritized_ ? attention_non_preceding_ : occlusion_attention_;
  }
  const std::vector<lanelet::CompoundPolygon3d> & attention_area() const
  {
    return is_prioritized_ ? attention_non_preceding_area_ : attention_area_;
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
  const std::optional<lanelet::ConstLanelet> & first_conflicting_lane() const
  {
    return first_conflicting_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_conflicting_area() const
  {
    return first_conflicting_area_;
  }
  const std::optional<lanelet::ConstLanelet> & first_attention_lane() const
  {
    return first_attention_lane_;
  }
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area() const
  {
    return first_attention_area_;
  }

  lanelet::ConstLanelets attention_;  // topologically merged lanelets
  std::vector<std::optional<lanelet::ConstLineString3d>>
    attention_stoplines_;  // the stop lines for each attention_ lanelets
  lanelet::ConstLanelets attention_non_preceding_;
  std::vector<std::optional<lanelet::ConstLineString3d>>
    attention_non_preceding_stoplines_;  // the stop lines for each attention_non_preceding_
                                         // lanelets
  lanelet::ConstLanelets conflicting_;
  lanelet::ConstLanelets adjacent_;
  lanelet::ConstLanelets occlusion_attention_;    // topologically merged lanelets
  std::vector<double> occlusion_attention_size_;  // the area() of each occlusion attention lanelets
  std::vector<lanelet::CompoundPolygon3d> attention_area_;  // topologically merged lanelets
  std::vector<lanelet::CompoundPolygon3d> attention_non_preceding_area_;
  std::vector<lanelet::CompoundPolygon3d> conflicting_area_;
  std::vector<lanelet::CompoundPolygon3d> adjacent_area_;
  std::vector<lanelet::CompoundPolygon3d>
    occlusion_attention_area_;  // topologically merged lanelets
  // the first area intersecting with the path
  // even if lane change/re-routing happened on the intersection, these areas area are supposed to
  // be invariant under the 'associative' lanes.
  std::optional<lanelet::ConstLanelet> first_conflicting_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> first_conflicting_area_{std::nullopt};
  std::optional<lanelet::ConstLanelet> first_attention_lane_{std::nullopt};
  std::optional<lanelet::CompoundPolygon3d> first_attention_area_{std::nullopt};
  bool is_prioritized_ = false;
};

struct IntersectionStopLines
{
  // NOTE: for baselink
  size_t closest_idx{0};
  // NOTE: null if path does not conflict with first_conflicting_area
  std::optional<size_t> stuck_stopline{std::nullopt};
  // NOTE: null if path is over map stopline OR its value is calculated negative
  std::optional<size_t> default_stopline{std::nullopt};
  // NOTE: null if the index is calculated negative
  std::optional<size_t> first_attention_stopline{std::nullopt};
  // NOTE: null if footprints do not change from outside to inside of detection area
  std::optional<size_t> occlusion_peeking_stopline{std::nullopt};
  // if the value is calculated negative, its value is 0
  size_t pass_judge_line{0};
  size_t occlusion_wo_tl_pass_judge_line{0};
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
                                         // conflicting lanelets plus the next lane part of the
                                         // path
};

struct TargetObject
{
  autoware_auto_perception_msgs::msg::PredictedObject object;
  std::optional<lanelet::ConstLanelet> attention_lanelet{std::nullopt};
  std::optional<lanelet::ConstLineString3d> stopline{std::nullopt};
  std::optional<double> dist_to_stopline{std::nullopt};
  void calc_dist_to_stopline();
};

struct TargetObjects
{
  std_msgs::msg::Header header;
  std::vector<TargetObject> attention_objects;
  std::vector<TargetObject> parked_attention_objects;
  std::vector<TargetObject> intersection_area_objects;
  std::vector<TargetObject> all_attention_objects;  // TODO(Mamoru Sobue): avoid copy
};

enum class TrafficPrioritizedLevel {
  // The target lane's traffic signal is red or the ego's traffic signal has an arrow.
  FULLY_PRIORITIZED = 0,
  // The target lane's traffic signal is amber
  PARTIALLY_PRIORITIZED,
  // The target lane's traffic signal is green
  NOT_PRIORITIZED
};
}  // namespace behavior_velocity_planner::util

#endif  // UTIL_TYPE_HPP_
