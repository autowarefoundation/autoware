// Copyright 2015-2019 Autoware Foundation
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

#ifndef AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__UTIL_HPP_
#define AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__UTIL_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/stop_reason.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Forward.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
struct DetectionRange
{
  bool use_right = true;
  bool use_left = true;
  double interval;
  double min_longitudinal_distance;
  double max_longitudinal_distance;
  double max_lateral_distance;
  double wheel_tread;
  double right_overhang;
  double left_overhang;
};

struct TrafficSignalStamped
{
  builtin_interfaces::msg::Time stamp;
  autoware_perception_msgs::msg::TrafficLightGroup signal;
};

using Pose = geometry_msgs::msg::Pose;
using Point2d = autoware::universe_utils::Point2d;
using LineString2d = autoware::universe_utils::LineString2d;
using Polygon2d = autoware::universe_utils::Polygon2d;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;
using Polygons2d = std::vector<Polygon2d>;
using autoware_perception_msgs::msg::PredictedObjects;
using tier4_planning_msgs::msg::PathPointWithLaneId;
using tier4_planning_msgs::msg::PathWithLaneId;
using tier4_planning_msgs::msg::StopFactor;
using tier4_planning_msgs::msg::StopReason;

namespace planning_utils
{
size_t calcSegmentIndexFromPointIndex(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & point, const size_t idx);
// create detection area from given range return false if creation failure
bool createDetectionAreaPolygons(
  Polygons2d & da_polys, const PathWithLaneId & path, const geometry_msgs::msg::Pose & target_pose,
  const size_t target_seg_idx, const DetectionRange & da_range, const double obstacle_vel_mps,
  const double min_velocity = 1.0);
Point2d calculateOffsetPoint2d(
  const geometry_msgs::msg::Pose & pose, const double offset_x, const double offset_y);
void extractClosePartition(
  const geometry_msgs::msg::Point position, const BasicPolygons2d & all_partitions,
  BasicPolygons2d & close_partition, const double distance_thresh = 30.0);
void getAllPartitionLanelets(const lanelet::LaneletMapConstPtr ll, BasicPolygons2d & polys);
void setVelocityFromIndex(const size_t begin_idx, const double vel, PathWithLaneId * input);
void insertVelocity(
  PathWithLaneId & path, const PathPointWithLaneId & path_point, const double v,
  size_t & insert_index, const double min_distance = 0.001);
inline int64_t bitShift(int64_t original_id)
{
  return original_id << (sizeof(int32_t) * 8 / 2);
}

bool isAheadOf(const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin);
geometry_msgs::msg::Pose getAheadPose(
  const size_t start_idx, const double ahead_dist,
  const tier4_planning_msgs::msg::PathWithLaneId & path);
Polygon2d generatePathPolygon(
  const PathWithLaneId & path, const size_t start_idx, const size_t end_idx, const double width);
double calcJudgeLineDistWithAccLimit(
  const double velocity, const double max_stop_acceleration, const double delay_response_time);

double calcJudgeLineDistWithJerkLimit(
  const double velocity, const double acceleration, const double max_stop_acceleration,
  const double max_stop_jerk, const double delay_response_time);

double calcDecelerationVelocityFromDistanceToTarget(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target);

double findReachTime(
  const double jerk, const double accel, const double velocity, const double distance,
  const double t_min, const double t_max);

StopReason initializeStopReason(const std::string & stop_reason);

void appendStopReason(const StopFactor stop_factor, StopReason * stop_reason);

std::vector<geometry_msgs::msg::Point> toRosPoints(const PredictedObjects & object);

LineString2d extendLine(
  const lanelet::ConstPoint3d & lanelet_point1, const lanelet::ConstPoint3d & lanelet_point2,
  const double & length);

template <class T>
std::vector<T> concatVector(const std::vector<T> & vec1, const std::vector<T> & vec2)
{
  auto concat_vec = vec1;
  concat_vec.insert(std::end(concat_vec), std::begin(vec2), std::end(vec2));
  return concat_vec;
}

std::optional<int64_t> getNearestLaneId(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose);

std::vector<int64_t> getSortedLaneIdsFromPath(const PathWithLaneId & path);

// return the set of lane_ids in the path after base_lane_id
std::vector<int64_t> getSubsequentLaneIdsSetOnPath(
  const PathWithLaneId & path, int64_t base_lane_id);

template <class T>
std::unordered_map<typename std::shared_ptr<const T>, lanelet::ConstLanelet> getRegElemMapOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose)
{
  std::unordered_map<typename std::shared_ptr<const T>, lanelet::ConstLanelet> reg_elem_map_on_path;

  // Add current lane id
  const auto nearest_lane_id = getNearestLaneId(path, lanelet_map, current_pose);

  std::vector<int64_t> unique_lane_ids;
  if (nearest_lane_id) {
    // Add subsequent lane_ids from nearest lane_id
    unique_lane_ids =
      autoware::behavior_velocity_planner::planning_utils::getSubsequentLaneIdsSetOnPath(
        path, *nearest_lane_id);
  } else {
    // Add all lane_ids in path
    unique_lane_ids =
      autoware::behavior_velocity_planner::planning_utils::getSortedLaneIdsFromPath(path);
  }

  for (const auto lane_id : unique_lane_ids) {
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    for (const auto & reg_elem : ll.regulatoryElementsAs<const T>()) {
      reg_elem_map_on_path.insert(std::make_pair(reg_elem, ll));
    }
  }

  return reg_elem_map_on_path;
}

template <class T>
std::set<int64_t> getRegElemIdSetOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose)
{
  std::set<int64_t> reg_elem_id_set;
  for (const auto & m : getRegElemMapOnPath<const T>(path, lanelet_map, current_pose)) {
    reg_elem_id_set.insert(m.first->id());
  }
  return reg_elem_id_set;
}

template <class T>
std::set<int64_t> getLaneletIdSetOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose)
{
  std::set<int64_t> id_set;
  for (const auto & m : getRegElemMapOnPath<const T>(path, lanelet_map, current_pose)) {
    id_set.insert(m.second.id());
  }
  return id_set;
}

std::optional<geometry_msgs::msg::Pose> insertDecelPoint(
  const geometry_msgs::msg::Point & stop_point, PathWithLaneId & output,
  const float target_velocity);

std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose);

std::set<int64_t> getLaneIdSetOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose);

bool isOverLine(
  const tier4_planning_msgs::msg::PathWithLaneId & path, const geometry_msgs::msg::Pose & self_pose,
  const geometry_msgs::msg::Pose & line_pose, const double offset = 0.0);

std::optional<geometry_msgs::msg::Pose> insertStopPoint(
  const geometry_msgs::msg::Point & stop_point, PathWithLaneId & output);
std::optional<geometry_msgs::msg::Pose> insertStopPoint(
  const geometry_msgs::msg::Point & stop_point, const size_t stop_seg_idx, PathWithLaneId & output);

/*
  @brief return 'associative' lanes in the intersection. 'associative' means that a lane shares same
  or lane-changeable parent lanes with `lane` and has same turn_direction value.
 */
std::set<lanelet::Id> getAssociativeIntersectionLanelets(
  lanelet::ConstLanelet lane, const lanelet::LaneletMapPtr lanelet_map,
  const lanelet::routing::RoutingGraphPtr routing_graph);

lanelet::ConstLanelets getConstLaneletsFromIds(
  lanelet::LaneletMapConstPtr map, const std::set<lanelet::Id> & ids);

}  // namespace planning_utils
}  // namespace autoware::behavior_velocity_planner

#endif  // AUTOWARE__BEHAVIOR_VELOCITY_PLANNER_COMMON__UTILIZATION__UTIL_HPP_
