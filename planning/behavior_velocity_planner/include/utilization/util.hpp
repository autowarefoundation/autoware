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

#ifndef UTILIZATION__UTIL_HPP_
#define UTILIZATION__UTIL_HPP_

#include <lanelet2_extension/utility/query.hpp>
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_point.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tier4_planning_msgs/msg/stop_reason.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <pcl/point_types.h>
#include <tf2/utils.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
struct SearchRangeIndex
{
  size_t min_idx;
  size_t max_idx;
};
struct DetectionRange
{
  bool use_right = true;
  bool use_left = true;
  double interval;
  double min_longitudinal_distance;
  double max_longitudinal_distance;
  double min_lateral_distance;
  double max_lateral_distance;
};
struct PointWithSearchRangeIndex
{
  geometry_msgs::msg::Point point;
  SearchRangeIndex index;
};

using geometry_msgs::msg::Pose;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;
using Polygons2d = std::vector<Polygon2d>;
using Point2d = boost::geometry::model::d2::point_xy<double>;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using motion_utils::calcLongitudinalOffsetToSegment;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using motion_utils::validateNonEmpty;
using tier4_autoware_utils::calcAzimuthAngle;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::calcSquaredDistance2d;
using tier4_autoware_utils::createQuaternionFromYaw;
using tier4_autoware_utils::getPoint;
using tier4_planning_msgs::msg::StopFactor;
using tier4_planning_msgs::msg::StopReason;

namespace planning_utils
{
// create detection area from given range return false if creation failure
bool createDetectionAreaPolygons(
  Polygons2d & slices, const PathWithLaneId & path, const geometry_msgs::msg::Pose & current_pose,
  const DetectionRange & da_range, const double obstacle_vel_mps, const double min_velocity = 1.0);
PathPoint getLerpPathPointWithLaneId(const PathPoint p0, const PathPoint p1, const double ratio);
Point2d calculateOffsetPoint2d(const Pose & pose, const double offset_x, const double offset_y);
void extractClosePartition(
  const geometry_msgs::msg::Point position, const BasicPolygons2d & all_partitions,
  BasicPolygons2d & close_partition, const double distance_thresh = 30.0);
void getAllPartitionLanelets(const lanelet::LaneletMapConstPtr ll, BasicPolygons2d & polys);
void setVelocityFromIndex(const size_t begin_idx, const double vel, PathWithLaneId * input);
void insertVelocity(
  PathWithLaneId & path, const PathPointWithLaneId & path_point, const double v,
  size_t & insert_index, const double min_distance = 0.001);
inline int64_t bitShift(int64_t original_id) { return original_id << (sizeof(int32_t) * 8 / 2); }

geometry_msgs::msg::Pose transformRelCoordinate2D(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin);
geometry_msgs::msg::Pose transformAbsCoordinate2D(
  const geometry_msgs::msg::Pose & relative, const geometry_msgs::msg::Pose & origin);
SearchRangeIndex getPathIndexRangeIncludeLaneId(const PathWithLaneId & path, const int64_t lane_id);
/**
 * @brief find nearest segment index with search range
 */
template <class T>
size_t findNearestSegmentIndex(const T & points, const PointWithSearchRangeIndex & point_with_index)
{
  const auto & index = point_with_index.index;
  const auto point = point_with_index.point;

  validateNonEmpty(points);

  double min_dist = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;

  for (size_t i = index.min_idx; i <= index.max_idx; ++i) {
    const auto dist = calcSquaredDistance2d(points.at(i), point);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == points.size() - 1) {
    return points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(points, nearest_idx, point);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}
/**
 * @brief find nearest segment index within distance threshold
 */
template <class T>
PointWithSearchRangeIndex findFirstNearSearchRangeIndex(
  const T & points, const geometry_msgs::msg::Point & point, const double distance_thresh = 9.0)
{
  validateNonEmpty(points);

  bool min_idx_found = false;
  bool max_idx_found = false;
  PointWithSearchRangeIndex point_with_range = {point, {static_cast<size_t>(0), points.size() - 1}};
  for (size_t i = 0; i < points.size(); i++) {
    const auto & p = points.at(i).point.pose.position;
    const double dist = std::hypot(point.x - p.x, point.y - p.y);
    if (dist < distance_thresh) {
      if (!min_idx_found) {
        point_with_range.index.min_idx = i;
        min_idx_found = true;
      }
      if (!max_idx_found) point_with_range.index.max_idx = i;
    } else if (min_idx_found) {
      // found close index and farther than distance_thresh, stop update max index
      max_idx_found = true;
    }
  }
  return point_with_range;
}
/**
 * @brief calcSignedArcLength from point to point with search range
 */
template <class T>
double calcSignedArcLengthWithSearchIndex(
  const T & points, const PointWithSearchRangeIndex & src_point_with_range,
  const PointWithSearchRangeIndex & dst_point_with_range)
{
  validateNonEmpty(points);
  const size_t src_idx = planning_utils::findNearestSegmentIndex(points, src_point_with_range);
  const size_t dst_idx = planning_utils::findNearestSegmentIndex(points, dst_point_with_range);
  const double signed_length = calcSignedArcLength(points, src_idx, dst_idx);
  const double signed_length_src_offset =
    calcLongitudinalOffsetToSegment(points, src_idx, src_point_with_range.point);
  const double signed_length_dst_offset =
    calcLongitudinalOffsetToSegment(points, dst_idx, dst_point_with_range.point);
  return signed_length - signed_length_src_offset + signed_length_dst_offset;
}
Polygon2d toFootprintPolygon(const PredictedObject & object);
bool isAheadOf(const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin);
geometry_msgs::msg::Pose getAheadPose(
  const size_t start_idx, const double ahead_dist,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path);
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

geometry_msgs::msg::Point toRosPoint(const pcl::PointXYZ & pcl_point);
geometry_msgs::msg::Point toRosPoint(const Point2d & boost_point, const double z);

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

boost::optional<int64_t> getNearestLaneId(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose, boost::optional<size_t> & nearest_segment_idx);

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
  boost::optional<size_t> nearest_segment_idx;
  const auto nearest_lane_id =
    getNearestLaneId(path, lanelet_map, current_pose, nearest_segment_idx);

  std::vector<int64_t> unique_lane_ids;
  if (nearest_lane_id) {
    // Add subsequent lane_ids from nearest lane_id
    unique_lane_ids = behavior_velocity_planner::planning_utils::getSubsequentLaneIdsSetOnPath(
      path, *nearest_lane_id);
  } else {
    // Add all lane_ids in path
    unique_lane_ids = behavior_velocity_planner::planning_utils::getSortedLaneIdsFromPath(path);
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

boost::optional<geometry_msgs::msg::Pose> insertDecelPoint(
  const geometry_msgs::msg::Point & stop_point, PathWithLaneId & output,
  const float target_velocity);

std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose);

std::set<int64_t> getLaneIdSetOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose);

bool isOverLine(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose,
  const double offset = 0.0);

boost::optional<geometry_msgs::msg::Pose> insertStopPoint(
  const geometry_msgs::msg::Point & stop_point, PathWithLaneId & output);
}  // namespace planning_utils
}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__UTIL_HPP_
