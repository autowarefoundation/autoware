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
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/trajectory/trajectory.hpp>
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
#include <string>
#include <vector>

namespace tier4_autoware_utils
{
template <>
inline geometry_msgs::msg::Point getPoint(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose.position;
}
}  // namespace tier4_autoware_utils

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

using Point2d = boost::geometry::model::d2::point_xy<double>;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;
using Polygons2d = std::vector<Polygon2d>;
namespace planning_utils
{
using geometry_msgs::msg::Pose;
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Point & p) { return p; }
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::Pose & p) { return p.position; }
inline geometry_msgs::msg::Point getPoint(const geometry_msgs::msg::PoseStamped & p)
{
  return p.pose.position;
}
inline geometry_msgs::msg::Point getPoint(const autoware_auto_planning_msgs::msg::PathPoint & p)
{
  return p.pose.position;
}
inline geometry_msgs::msg::Point getPoint(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose.position;
}
inline geometry_msgs::msg::Point getPoint(
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & p)
{
  return p.pose.position;
}
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_planning_msgs::msg::Path & path, int idx)
{
  return path.points.at(idx).pose;
}
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, int idx)
{
  return path.points.at(idx).point.pose;
}
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, int idx)
{
  return traj.points.at(idx).pose;
}

// create detection area from given range return false if creation failure
bool createDetectionAreaPolygons(
  Polygons2d & slices, const PathWithLaneId & path, const DetectionRange da_range,
  const double obstacle_vel_mps);
PathPoint getLerpPathPointWithLaneId(const PathPoint p0, const PathPoint p1, const double ratio);
Point2d calculateLateralOffsetPoint2d(const Pose & p, const double offset);
void extractClosePartition(
  const geometry_msgs::msg::Point position, const BasicPolygons2d & all_partitions,
  BasicPolygons2d & close_partition, const double distance_thresh = 30.0);
void getAllPartitionLanelets(const lanelet::LaneletMapConstPtr ll, BasicPolygons2d & polys);
void setVelocityFrom(const size_t idx, const double vel, PathWithLaneId * input);
void insertVelocity(
  PathWithLaneId & path, const PathPointWithLaneId & path_point, const double v,
  size_t & insert_index, const double min_distance = 0.001);
inline int64_t bitShift(int64_t original_id) { return original_id << (sizeof(int32_t) * 8 / 2); }

inline double square(const double & a) { return a * a; }
double normalizeEulerAngle(double euler);
geometry_msgs::msg::Quaternion getQuaternionFromYaw(double yaw);

template <class T1, class T2>
double calcSquaredDist2d(const T1 & a, const T2 & b)
{
  return square(getPoint(a).x - getPoint(b).x) + square(getPoint(a).y - getPoint(b).y);
}

template <class T1, class T2>
double calcDist2d(const T1 & a, const T2 & b)
{
  return std::sqrt(calcSquaredDist2d<T1, T2>(a, b));
}

template <class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::msg::Pose & pose, int & closest, double dist_thr = 3.0,
  double angle_thr = M_PI_4);

template <class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::msg::Point & point, int & closest, double dist_thr = 3.0);

geometry_msgs::msg::Pose transformRelCoordinate2D(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin);
geometry_msgs::msg::Pose transformAbsCoordinate2D(
  const geometry_msgs::msg::Pose & relative, const geometry_msgs::msg::Pose & origin);
SearchRangeIndex getPathIndexRangeIncludeLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int64_t lane_id);
/**
 * @brief find nearest segment index with search range
 */
template <class T>
size_t findNearestSegmentIndex(const T & points, const PointWithSearchRangeIndex & point_with_index)
{
  const auto & index = point_with_index.index;
  const auto point = point_with_index.point;

  tier4_autoware_utils::validateNonEmpty(points);

  double min_dist = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;

  for (size_t i = index.min_idx; i <= index.max_idx; ++i) {
    const auto dist = tier4_autoware_utils::calcSquaredDistance2d(points.at(i), point);
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

  const double signed_length =
    tier4_autoware_utils::calcLongitudinalOffsetToSegment(points, nearest_idx, point);

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
  tier4_autoware_utils::validateNonEmpty(points);

  bool min_idx_found = false;
  PointWithSearchRangeIndex point_with_range = {point, {static_cast<size_t>(0), points.size() - 1}};
  for (size_t i = 0; i < points.size(); i++) {
    const auto & p = points.at(i).point.pose.position;
    const double dist = std::hypot(point.x - p.x, point.y - p.y);
    if (dist < distance_thresh) {
      if (!min_idx_found) {
        point_with_range.index.min_idx = i;
        min_idx_found = true;
      }
      point_with_range.index.max_idx = i;
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
  tier4_autoware_utils::validateNonEmpty(points);
  const size_t src_idx = planning_utils::findNearestSegmentIndex(points, src_point_with_range);
  const size_t dst_idx = planning_utils::findNearestSegmentIndex(points, dst_point_with_range);
  const double signed_length = tier4_autoware_utils::calcSignedArcLength(points, src_idx, dst_idx);
  const double signed_length_src_offset = tier4_autoware_utils::calcLongitudinalOffsetToSegment(
    points, src_idx, src_point_with_range.point);
  const double signed_length_dst_offset = tier4_autoware_utils::calcLongitudinalOffsetToSegment(
    points, dst_idx, dst_point_with_range.point);
  return signed_length - signed_length_src_offset + signed_length_dst_offset;
}
Polygon2d toFootprintPolygon(const autoware_auto_perception_msgs::msg::PredictedObject & object);
bool isAheadOf(const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin);
Polygon2d generatePathPolygon(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t start_idx,
  const size_t end_idx, const double width);

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

tier4_planning_msgs::msg::StopReason initializeStopReason(const std::string & stop_reason);

void appendStopReason(
  const tier4_planning_msgs::msg::StopFactor stop_factor,
  tier4_planning_msgs::msg::StopReason * stop_reason);

std::vector<geometry_msgs::msg::Point> toRosPoints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & object);

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
}  // namespace planning_utils
}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__UTIL_HPP_
