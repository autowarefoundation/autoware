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
#include <utilization/boost_geometry_helper.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tier4_planning_msgs/msg/stop_reason.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <pcl/point_types.h>
#include <tf2/utils.h>

#include <string>
#include <vector>

namespace behavior_velocity_planner
{
using Point2d = boost::geometry::model::d2::point_xy<double>;
namespace planning_utils
{
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
