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

#include <utilization/util.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
namespace planning_utils
{
Polygon2d toFootprintPolygon(const autoware_auto_perception_msgs::msg::PredictedObject & object)
{
  Polygon2d obj_footprint;
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    obj_footprint = toBoostPoly(object.shape.footprint);
  } else {
    // cylinder type is treated as square-polygon
    obj_footprint =
      obj2polygon(object.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions);
  }
  return obj_footprint;
}

bool isAheadOf(const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin)
{
  geometry_msgs::msg::Pose p = planning_utils::transformRelCoordinate2D(target, origin);
  const bool is_target_ahead = (p.position.x > 0.0);
  return is_target_ahead;
}

Polygon2d generatePathPolygon(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t start_idx,
  const size_t end_idx, const double width)
{
  Polygon2d ego_area;  // open polygon
  for (size_t i = start_idx; i <= end_idx; ++i) {
    const double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    const double x = path.points.at(i).point.pose.position.x + width * std::sin(yaw);
    const double y = path.points.at(i).point.pose.position.y - width * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }
  for (size_t i = end_idx; i >= start_idx; --i) {
    const double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    const double x = path.points.at(i).point.pose.position.x - width * std::sin(yaw);
    const double y = path.points.at(i).point.pose.position.y + width * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }
  return ego_area;
}

double normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI) {
    res -= (2.0 * M_PI);
  }
  while (res < -M_PI) {
    res += 2.0 * M_PI;
  }

  return res;
}

geometry_msgs::msg::Quaternion getQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

template <class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::msg::Pose & pose, int & closest, double dist_thr,
  double angle_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  double yaw_pose = tf2::getYaw(pose.orientation);
  closest = -1;

  for (int i = 0; i < static_cast<int>(path.points.size()); ++i) {
    const double dist_squared = calcSquaredDist2d(getPose(path, i), pose);

    /* check distance threshold */
    if (dist_squared > dist_thr * dist_thr) {
      continue;
    }

    /* check angle threshold */
    double yaw_i = tf2::getYaw(getPose(path, i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_i);

    if (std::fabs(yaw_diff) > angle_thr) {
      continue;
    }

    if (dist_squared < dist_squared_min) {
      dist_squared_min = dist_squared;
      closest = i;
    }
  }

  return closest == -1 ? false : true;
}

template bool calcClosestIndex<autoware_auto_planning_msgs::msg::Trajectory>(
  const autoware_auto_planning_msgs::msg::Trajectory & path, const geometry_msgs::msg::Pose & pose,
  int & closest, double dist_thr, double angle_thr);
template bool calcClosestIndex<autoware_auto_planning_msgs::msg::PathWithLaneId>(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & pose, int & closest, double dist_thr, double angle_thr);
template bool calcClosestIndex<autoware_auto_planning_msgs::msg::Path>(
  const autoware_auto_planning_msgs::msg::Path & path, const geometry_msgs::msg::Pose & pose,
  int & closest, double dist_thr, double angle_thr);

template <class T>
bool calcClosestIndex(
  const T & path, const geometry_msgs::msg::Point & point, int & closest, double dist_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  closest = -1;

  for (int i = 0; i < static_cast<int>(path.points.size()); ++i) {
    const double dist_squared = calcSquaredDist2d(getPose(path, i), point);

    /* check distance threshold */
    if (dist_squared > dist_thr * dist_thr) {
      continue;
    }

    if (dist_squared < dist_squared_min) {
      dist_squared_min = dist_squared;
      closest = i;
    }
  }

  return closest == -1 ? false : true;
}
template bool calcClosestIndex<autoware_auto_planning_msgs::msg::Trajectory>(
  const autoware_auto_planning_msgs::msg::Trajectory & path,
  const geometry_msgs::msg::Point & point, int & closest, double dist_thr);
template bool calcClosestIndex<autoware_auto_planning_msgs::msg::PathWithLaneId>(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & point, int & closest, double dist_thr);
template bool calcClosestIndex<autoware_auto_planning_msgs::msg::Path>(
  const autoware_auto_planning_msgs::msg::Path & path, const geometry_msgs::msg::Point & point,
  int & closest, double dist_thr);

geometry_msgs::msg::Pose transformRelCoordinate2D(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin)
{
  // translation
  geometry_msgs::msg::Point trans_p;
  trans_p.x = target.position.x - origin.position.x;
  trans_p.y = target.position.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::msg::Pose res;
  res.position.x = (std::cos(yaw) * trans_p.x) + (std::sin(yaw) * trans_p.y);
  res.position.y = ((-1.0) * std::sin(yaw) * trans_p.x) + (std::cos(yaw) * trans_p.y);
  res.position.z = target.position.z - origin.position.z;
  res.orientation = getQuaternionFromYaw(tf2::getYaw(target.orientation) - yaw);

  return res;
}

geometry_msgs::msg::Pose transformAbsCoordinate2D(
  const geometry_msgs::msg::Pose & relative, const geometry_msgs::msg::Pose & origin)
{
  // rotation
  geometry_msgs::msg::Point rot_p;
  double yaw = tf2::getYaw(origin.orientation);
  rot_p.x = (std::cos(yaw) * relative.position.x) + (-std::sin(yaw) * relative.position.y);
  rot_p.y = (std::sin(yaw) * relative.position.x) + (std::cos(yaw) * relative.position.y);

  // translation
  geometry_msgs::msg::Pose absolute;
  absolute.position.x = rot_p.x + origin.position.x;
  absolute.position.y = rot_p.y + origin.position.y;
  absolute.position.z = relative.position.z + origin.position.z;
  absolute.orientation = getQuaternionFromYaw(tf2::getYaw(relative.orientation) + yaw);

  return absolute;
}

double calcJudgeLineDistWithAccLimit(
  const double velocity, const double max_stop_acceleration, const double delay_response_time)
{
  double judge_line_dist =
    (velocity * velocity) / (2.0 * (-max_stop_acceleration)) + delay_response_time * velocity;
  return judge_line_dist;
}

double calcJudgeLineDistWithJerkLimit(
  const double velocity, const double acceleration, const double max_stop_acceleration,
  const double max_stop_jerk, const double delay_response_time)
{
  if (velocity <= 0.0) {
    return 0.0;
  }

  /* t0: subscribe traffic light state and decide to stop */
  /* t1: braking start (with jerk limitation) */
  /* t2: reach max stop acceleration */
  /* t3: stop */

  const double t1 = delay_response_time;
  const double x1 = velocity * t1;

  const double v2 = velocity + (std::pow(max_stop_acceleration, 2) - std::pow(acceleration, 2)) /
                                 (2.0 * max_stop_jerk);

  if (v2 <= 0.0) {
    const double t2 = -1.0 *
                      (max_stop_acceleration +
                       std::sqrt(acceleration * acceleration - 2.0 * max_stop_jerk * velocity)) /
                      max_stop_jerk;
    const double x2 =
      velocity * t2 + acceleration * std::pow(t2, 2) / 2.0 + max_stop_jerk * std::pow(t2, 3) / 6.0;
    return std::max(0.0, x1 + x2);
  }

  const double t2 = (max_stop_acceleration - acceleration) / max_stop_jerk;
  const double x2 =
    velocity * t2 + acceleration * std::pow(t2, 2) / 2.0 + max_stop_jerk * std::pow(t2, 3) / 6.0;

  const double x3 = -1.0 * std::pow(v2, 2) / (2.0 * max_stop_acceleration);
  return std::max(0.0, x1 + x2 + x3);
}

autoware_planning_msgs::msg::StopReason initializeStopReason(const std::string & stop_reason)
{
  autoware_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = stop_reason;
  return stop_reason_msg;
}

void appendStopReason(
  const autoware_planning_msgs::msg::StopFactor stop_factor,
  autoware_planning_msgs::msg::StopReason * stop_reason)
{
  stop_reason->stop_factors.emplace_back(stop_factor);
}

std::vector<geometry_msgs::msg::Point> toRosPoints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & object)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & obj : object.objects) {
    points.emplace_back(obj.kinematics.initial_pose_with_covariance.pose.position);
  }
  return points;
}

geometry_msgs::msg::Point toRosPoint(const pcl::PointXYZ & pcl_point)
{
  geometry_msgs::msg::Point point;
  point.x = pcl_point.x;
  point.y = pcl_point.y;
  point.z = pcl_point.z;
  return point;
}

geometry_msgs::msg::Point toRosPoint(const Point2d & boost_point, const double z)
{
  geometry_msgs::msg::Point point;
  point.x = boost_point.x();
  point.y = boost_point.y();
  point.z = z;
  return point;
}

LineString2d extendLine(
  const lanelet::ConstPoint3d & lanelet_point1, const lanelet::ConstPoint3d & lanelet_point2,
  const double & length)
{
  const Eigen::Vector2d p1(lanelet_point1.x(), lanelet_point1.y());
  const Eigen::Vector2d p2(lanelet_point2.x(), lanelet_point2.y());
  const Eigen::Vector2d t = (p2 - p1).normalized();
  return {
    {(p1 - length * t).x(), (p1 - length * t).y()}, {(p2 + length * t).x(), (p2 + length * t).y()}};
}
}  // namespace planning_utils
}  // namespace behavior_velocity_planner
