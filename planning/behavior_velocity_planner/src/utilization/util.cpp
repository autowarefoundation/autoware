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
Point2d calculateLateralOffsetPoint2d(const Pose & pose, const double offset)
{
  using tier4_autoware_utils::calcOffsetPose;
  return to_bg2d(calcOffsetPose(pose, 0.0, offset, 0.0));
}

PathPoint getLerpPathPointWithLaneId(const PathPoint p0, const PathPoint p1, const double ratio)
{
  auto lerp = [](const double a, const double b, const double t) { return a + t * (b - a); };
  PathPoint p;
  Pose pose;
  const auto pp0 = p0.pose.position;
  const auto pp1 = p1.pose.position;
  pose.position.x = lerp(pp0.x, pp1.x, ratio);
  pose.position.y = lerp(pp0.y, pp1.y, ratio);
  pose.position.z = lerp(pp0.z, pp1.z, ratio);
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(pp0, pp1);
  pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);
  p.pose = pose;
  const double v = lerp(p0.longitudinal_velocity_mps, p1.longitudinal_velocity_mps, ratio);
  p.longitudinal_velocity_mps = v;
  return p;
}

bool createDetectionAreaPolygons(
  Polygons2d & da_polys, const PathWithLaneId & path, const DetectionRange da_range,
  const double obstacle_vel_mps)
{
  /**
   * @brief relationships for interpolated polygon
   *
   * +(min_length,max_distance)-+ - +---+(max_length,max_distance) = outer_polygons
   * |                                  |
   * +--------------------------+ - +---+(max_length,min_distance) = inner_polygons
   */
  const double min_len = da_range.min_longitudinal_distance;
  const double max_len = da_range.max_longitudinal_distance;
  const double min_dst = da_range.min_lateral_distance;
  const double max_dst = da_range.max_lateral_distance;
  const double interval = da_range.interval;
  const double min_velocity = 0.5;  // min velocity that autoware can cruise stably
  //! max index is the last index of path point
  const size_t max_index = static_cast<size_t>(path.points.size() - 1);
  double dist_sum = 0;
  size_t first_idx = 0;  // first path point found in front of ego front bumper + offset
  const auto & pp = path.points;
  //! avoid bug with same point polygon
  const double eps = 1e-3;
  if (path.points.size() < 2) return false;  // case of path point is only one
  auto p0 = path.points.front().point;
  // handle the first point
  {
    double current_dist = 0.0;
    for (size_t i = 1; i <= max_index - 1; i++) {
      dist_sum += tier4_autoware_utils::calcDistance2d(pp.at(i - 1), pp.at(i));
      if (dist_sum > min_len) {
        first_idx = i;
        break;
      }
      current_dist = dist_sum;
    }
    if (first_idx == 0) return false;  // case of all path point is behind ego front bumper + offset
    const double ds = dist_sum - current_dist;
    if (std::abs(ds) < eps) {
      p0 = pp.at(first_idx - 1).point;
    } else {
      const double ratio = (min_len - current_dist) / ds;
      p0 = getLerpPathPointWithLaneId(pp.at(first_idx - 1).point, pp.at(first_idx).point, ratio);
    }
  }
  double ttc = 0.0;
  dist_sum = min_len;
  double length = 0;
  // initial point of detection area polygon
  LineString2d left_inner_bound = {calculateLateralOffsetPoint2d(p0.pose, min_dst)};
  LineString2d left_outer_bound = {calculateLateralOffsetPoint2d(p0.pose, min_dst + eps)};
  LineString2d right_inner_bound = {calculateLateralOffsetPoint2d(p0.pose, -min_dst)};
  LineString2d right_outer_bound = {calculateLateralOffsetPoint2d(p0.pose, -min_dst - eps)};
  for (size_t s = first_idx; s <= max_index; s++) {
    const auto p1 = path.points.at(s).point;
    const double ds = tier4_autoware_utils::calcDistance2d(p0, p1);
    dist_sum += ds;
    length += ds;
    // calculate the distance that obstacles can move until ego reach the trajectory point
    const double v_average = 0.5 * (p0.longitudinal_velocity_mps + p1.longitudinal_velocity_mps);
    const double v = std::max(v_average, min_velocity);
    const double dt = ds / v;
    ttc += dt;
    // for offset calculation
    const double max_lateral_distance = std::min(max_dst, min_dst + ttc * obstacle_vel_mps + eps);
    // left bound
    if (da_range.use_left) {
      left_inner_bound.emplace_back(calculateLateralOffsetPoint2d(p1.pose, min_dst));
      left_outer_bound.emplace_back(calculateLateralOffsetPoint2d(p1.pose, max_lateral_distance));
    }
    // right bound
    if (da_range.use_right) {
      right_inner_bound.emplace_back(calculateLateralOffsetPoint2d(p1.pose, -min_dst));
      right_outer_bound.emplace_back(calculateLateralOffsetPoint2d(p1.pose, -max_lateral_distance));
    }
    // replace previous point with next point
    p0 = p1;
    // separate detection area polygon with fixed interval or at the end of detection max length
    if (length > interval || max_len < dist_sum || s == max_index) {
      if (left_inner_bound.size() > 1)
        da_polys.emplace_back(lines2polygon(left_inner_bound, left_outer_bound));
      if (right_inner_bound.size() > 1)
        da_polys.emplace_back(lines2polygon(right_outer_bound, right_inner_bound));
      left_inner_bound = {left_inner_bound.back()};
      left_outer_bound = {left_outer_bound.back()};
      right_inner_bound = {right_inner_bound.back()};
      right_outer_bound = {right_outer_bound.back()};
      length = 0;
      if (max_len < dist_sum || s == max_index) return true;
    }
  }
  return true;
}

void extractClosePartition(
  const geometry_msgs::msg::Point position, const BasicPolygons2d & all_partitions,
  BasicPolygons2d & close_partition, const double distance_thresh)
{
  close_partition.clear();
  for (const auto & p : all_partitions) {
    if (boost::geometry::distance(Point2d(position.x, position.y), p) < distance_thresh) {
      close_partition.emplace_back(p);
    }
  }
  return;
}

void getAllPartitionLanelets(const lanelet::LaneletMapConstPtr ll, BasicPolygons2d & polys)
{
  const lanelet::ConstLineStrings3d partitions = lanelet::utils::query::getAllPartitions(ll);
  for (const auto & partition : partitions) {
    lanelet::BasicLineString2d line;
    for (const auto & p : partition) {
      line.emplace_back(lanelet::BasicPoint2d{p.x(), p.y()});
    }
    // corect line to calculate distance accuratry
    boost::geometry::correct(line);
    polys.emplace_back(lanelet::BasicPolygon2d(line));
  }
}

SearchRangeIndex getPathIndexRangeIncludeLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int64_t lane_id)
{
  /**
   * @brief find path index range include given lane_id
   *        |<-min_idx       |<-max_idx
   *  ------|oooooooooooooooo|-------
   */
  SearchRangeIndex search_range = {0, path.points.size() - 1};
  bool found_first_idx = false;
  for (size_t i = 0; i < path.points.size(); i++) {
    const auto & p = path.points.at(i);
    for (const auto & id : p.lane_ids) {
      if (id == lane_id) {
        if (!found_first_idx) {
          search_range.min_idx = i;
          found_first_idx = true;
        }
        search_range.max_idx = i;
      }
    }
  }
  return search_range;
}

void setVelocityFromIndex(const size_t begin_idx, const double vel, PathWithLaneId * input)
{
  for (size_t i = begin_idx; i < input->points.size(); ++i) {
    input->points.at(i).point.longitudinal_velocity_mps =
      std::min(static_cast<float>(vel), input->points.at(i).point.longitudinal_velocity_mps);
  }
  return;
}

void insertVelocity(
  PathWithLaneId & path, const PathPointWithLaneId & path_point, const double v,
  size_t & insert_index, const double min_distance)
{
  bool already_has_path_point = false;
  // consider front/back point is near to insert point or not
  int min_idx = std::max(0, static_cast<int>(insert_index - 1));
  int max_idx =
    std::min(static_cast<int>(insert_index + 1), static_cast<int>(path.points.size() - 1));
  for (int i = min_idx; i <= max_idx; i++) {
    if (
      tier4_autoware_utils::calcDistance2d(path.points.at(static_cast<size_t>(i)), path_point) <
      min_distance) {
      path.points.at(i).point.longitudinal_velocity_mps = 0;
      already_has_path_point = true;
      insert_index = static_cast<size_t>(i);
      // set velocity from is going to insert min velocity later
      break;
    }
  }
  //! insert velocity point only if there is no close point on path
  if (!already_has_path_point) {
    path.points.insert(path.points.begin() + insert_index, path_point);
  }
  // set zero velocity from insert index
  setVelocityFromIndex(insert_index, v, &path);
}

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

double findReachTime(
  const double jerk, const double accel, const double velocity, const double distance,
  const double t_min, const double t_max)
{
  const double j = jerk;
  const double a = accel;
  const double v = velocity;
  const double d = distance;
  const double min = t_min;
  const double max = t_max;
  auto f = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  if (f(min, j, a, v, d) > 0 || f(max, j, a, v, d) < 0) {
    std::logic_error("[behavior_velocity](findReachTime): search range is invalid");
  }
  const double eps = 1e-5;
  const int warn_iter = 100;
  double lower = min;
  double upper = max;
  double t;
  int iter = 0;
  for (int i = 0;; i++) {
    t = 0.5 * (lower + upper);
    const double fx = f(t, j, a, v, d);
    // std::cout<<"fx: "<<fx<<" up: "<<upper<<" lo: "<<lower<<" t: "<<t<<std::endl;
    if (std::abs(fx) < eps) {
      break;
    } else if (fx > 0.0) {
      upper = t;
    } else {
      lower = t;
    }
    iter++;
    if (iter > warn_iter)
      std::cerr << "[behavior_velocity](findReachTime): current iter is over warning" << std::endl;
  }
  // std::cout<<"iter: "<<iter<<std::endl;
  return t;
}

double calcDecelerationVelocityFromDistanceToTarget(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target)
{
  if (max_slowdown_jerk > 0 || max_slowdown_accel > 0) {
    std::logic_error("max_slowdown_jerk and max_slowdown_accel should be negative");
  }
  // case0: distance to target is behind ego
  if (distance_to_target <= 0) return current_velocity;
  auto ft = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  auto vt = [](const double t, const double j, const double a, const double v) {
    return j * t * t / 2.0 + a * t + v;
  };
  const double j_max = max_slowdown_jerk;
  const double a0 = current_accel;
  const double a_max = max_slowdown_accel;
  const double v0 = current_velocity;
  const double l = distance_to_target;
  const double t_const_jerk = (a_max - a0) / j_max;
  const double d_const_jerk_stop = ft(t_const_jerk, j_max, a0, v0, 0.0);
  const double d_const_acc_stop = l - d_const_jerk_stop;

  if (d_const_acc_stop < 0) {
    // case0: distance to target is within constant jerk deceleration
    // use binary search instead of solving cubic equation
    const double t_jerk = findReachTime(j_max, a0, v0, l, 0, t_const_jerk);
    const double velocity = vt(t_jerk, j_max, a0, v0);
    return velocity;
  } else {
    const double v1 = vt(t_const_jerk, j_max, a0, v0);
    const double discriminant_of_stop = 2.0 * a_max * d_const_acc_stop + v1 * v1;
    // case3: distance to target is farther than distance to stop
    if (discriminant_of_stop <= 0) {
      return 0.0;
    }
    // case2: distance to target is within constant accel deceleration
    // solve d = 0.5*a^2+v*t by t
    const double t_acc = (-v1 + std::sqrt(discriminant_of_stop)) / a_max;
    return vt(t_acc, 0.0, a_max, v1);
  }
  return current_velocity;
}

tier4_planning_msgs::msg::StopReason initializeStopReason(const std::string & stop_reason)
{
  tier4_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = stop_reason;
  return stop_reason_msg;
}

void appendStopReason(
  const tier4_planning_msgs::msg::StopFactor stop_factor,
  tier4_planning_msgs::msg::StopReason * stop_reason)
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
