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

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <autoware_auto_planning_msgs/msg/path_point.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace
{
size_t calcPointIndexFromSegmentIndex(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & point, const size_t seg_idx)
{
  const size_t prev_point_idx = seg_idx;
  const size_t next_point_idx = seg_idx + 1;

  const double prev_dist = tier4_autoware_utils::calcDistance2d(point, points.at(prev_point_idx));
  const double next_dist = tier4_autoware_utils::calcDistance2d(point, points.at(next_point_idx));

  if (prev_dist < next_dist) {
    return prev_point_idx;
  }
  return next_point_idx;
}

using autoware_auto_planning_msgs::msg::PathPoint;

PathPoint getLerpPathPointWithLaneId(const PathPoint p0, const PathPoint p1, const double ratio)
{
  auto lerp = [](const double a, const double b, const double t) { return a + t * (b - a); };
  PathPoint p;
  p.pose = tier4_autoware_utils::calcInterpolatedPose(p0, p1, ratio);
  const double v = lerp(p0.longitudinal_velocity_mps, p1.longitudinal_velocity_mps, ratio);
  p.longitudinal_velocity_mps = v;
  return p;
}

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
  res.orientation =
    tier4_autoware_utils::createQuaternionFromYaw(tf2::getYaw(target.orientation) - yaw);

  return res;
}

}  // namespace

namespace behavior_velocity_planner
{
namespace planning_utils
{
using autoware_auto_planning_msgs::msg::PathPoint;
using motion_utils::calcLongitudinalOffsetToSegment;
using motion_utils::calcSignedArcLength;
using motion_utils::validateNonEmpty;
using tier4_autoware_utils::calcAzimuthAngle;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::calcSquaredDistance2d;
using tier4_autoware_utils::createQuaternionFromYaw;
using tier4_autoware_utils::getPoint;

size_t calcSegmentIndexFromPointIndex(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & point, const size_t idx)
{
  if (idx == 0) {
    return 0;
  }
  if (idx == points.size() - 1) {
    return idx - 1;
  }
  if (points.size() < 3) {
    return 0;
  }

  const double offset_to_seg = motion_utils::calcLongitudinalOffsetToSegment(points, idx, point);
  if (0 < offset_to_seg) {
    return idx;
  }
  return idx - 1;
}

Point2d calculateOffsetPoint2d(
  const geometry_msgs::msg::Pose & pose, const double offset_x, const double offset_y)
{
  return to_bg2d(calcOffsetPose(pose, offset_x, offset_y, 0.0));
}

bool createDetectionAreaPolygons(
  Polygons2d & da_polys, const PathWithLaneId & path, const geometry_msgs::msg::Pose & target_pose,
  const size_t target_seg_idx, const DetectionRange & da_range, const double obstacle_vel_mps,
  const double min_velocity)
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
  const double max_dst = da_range.max_lateral_distance;
  const double interval = da_range.interval;
  const double offset_left = (da_range.wheel_tread / 2.0) + da_range.left_overhang;
  const double offset_right = (da_range.wheel_tread / 2.0) + da_range.right_overhang;

  //! max index is the last index of path point
  const size_t max_index = static_cast<size_t>(path.points.size() - 1);
  //! avoid bug with same point polygon
  const double eps = 1e-3;
  auto nearest_idx =
    calcPointIndexFromSegmentIndex(path.points, target_pose.position, target_seg_idx);
  if (max_index == nearest_idx) return false;  // case of path point is not enough size
  auto p0 = path.points.at(nearest_idx).point;
  auto first_idx = nearest_idx + 1;

  // use ego point as start point if same point as ego is not in the path
  const auto dist_to_nearest =
    std::fabs(calcSignedArcLength(path.points, target_pose.position, target_seg_idx, nearest_idx));
  if (dist_to_nearest > eps) {
    // interpolate ego point
    const auto & pp = path.points;
    const double ds = calcDistance2d(pp.at(target_seg_idx), pp.at(target_seg_idx + 1));
    const double dist_to_target_seg =
      calcSignedArcLength(path.points, target_seg_idx, target_pose.position, target_seg_idx);
    const double ratio = dist_to_target_seg / ds;
    p0 = getLerpPathPointWithLaneId(
      pp.at(target_seg_idx).point, pp.at(target_seg_idx + 1).point, ratio);

    // new first index should be ahead of p0
    first_idx = target_seg_idx + 1;
  }

  double ttc = 0.0;
  double dist_sum = 0.0;
  double length = 0;
  // initial point of detection area polygon
  LineString2d left_inner_bound = {calculateOffsetPoint2d(p0.pose, min_len, offset_left)};
  LineString2d left_outer_bound = {calculateOffsetPoint2d(p0.pose, min_len, offset_left + eps)};
  LineString2d right_inner_bound = {calculateOffsetPoint2d(p0.pose, min_len, -offset_right)};
  LineString2d right_outer_bound = {calculateOffsetPoint2d(p0.pose, min_len, -offset_right - eps)};
  for (size_t s = first_idx; s <= max_index; s++) {
    const auto p1 = path.points.at(s).point;
    const double ds = calcDistance2d(p0, p1);
    dist_sum += ds;
    length += ds;
    // calculate the distance that obstacles can move until ego reach the trajectory point
    const double v_average = 0.5 * (p0.longitudinal_velocity_mps + p1.longitudinal_velocity_mps);
    const double v = std::max(v_average, min_velocity);
    const double dt = ds / v;
    ttc += dt;

    // for offset calculation
    const double max_lateral_distance_right =
      std::min(max_dst, offset_right + ttc * obstacle_vel_mps + eps);
    const double max_lateral_distance_left =
      std::min(max_dst, offset_left + ttc * obstacle_vel_mps + eps);

    // left bound
    if (da_range.use_left) {
      left_inner_bound.emplace_back(calculateOffsetPoint2d(p1.pose, min_len, offset_left));
      left_outer_bound.emplace_back(
        calculateOffsetPoint2d(p1.pose, min_len, max_lateral_distance_left));
    }
    // right bound
    if (da_range.use_right) {
      right_inner_bound.emplace_back(calculateOffsetPoint2d(p1.pose, min_len, -offset_right));
      right_outer_bound.emplace_back(
        calculateOffsetPoint2d(p1.pose, min_len, -max_lateral_distance_right));
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
    // correct line to calculate distance in accurate
    boost::geometry::correct(line);
    polys.emplace_back(lanelet::BasicPolygon2d(line));
  }
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
    if (calcDistance2d(path.points.at(static_cast<size_t>(i)), path_point) < min_distance) {
      path.points.at(i).point.longitudinal_velocity_mps = v;
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

bool isAheadOf(const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & origin)
{
  geometry_msgs::msg::Pose p = transformRelCoordinate2D(target, origin);
  const bool is_target_ahead = (p.position.x > 0.0);
  return is_target_ahead;
}

geometry_msgs::msg::Pose getAheadPose(
  const size_t start_idx, const double ahead_dist,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  if (path.points.size() == 0) {
    return geometry_msgs::msg::Pose{};
  }

  double curr_dist = 0.0;
  double prev_dist = 0.0;
  for (size_t i = start_idx; i < path.points.size() - 1; ++i) {
    const geometry_msgs::msg::Pose p0 = path.points.at(i).point.pose;
    const geometry_msgs::msg::Pose p1 = path.points.at(i + 1).point.pose;
    curr_dist += tier4_autoware_utils::calcDistance2d(p0, p1);
    if (curr_dist > ahead_dist) {
      const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
      const double w_p0 = (curr_dist - ahead_dist) / dl;
      const double w_p1 = (ahead_dist - prev_dist) / dl;
      geometry_msgs::msg::Pose p;
      p.position.x = w_p0 * p0.position.x + w_p1 * p1.position.x;
      p.position.y = w_p0 * p0.position.y + w_p1 * p1.position.y;
      p.position.z = w_p0 * p0.position.z + w_p1 * p1.position.z;
      tf2::Quaternion q0_tf, q1_tf;
      tf2::fromMsg(p0.orientation, q0_tf);
      tf2::fromMsg(p1.orientation, q1_tf);
      p.orientation = tf2::toMsg(q0_tf.slerp(q1_tf, w_p1));
      return p;
    }
    prev_dist = curr_dist;
  }
  return path.points.back().point.pose;
}

Polygon2d generatePathPolygon(
  const PathWithLaneId & path, const size_t start_idx, const size_t end_idx, const double width)
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

  bg::correct(ego_area);
  return ego_area;
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

StopReason initializeStopReason(const std::string & stop_reason)
{
  StopReason stop_reason_msg;
  stop_reason_msg.reason = stop_reason;
  return stop_reason_msg;
}

void appendStopReason(const StopFactor stop_factor, StopReason * stop_reason)
{
  stop_reason->stop_factors.emplace_back(stop_factor);
}

std::vector<geometry_msgs::msg::Point> toRosPoints(const PredictedObjects & object)
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto & obj : object.objects) {
    points.emplace_back(obj.kinematics.initial_pose_with_covariance.pose.position);
  }
  return points;
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

std::optional<int64_t> getNearestLaneId(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose)
{
  lanelet::ConstLanelets lanes;
  const auto lane_ids = getSortedLaneIdsFromPath(path);
  for (const auto & lane_id : lane_ids) {
    lanes.push_back(lanelet_map->laneletLayer.get(lane_id));
  }

  lanelet::Lanelet closest_lane;
  if (lanelet::utils::query::getClosestLanelet(lanes, current_pose, &closest_lane)) {
    return closest_lane.id();
  }
  return std::nullopt;
}

std::vector<lanelet::ConstLanelet> getLaneletsOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto nearest_lane_id = getNearestLaneId(path, lanelet_map, current_pose);

  std::vector<int64_t> unique_lane_ids;
  if (nearest_lane_id) {
    // Add subsequent lane_ids from nearest lane_id
    unique_lane_ids = behavior_velocity_planner::planning_utils::getSubsequentLaneIdsSetOnPath(
      path, *nearest_lane_id);
  } else {
    // Add all lane_ids in path
    unique_lane_ids = behavior_velocity_planner::planning_utils::getSortedLaneIdsFromPath(path);
  }

  std::vector<lanelet::ConstLanelet> lanelets;
  for (const auto lane_id : unique_lane_ids) {
    lanelets.push_back(lanelet_map->laneletLayer.get(lane_id));
  }

  return lanelets;
}

std::set<int64_t> getLaneIdSetOnPath(
  const PathWithLaneId & path, const lanelet::LaneletMapPtr lanelet_map,
  const geometry_msgs::msg::Pose & current_pose)
{
  std::set<int64_t> lane_id_set;
  for (const auto & lane : getLaneletsOnPath(path, lanelet_map, current_pose)) {
    lane_id_set.insert(lane.id());
  }

  return lane_id_set;
}

std::vector<int64_t> getSortedLaneIdsFromPath(const PathWithLaneId & path)
{
  std::vector<int64_t> sorted_lane_ids;
  for (const auto & path_points : path.points) {
    for (const auto lane_id : path_points.lane_ids)
      if (
        std::find(sorted_lane_ids.begin(), sorted_lane_ids.end(), lane_id) ==
        sorted_lane_ids.end()) {
        sorted_lane_ids.emplace_back(lane_id);
      }
  }
  return sorted_lane_ids;
}

std::vector<int64_t> getSubsequentLaneIdsSetOnPath(
  const PathWithLaneId & path, int64_t base_lane_id)
{
  const auto all_lane_ids = getSortedLaneIdsFromPath(path);
  const auto base_index = std::find(all_lane_ids.begin(), all_lane_ids.end(), base_lane_id);

  // cannot find base_index in all_lane_ids
  if (base_index == all_lane_ids.end()) {
    return std::vector<int64_t>();
  }

  std::vector<int64_t> subsequent_lane_ids;

  std::copy(base_index, all_lane_ids.end(), std::back_inserter(subsequent_lane_ids));
  return subsequent_lane_ids;
}

// TODO(murooka) remove calcSignedArcLength using findNearestSegmentIndex
bool isOverLine(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose,
  const double offset)
{
  return motion_utils::calcSignedArcLength(path.points, self_pose.position, line_pose.position) +
           offset <
         0.0;
}

std::optional<geometry_msgs::msg::Pose> insertDecelPoint(
  const geometry_msgs::msg::Point & stop_point, PathWithLaneId & output,
  const float target_velocity)
{
  // TODO(tanaka): consider proper overlap threshold for inserting decel point
  const double overlap_threshold = 5e-2;
  // TODO(murooka): remove this function for u-turn and crossing-path
  const size_t base_idx = motion_utils::findNearestSegmentIndex(output.points, stop_point);
  const auto insert_idx =
    motion_utils::insertTargetPoint(base_idx, stop_point, output.points, overlap_threshold);

  if (!insert_idx) {
    return {};
  }

  for (size_t i = insert_idx.value(); i < output.points.size(); ++i) {
    const auto & original_velocity = output.points.at(i).point.longitudinal_velocity_mps;
    output.points.at(i).point.longitudinal_velocity_mps =
      std::min(original_velocity, target_velocity);
  }
  return tier4_autoware_utils::getPose(output.points.at(insert_idx.value()));
}

// TODO(murooka): remove this function for u-turn and crossing-path
std::optional<geometry_msgs::msg::Pose> insertStopPoint(
  const geometry_msgs::msg::Point & stop_point, PathWithLaneId & output)
{
  const size_t base_idx = motion_utils::findNearestSegmentIndex(output.points, stop_point);
  const auto insert_idx = motion_utils::insertStopPoint(base_idx, stop_point, output.points);

  if (!insert_idx) {
    return {};
  }

  return tier4_autoware_utils::getPose(output.points.at(insert_idx.value()));
}

std::optional<geometry_msgs::msg::Pose> insertStopPoint(
  const geometry_msgs::msg::Point & stop_point, const size_t stop_seg_idx, PathWithLaneId & output)
{
  const auto insert_idx = motion_utils::insertStopPoint(stop_seg_idx, stop_point, output.points);

  if (!insert_idx) {
    return {};
  }

  return tier4_autoware_utils::getPose(output.points.at(insert_idx.value()));
}

std::set<lanelet::Id> getAssociativeIntersectionLanelets(
  lanelet::ConstLanelet lane, const lanelet::LaneletMapPtr lanelet_map,
  const lanelet::routing::RoutingGraphPtr routing_graph)
{
  const std::string turn_direction = lane.attributeOr("turn_direction", "else");
  if (turn_direction.compare("else") == 0) {
    return {};
  }

  const auto parents = routing_graph->previous(lane);
  std::set<lanelet::Id> parent_neighbors;
  for (const auto & parent : parents) {
    const auto neighbors = routing_graph->besides(parent);
    for (const auto & neighbor : neighbors) parent_neighbors.insert(neighbor.id());
  }
  std::set<lanelet::Id> associative_intersection_lanelets;
  associative_intersection_lanelets.insert(lane.id());
  for (const auto & parent_neighbor_id : parent_neighbors) {
    const auto parent_neighbor = lanelet_map->laneletLayer.get(parent_neighbor_id);
    const auto followings = routing_graph->following(parent_neighbor);
    for (const auto & following : followings) {
      if (following.attributeOr("turn_direction", "else") == turn_direction) {
        associative_intersection_lanelets.insert(following.id());
      }
    }
  }
  return associative_intersection_lanelets;
}

}  // namespace planning_utils
}  // namespace behavior_velocity_planner
