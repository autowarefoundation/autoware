// Copyright 2020 Tier IV, Inc.
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

#include "obstacle_stop_planner/planner_utils.hpp"

#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <diagnostic_msgs/msg/key_value.hpp>

#include <boost/format.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace motion_planning
{

using motion_utils::findFirstNearestIndexWithSoftConstraints;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getRPY;

bool validCheckDecelPlan(
  const double v_end, const double a_end, const double v_target, const double a_target,
  const double v_margin, const double a_margin)
{
  const double v_min = v_target - std::abs(v_margin);
  const double v_max = v_target + std::abs(v_margin);
  const double a_min = a_target - std::abs(a_margin);
  const double a_max = a_target + std::abs(a_margin);

  if (v_end < v_min || v_max < v_end) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("validCheckDecelPlan"),
      "[validCheckDecelPlan] valid check error! v_target = " << v_target << ", v_end = " << v_end);
    return false;
  }
  if (a_end < a_min || a_max < a_end) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("validCheckDecelPlan"),
      "[validCheckDecelPlan] valid check error! a_target = " << a_target << ", a_end = " << a_end);
    return false;
  }

  return true;
}

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE1)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @param (t_min) duration of constant deceleration [s]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType1(
  const double v0, const double vt, const double a0, const double am, const double ja,
  const double jd, const double t_min)
{
  constexpr double epsilon = 1e-3;

  const double j1 = am < a0 ? jd : ja;
  const double t1 = epsilon < (am - a0) / j1 ? (am - a0) / j1 : 0.0;
  const double a1 = a0 + j1 * t1;
  const double v1 = v0 + a0 * t1 + 0.5 * j1 * t1 * t1;
  const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * j1 * t1 * t1 * t1;

  const double t2 = epsilon < t_min ? t_min : 0.0;
  const double a2 = a1;
  const double v2 = v1 + a1 * t2;
  const double x2 = x1 + v1 * t2 + 0.5 * a1 * t2 * t2;

  const double t3 = epsilon < (0.0 - am) / ja ? (0.0 - am) / ja : 0.0;
  const double a3 = a2 + ja * t3;
  const double v3 = v2 + a2 * t3 + 0.5 * ja * t3 * t3;
  const double x3 = x2 + v2 * t3 + 0.5 * a2 * t3 * t3 + (1.0 / 6.0) * ja * t3 * t3 * t3;

  const double a_target = 0.0;
  const double v_margin = 0.3;  // [m/s]
  const double a_margin = 0.1;  // [m/s^2]

  if (!validCheckDecelPlan(v3, a3, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x3;
}

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE2)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (am) minimum deceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @param (jd) minimum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType2(
  const double v0, const double vt, const double a0, const double ja, const double jd)
{
  constexpr double epsilon = 1e-3;

  const double a1_square = (vt - v0 - 0.5 * (0.0 - a0) / jd * a0) * (2.0 * ja * jd / (ja - jd));
  const double a1 = -std::sqrt(a1_square);

  const double t1 = epsilon < (a1 - a0) / jd ? (a1 - a0) / jd : 0.0;
  const double v1 = v0 + a0 * t1 + 0.5 * jd * t1 * t1;
  const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * jd * t1 * t1 * t1;

  const double t2 = epsilon < (0.0 - a1) / ja ? (0.0 - a1) / ja : 0.0;
  const double a2 = a1 + ja * t2;
  const double v2 = v1 + a1 * t2 + 0.5 * ja * t2 * t2;
  const double x2 = x1 + v1 * t2 + 0.5 * a1 * t2 * t2 + (1.0 / 6.0) * ja * t2 * t2 * t2;

  const double a_target = 0.0;
  const double v_margin = 0.3;
  const double a_margin = 0.1;

  if (!validCheckDecelPlan(v2, a2, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x2;
}

/**
 * @brief calculate distance until velocity is reached target velocity (TYPE3)
 * @param (v0) current velocity [m/s]
 * @param (vt) target velocity [m/s]
 * @param (a0) current acceleration [m/ss]
 * @param (ja) maximum jerk [m/sss]
 * @return moving distance until velocity is reached vt [m]
 * @detail TODO(Satoshi Ota)
 */
boost::optional<double> calcDecelDistPlanType3(
  const double v0, const double vt, const double a0, const double ja)
{
  constexpr double epsilon = 1e-3;

  const double t_acc = (0.0 - a0) / ja;

  const double t1 = epsilon < t_acc ? t_acc : 0.0;
  const double a1 = a0 + ja * t1;
  const double v1 = v0 + a0 * t1 + 0.5 * ja * t1 * t1;
  const double x1 = v0 * t1 + 0.5 * a0 * t1 * t1 + (1.0 / 6.0) * ja * t1 * t1 * t1;

  const double a_target = 0.0;
  const double v_margin = 0.3;
  const double a_margin = 0.1;

  if (!validCheckDecelPlan(v1, a1, vt, a_target, v_margin, a_margin)) {
    return {};
  }

  return x1;
}

boost::optional<double> calcDecelDistWithJerkAndAccConstraints(
  const double current_vel, const double target_vel, const double current_acc, const double acc_min,
  const double jerk_acc, const double jerk_dec)
{
  constexpr double epsilon = 1e-3;
  const double t_dec =
    acc_min < current_acc ? (acc_min - current_acc) / jerk_dec : (acc_min - current_acc) / jerk_acc;
  const double t_acc = (0.0 - acc_min) / jerk_acc;
  const double t_min = (target_vel - current_vel - current_acc * t_dec -
                        0.5 * jerk_dec * t_dec * t_dec - 0.5 * acc_min * t_acc) /
                       acc_min;

  // check if it is possible to decelerate to the target velocity
  // by simply bringing the current acceleration to zero.
  const auto is_decel_needed =
    0.5 * (0.0 - current_acc) / jerk_acc * current_acc > target_vel - current_vel;

  if (t_min > epsilon) {
    return calcDecelDistPlanType1(
      current_vel, target_vel, current_acc, acc_min, jerk_acc, jerk_dec, t_min);
  } else if (is_decel_needed || current_acc > epsilon) {
    return calcDecelDistPlanType2(current_vel, target_vel, current_acc, jerk_acc, jerk_dec);
  } else {
    return calcDecelDistPlanType3(current_vel, target_vel, current_acc, jerk_acc);
  }

  return {};
}

boost::optional<std::pair<double, double>> calcFeasibleMarginAndVelocity(
  const SlowDownParam & slow_down_param, const double dist_baselink_to_obstacle,
  const double current_vel, const double current_acc)
{
  const auto & p = slow_down_param;
  const auto & logger = rclcpp::get_logger("calcFeasibleMarginAndVelocity");
  constexpr double epsilon = 1e-4;

  if (current_vel < p.slow_down_velocity + epsilon) {
    return std::make_pair(p.longitudinal_forward_margin, p.slow_down_velocity);
  }

  for (double planning_jerk = p.jerk_start; planning_jerk > p.slow_down_min_jerk - epsilon;
       planning_jerk += p.jerk_span) {
    const double jerk_dec = planning_jerk;
    const double jerk_acc = std::abs(planning_jerk);

    const auto planning_dec =
      planning_jerk > p.normal_min_jerk ? p.limit_min_acc : p.normal_min_acc;
    const auto stop_dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, p.slow_down_velocity, current_acc, planning_dec, jerk_acc, jerk_dec);

    if (!stop_dist) {
      continue;
    }

    if (stop_dist.get() + p.longitudinal_forward_margin < dist_baselink_to_obstacle) {
      RCLCPP_DEBUG(
        logger, "[found plan] dist:%-6.2f jerk:%-6.2f margin:%-6.2f v0:%-6.2f vt:%-6.2f",
        stop_dist.get(), planning_jerk, p.longitudinal_forward_margin, p.slow_down_velocity,
        current_vel);
      return std::make_pair(p.longitudinal_forward_margin, p.slow_down_velocity);
    }
  }

  {
    const double jerk_dec = p.slow_down_min_jerk;
    const double jerk_acc = std::abs(p.slow_down_min_jerk);

    const auto planning_dec =
      p.slow_down_min_jerk > p.normal_min_jerk ? p.limit_min_acc : p.normal_min_acc;
    const auto stop_dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, p.slow_down_velocity, current_acc, planning_dec, jerk_acc, jerk_dec);

    if (!stop_dist) {
      return {};
    }

    if (stop_dist.get() + p.min_longitudinal_forward_margin < dist_baselink_to_obstacle) {
      const auto planning_margin = dist_baselink_to_obstacle - stop_dist.get();
      RCLCPP_DEBUG(
        logger, "[relax margin] dist:%-6.2f jerk:%-6.2f margin:%-6.2f v0:%-6.2f vt%-6.2f",
        stop_dist.get(), p.slow_down_min_jerk, planning_margin, p.slow_down_velocity, current_vel);
      return std::make_pair(planning_margin, p.slow_down_velocity);
    }
  }

  RCLCPP_DEBUG(logger, "relax target slow down velocity");
  return {};
}

boost::optional<std::pair<size_t, TrajectoryPoint>> getForwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin)
{
  if (base_idx + 1 > trajectory.size()) {
    return {};
  }

  if (margin < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(base_idx, trajectory.at(base_idx));
  }

  double length_sum = 0.0;
  double length_residual = 0.0;

  for (size_t i = base_idx; i < trajectory.size() - 1; ++i) {
    const auto & p_front = trajectory.at(i);
    const auto & p_back = trajectory.at(i + 1);

    length_sum += calcDistance2d(p_front, p_back);
    length_residual = length_sum - margin;

    if (length_residual >= std::numeric_limits<double>::epsilon()) {
      const auto p_insert = getBackwardPointFromBasePoint(p_back, p_front, p_back, length_residual);

      // p_front(trajectory.points.at(i)) is insert base point
      return std::make_pair(i, p_insert);
    }
  }

  if (length_residual < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(trajectory.size() - 1, trajectory.back());
  }

  return {};
}

boost::optional<std::pair<size_t, TrajectoryPoint>> getBackwardInsertPointFromBasePoint(
  const size_t base_idx, const TrajectoryPoints & trajectory, const double margin)
{
  if (base_idx + 1 > trajectory.size()) {
    return {};
  }

  if (margin < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(base_idx, trajectory.at(base_idx));
  }

  double length_sum = 0.0;
  double length_residual = 0.0;

  for (size_t i = base_idx; 0 < i; --i) {
    const auto & p_front = trajectory.at(i - 1);
    const auto & p_back = trajectory.at(i);

    length_sum += calcDistance2d(p_front, p_back);
    length_residual = length_sum - margin;

    if (length_residual >= std::numeric_limits<double>::epsilon()) {
      const auto p_insert =
        getBackwardPointFromBasePoint(p_front, p_back, p_front, length_residual);

      // p_front(trajectory.points.at(i-1)) is insert base point
      return std::make_pair(i - 1, p_insert);
    }
  }

  if (length_residual < std::numeric_limits<double>::epsilon()) {
    return std::make_pair(size_t(0), trajectory.front());
  }

  return {};
}

boost::optional<std::pair<size_t, double>> findNearestFrontIndex(
  const size_t start_idx, const TrajectoryPoints & trajectory, const Point & point)
{
  for (size_t i = start_idx; i < trajectory.size(); ++i) {
    const auto & p_traj = trajectory.at(i).pose;
    const auto yaw = getRPY(p_traj).z;
    const Point2d p_traj_direction(std::cos(yaw), std::sin(yaw));
    const Point2d p_traj_to_target(point.x - p_traj.position.x, point.y - p_traj.position.y);

    const auto is_in_front_of_target_point = p_traj_direction.dot(p_traj_to_target) < 0.0;
    const auto is_trajectory_end = i + 1 == trajectory.size();

    if (is_in_front_of_target_point || is_trajectory_end) {
      const auto dist_p_traj_to_target = p_traj_direction.normalized().dot(p_traj_to_target);
      return std::make_pair(i, dist_p_traj_to_target);
    }
  }

  return {};
}

bool isInFrontOfTargetPoint(const Pose & pose, const Point & point)
{
  const auto yaw = getRPY(pose).z;
  const Point2d pose_direction(std::cos(yaw), std::sin(yaw));
  const Point2d to_target(point.x - pose.position.x, point.y - pose.position.y);

  return pose_direction.dot(to_target) < 0.0;
}

bool checkValidIndex(const Pose & p_base, const Pose & p_next, const Pose & p_target)
{
  const Point2d base2target(
    p_target.position.x - p_base.position.x, p_target.position.y - p_base.position.y);
  const Point2d target2next(
    p_next.position.x - p_target.position.x, p_next.position.y - p_target.position.y);
  return base2target.dot(target2next) > 0.0;
}

std::string jsonDumpsPose(const Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}

DiagnosticStatus makeStopReasonDiag(const std::string stop_reason, const Pose & stop_pose)
{
  DiagnosticStatus stop_reason_diag;
  KeyValue stop_reason_diag_kv;
  stop_reason_diag.level = DiagnosticStatus::OK;
  stop_reason_diag.name = "stop_reason";
  stop_reason_diag.message = stop_reason;
  stop_reason_diag_kv.key = "stop_pose";
  stop_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  stop_reason_diag.values.push_back(stop_reason_diag_kv);
  return stop_reason_diag;
}

TrajectoryPoint getBackwardPointFromBasePoint(
  const TrajectoryPoint & p_from, const TrajectoryPoint & p_to, const TrajectoryPoint & p_base,
  const double backward_length)
{
  TrajectoryPoint output;
  const double dx = p_to.pose.position.x - p_from.pose.position.x;
  const double dy = p_to.pose.position.y - p_from.pose.position.y;
  const double norm = std::hypot(dx, dy);

  output = p_base;
  output.pose.position.x += backward_length * dx / norm;
  output.pose.position.y += backward_length * dy / norm;

  return output;
}

rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}

bool withinPolygon(
  const std::vector<cv::Point2d> & cv_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, PointCloud::Ptr candidate_points_ptr,
  PointCloud::Ptr within_points_ptr)
{
  Polygon2d boost_polygon;
  bool find_within_points = false;
  for (const auto & point : cv_polygon) {
    boost_polygon.outer().push_back(bg::make<Point2d>(point.x, point.y));
  }
  boost_polygon.outer().push_back(bg::make<Point2d>(cv_polygon.front().x, cv_polygon.front().y));

  for (size_t j = 0; j < candidate_points_ptr->size(); ++j) {
    Point2d point(candidate_points_ptr->at(j).x, candidate_points_ptr->at(j).y);
    if (bg::distance(prev_point, point) < radius || bg::distance(next_point, point) < radius) {
      if (bg::within(point, boost_polygon)) {
        within_points_ptr->push_back(candidate_points_ptr->at(j));
        find_within_points = true;
      }
    }
  }
  return find_within_points;
}

bool convexHull(
  const std::vector<cv::Point2d> & pointcloud, std::vector<cv::Point2d> & polygon_points)
{
  cv::Point2d centroid;
  centroid.x = 0;
  centroid.y = 0;
  for (const auto & point : pointcloud) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x = centroid.x / static_cast<double>(pointcloud.size());
  centroid.y = centroid.y / static_cast<double>(pointcloud.size());

  std::vector<cv::Point> normalized_pointcloud;
  std::vector<cv::Point> normalized_polygon_points;
  for (const auto & p : pointcloud) {
    normalized_pointcloud.emplace_back(
      cv::Point((p.x - centroid.x) * 1000.0, (p.y - centroid.y) * 1000.0));
  }
  cv::convexHull(normalized_pointcloud, normalized_polygon_points);

  for (const auto & p : normalized_polygon_points) {
    cv::Point2d polygon_point;
    polygon_point.x = (p.x / 1000.0 + centroid.x);
    polygon_point.y = (p.y / 1000.0 + centroid.y);
    polygon_points.push_back(polygon_point);
  }
  return true;
}

void createOneStepPolygon(
  const Pose & base_step_pose, const Pose & next_step_pose, std::vector<cv::Point2d> & polygon,
  const VehicleInfo & vehicle_info, const double expand_width)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;

  const auto & i = vehicle_info;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0 + expand_width;
  const auto & back_m = i.rear_overhang_m;
  // start step
  {
    const auto yaw = getRPY(base_step_pose).z;
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
      base_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
      base_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
      base_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
      base_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
  }
  // next step
  {
    const auto yaw = getRPY(next_step_pose).z;
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
      next_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
      next_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
      next_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
      next_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
  }
  convexHull(one_step_move_vehicle_corner_points, polygon);
}

void insertStopPoint(
  const StopPoint & stop_point, TrajectoryPoints & output, DiagnosticStatus & stop_reason_diag)
{
  const auto traj_end_idx = output.size() - 1;
  const auto & stop_idx = stop_point.index;

  const auto & p_base = output.at(stop_idx);
  const auto & p_next = output.at(std::min(stop_idx + 1, traj_end_idx));
  const auto & p_insert = stop_point.point;

  constexpr double min_dist = 1e-3;

  const auto is_p_base_and_p_insert_overlap = calcDistance2d(p_base, p_insert) < min_dist;
  const auto is_p_next_and_p_insert_overlap = calcDistance2d(p_next, p_insert) < min_dist;
  const auto is_valid_index = checkValidIndex(p_base.pose, p_next.pose, p_insert.pose);

  auto update_stop_idx = stop_idx;

  if (!is_p_base_and_p_insert_overlap && !is_p_next_and_p_insert_overlap && is_valid_index) {
    // insert: start_idx and end_idx are shifted by one
    output.insert(output.begin() + stop_idx + 1, p_insert);
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  } else if (is_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_stop_idx = std::min(update_stop_idx + 1, traj_end_idx);
  }

  for (size_t i = update_stop_idx; i < output.size(); ++i) {
    output.at(i).longitudinal_velocity_mps = 0.0;
  }

  stop_reason_diag = makeStopReasonDiag("obstacle", p_insert.pose);
}

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point)
{
  tf2::Transform map2goal;
  tf2::fromMsg(goal_point.pose, map2goal);
  tf2::Transform local_extend_point;
  local_extend_point.setOrigin(tf2::Vector3(extend_distance, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  local_extend_point.setRotation(q);
  const auto map2extend_point = map2goal * local_extend_point;
  Pose extend_pose;
  tf2::toMsg(map2extend_point, extend_pose);
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = extend_pose;
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}

TrajectoryPoints decimateTrajectory(
  const TrajectoryPoints & input, const double step_length,
  std::map<size_t /* decimate */, size_t /* origin */> & index_map)
{
  TrajectoryPoints output{};

  double trajectory_length_sum = 0.0;
  double next_length = 0.0;

  for (int i = 0; i < static_cast<int>(input.size()) - 1; ++i) {
    const auto & p_front = input.at(i);
    const auto & p_back = input.at(i + 1);
    constexpr double epsilon = 1e-3;

    if (next_length <= trajectory_length_sum + epsilon) {
      const auto p_interpolate =
        getBackwardPointFromBasePoint(p_front, p_back, p_back, next_length - trajectory_length_sum);
      output.push_back(p_interpolate);

      index_map.insert(std::make_pair(output.size() - 1, size_t(i)));
      next_length += step_length;
      continue;
    }

    trajectory_length_sum += calcDistance2d(p_front, p_back);
  }
  if (!input.empty()) {
    output.push_back(input.back());
    index_map.insert(std::make_pair(output.size() - 1, input.size() - 1));
  }

  return output;
}

TrajectoryPoints extendTrajectory(const TrajectoryPoints & input, const double extend_distance)
{
  TrajectoryPoints output = input;

  if (extend_distance < std::numeric_limits<double>::epsilon()) {
    return output;
  }

  const auto goal_point = input.back();
  double interpolation_distance = 0.1;

  double extend_sum = 0.0;
  while (extend_sum <= (extend_distance - interpolation_distance)) {
    const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_sum, goal_point);
    output.push_back(extend_trajectory_point);
    extend_sum += interpolation_distance;
  }
  const auto extend_trajectory_point = getExtendTrajectoryPoint(extend_distance, goal_point);
  output.push_back(extend_trajectory_point);

  return output;
}

bool getSelfPose(const Header & header, const tf2_ros::Buffer & tf_buffer, Pose & self_pose)
{
  try {
    TransformStamped transform;
    transform = tf_buffer.lookupTransform(
      header.frame_id, "base_link", header.stamp, rclcpp::Duration::from_seconds(0.1));
    self_pose.position.x = transform.transform.translation.x;
    self_pose.position.y = transform.transform.translation.y;
    self_pose.position.z = transform.transform.translation.z;
    self_pose.orientation.x = transform.transform.rotation.x;
    self_pose.orientation.y = transform.transform.rotation.y;
    self_pose.orientation.z = transform.transform.rotation.z;
    self_pose.orientation.w = transform.transform.rotation.w;
    return true;
  } catch (tf2::TransformException & ex) {
    return false;
  }
}

void getNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * nearest_collision_point,
  rclcpp::Time * nearest_collision_point_time)
{
  double min_norm = 0.0;
  bool is_init = false;
  const auto yaw = getRPY(base_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));

  for (const auto & p : pointcloud) {
    const Eigen::Vector2d pointcloud_vec(p.x - base_pose.position.x, p.y - base_pose.position.y);
    double norm = base_pose_vec.dot(pointcloud_vec);
    if (norm < min_norm || !is_init) {
      min_norm = norm;
      *nearest_collision_point = p;
      *nearest_collision_point_time = pcl_conversions::fromPCL(pointcloud.header).stamp;
      is_init = true;
    }
  }
}

void getLateralNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * lateral_nearest_point,
  double * deviation)
{
  double min_norm = std::numeric_limits<double>::max();
  const auto yaw = getRPY(base_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    const Eigen::Vector2d pointcloud_vec(
      pointcloud.at(i).x - base_pose.position.x, pointcloud.at(i).y - base_pose.position.y);
    double norm =
      std::abs(base_pose_vec.x() * pointcloud_vec.y() - base_pose_vec.y() * pointcloud_vec.x());
    if (norm < min_norm) {
      min_norm = norm;
      *lateral_nearest_point = pointcloud.at(i);
    }
  }
  *deviation = min_norm;
}

Pose getVehicleCenterFromBase(const Pose & base_pose, const VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;
  const auto yaw = getRPY(base_pose).z;

  Pose center_pose;
  center_pose.position.x =
    base_pose.position.x + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::cos(yaw);
  center_pose.position.y =
    base_pose.position.y + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::sin(yaw);
  center_pose.position.z = base_pose.position.z;
  center_pose.orientation = base_pose.orientation;
  return center_pose;
}
}  // namespace motion_planning
