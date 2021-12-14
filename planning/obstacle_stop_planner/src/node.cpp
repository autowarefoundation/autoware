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

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "obstacle_stop_planner/node.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <diagnostic_msgs/msg/key_value.hpp>

#include <pcl/filters/voxel_grid.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>

namespace motion_planning
{
using tier4_autoware_utils::calcAzimuthAngle;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcSignedArcLength;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::findNearestIndex;
using tier4_autoware_utils::getRPY;

namespace
{
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
  const ObstacleStopPlannerNode::SlowDownParam & slow_down_param,
  const double dist_baselink_to_obstacle, const double current_vel, const double current_acc)
{
  const auto & p = slow_down_param;
  const auto & logger = rclcpp::get_logger("calcFeasibleMarginAndVelocity");
  constexpr double epsilon = 1e-4;

  if (current_vel < p.slow_down_vel + epsilon) {
    return std::make_pair(p.forward_margin, p.slow_down_vel);
  }

  for (double planning_jerk = p.jerk_start; planning_jerk > p.slow_down_min_jerk - epsilon;
       planning_jerk += p.jerk_span) {
    const double jerk_dec = planning_jerk;
    const double jerk_acc = std::abs(planning_jerk);

    const auto planning_dec =
      planning_jerk > p.normal_min_jerk ? p.limit_min_acc : p.normal_min_acc;
    const auto stop_dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, p.slow_down_vel, current_acc, planning_dec, jerk_acc, jerk_dec);

    if (!stop_dist) {
      continue;
    }

    if (stop_dist.get() + p.forward_margin < dist_baselink_to_obstacle) {
      RCLCPP_DEBUG(
        logger, "[found plan] dist:%-6.2f jerk:%-6.2f margin:%-6.2f v0:%-6.2f vt:%-6.2f",
        stop_dist.get(), planning_jerk, p.forward_margin, p.slow_down_vel, current_vel);
      return std::make_pair(p.forward_margin, p.slow_down_vel);
    }
  }

  {
    const double jerk_dec = p.slow_down_min_jerk;
    const double jerk_acc = std::abs(p.slow_down_min_jerk);

    const auto planning_dec =
      p.slow_down_min_jerk > p.normal_min_jerk ? p.limit_min_acc : p.normal_min_acc;
    const auto stop_dist = calcDecelDistWithJerkAndAccConstraints(
      current_vel, p.slow_down_vel, current_acc, planning_dec, jerk_acc, jerk_dec);

    if (!stop_dist) {
      return {};
    }

    if (stop_dist.get() + p.forward_margin_min < dist_baselink_to_obstacle) {
      const auto planning_margin = dist_baselink_to_obstacle - stop_dist.get();
      RCLCPP_DEBUG(
        logger, "[relax margin] dist:%-6.2f jerk:%-6.2f margin:%-6.2f v0:%-6.2f vt%-6.2f",
        stop_dist.get(), p.slow_down_min_jerk, planning_margin, p.slow_down_vel, current_vel);
      return std::make_pair(planning_margin, p.slow_down_vel);
    }
  }

  RCLCPP_DEBUG(logger, "relax target slow down velocity");
  return {};
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
  const size_t start_idx, const TrajectoryPoints & trajectory,
  const geometry_msgs::msg::Point & point)
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
bool isInFrontOfTargetPoint(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & point)
{
  const auto yaw = getRPY(pose).z;
  const Point2d pose_direction(std::cos(yaw), std::sin(yaw));
  const Point2d to_target(point.x - pose.position.x, point.y - pose.position.y);

  return pose_direction.dot(to_target) < 0.0;
}
bool checkValidIndex(
  const geometry_msgs::msg::Pose & p_base, const geometry_msgs::msg::Pose & p_next,
  const geometry_msgs::msg::Pose & p_target)
{
  const Point2d base2target(
    p_target.position.x - p_base.position.x, p_target.position.y - p_base.position.y);
  const Point2d target2next(
    p_next.position.x - p_target.position.x, p_next.position.y - p_target.position.y);
  return base2target.dot(target2next) > 0.0;
}
std::string jsonDumpsPose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}
diagnostic_msgs::msg::DiagnosticStatus makeStopReasonDiag(
  const std::string stop_reason, const geometry_msgs::msg::Pose & stop_pose)
{
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag;
  diagnostic_msgs::msg::KeyValue stop_reason_diag_kv;
  stop_reason_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stop_reason_diag.name = "stop_reason";
  stop_reason_diag.message = stop_reason;
  stop_reason_diag_kv.key = "stop_pose";
  stop_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  stop_reason_diag.values.push_back(stop_reason_diag_kv);
  return stop_reason_diag;
}
}  // namespace

ObstacleStopPlannerNode::ObstacleStopPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_stop_planner", node_options)
{
  // Vehicle Parameters
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  const auto & i = vehicle_info_;

  // Parameters
  {
    auto & p = node_param_;
    p.enable_slow_down = declare_parameter("enable_slow_down", false);
    p.max_velocity = declare_parameter("max_velocity", 20.0);
    p.hunting_threshold = declare_parameter("hunting_threshold", 0.5);
    p.lowpass_gain = declare_parameter("lowpass_gain", 0.9);
    lpf_acc_ = std::make_shared<LowpassFilter1d>(0.0, p.lowpass_gain);
  }

  {
    auto & p = stop_param_;
    const std::string ns = "stop_planner.";
    p.stop_margin = declare_parameter(ns + "stop_margin", 5.0);
    p.min_behavior_stop_margin = declare_parameter(ns + "min_behavior_stop_margin", 2.0);
    p.expand_stop_range = declare_parameter(ns + "expand_stop_range", 0.0);
    p.extend_distance = declare_parameter(ns + "extend_distance", 0.0);
    p.step_length = declare_parameter(ns + "step_length", 1.0);
    p.stop_margin += i.max_longitudinal_offset_m;
    p.min_behavior_stop_margin += i.max_longitudinal_offset_m;
    p.stop_search_radius =
      p.step_length +
      std::hypot(i.vehicle_width_m / 2.0 + p.expand_stop_range, i.vehicle_length_m / 2.0);
  }

  {
    auto & p = slow_down_param_;
    const std::string ns = "slow_down_planner.";
    // common param
    p.normal_min_jerk = declare_parameter("normal.min_jerk", -0.3);
    p.normal_min_acc = declare_parameter("normal.min_acc", -1.0);
    p.limit_min_jerk = declare_parameter("limit.min_jerk", -1.5);
    p.limit_min_acc = declare_parameter("limit.min_acc", -2.5);
    // slow down planner specific parameters
    p.forward_margin = declare_parameter(ns + "forward_margin", 5.0);
    p.backward_margin = declare_parameter(ns + "backward_margin", 5.0);
    p.expand_slow_down_range = declare_parameter(ns + "expand_slow_down_range", 1.0);
    p.max_slow_down_vel = declare_parameter(ns + "max_slow_down_vel", 4.0);
    p.min_slow_down_vel = declare_parameter(ns + "min_slow_down_vel", 2.0);
    // consider jerk/dec constraints in slow down
    p.consider_constraints = declare_parameter(ns + "consider_constraints", false);
    p.forward_margin_min = declare_parameter(ns + "forward_margin_min", 1.0);
    p.forward_margin_span = declare_parameter(ns + "forward_margin_span", -0.1);
    p.slow_down_min_jerk = declare_parameter(ns + "jerk_min_slow_down", -0.3);
    p.jerk_start = declare_parameter(ns + "jerk_start", -0.1);
    p.jerk_span = declare_parameter(ns + "jerk_span", -0.01);
    p.slow_down_vel = declare_parameter(ns + "slow_down_vel", 1.39);
    p.vel_threshold_reset_velocity_limit_ =
      declare_parameter(ns + "vel_threshold_reset_velocity_limit_", 0.2);
    p.dec_threshold_reset_velocity_limit_ =
      declare_parameter(ns + "dec_threshold_reset_velocity_limit_", 0.1);
    p.forward_margin += i.max_longitudinal_offset_m;
    p.forward_margin_min += i.wheel_base_m + i.front_overhang_m;
    p.backward_margin += i.rear_overhang_m;
    p.slow_down_search_radius =
      stop_param_.step_length +
      std::hypot(i.vehicle_width_m / 2.0 + p.expand_slow_down_range, i.vehicle_length_m / 2.0);
  }

  // Initializer
  acc_controller_ = std::make_unique<motion_planning::AdaptiveCruiseController>(
    this, i.vehicle_width_m, i.vehicle_length_m, i.max_longitudinal_offset_m);
  debug_ptr_ = std::make_shared<ObstacleStopPlannerDebugNode>(this, i.max_longitudinal_offset_m);
  last_detection_time_ = this->now();

  // Publishers
  path_pub_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/stop_reason", 1);
  pub_clear_velocity_limit_ =
    this->create_publisher<VelocityLimitClearCommand>("~/output/velocity_limit_clear_command", 1);
  pub_velocity_limit_ = this->create_publisher<VelocityLimit>("~/output/max_velocity", 1);

  // Subscribers
  obstacle_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ObstacleStopPlannerNode::obstaclePointcloudCallback, this, std::placeholders::_1));
  path_sub_ = this->create_subscription<Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&ObstacleStopPlannerNode::pathCallback, this, std::placeholders::_1));
  current_velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&ObstacleStopPlannerNode::currentVelocityCallback, this, std::placeholders::_1));
  dynamic_object_sub_ = this->create_subscription<PredictedObjects>(
    "~/input/objects", 1,
    std::bind(&ObstacleStopPlannerNode::dynamicObjectCallback, this, std::placeholders::_1));
  expand_stop_range_sub_ = this->create_subscription<ExpandStopRange>(
    "~/input/expand_stop_range", 1,
    std::bind(
      &ObstacleStopPlannerNode::externalExpandStopRangeCallback, this, std::placeholders::_1));
}

void ObstacleStopPlannerNode::obstaclePointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  obstacle_ros_pointcloud_ptr_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_height_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_height_filtered_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*input_msg, *pointcloud_ptr);

  for (const auto & point : pointcloud_ptr->points) {
    no_height_pointcloud_ptr->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
  }
  filter.setInputCloud(no_height_pointcloud_ptr);
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);
  filter.filter(*no_height_filtered_pointcloud_ptr);
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *obstacle_ros_pointcloud_ptr_);
  obstacle_ros_pointcloud_ptr_->header = input_msg->header;
}

void ObstacleStopPlannerNode::pathCallback(const Trajectory::ConstSharedPtr input_msg)
{
  if (!obstacle_ros_pointcloud_ptr_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for obstacle pointcloud...");
    return;
  }

  if (!current_velocity_ptr_ && node_param_.enable_slow_down) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
      "waiting for current velocity...");
    return;
  }

  if (input_msg->points.empty()) {
    return;
  }

  PlannerData planner_data{};

  getSelfPose(input_msg->header, tf_buffer_, planner_data.current_pose);

  Trajectory output_trajectory = *input_msg;
  TrajectoryPoints output_trajectory_points =
    tier4_autoware_utils::convertToTrajectoryPointArray(*input_msg);

  // trim trajectory from self pose
  const auto base_trajectory = trimTrajectoryWithIndexFromSelfPose(
    tier4_autoware_utils::convertToTrajectoryPointArray(*input_msg), planner_data.current_pose,
    planner_data.trajectory_trim_index);
  // extend trajectory to consider obstacles after the goal
  const auto extend_trajectory = extendTrajectory(base_trajectory, stop_param_.extend_distance);
  // decimate trajectory for calculation cost
  const auto decimate_trajectory = decimateTrajectory(
    extend_trajectory, stop_param_.step_length, planner_data.decimate_trajectory_index_map);

  // search obstacles within slow-down/collision area
  searchObstacle(decimate_trajectory, output_trajectory_points, planner_data, input_msg->header);
  // insert slow-down-section/stop-point
  insertVelocity(output_trajectory_points, planner_data, input_msg->header);

  const auto no_slow_down_section = !planner_data.slow_down_require && !latest_slow_down_section_;
  const auto no_hunting = (rclcpp::Time(input_msg->header.stamp) - last_detection_time_).seconds() >
                          node_param_.hunting_threshold;
  if (node_param_.enable_slow_down && no_slow_down_section && set_velocity_limit_ && no_hunting) {
    resetExternalVelocityLimit();
  }

  auto trajectory = tier4_autoware_utils::convertToTrajectory(output_trajectory_points);
  trajectory.header = input_msg->header;
  path_pub_->publish(trajectory);
  publishDebugData(planner_data);
}

void ObstacleStopPlannerNode::searchObstacle(
  const TrajectoryPoints & decimate_trajectory, TrajectoryPoints & output,
  PlannerData & planner_data, const std_msgs::msg::Header & trajectory_header)
{
  // search candidate obstacle pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr slow_down_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_candidate_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  if (!searchPointcloudNearTrajectory(
        decimate_trajectory, obstacle_ros_pointcloud_ptr_, obstacle_candidate_pointcloud_ptr,
        trajectory_header)) {
    return;
  }

  for (size_t i = 0; i < decimate_trajectory.size() - 1; ++i) {
    // create one step circle center for vehicle
    const auto & p_front = decimate_trajectory.at(i).pose;
    const auto & p_back = decimate_trajectory.at(i + 1).pose;
    const auto prev_center_pose = getVehicleCenterFromBase(p_front);
    const Point2d prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose = getVehicleCenterFromBase(p_back);
    const Point2d next_center_point(next_center_pose.position.x, next_center_pose.position.y);

    if (node_param_.enable_slow_down) {
      std::vector<cv::Point2d> one_step_move_slow_down_range_polygon;
      // create one step polygon for slow_down range
      createOneStepPolygon(
        p_front, p_back, one_step_move_slow_down_range_polygon,
        slow_down_param_.expand_slow_down_range);
      debug_ptr_->pushPolygon(
        one_step_move_slow_down_range_polygon, p_front.position.z, PolygonType::SlowDownRange);

      planner_data.found_slow_down_points = withinPolygon(
        one_step_move_slow_down_range_polygon, slow_down_param_.slow_down_search_radius,
        prev_center_point, next_center_point, obstacle_candidate_pointcloud_ptr,
        slow_down_pointcloud_ptr);

      const auto found_first_slow_down_points =
        planner_data.found_slow_down_points && !planner_data.slow_down_require;

      if (found_first_slow_down_points) {
        // found nearest slow down obstacle
        planner_data.decimate_trajectory_slow_down_index = i;
        planner_data.slow_down_require = true;
        getNearestPoint(
          *slow_down_pointcloud_ptr, p_front, &planner_data.nearest_slow_down_point,
          &planner_data.nearest_collision_point_time);
        getLateralNearestPoint(
          *slow_down_pointcloud_ptr, p_front, &planner_data.lateral_nearest_slow_down_point,
          &planner_data.lateral_deviation);

        debug_ptr_->pushObstaclePoint(planner_data.nearest_slow_down_point, PointType::SlowDown);
        debug_ptr_->pushPolygon(
          one_step_move_slow_down_range_polygon, p_front.position.z, PolygonType::SlowDown);
      }

    } else {
      slow_down_pointcloud_ptr = obstacle_candidate_pointcloud_ptr;
    }

    {
      std::vector<cv::Point2d> one_step_move_vehicle_polygon;
      // create one step polygon for vehicle
      createOneStepPolygon(
        p_front, p_back, one_step_move_vehicle_polygon, stop_param_.expand_stop_range);
      debug_ptr_->pushPolygon(
        one_step_move_vehicle_polygon, decimate_trajectory.at(i).pose.position.z,
        PolygonType::Vehicle);

      pcl::PointCloud<pcl::PointXYZ>::Ptr collision_pointcloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
      collision_pointcloud_ptr->header = obstacle_candidate_pointcloud_ptr->header;

      planner_data.found_collision_points = withinPolygon(
        one_step_move_vehicle_polygon, stop_param_.stop_search_radius, prev_center_point,
        next_center_point, slow_down_pointcloud_ptr, collision_pointcloud_ptr);

      if (planner_data.found_collision_points) {
        planner_data.decimate_trajectory_collision_index = i;
        getNearestPoint(
          *collision_pointcloud_ptr, p_front, &planner_data.nearest_collision_point,
          &planner_data.nearest_collision_point_time);

        debug_ptr_->pushObstaclePoint(planner_data.nearest_collision_point, PointType::Stop);
        debug_ptr_->pushPolygon(
          one_step_move_vehicle_polygon, p_front.position.z, PolygonType::Collision);

        planner_data.stop_require = planner_data.found_collision_points;
        acc_controller_->insertAdaptiveCruiseVelocity(
          decimate_trajectory, planner_data.decimate_trajectory_collision_index,
          planner_data.current_pose, planner_data.nearest_collision_point,
          planner_data.nearest_collision_point_time, object_ptr_, current_velocity_ptr_,
          &planner_data.stop_require, &output, trajectory_header);

        break;
      }
    }
  }
}

void ObstacleStopPlannerNode::insertVelocity(
  TrajectoryPoints & output, PlannerData & planner_data,
  const std_msgs::msg::Header & trajectory_header)
{
  if (planner_data.stop_require) {
    // insert stop point
    const auto traj_end_idx = output.size() - 1;
    const auto idx = planner_data.decimate_trajectory_index_map.at(
                       planner_data.decimate_trajectory_collision_index) +
                     planner_data.trajectory_trim_index;
    const auto index_with_dist_remain = findNearestFrontIndex(
      std::min(idx, traj_end_idx), output,
      createPoint(
        planner_data.nearest_collision_point.x, planner_data.nearest_collision_point.y, 0));

    if (index_with_dist_remain) {
      const auto stop_point = searchInsertPoint(
        index_with_dist_remain.get().first, output, index_with_dist_remain.get().second);
      insertStopPoint(stop_point, output, planner_data.stop_reason_diag);
    }
  }

  if (planner_data.slow_down_require) {
    // insert slow down point
    const auto traj_end_idx = output.size() - 1;
    const auto idx = planner_data.decimate_trajectory_index_map.at(
      planner_data.decimate_trajectory_slow_down_index);
    const auto index_with_dist_remain = findNearestFrontIndex(
      std::min(idx, traj_end_idx), output,
      createPoint(
        planner_data.nearest_slow_down_point.x, planner_data.nearest_slow_down_point.y, 0));

    if (index_with_dist_remain) {
      const auto vehicle_idx = std::min(planner_data.trajectory_trim_index, traj_end_idx);
      const auto dist_baselink_to_obstacle =
        calcSignedArcLength(output, vehicle_idx, index_with_dist_remain.get().first);

      debug_ptr_->setDebugValues(
        DebugValues::TYPE::OBSTACLE_DISTANCE,
        dist_baselink_to_obstacle + index_with_dist_remain.get().second);
      const auto slow_down_section = createSlowDownSection(
        index_with_dist_remain.get().first, output, planner_data.lateral_deviation,
        index_with_dist_remain.get().second, dist_baselink_to_obstacle);

      if (
        !latest_slow_down_section_ &&
        dist_baselink_to_obstacle + index_with_dist_remain.get().second <
          vehicle_info_.max_longitudinal_offset_m) {
        latest_slow_down_section_ = slow_down_section;
      }

      insertSlowDownSection(slow_down_section, output);
    }

    last_detection_time_ = trajectory_header.stamp;
  }

  if (node_param_.enable_slow_down && latest_slow_down_section_) {
    // check whether ego is in slow down section or not
    const auto & p_start = latest_slow_down_section_.get().start_point.pose.position;
    const auto & p_end = latest_slow_down_section_.get().end_point.pose.position;
    const auto reach_slow_down_start_point =
      isInFrontOfTargetPoint(planner_data.current_pose, p_start);
    const auto reach_slow_down_end_point = isInFrontOfTargetPoint(planner_data.current_pose, p_end);
    const auto is_in_slow_down_section = reach_slow_down_start_point && !reach_slow_down_end_point;
    const auto index_with_dist_remain = findNearestFrontIndex(0, output, p_end);

    if (is_in_slow_down_section && index_with_dist_remain) {
      const auto end_insert_point_with_idx = getBackwardInsertPointFromBasePoint(
        index_with_dist_remain.get().first, output, -index_with_dist_remain.get().second);

      SlowDownSection slow_down_section{};
      slow_down_section.slow_down_start_idx = 0;
      slow_down_section.start_point = output.front();
      slow_down_section.slow_down_end_idx = end_insert_point_with_idx.get().first;
      slow_down_section.end_point = end_insert_point_with_idx.get().second;
      slow_down_section.velocity =
        set_velocity_limit_ ? std::numeric_limits<double>::max() : slow_down_param_.slow_down_vel;

      insertSlowDownSection(slow_down_section, output);
    } else {
      latest_slow_down_section_ = {};
    }
  }

  for (size_t i = 0; i < output.size() - 2; ++i) {
    const auto & p_base = output.at(i).pose;
    const auto & p_target = output.at(i + 1).pose;
    const auto & p_next = output.at(i + 2).pose;
    if (!checkValidIndex(p_base, p_next, p_target)) {
      RCLCPP_ERROR(get_logger(), "detect bad index");
    }
  }

  stop_reason_diag_pub_->publish(planner_data.stop_reason_diag);
}

bool ObstacleStopPlannerNode::withinPolygon(
  const std::vector<cv::Point2d> & cv_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_points_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr within_points_ptr)
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

void ObstacleStopPlannerNode::externalExpandStopRangeCallback(
  const ExpandStopRange::ConstSharedPtr input_msg)
{
  const auto & i = vehicle_info_;
  stop_param_.expand_stop_range = input_msg->expand_stop_range;
  stop_param_.stop_search_radius =
    stop_param_.step_length +
    std::hypot(i.vehicle_width_m / 2.0 + stop_param_.expand_stop_range, i.vehicle_length_m / 2.0);
}

void ObstacleStopPlannerNode::insertStopPoint(
  const StopPoint & stop_point, TrajectoryPoints & output,
  diagnostic_msgs::msg::DiagnosticStatus & stop_reason_diag)
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
  debug_ptr_->pushPose(p_insert.pose, PoseType::Stop);
}

StopPoint ObstacleStopPlannerNode::searchInsertPoint(
  const int idx, const TrajectoryPoints & base_trajectory, const double dist_remain)
{
  const auto max_dist_stop_point =
    createTargetPoint(idx, stop_param_.stop_margin, base_trajectory, dist_remain);
  const auto min_dist_stop_point =
    createTargetPoint(idx, stop_param_.min_behavior_stop_margin, base_trajectory, dist_remain);

  // check if stop point is already inserted by behavior planner
  bool is_inserted_already_stop_point = false;
  const double epsilon = 1e-3;
  for (int j = max_dist_stop_point.index - 1; j < static_cast<int>(idx); ++j) {
    if (std::abs(base_trajectory.at(std::max(j, 0)).longitudinal_velocity_mps) < epsilon) {
      is_inserted_already_stop_point = true;
      break;
    }
  }
  // insert stop point
  StopPoint stop_point{};
  stop_point.index =
    !is_inserted_already_stop_point ? max_dist_stop_point.index : min_dist_stop_point.index;
  stop_point.point =
    !is_inserted_already_stop_point ? max_dist_stop_point.point : min_dist_stop_point.point;
  return stop_point;
}

StopPoint ObstacleStopPlannerNode::createTargetPoint(
  const int idx, const double margin, const TrajectoryPoints & base_trajectory,
  const double dist_remain)
{
  const auto update_margin_from_vehicle = margin - dist_remain;
  const auto insert_point_with_idx =
    getBackwardInsertPointFromBasePoint(idx, base_trajectory, update_margin_from_vehicle);

  if (!insert_point_with_idx) {
    // TODO(Satoshi Ota)
    return StopPoint{};
  }

  StopPoint stop_point{};
  stop_point.index = insert_point_with_idx.get().first;
  stop_point.point = insert_point_with_idx.get().second;

  return stop_point;
}

SlowDownSection ObstacleStopPlannerNode::createSlowDownSection(
  const int idx, const TrajectoryPoints & base_trajectory, const double lateral_deviation,
  const double dist_remain, const double dist_baselink_to_obstacle)
{
  if (!current_velocity_ptr_) {
    // TODO(Satoshi Ota)
    return SlowDownSection{};
  }

  if (slow_down_param_.consider_constraints) {
    const auto & current_vel = current_velocity_ptr_->twist.twist.linear.x;
    const auto margin_with_vel = calcFeasibleMarginAndVelocity(
      slow_down_param_, dist_baselink_to_obstacle + dist_remain, current_vel, current_acc_);

    const auto relax_target_vel = margin_with_vel == boost::none;
    if (relax_target_vel && !set_velocity_limit_) {
      setExternalVelocityLimit();
    }

    const auto no_need_velocity_limit =
      dist_baselink_to_obstacle + dist_remain > slow_down_param_.forward_margin;
    if (set_velocity_limit_ && no_need_velocity_limit) {
      resetExternalVelocityLimit();
    }

    const auto use_velocity_limit = relax_target_vel || set_velocity_limit_;

    const auto update_forward_margin_from_vehicle =
      use_velocity_limit ? slow_down_param_.forward_margin_min - dist_remain
                         : margin_with_vel.get().first - dist_remain;
    const auto update_backward_margin_from_vehicle = slow_down_param_.backward_margin + dist_remain;

    const auto velocity =
      use_velocity_limit ? std::numeric_limits<double>::max() : margin_with_vel.get().second;

    return createSlowDownSectionFromMargin(
      idx, base_trajectory, update_forward_margin_from_vehicle, update_backward_margin_from_vehicle,
      velocity);
  } else {
    const auto update_forward_margin_from_vehicle = slow_down_param_.forward_margin - dist_remain;
    const auto update_backward_margin_from_vehicle = slow_down_param_.backward_margin + dist_remain;

    const auto velocity =
      slow_down_param_.min_slow_down_vel +
      (slow_down_param_.max_slow_down_vel - slow_down_param_.min_slow_down_vel) *
        std::max(lateral_deviation - vehicle_info_.vehicle_width_m / 2, 0.0) /
        slow_down_param_.expand_slow_down_range;

    return createSlowDownSectionFromMargin(
      idx, base_trajectory, update_forward_margin_from_vehicle, update_backward_margin_from_vehicle,
      velocity);
  }

  return SlowDownSection{};
}

SlowDownSection ObstacleStopPlannerNode::createSlowDownSectionFromMargin(
  const int idx, const TrajectoryPoints & base_trajectory, const double forward_margin,
  const double backward_margin, const double velocity)
{
  // calc slow down start point
  const auto start_insert_point_with_idx =
    getBackwardInsertPointFromBasePoint(idx, base_trajectory, forward_margin);
  // calc slow down end point
  const auto end_insert_point_with_idx =
    getForwardInsertPointFromBasePoint(idx, base_trajectory, backward_margin);

  if (!start_insert_point_with_idx || !end_insert_point_with_idx) {
    // TODO(Satoshi Ota)
    return SlowDownSection{};
  }

  SlowDownSection slow_down_section{};
  slow_down_section.slow_down_start_idx = start_insert_point_with_idx.get().first;
  slow_down_section.start_point = start_insert_point_with_idx.get().second;
  slow_down_section.slow_down_end_idx = end_insert_point_with_idx.get().first;
  slow_down_section.end_point = end_insert_point_with_idx.get().second;
  slow_down_section.velocity = velocity;

  return slow_down_section;
}

void ObstacleStopPlannerNode::insertSlowDownSection(
  const SlowDownSection & slow_down_section, TrajectoryPoints & output)
{
  const auto traj_end_idx = output.size() - 1;
  const auto & start_idx = slow_down_section.slow_down_start_idx;
  const auto & end_idx = slow_down_section.slow_down_end_idx;

  const auto p_base_start = output.at(start_idx);
  const auto p_next_start = output.at(std::min(start_idx + 1, traj_end_idx));
  const auto & p_insert_start = slow_down_section.start_point;

  const auto p_base_end = output.at(end_idx);
  const auto p_next_end = output.at(std::min(end_idx + 1, traj_end_idx));
  const auto & p_insert_end = slow_down_section.end_point;

  constexpr double min_dist = 1e-3;

  const auto is_valid_index_start =
    checkValidIndex(p_base_start.pose, p_next_start.pose, p_insert_start.pose);
  const auto is_start_p_base_and_p_insert_overlap =
    calcDistance2d(p_base_start, p_insert_start) < min_dist;
  const auto is_start_p_next_and_p_insert_overlap =
    calcDistance2d(p_next_start, p_insert_start) < min_dist;

  auto update_start_idx = start_idx;
  auto update_end_idx = end_idx;

  if (
    !is_start_p_base_and_p_insert_overlap && !is_start_p_next_and_p_insert_overlap &&
    is_valid_index_start) {
    // insert: start_idx and end_idx are shifted by one
    output.insert(output.begin() + start_idx + 1, p_insert_start);
    update_start_idx = std::min(update_start_idx + 1, traj_end_idx);
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  } else if (is_start_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_start_idx = std::min(update_start_idx + 1, traj_end_idx);
  }

  const auto is_end_p_base_and_p_insert_overlap =
    calcDistance2d(p_base_end, p_insert_end) < min_dist;
  const auto is_end_p_next_and_p_insert_overlap =
    calcDistance2d(p_next_end, p_insert_end) < min_dist;
  const auto is_valid_index_end =
    checkValidIndex(p_base_end.pose, p_next_end.pose, p_insert_end.pose);

  if (
    !is_end_p_base_and_p_insert_overlap && !is_end_p_next_and_p_insert_overlap &&
    is_valid_index_end) {
    // insert: end_idx is shifted by one
    output.insert(output.begin() + update_end_idx + 1, p_insert_end);
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  } else if (is_end_p_next_and_p_insert_overlap) {
    // not insert: p_insert is merged into p_next
    update_end_idx = std::min(update_end_idx + 1, traj_end_idx);
  }

  for (size_t i = update_start_idx; i <= update_end_idx; ++i) {
    output.at(i).longitudinal_velocity_mps = std::min(
      slow_down_section.velocity, static_cast<double>(output.at(i).longitudinal_velocity_mps));
  }

  debug_ptr_->pushPose(p_base_start.pose, PoseType::SlowDownStart);
  debug_ptr_->pushPose(p_base_end.pose, PoseType::SlowDownEnd);
}

void ObstacleStopPlannerNode::dynamicObjectCallback(
  const PredictedObjects::ConstSharedPtr input_msg)
{
  object_ptr_ = input_msg;
}

void ObstacleStopPlannerNode::currentVelocityCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr input_msg)
{
  current_velocity_ptr_ = input_msg;

  if (!prev_velocity_ptr_) {
    prev_velocity_ptr_ = current_velocity_ptr_;
    return;
  }

  const double dv =
    current_velocity_ptr_->twist.twist.linear.x - prev_velocity_ptr_->twist.twist.linear.x;
  const double dt = std::max(
    (rclcpp::Time(current_velocity_ptr_->header.stamp) -
     rclcpp::Time(prev_velocity_ptr_->header.stamp))
      .seconds(),
    1e-03);

  const double accel = dv / dt;

  current_acc_ = lpf_acc_->filter(accel);
  prev_velocity_ptr_ = current_velocity_ptr_;
}

TrajectoryPoint ObstacleStopPlannerNode::getExtendTrajectoryPoint(
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
  geometry_msgs::msg::Pose extend_pose;
  tf2::toMsg(map2extend_point, extend_pose);
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = extend_pose;
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}

TrajectoryPoints ObstacleStopPlannerNode::extendTrajectory(
  const TrajectoryPoints & input, const double extend_distance)
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

TrajectoryPoints ObstacleStopPlannerNode::decimateTrajectory(
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

TrajectoryPoints ObstacleStopPlannerNode::trimTrajectoryWithIndexFromSelfPose(
  const TrajectoryPoints & input, const geometry_msgs::msg::Pose & self_pose, size_t & index)
{
  TrajectoryPoints output{};

  double min_distance = 0.0;
  size_t min_distance_index = 0;
  bool is_init = false;
  for (size_t i = 0; i < input.size(); ++i) {
    const double x = input.at(i).pose.position.x - self_pose.position.x;
    const double y = input.at(i).pose.position.y - self_pose.position.y;
    const double squared_distance = x * x + y * y;
    if (!is_init || squared_distance < min_distance * min_distance) {
      is_init = true;
      min_distance = std::sqrt(squared_distance);
      min_distance_index = i;
    }
  }
  for (size_t i = min_distance_index; i < input.size(); ++i) {
    output.push_back(input.at(i));
  }
  index = min_distance_index;

  return output;
}

bool ObstacleStopPlannerNode::searchPointcloudNearTrajectory(
  const TrajectoryPoints & trajectory,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_points_ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_points_ptr,
  const std_msgs::msg::Header & trajectory_header)
{
  // transform pointcloud
  geometry_msgs::msg::TransformStamped transform_stamped{};
  try {
    transform_stamped = tf_buffer_.lookupTransform(
      trajectory_header.frame_id, input_points_ptr->header.frame_id, input_points_ptr->header.stamp,
      rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Failed to look up transform from " << trajectory_header.frame_id << " to "
                                                        << input_points_ptr->header.frame_id);
    return false;
  }

  sensor_msgs::msg::PointCloud2 transformed_points{};
  const Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(affine_matrix, *input_points_ptr, transformed_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(transformed_points, *transformed_points_ptr);

  output_points_ptr->header = transformed_points_ptr->header;

  // search obstacle candidate pointcloud to reduce calculation cost
  const double search_radius = node_param_.enable_slow_down
                                 ? slow_down_param_.slow_down_search_radius
                                 : stop_param_.stop_search_radius;
  const double squared_radius = search_radius * search_radius;
  for (const auto & trajectory_point : trajectory) {
    const auto center_pose = getVehicleCenterFromBase(trajectory_point.pose);
    for (const auto & point : transformed_points_ptr->points) {
      const double x = center_pose.position.x - point.x;
      const double y = center_pose.position.y - point.y;
      const double squared_distance = x * x + y * y;
      if (squared_distance < squared_radius) {
        output_points_ptr->points.push_back(point);
      }
    }
  }
  return true;
}

void ObstacleStopPlannerNode::createOneStepPolygon(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  std::vector<cv::Point2d> & polygon, const double expand_width)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;

  const auto & i = vehicle_info_;
  const auto & front_m = i.max_longitudinal_offset_m;
  const auto & width_m = i.vehicle_width_m / 2.0 + expand_width;
  const auto & back_m = i.rear_overhang_m;
  // start step
  {
    const auto yaw = getRPY(base_step_pose).z;
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
      base_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
      base_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
      base_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
      base_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
  }
  // next step
  {
    const auto yaw = getRPY(next_step_pose).z;
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
      next_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
      next_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
      next_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.push_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
      next_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
  }
  convexHull(one_step_move_vehicle_corner_points, polygon);
}

bool ObstacleStopPlannerNode::convexHull(
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
  for (size_t i = 0; i < pointcloud.size(); ++i) {
    normalized_pointcloud.push_back(cv::Point(
      (pointcloud.at(i).x - centroid.x) * 1000.0, (pointcloud.at(i).y - centroid.y) * 1000.0));
  }
  cv::convexHull(normalized_pointcloud, normalized_polygon_points);

  for (size_t i = 0; i < normalized_polygon_points.size(); ++i) {
    cv::Point2d polygon_point;
    polygon_point.x = (normalized_polygon_points.at(i).x / 1000.0 + centroid.x);
    polygon_point.y = (normalized_polygon_points.at(i).y / 1000.0 + centroid.y);
    polygon_points.push_back(polygon_point);
  }
  return true;
}

bool ObstacleStopPlannerNode::getSelfPose(
  const std_msgs::msg::Header & header, const tf2_ros::Buffer & tf_buffer,
  geometry_msgs::msg::Pose & self_pose)
{
  try {
    geometry_msgs::msg::TransformStamped transform;
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

void ObstacleStopPlannerNode::getNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
  pcl::PointXYZ * nearest_collision_point, rclcpp::Time * nearest_collision_point_time)
{
  double min_norm = 0.0;
  bool is_init = false;
  const auto yaw = getRPY(base_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));

  for (size_t i = 0; i < pointcloud.size(); ++i) {
    const Eigen::Vector2d pointcloud_vec(
      pointcloud.at(i).x - base_pose.position.x, pointcloud.at(i).y - base_pose.position.y);
    double norm = base_pose_vec.dot(pointcloud_vec);
    if (norm < min_norm || !is_init) {
      min_norm = norm;
      *nearest_collision_point = pointcloud.at(i);
      *nearest_collision_point_time = pcl_conversions::fromPCL(pointcloud.header).stamp;
      is_init = true;
    }
  }
}

void ObstacleStopPlannerNode::getLateralNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const geometry_msgs::msg::Pose & base_pose,
  pcl::PointXYZ * lateral_nearest_point, double * deviation)
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

geometry_msgs::msg::Pose ObstacleStopPlannerNode::getVehicleCenterFromBase(
  const geometry_msgs::msg::Pose & base_pose)
{
  const auto & i = vehicle_info_;
  const auto yaw = getRPY(base_pose).z;

  geometry_msgs::msg::Pose center_pose;
  center_pose.position.x =
    base_pose.position.x + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::cos(yaw);
  center_pose.position.y =
    base_pose.position.y + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::sin(yaw);
  center_pose.position.z = base_pose.position.z;
  center_pose.orientation = base_pose.orientation;
  return center_pose;
}

void ObstacleStopPlannerNode::setExternalVelocityLimit()
{
  const auto & p = slow_down_param_;
  auto slow_down_limit_vel = std::make_shared<VelocityLimit>();
  slow_down_limit_vel->stamp = this->now();
  slow_down_limit_vel->max_velocity = p.slow_down_vel;
  slow_down_limit_vel->constraints.min_acceleration =
    p.slow_down_min_jerk < p.normal_min_jerk ? p.limit_min_acc : p.normal_min_acc;
  slow_down_limit_vel->constraints.max_jerk = std::abs(p.slow_down_min_jerk);
  slow_down_limit_vel->constraints.min_jerk = p.slow_down_min_jerk;
  slow_down_limit_vel->use_constraints = true;
  slow_down_limit_vel->sender = "obstacle_stop_planner";

  pub_velocity_limit_->publish(*slow_down_limit_vel);
  set_velocity_limit_ = true;

  RCLCPP_INFO(
    get_logger(), "set velocity limit. jerk:%-6.2f dec:%-6.2f",
    slow_down_limit_vel->constraints.min_jerk, slow_down_limit_vel->constraints.min_acceleration);
}

void ObstacleStopPlannerNode::resetExternalVelocityLimit()
{
  const auto current_vel = current_velocity_ptr_->twist.twist.linear.x;
  const auto reach_target_vel =
    current_vel <
    slow_down_param_.slow_down_vel + slow_down_param_.vel_threshold_reset_velocity_limit_;
  const auto constant_vel =
    std::abs(current_acc_) < slow_down_param_.dec_threshold_reset_velocity_limit_;
  const auto no_undershoot = reach_target_vel && constant_vel;

  if (!no_undershoot) {
    return;
  }

  auto velocity_limit_clear_command = std::make_shared<VelocityLimitClearCommand>();
  velocity_limit_clear_command->stamp = this->now();
  velocity_limit_clear_command->command = true;
  velocity_limit_clear_command->sender = "obstacle_stop_planner";

  pub_clear_velocity_limit_->publish(*velocity_limit_clear_command);
  set_velocity_limit_ = false;

  RCLCPP_INFO(get_logger(), "reset velocity limit");
}

void ObstacleStopPlannerNode::publishDebugData(const PlannerData & planner_data)
{
  const auto & current_vel = current_velocity_ptr_->twist.twist.linear.x;
  debug_ptr_->setDebugValues(DebugValues::TYPE::CURRENT_VEL, current_vel);
  debug_ptr_->setDebugValues(DebugValues::TYPE::CURRENT_ACC, current_acc_);
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::FLAG_FIND_SLOW_DOWN_OBSTACLE, planner_data.slow_down_require);
  debug_ptr_->setDebugValues(
    DebugValues::TYPE::FLAG_FIND_COLLISION_OBSTACLE, planner_data.stop_require);
  debug_ptr_->setDebugValues(DebugValues::TYPE::FLAG_EXTERNAL, set_velocity_limit_);

  const auto now_adaptive_cruise =
    !planner_data.stop_require && planner_data.found_collision_points;
  debug_ptr_->setDebugValues(DebugValues::TYPE::FLAG_ADAPTIVE_CRUISE, now_adaptive_cruise);

  debug_ptr_->publish();
}

}  // namespace motion_planning

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(motion_planning::ObstacleStopPlannerNode)
