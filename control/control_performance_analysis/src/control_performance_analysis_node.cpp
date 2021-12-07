// Copyright 2021 Tier IV, Inc.
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

#include "control_performance_analysis/control_performance_analysis_node.hpp"

#include "control_performance_analysis/msg/error_stamped.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

#include <memory>
#include <utility>

namespace
{
using control_performance_analysis::TargetPerformanceMsgVars;
using control_performance_analysis::msg::ErrorStamped;

ErrorStamped createPerformanceMsgVars(const TargetPerformanceMsgVars & target_performance_vars)
{
  ErrorStamped error_msgs{};

  error_msgs.error.lateral_error = target_performance_vars.lateral_error;
  error_msgs.error.heading_error = target_performance_vars.heading_error;
  error_msgs.error.control_effort_energy = target_performance_vars.control_effort_energy;
  error_msgs.error.error_energy = target_performance_vars.error_energy;
  error_msgs.error.value_approximation = target_performance_vars.value_approximation;
  error_msgs.error.curvature_estimate = target_performance_vars.curvature_estimate;
  error_msgs.error.curvature_estimate_pp = target_performance_vars.curvature_estimate_pp;
  error_msgs.error.lateral_error_velocity = target_performance_vars.lateral_error_velocity;
  error_msgs.error.lateral_error_acceleration = target_performance_vars.lateral_error_acceleration;

  return error_msgs;
}
}  // namespace

namespace control_performance_analysis
{
using vehicle_info_util::VehicleInfoUtil;

ControlPerformanceAnalysisNode::ControlPerformanceAnalysisNode(
  const rclcpp::NodeOptions & node_options)
: Node("control_performance_analysis", node_options)
{
  using std::placeholders::_1;

  // Implement Reading Global and Local Variables.
  const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;

  // Node Parameters.
  param_.control_period = declare_parameter("control_period", 0.033);
  param_.curvature_interval_length = declare_parameter("curvature_interval_length", 10.0);

  // Prepare error computation class with the wheelbase parameter.
  control_performance_core_ptr_ = std::make_unique<ControlPerformanceAnalysisCore>(
    param_.wheel_base, param_.curvature_interval_length);

  // Subscribers.
  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/reference_trajectory", 1,
    std::bind(&ControlPerformanceAnalysisNode::onTrajectory, this, _1));

  sub_control_steering_ = create_subscription<AckermannLateralCommand>(
    "~/input/control_raw", 1, std::bind(&ControlPerformanceAnalysisNode::onControlRaw, this, _1));

  sub_vehicle_steering_ = create_subscription<SteeringReport>(
    "~/input/measured_steering", 1,
    std::bind(&ControlPerformanceAnalysisNode::onVecSteeringMeasured, this, _1));

  sub_velocity_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&ControlPerformanceAnalysisNode::onVelocity, this, _1));

  // Publishers
  pub_error_msg_ = create_publisher<ErrorStamped>("~/output/error_stamped", 1);

  // Timer
  {
    auto on_timer = std::bind(&ControlPerformanceAnalysisNode::onTimer, this);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(param_.control_period));
    timer_publish_ = std::make_shared<rclcpp::GenericTimer<decltype(on_timer)>>(
      this->get_clock(), period, std::move(on_timer),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_publish_, nullptr);
  }

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();
}

void ControlPerformanceAnalysisNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  if (msg->points.size() < 3) {
    RCLCPP_DEBUG(get_logger(), "received path size < 3, is not sufficient.");
    return;
  }

  if (!isValidTrajectory(*msg)) {
    RCLCPP_ERROR(get_logger(), "Trajectory is invalid!, stop computing.");
    return;
  }

  current_trajectory_ptr_ = msg;
}

void ControlPerformanceAnalysisNode::onControlRaw(
  const AckermannLateralCommand::ConstSharedPtr control_msg)
{
  if (!control_msg) {
    RCLCPP_ERROR(get_logger(), "steering signal has not been received yet ...");
    return;
  }
  current_control_msg_ptr_ = control_msg;
}

void ControlPerformanceAnalysisNode::onVecSteeringMeasured(
  const SteeringReport::ConstSharedPtr meas_steer_msg)
{
  if (!meas_steer_msg) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "waiting for vehicle measured steering message ...");
    return;
  }
  current_vec_steering_msg_ptr_ = meas_steer_msg;
}

void ControlPerformanceAnalysisNode::onVelocity(const Odometry::ConstSharedPtr msg)
{
  current_odom_ptr_ = msg;
}

void ControlPerformanceAnalysisNode::onTimer()
{
  // Read and Update Current Pose updating  var:current_pose_.
  current_pose_ = self_pose_listener_.getCurrentPose();

  // Check Data Stream
  if (!isDataReady()) {
    // Publish Here
    return;
  }

  // Compute Control Performance Variables.
  auto performanceVars = computeTargetPerformanceMsgVars();
  if (!performanceVars) {
    RCLCPP_ERROR(get_logger(), "steering signal has not been received yet ...");
    return;
  }

  // If successful publish.
  publishErrorMsg(*performanceVars);
}

void ControlPerformanceAnalysisNode::publishErrorMsg(
  const TargetPerformanceMsgVars & control_performance_vars)
{
  control_performance_analysis::ErrorStamped error_msgs =
    createPerformanceMsgVars(control_performance_vars);

  pub_error_msg_->publish(error_msgs);
}

bool ControlPerformanceAnalysisNode::isDataReady() const
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  if (!current_pose_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for current_pose ...");
    return false;
  }

  if (!current_trajectory_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for trajectory ... ");
    return false;
  }

  if (!current_odom_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for current_odom ...");
    return false;
  }

  if (!current_control_msg_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for current_control_steering_val ...");
    return false;
  }

  return true;
}

/*
 *  - Pass trajectory and current pose to control_performance_analysis -> setCurrentPose()
 *                                                               -> setWayPoints()
 *                                                               -> findClosestPoint
 *                                                               -> computePerformanceVars
 * */

boost::optional<TargetPerformanceMsgVars>
ControlPerformanceAnalysisNode::computeTargetPerformanceMsgVars() const
{
  // Set trajectory and current pose of controller_performance_core.
  control_performance_core_ptr_->setCurrentWaypoints(*current_trajectory_ptr_);
  control_performance_core_ptr_->setCurrentPose(current_pose_->pose);
  control_performance_core_ptr_->setCurrentVelocities(current_odom_ptr_->twist.twist);
  control_performance_core_ptr_->setCurrentControlValue(*current_control_msg_ptr_);

  // Find the index of the next waypoint.
  std::pair<bool, int32_t> prev_closest_wp_pose_idx =
    control_performance_core_ptr_->findClosestPrevWayPointIdx_path_direction();

  if (!prev_closest_wp_pose_idx.first) {
    RCLCPP_ERROR(get_logger(), "Cannot find closest waypoint");
    return {};
  }

  // Compute control performance values.
  const std::pair<bool, TargetPerformanceMsgVars> target_performance_vars =
    control_performance_core_ptr_->getPerformanceVars();

  if (!target_performance_vars.first) {
    RCLCPP_ERROR(get_logger(), "Cannot compute control performance vars ...");
    return {};
  }

  return target_performance_vars.second;
}

bool ControlPerformanceAnalysisNode::isValidTrajectory(const Trajectory & traj)
{
  bool check_condition = std::all_of(traj.points.cbegin(), traj.points.cend(), [](auto point) {
    const auto & p = point.pose.position;
    const auto & o = point.pose.orientation;
    const auto & t = point.longitudinal_velocity_mps;
    const auto & a = point.acceleration_mps2;

    if (
      !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) || !isfinite(o.x) || !isfinite(o.y) ||
      !isfinite(o.z) || !isfinite(o.w) || !isfinite(t) || !isfinite(a)) {
      return false;
    } else {
      return true;
    }
  });

  return check_condition;
}
}  // namespace control_performance_analysis

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(control_performance_analysis::ControlPerformanceAnalysisNode)
