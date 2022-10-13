// Copyright 2021 - 2022 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "control_performance_analysis/msg/driving_monitor_stamped.hpp"
#include "control_performance_analysis/msg/error_stamped.hpp"

#include <vehicle_info_util/vehicle_info_util.hpp>

#include <memory>
#include <utility>

namespace
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using control_performance_analysis::msg::DrivingMonitorStamped;
using control_performance_analysis::msg::ErrorStamped;

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
  const auto & vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
  param_.wheelbase_ = vehicle_info.wheel_base_m;

  // Node Parameters.
  param_.curvature_interval_length_ = declare_parameter("curvature_interval_length", 10.0);
  param_.prevent_zero_division_value_ = declare_parameter("prevent_zero_division_value", 0.001);
  param_.odom_interval_ = declare_parameter("odom_interval", 2);
  param_.acceptable_max_distance_to_waypoint_ =
    declare_parameter("acceptable_max_distance_to_waypoint", 1.5);
  param_.acceptable_max_yaw_difference_rad_ =
    declare_parameter("acceptable_max_yaw_difference_rad", 1.0472);
  param_.lpf_gain_ = declare_parameter("low_pass_filter_gain", 0.8);

  // Prepare error computation class with the wheelbase parameter.
  control_performance_core_ptr_ = std::make_unique<ControlPerformanceAnalysisCore>(param_);

  // Subscribers.
  sub_trajectory_ = create_subscription<Trajectory>(
    "~/input/reference_trajectory", 1,
    std::bind(&ControlPerformanceAnalysisNode::onTrajectory, this, _1));

  sub_control_cmd_ = create_subscription<AckermannControlCommand>(
    "~/input/control_raw", 1, std::bind(&ControlPerformanceAnalysisNode::onControlRaw, this, _1));

  sub_vehicle_steering_ = create_subscription<SteeringReport>(
    "~/input/measured_steering", 1,
    std::bind(&ControlPerformanceAnalysisNode::onVecSteeringMeasured, this, _1));

  sub_velocity_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&ControlPerformanceAnalysisNode::onVelocity, this, _1));

  // Publishers
  pub_error_msg_ = create_publisher<ErrorStamped>("~/output/error_stamped", 1);

  pub_driving_msg_ = create_publisher<DrivingMonitorStamped>("~/output/driving_status_stamped", 1);

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
  const AckermannControlCommand::ConstSharedPtr control_msg)
{
  static bool initialized = false;
  if (!control_msg) {
    RCLCPP_ERROR(get_logger(), "control command has not been received yet ...");

    return;
  } else if (!initialized) {
    initialized = true;
    current_control_msg_ptr_ = control_msg;
    last_control_cmd_.stamp = current_control_msg_ptr_->stamp;

  } else {
    current_control_msg_ptr_ = control_msg;
    const rclcpp::Duration & duration =
      (rclcpp::Time(current_control_msg_ptr_->stamp) - rclcpp::Time(last_control_cmd_.stamp));
    d_control_cmd_ = duration.seconds() * 1000;  // ms
    last_control_cmd_.stamp = current_control_msg_ptr_->stamp;
  }
}

void ControlPerformanceAnalysisNode::onVecSteeringMeasured(
  const SteeringReport::ConstSharedPtr meas_steer_msg)
{
  if (!meas_steer_msg) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for vehicle measured steering message ...");
    return;
  }
  current_vec_steering_msg_ptr_ = meas_steer_msg;
}

void ControlPerformanceAnalysisNode::onVelocity(const Odometry::ConstSharedPtr msg)
{
  // Sent previous state to error calculation because we need to calculate current acceleration.

  if (!current_odom_ptr_) {
    if (isDataReady()) {
      current_odom_ptr_ = msg;
      current_pose_ = self_pose_listener_.getCurrentPose();
      control_performance_core_ptr_->setOdomHistory(*current_odom_ptr_);
      prev_traj = *current_trajectory_ptr_;
      prev_cmd = *current_control_msg_ptr_;
      prev_steering = *current_vec_steering_msg_ptr_;
    }
    return;
  }

  control_performance_core_ptr_->setCurrentWaypoints(prev_traj);
  control_performance_core_ptr_->setCurrentPose(current_pose_->pose);
  control_performance_core_ptr_->setCurrentControlValue(prev_cmd);
  control_performance_core_ptr_->setOdomHistory(*msg);  // k+1, k, k-1
  control_performance_core_ptr_->setSteeringStatus(prev_steering);

  if (!control_performance_core_ptr_->isDataReady()) {
    return;
  }
  // Find the index of the next waypoint.
  const std::pair<bool, int32_t> & prev_closest_wp_pose_idx =
    control_performance_core_ptr_->findClosestPrevWayPointIdx_path_direction();

  if (!prev_closest_wp_pose_idx.first) {
    RCLCPP_ERROR(get_logger(), "Cannot find closest waypoint");
    return;
  }

  // Compute control performance values.
  if (control_performance_core_ptr_->calculateErrorVars()) {
    pub_error_msg_->publish(control_performance_core_ptr_->error_vars);
  } else {
    RCLCPP_ERROR(get_logger(), "Cannot compute error vars ...");
  }
  if (control_performance_core_ptr_->calculateDrivingVars()) {
    control_performance_core_ptr_->driving_status_vars.controller_processing_time.header.stamp =
      current_control_msg_ptr_->stamp;
    control_performance_core_ptr_->driving_status_vars.controller_processing_time.data =
      d_control_cmd_;
    pub_driving_msg_->publish(control_performance_core_ptr_->driving_status_vars);
  } else {
    RCLCPP_ERROR(get_logger(), "Cannot compute driving vars ...");
  }
  prev_traj = *current_trajectory_ptr_;
  prev_cmd = *current_control_msg_ptr_;
  prev_steering = *current_vec_steering_msg_ptr_;
  current_odom_ptr_ = msg;
  current_pose_ = self_pose_listener_.getCurrentPose();
}

bool ControlPerformanceAnalysisNode::isDataReady() const
{
  rclcpp::Clock clock{RCL_ROS_TIME};

  if (!current_trajectory_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for trajectory ... ");
    return false;
  }

  if (!current_control_msg_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "waiting for current_control_cmd ...");
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
