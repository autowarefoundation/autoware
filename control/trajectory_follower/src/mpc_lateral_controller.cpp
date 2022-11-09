// Copyright 2021 The Autoware Foundation
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

#include "trajectory_follower/mpc_lateral_controller.hpp"

#include "motion_utils/motion_utils.hpp"
#include "tf2_ros/create_timer_ros.h"

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
namespace
{
using namespace std::literals::chrono_literals;

template <typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != parameters.cend()) {
    value = static_cast<T>(it->template get_value<T>());
  }
}
}  // namespace

MpcLateralController::MpcLateralController(rclcpp::Node & node) : node_{&node}
{
  using std::placeholders::_1;

  m_mpc.m_ctrl_period = node_->get_parameter("ctrl_period").as_double();
  m_enable_path_smoothing = node_->declare_parameter<bool>("enable_path_smoothing");
  m_path_filter_moving_ave_num = node_->declare_parameter<int>("path_filter_moving_ave_num");
  m_curvature_smoothing_num_traj = node_->declare_parameter<int>("curvature_smoothing_num_traj");
  m_curvature_smoothing_num_ref_steer =
    node_->declare_parameter<int>("curvature_smoothing_num_ref_steer");
  m_traj_resample_dist = node_->declare_parameter<double>("traj_resample_dist");
  m_mpc.m_admissible_position_error = node_->declare_parameter<double>("admissible_position_error");
  m_mpc.m_admissible_yaw_error_rad = node_->declare_parameter<double>("admissible_yaw_error_rad");
  m_mpc.m_use_steer_prediction = node_->declare_parameter<bool>("use_steer_prediction");
  m_mpc.m_param.steer_tau = node_->declare_parameter<double>("vehicle_model_steer_tau");

  /* stop state parameters */
  m_stop_state_entry_ego_speed = node_->declare_parameter<double>("stop_state_entry_ego_speed");
  m_stop_state_entry_target_speed =
    node_->declare_parameter<double>("stop_state_entry_target_speed");
  m_converged_steer_rad = node_->declare_parameter<double>("converged_steer_rad");
  m_keep_steer_control_until_converged =
    node_->declare_parameter<bool>("keep_steer_control_until_converged");
  m_new_traj_duration_time = node_->declare_parameter<double>("new_traj_duration_time");  // [s]
  m_new_traj_end_dist = node_->declare_parameter<double>("new_traj_end_dist");            // [m]

  /* mpc parameters */
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*node_).getVehicleInfo();
  const double wheelbase = vehicle_info.wheel_base_m;
  const double steer_rate_lim_dps = node_->declare_parameter<double>("steer_rate_lim_dps");
  constexpr double deg2rad = static_cast<double>(M_PI) / 180.0;
  m_mpc.m_steer_lim = vehicle_info.max_steer_angle_rad;
  m_mpc.m_steer_rate_lim = steer_rate_lim_dps * deg2rad;

  /* vehicle model setup */
  const std::string vehicle_model_type =
    node_->declare_parameter<std::string>("vehicle_model_type");
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr;
  if (vehicle_model_type == "kinematics") {
    vehicle_model_ptr = std::make_shared<trajectory_follower::KinematicsBicycleModel>(
      wheelbase, m_mpc.m_steer_lim, m_mpc.m_param.steer_tau);
  } else if (vehicle_model_type == "kinematics_no_delay") {
    vehicle_model_ptr = std::make_shared<trajectory_follower::KinematicsBicycleModelNoDelay>(
      wheelbase, m_mpc.m_steer_lim);
  } else if (vehicle_model_type == "dynamics") {
    const double mass_fl = node_->declare_parameter<double>("vehicle.mass_fl");
    const double mass_fr = node_->declare_parameter<double>("vehicle.mass_fr");
    const double mass_rl = node_->declare_parameter<double>("vehicle.mass_rl");
    const double mass_rr = node_->declare_parameter<double>("vehicle.mass_rr");
    const double cf = node_->declare_parameter<double>("vehicle.cf");
    const double cr = node_->declare_parameter<double>("vehicle.cr");

    // vehicle_model_ptr is only assigned in ctor, so parameter value have to be passed at init time
    // // NOLINT
    vehicle_model_ptr = std::make_shared<trajectory_follower::DynamicsBicycleModel>(
      wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "vehicle_model_type is undefined");
  }

  /* QP solver setup */
  const std::string qp_solver_type = node_->declare_parameter<std::string>("qp_solver_type");
  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr;
  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr = std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  } else if (qp_solver_type == "osqp") {
    qpsolver_ptr = std::make_shared<trajectory_follower::QPSolverOSQP>(node_->get_logger());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "qp_solver_type is undefined");
  }

  /* delay compensation */
  {
    const double delay_tmp = node_->declare_parameter<double>("input_delay");
    const double delay_step = std::round(delay_tmp / m_mpc.m_ctrl_period);
    m_mpc.m_param.input_delay = delay_step * m_mpc.m_ctrl_period;
    m_mpc.m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
  }

  /* initialize lowpass filter */
  {
    const double steering_lpf_cutoff_hz =
      node_->declare_parameter<double>("steering_lpf_cutoff_hz");
    const double error_deriv_lpf_cutoff_hz =
      node_->declare_parameter<double>("error_deriv_lpf_cutoff_hz");
    m_mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
  }

  // ego nearest index search
  m_ego_nearest_dist_threshold =
    node_->has_parameter("ego_nearest_dist_threshold")
      ? node_->get_parameter("ego_nearest_dist_threshold").as_double()
      : node_->declare_parameter<double>("ego_nearest_dist_threshold");  // [m]
  m_ego_nearest_yaw_threshold =
    node_->has_parameter("ego_nearest_yaw_threshold")
      ? node_->get_parameter("ego_nearest_yaw_threshold").as_double()
      : node_->declare_parameter<double>("ego_nearest_yaw_threshold");  // [rad]
  m_mpc.ego_nearest_dist_threshold = m_ego_nearest_dist_threshold;
  m_mpc.ego_nearest_yaw_threshold = m_ego_nearest_yaw_threshold;

  m_pub_predicted_traj = node_->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/output/predicted_trajectory", 1);
  m_pub_debug_values = node_->create_publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>(
    "~/output/lateral_diagnostic", 1);

  // TODO(Frederik.Beaujean) ctor is too long, should factor out parameter declarations
  declareMPCparameters();

  /* get parameter updates */
  m_set_param_res = node_->add_on_set_parameters_callback(
    std::bind(&MpcLateralController::paramCallback, this, _1));

  m_mpc.setQPSolver(qpsolver_ptr);
  m_mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);

  m_mpc.setLogger(node_->get_logger());
  m_mpc.setClock(node_->get_clock());
}

MpcLateralController::~MpcLateralController() {}

boost::optional<LateralOutput> MpcLateralController::run()
{
  if (!checkData()) {
    return boost::none;
  }

  autoware_auto_control_msgs::msg::AckermannLateralCommand ctrl_cmd;
  autoware_auto_planning_msgs::msg::Trajectory predicted_traj;
  tier4_debug_msgs::msg::Float32MultiArrayStamped debug_values;

  if (!m_is_ctrl_cmd_prev_initialized) {
    m_ctrl_cmd_prev = getInitialControlCommand();
    m_is_ctrl_cmd_prev_initialized = true;
  }

  const bool is_mpc_solved = m_mpc.calculateMPC(
    *m_current_steering_ptr, m_current_kinematic_state_ptr->twist.twist.linear.x,
    m_current_kinematic_state_ptr->pose.pose, ctrl_cmd, predicted_traj, debug_values);

  publishPredictedTraj(predicted_traj);
  publishDebugValues(debug_values);

  const auto createLateralOutput = [this](const auto & cmd) {
    LateralOutput output;
    output.control_cmd = createCtrlCmdMsg(cmd);
    output.sync_data.is_steer_converged = isSteerConverged(cmd);
    return boost::optional<LateralOutput>(output);
  };

  if (isStoppedState()) {
    // Reset input buffer
    for (auto & value : m_mpc.m_input_buffer) {
      value = m_ctrl_cmd_prev.steering_tire_angle;
    }
    // Use previous command value as previous raw steer command
    m_mpc.m_raw_steer_cmd_prev = m_ctrl_cmd_prev.steering_tire_angle;
    return createLateralOutput(m_ctrl_cmd_prev);
  }

  if (!is_mpc_solved) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000 /*ms*/,
      "MPC is not solved. publish 0 velocity.");
    ctrl_cmd = getStopControlCommand();
  }

  m_ctrl_cmd_prev = ctrl_cmd;
  return createLateralOutput(ctrl_cmd);
}

void MpcLateralController::setInputData(InputData const & input_data)
{
  setTrajectory(input_data.current_trajectory_ptr);
  m_current_kinematic_state_ptr = input_data.current_odometry_ptr;
  m_current_steering_ptr = input_data.current_steering_ptr;
}

bool MpcLateralController::isSteerConverged(
  const autoware_auto_control_msgs::msg::AckermannLateralCommand & cmd) const
{
  // wait for a while to propagate the trajectory shape to the output command when the trajectory
  // shape is changed.
  if (!m_has_received_first_trajectory || isTrajectoryShapeChanged()) {
    return false;
  }

  const bool is_converged =
    std::abs(cmd.steering_tire_angle - m_current_steering_ptr->steering_tire_angle) <
    static_cast<float>(m_converged_steer_rad);

  return is_converged;
}

bool MpcLateralController::checkData() const
{
  if (!m_mpc.hasVehicleModel()) {
    RCLCPP_DEBUG(node_->get_logger(), "MPC does not have a vehicle model");
    return false;
  }
  if (!m_mpc.hasQPSolver()) {
    RCLCPP_DEBUG(node_->get_logger(), "MPC does not have a QP solver");
    return false;
  }

  if (!m_current_kinematic_state_ptr) {
    RCLCPP_DEBUG(
      node_->get_logger(), "waiting data. kinematic_state = %d",
      m_current_kinematic_state_ptr != nullptr);
    return false;
  }

  if (!m_current_steering_ptr) {
    RCLCPP_DEBUG(
      node_->get_logger(), "waiting data. current_steering = %d",
      m_current_steering_ptr != nullptr);
    return false;
  }

  if (m_mpc.m_ref_traj.size() == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "trajectory size is zero.");
    return false;
  }

  return true;
}

void MpcLateralController::setTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  if (!msg) return;

  m_current_trajectory_ptr = msg;

  if (!m_current_kinematic_state_ptr) {
    RCLCPP_DEBUG(node_->get_logger(), "Current kinematic state is not received yet.");
    return;
  }

  if (msg->points.size() < 3) {
    RCLCPP_DEBUG(node_->get_logger(), "received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(*msg)) {
    RCLCPP_ERROR(node_->get_logger(), "Trajectory is invalid!! stop computing.");
    return;
  }

  m_mpc.setReferenceTrajectory(
    *msg, m_traj_resample_dist, m_enable_path_smoothing, m_path_filter_moving_ave_num,
    m_curvature_smoothing_num_traj, m_curvature_smoothing_num_ref_steer);

  // update trajectory buffer to check the trajectory shape change.
  m_trajectory_buffer.push_back(*m_current_trajectory_ptr);
  while (rclcpp::ok()) {
    const auto time_diff = rclcpp::Time(m_trajectory_buffer.back().header.stamp) -
                           rclcpp::Time(m_trajectory_buffer.front().header.stamp);

    const double first_trajectory_duration_time = 5.0;
    const double duration_time =
      m_has_received_first_trajectory ? m_new_traj_duration_time : first_trajectory_duration_time;
    if (time_diff.seconds() < duration_time) {
      m_has_received_first_trajectory = true;
      break;
    }
    m_trajectory_buffer.pop_front();
  }
}

autoware_auto_control_msgs::msg::AckermannLateralCommand
MpcLateralController::getStopControlCommand() const
{
  autoware_auto_control_msgs::msg::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = static_cast<decltype(cmd.steering_tire_angle)>(m_steer_cmd_prev);
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

autoware_auto_control_msgs::msg::AckermannLateralCommand
MpcLateralController::getInitialControlCommand() const
{
  autoware_auto_control_msgs::msg::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = m_current_steering_ptr->steering_tire_angle;
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

bool MpcLateralController::isStoppedState() const
{
  // If the nearest index is not found, return false
  if (m_current_trajectory_ptr->points.empty()) {
    return false;
  }

  // Note: This function used to take into account the distance to the stop line
  // for the stop state judgement. However, it has been removed since the steering
  // control was turned off when approaching/exceeding the stop line on a curve or
  // emergency stop situation and it caused large tracking error.
  const size_t nearest = motion_utils::findFirstNearestIndexWithSoftConstraints(
    m_current_trajectory_ptr->points, m_current_kinematic_state_ptr->pose.pose,
    m_ego_nearest_dist_threshold, m_ego_nearest_yaw_threshold);

  const double current_vel = m_current_kinematic_state_ptr->twist.twist.linear.x;
  const double target_vel =
    m_current_trajectory_ptr->points.at(static_cast<size_t>(nearest)).longitudinal_velocity_mps;

  const auto latest_published_cmd = m_ctrl_cmd_prev;  // use prev_cmd as a latest published command
  if (m_keep_steer_control_until_converged && !isSteerConverged(latest_published_cmd)) {
    return false;  // not stopState: keep control
  }

  if (
    std::fabs(current_vel) < m_stop_state_entry_ego_speed &&
    std::fabs(target_vel) < m_stop_state_entry_target_speed) {
    return true;
  } else {
    return false;
  }
}

autoware_auto_control_msgs::msg::AckermannLateralCommand MpcLateralController::createCtrlCmdMsg(
  autoware_auto_control_msgs::msg::AckermannLateralCommand ctrl_cmd)
{
  ctrl_cmd.stamp = node_->now();
  m_steer_cmd_prev = ctrl_cmd.steering_tire_angle;
  return ctrl_cmd;
}

void MpcLateralController::publishPredictedTraj(
  autoware_auto_planning_msgs::msg::Trajectory & predicted_traj) const
{
  predicted_traj.header.stamp = node_->now();
  predicted_traj.header.frame_id = m_current_trajectory_ptr->header.frame_id;
  m_pub_predicted_traj->publish(predicted_traj);
}

void MpcLateralController::publishDebugValues(
  tier4_debug_msgs::msg::Float32MultiArrayStamped & debug_values) const
{
  debug_values.stamp = node_->now();
  m_pub_debug_values->publish(debug_values);
}

void MpcLateralController::declareMPCparameters()
{
  m_mpc.m_param.prediction_horizon = node_->declare_parameter<int>("mpc_prediction_horizon");
  m_mpc.m_param.prediction_dt = node_->declare_parameter<double>("mpc_prediction_dt");
  m_mpc.m_param.weight_lat_error = node_->declare_parameter<double>("mpc_weight_lat_error");
  m_mpc.m_param.weight_heading_error = node_->declare_parameter<double>("mpc_weight_heading_error");
  m_mpc.m_param.weight_heading_error_squared_vel =
    node_->declare_parameter<double>("mpc_weight_heading_error_squared_vel");
  m_mpc.m_param.weight_steering_input =
    node_->declare_parameter<double>("mpc_weight_steering_input");
  m_mpc.m_param.weight_steering_input_squared_vel =
    node_->declare_parameter<double>("mpc_weight_steering_input_squared_vel");
  m_mpc.m_param.weight_lat_jerk = node_->declare_parameter<double>("mpc_weight_lat_jerk");
  m_mpc.m_param.weight_steer_rate = node_->declare_parameter<double>("mpc_weight_steer_rate");
  m_mpc.m_param.weight_steer_acc = node_->declare_parameter<double>("mpc_weight_steer_acc");
  m_mpc.m_param.low_curvature_weight_lat_error =
    node_->declare_parameter<double>("mpc_low_curvature_weight_lat_error");
  m_mpc.m_param.low_curvature_weight_heading_error =
    node_->declare_parameter<double>("mpc_low_curvature_weight_heading_error");
  m_mpc.m_param.low_curvature_weight_heading_error_squared_vel =
    node_->declare_parameter<double>("mpc_low_curvature_weight_heading_error_squared_vel");
  m_mpc.m_param.low_curvature_weight_steering_input =
    node_->declare_parameter<double>("mpc_low_curvature_weight_steering_input");
  m_mpc.m_param.low_curvature_weight_steering_input_squared_vel =
    node_->declare_parameter<double>("mpc_low_curvature_weight_steering_input_squared_vel");
  m_mpc.m_param.low_curvature_weight_lat_jerk =
    node_->declare_parameter<double>("mpc_low_curvature_weight_lat_jerk");
  m_mpc.m_param.low_curvature_weight_steer_rate =
    node_->declare_parameter<double>("mpc_low_curvature_weight_steer_rate");
  m_mpc.m_param.low_curvature_weight_steer_acc =
    node_->declare_parameter<double>("mpc_low_curvature_weight_steer_acc");
  m_mpc.m_param.low_curvature_thresh_curvature =
    node_->declare_parameter<double>("mpc_low_curvature_thresh_curvature");
  m_mpc.m_param.weight_terminal_lat_error =
    node_->declare_parameter<double>("mpc_weight_terminal_lat_error");
  m_mpc.m_param.weight_terminal_heading_error =
    node_->declare_parameter<double>("mpc_weight_terminal_heading_error");
  m_mpc.m_param.zero_ff_steer_deg = node_->declare_parameter<double>("mpc_zero_ff_steer_deg");
  m_mpc.m_param.acceleration_limit = node_->declare_parameter<double>("mpc_acceleration_limit");
  m_mpc.m_param.velocity_time_constant =
    node_->declare_parameter<double>("mpc_velocity_time_constant");
  m_mpc.m_param.min_prediction_length =
    node_->declare_parameter<double>("mpc_min_prediction_length");
}

rcl_interfaces::msg::SetParametersResult MpcLateralController::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  trajectory_follower::MPCParam param = m_mpc.m_param;
  try {
    update_param(parameters, "mpc_prediction_horizon", param.prediction_horizon);
    update_param(parameters, "mpc_prediction_dt", param.prediction_dt);
    update_param(parameters, "mpc_weight_lat_error", param.weight_lat_error);
    update_param(parameters, "mpc_weight_heading_error", param.weight_heading_error);
    update_param(
      parameters, "mpc_weight_heading_error_squared_vel", param.weight_heading_error_squared_vel);
    update_param(parameters, "mpc_weight_steering_input", param.weight_steering_input);
    update_param(
      parameters, "mpc_weight_steering_input_squared_vel", param.weight_steering_input_squared_vel);
    update_param(parameters, "mpc_weight_lat_jerk", param.weight_lat_jerk);
    update_param(parameters, "mpc_weight_steer_rate", param.weight_steer_rate);
    update_param(parameters, "mpc_weight_steer_acc", param.weight_steer_acc);
    update_param(
      parameters, "mpc_low_curvature_weight_lat_error", param.low_curvature_weight_lat_error);
    update_param(
      parameters, "mpc_low_curvature_weight_heading_error",
      param.low_curvature_weight_heading_error);
    update_param(
      parameters, "mpc_low_curvature_weight_heading_error_squared_vel",
      param.low_curvature_weight_heading_error_squared_vel);
    update_param(
      parameters, "mpc_low_curvature_weight_steering_input",
      param.low_curvature_weight_steering_input);
    update_param(
      parameters, "mpc_low_curvature_weight_steering_input_squared_vel",
      param.low_curvature_weight_steering_input_squared_vel);
    update_param(
      parameters, "mpc_low_curvature_weight_lat_jerk", param.low_curvature_weight_lat_jerk);
    update_param(
      parameters, "mpc_low_curvature_weight_steer_rate", param.low_curvature_weight_steer_rate);
    update_param(
      parameters, "mpc_low_curvature_weight_steer_acc", param.low_curvature_weight_steer_acc);
    update_param(
      parameters, "mpc_low_curvature_thresh_curvature", param.low_curvature_thresh_curvature);
    update_param(parameters, "mpc_weight_terminal_lat_error", param.weight_terminal_lat_error);
    update_param(
      parameters, "mpc_weight_terminal_heading_error", param.weight_terminal_heading_error);
    update_param(parameters, "mpc_zero_ff_steer_deg", param.zero_ff_steer_deg);
    update_param(parameters, "mpc_acceleration_limit", param.acceleration_limit);
    update_param(parameters, "mpc_velocity_time_constant", param.velocity_time_constant);
    update_param(parameters, "mpc_min_prediction_length", param.min_prediction_length);

    // initialize input buffer
    update_param(parameters, "input_delay", param.input_delay);
    const double delay_step = std::round(param.input_delay / m_mpc.m_ctrl_period);
    const double delay = delay_step * m_mpc.m_ctrl_period;
    if (param.input_delay != delay) {
      param.input_delay = delay;
      m_mpc.m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
    }

    // transaction succeeds, now assign values
    m_mpc.m_param = param;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

bool MpcLateralController::isTrajectoryShapeChanged() const
{
  // TODO(Horibe): update implementation to check trajectory shape around ego vehicle.
  // Now temporally check the goal position.
  for (const auto & trajectory : m_trajectory_buffer) {
    if (
      tier4_autoware_utils::calcDistance2d(
        trajectory.points.back().pose, m_current_trajectory_ptr->points.back().pose) >
      m_new_traj_end_dist) {
      return true;
    }
  }
  return false;
}

bool MpcLateralController::isValidTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & traj) const
{
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.pose.position.x) || !isfinite(p.pose.position.y) ||
      !isfinite(p.pose.orientation.w) || !isfinite(p.pose.orientation.x) ||
      !isfinite(p.pose.orientation.y) || !isfinite(p.pose.orientation.z) ||
      !isfinite(p.longitudinal_velocity_mps) || !isfinite(p.lateral_velocity_mps) ||
      !isfinite(p.lateral_velocity_mps) || !isfinite(p.heading_rate_rps) ||
      !isfinite(p.front_wheel_angle_rad) || !isfinite(p.rear_wheel_angle_rad)) {
      return false;
    }
  }
  return true;
}

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
