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

#include "mpc_lateral_controller/mpc_lateral_controller.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_osqp.hpp"
#include "mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "mpc_lateral_controller/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "tf2/utils.h"
#include "tf2_ros/create_timer_ros.h"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

MpcLateralController::MpcLateralController(rclcpp::Node & node) : node_{&node}
{
  const auto dp_int = [&](const std::string & s) { return node_->declare_parameter<int>(s); };
  const auto dp_bool = [&](const std::string & s) { return node_->declare_parameter<bool>(s); };
  const auto dp_double = [&](const std::string & s) { return node_->declare_parameter<double>(s); };

  m_mpc.m_ctrl_period = node_->get_parameter("ctrl_period").as_double();

  auto & p_filt = m_trajectory_filtering_param;
  p_filt.enable_path_smoothing = dp_bool("enable_path_smoothing");
  p_filt.path_filter_moving_ave_num = dp_int("path_filter_moving_ave_num");
  p_filt.curvature_smoothing_num_traj = dp_int("curvature_smoothing_num_traj");
  p_filt.curvature_smoothing_num_ref_steer = dp_int("curvature_smoothing_num_ref_steer");
  p_filt.traj_resample_dist = dp_double("traj_resample_dist");
  p_filt.extend_trajectory_for_end_yaw_control = dp_bool("extend_trajectory_for_end_yaw_control");

  m_mpc.m_admissible_position_error = dp_double("admissible_position_error");
  m_mpc.m_admissible_yaw_error_rad = dp_double("admissible_yaw_error_rad");
  m_mpc.m_use_steer_prediction = dp_bool("use_steer_prediction");
  m_mpc.m_param.steer_tau = dp_double("vehicle_model_steer_tau");

  /* stop state parameters */
  m_stop_state_entry_ego_speed = dp_double("stop_state_entry_ego_speed");
  m_stop_state_entry_target_speed = dp_double("stop_state_entry_target_speed");
  m_converged_steer_rad = dp_double("converged_steer_rad");
  m_keep_steer_control_until_converged = dp_bool("keep_steer_control_until_converged");
  m_new_traj_duration_time = dp_double("new_traj_duration_time");            // [s]
  m_new_traj_end_dist = dp_double("new_traj_end_dist");                      // [m]
  m_mpc_converged_threshold_rps = dp_double("mpc_converged_threshold_rps");  // [rad/s]

  /* mpc parameters */
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*node_).getVehicleInfo();
  const double wheelbase = vehicle_info.wheel_base_m;
  constexpr double deg2rad = static_cast<double>(M_PI) / 180.0;
  m_mpc.m_steer_lim = vehicle_info.max_steer_angle_rad;

  // steer rate limit depending on curvature
  const auto steer_rate_lim_dps_list_by_curvature =
    node_->declare_parameter<std::vector<double>>("steer_rate_lim_dps_list_by_curvature");
  const auto curvature_list_for_steer_rate_lim =
    node_->declare_parameter<std::vector<double>>("curvature_list_for_steer_rate_lim");
  for (size_t i = 0; i < steer_rate_lim_dps_list_by_curvature.size(); ++i) {
    m_mpc.m_steer_rate_lim_map_by_curvature.emplace_back(
      curvature_list_for_steer_rate_lim.at(i),
      steer_rate_lim_dps_list_by_curvature.at(i) * deg2rad);
  }

  // steer rate limit depending on velocity
  const auto steer_rate_lim_dps_list_by_velocity =
    node_->declare_parameter<std::vector<double>>("steer_rate_lim_dps_list_by_velocity");
  const auto velocity_list_for_steer_rate_lim =
    node_->declare_parameter<std::vector<double>>("velocity_list_for_steer_rate_lim");
  for (size_t i = 0; i < steer_rate_lim_dps_list_by_velocity.size(); ++i) {
    m_mpc.m_steer_rate_lim_map_by_velocity.emplace_back(
      velocity_list_for_steer_rate_lim.at(i), steer_rate_lim_dps_list_by_velocity.at(i) * deg2rad);
  }

  /* vehicle model setup */
  auto vehicle_model_ptr =
    createVehicleModel(wheelbase, m_mpc.m_steer_lim, m_mpc.m_param.steer_tau);
  m_mpc.setVehicleModel(vehicle_model_ptr);

  /* QP solver setup */
  m_mpc.setVehicleModel(vehicle_model_ptr);
  auto qpsolver_ptr = createQPSolverInterface();
  m_mpc.setQPSolver(qpsolver_ptr);

  /* delay compensation */
  {
    const double delay_tmp = dp_double("input_delay");
    const double delay_step = std::round(delay_tmp / m_mpc.m_ctrl_period);
    m_mpc.m_param.input_delay = delay_step * m_mpc.m_ctrl_period;
    m_mpc.m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
  }

  /* steering offset compensation */
  enable_auto_steering_offset_removal_ =
    dp_bool("steering_offset.enable_auto_steering_offset_removal");
  steering_offset_ = createSteerOffsetEstimator(wheelbase);

  /* initialize low-pass filter */
  {
    const double steering_lpf_cutoff_hz = dp_double("steering_lpf_cutoff_hz");
    const double error_deriv_lpf_cutoff_hz = dp_double("error_deriv_lpf_cutoff_hz");
    m_mpc.initializeLowPassFilters(steering_lpf_cutoff_hz, error_deriv_lpf_cutoff_hz);
  }

  // ego nearest index search
  const auto check_and_get_param = [&](const auto & param) {
    return node_->has_parameter(param) ? node_->get_parameter(param).as_double() : dp_double(param);
  };
  m_ego_nearest_dist_threshold = check_and_get_param("ego_nearest_dist_threshold");
  m_ego_nearest_yaw_threshold = check_and_get_param("ego_nearest_yaw_threshold");
  m_mpc.ego_nearest_dist_threshold = m_ego_nearest_dist_threshold;
  m_mpc.ego_nearest_yaw_threshold = m_ego_nearest_yaw_threshold;

  m_pub_predicted_traj = node_->create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  m_pub_debug_values =
    node_->create_publisher<Float32MultiArrayStamped>("~/output/lateral_diagnostic", 1);
  m_pub_steer_offset =
    node_->create_publisher<Float32Stamped>("~/output/estimated_steer_offset", 1);

  declareMPCparameters();

  /* get parameter updates */
  using std::placeholders::_1;
  m_set_param_res = node_->add_on_set_parameters_callback(
    std::bind(&MpcLateralController::paramCallback, this, _1));

  m_mpc.initializeSteeringPredictor();

  m_mpc.setLogger(node_->get_logger());
  m_mpc.setClock(node_->get_clock());
}

MpcLateralController::~MpcLateralController()
{
}

std::shared_ptr<VehicleModelInterface> MpcLateralController::createVehicleModel(
  const double wheelbase, const double steer_lim, const double steer_tau)
{
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr;

  const std::string vehicle_model_type =
    node_->declare_parameter<std::string>("vehicle_model_type");

  if (vehicle_model_type == "kinematics") {
    vehicle_model_ptr = std::make_shared<KinematicsBicycleModel>(wheelbase, steer_lim, steer_tau);
    return vehicle_model_ptr;
  }

  if (vehicle_model_type == "kinematics_no_delay") {
    vehicle_model_ptr = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase, steer_lim);
    return vehicle_model_ptr;
  }

  if (vehicle_model_type == "dynamics") {
    const double mass_fl = node_->declare_parameter<double>("vehicle.mass_fl");
    const double mass_fr = node_->declare_parameter<double>("vehicle.mass_fr");
    const double mass_rl = node_->declare_parameter<double>("vehicle.mass_rl");
    const double mass_rr = node_->declare_parameter<double>("vehicle.mass_rr");
    const double cf = node_->declare_parameter<double>("vehicle.cf");
    const double cr = node_->declare_parameter<double>("vehicle.cr");

    // vehicle_model_ptr is only assigned in ctor, so parameter value have to be passed at init time
    vehicle_model_ptr =
      std::make_shared<DynamicsBicycleModel>(wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
    return vehicle_model_ptr;
  }

  RCLCPP_ERROR(node_->get_logger(), "vehicle_model_type is undefined");
  return vehicle_model_ptr;
}

std::shared_ptr<QPSolverInterface> MpcLateralController::createQPSolverInterface()
{
  std::shared_ptr<QPSolverInterface> qpsolver_ptr;

  const std::string qp_solver_type = node_->declare_parameter<std::string>("qp_solver_type");

  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr = std::make_shared<QPSolverEigenLeastSquareLLT>();
    return qpsolver_ptr;
  }

  if (qp_solver_type == "osqp") {
    qpsolver_ptr = std::make_shared<QPSolverOSQP>(node_->get_logger());
    return qpsolver_ptr;
  }

  RCLCPP_ERROR(node_->get_logger(), "qp_solver_type is undefined");
  return qpsolver_ptr;
}

std::shared_ptr<SteeringOffsetEstimator> MpcLateralController::createSteerOffsetEstimator(
  const double wheelbase)
{
  const std::string ns = "steering_offset.";
  const auto vel_thres = node_->declare_parameter<double>(ns + "update_vel_threshold");
  const auto steer_thres = node_->declare_parameter<double>(ns + "update_steer_threshold");
  const auto limit = node_->declare_parameter<double>(ns + "steering_offset_limit");
  const auto num = node_->declare_parameter<int>(ns + "average_num");
  steering_offset_ =
    std::make_shared<SteeringOffsetEstimator>(wheelbase, num, vel_thres, steer_thres, limit);
  return steering_offset_;
}

trajectory_follower::LateralOutput MpcLateralController::run(
  trajectory_follower::InputData const & input_data)
{
  // set input data
  setTrajectory(input_data.current_trajectory);

  m_current_kinematic_state = input_data.current_odometry;
  m_current_steering = input_data.current_steering;
  if (enable_auto_steering_offset_removal_) {
    m_current_steering.steering_tire_angle -= steering_offset_->getOffset();
  }

  AckermannLateralCommand ctrl_cmd;
  Trajectory predicted_traj;
  Float32MultiArrayStamped debug_values;

  if (!m_is_ctrl_cmd_prev_initialized) {
    m_ctrl_cmd_prev = getInitialControlCommand();
    m_is_ctrl_cmd_prev_initialized = true;
  }

  const bool is_mpc_solved = m_mpc.calculateMPC(
    m_current_steering, m_current_kinematic_state, ctrl_cmd, predicted_traj, debug_values);

  // reset previous MPC result
  // Note: When a large deviation from the trajectory occurs, the optimization stops and
  // the vehicle will return to the path by re-planning the trajectory or external operation.
  // After the recovery, the previous value of the optimization may deviate greatly from
  // the actual steer angle, and it may make the optimization result unstable.
  if (!is_mpc_solved) {
    m_mpc.resetPrevResult(m_current_steering);
  } else {
    setSteeringToHistory(ctrl_cmd);
  }

  if (enable_auto_steering_offset_removal_) {
    steering_offset_->updateOffset(
      m_current_kinematic_state.twist.twist,
      input_data.current_steering.steering_tire_angle);  // use unbiased steering
    ctrl_cmd.steering_tire_angle += steering_offset_->getOffset();
  }

  publishPredictedTraj(predicted_traj);
  publishDebugValues(debug_values);

  const auto createLateralOutput = [this](const auto & cmd, const bool is_mpc_solved) {
    trajectory_follower::LateralOutput output;
    output.control_cmd = createCtrlCmdMsg(cmd);
    // To be sure current steering of the vehicle is desired steering angle, we need to check
    // following conditions.
    // 1. At the last loop, mpc should be solved because command should be optimized output.
    // 2. The mpc should be converged.
    // 3. The steer angle should be converged.
    output.sync_data.is_steer_converged =
      is_mpc_solved && isMpcConverged() && isSteerConverged(cmd);

    return output;
  };

  if (isStoppedState()) {
    // Reset input buffer
    for (auto & value : m_mpc.m_input_buffer) {
      value = m_ctrl_cmd_prev.steering_tire_angle;
    }
    // Use previous command value as previous raw steer command
    m_mpc.m_raw_steer_cmd_prev = m_ctrl_cmd_prev.steering_tire_angle;
    return createLateralOutput(m_ctrl_cmd_prev, false);
  }

  if (!is_mpc_solved) {
    warn_throttle("MPC is not solved. publish 0 velocity.");
    ctrl_cmd = getStopControlCommand();
  }

  m_ctrl_cmd_prev = ctrl_cmd;
  return createLateralOutput(ctrl_cmd, is_mpc_solved);
}

bool MpcLateralController::isSteerConverged(const AckermannLateralCommand & cmd) const
{
  // wait for a while to propagate the trajectory shape to the output command when the trajectory
  // shape is changed.
  if (!m_has_received_first_trajectory || isTrajectoryShapeChanged()) {
    RCLCPP_DEBUG(node_->get_logger(), "trajectory shaped is changed");
    return false;
  }

  const bool is_converged =
    std::abs(cmd.steering_tire_angle - m_current_steering.steering_tire_angle) <
    static_cast<float>(m_converged_steer_rad);

  return is_converged;
}

bool MpcLateralController::isReady(const trajectory_follower::InputData & input_data)
{
  setTrajectory(input_data.current_trajectory);
  m_current_kinematic_state = input_data.current_odometry;
  m_current_steering = input_data.current_steering;

  if (!m_mpc.hasVehicleModel()) {
    info_throttle("MPC does not have a vehicle model");
    return false;
  }
  if (!m_mpc.hasQPSolver()) {
    info_throttle("MPC does not have a QP solver");
    return false;
  }
  if (m_mpc.m_reference_trajectory.empty()) {
    info_throttle("trajectory size is zero.");
    return false;
  }

  return true;
}

void MpcLateralController::setTrajectory(const Trajectory & msg)
{
  m_current_trajectory = msg;

  if (msg.points.size() < 3) {
    RCLCPP_DEBUG(node_->get_logger(), "received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(msg)) {
    RCLCPP_ERROR(node_->get_logger(), "Trajectory is invalid!! stop computing.");
    return;
  }

  m_mpc.setReferenceTrajectory(msg, m_trajectory_filtering_param);

  // update trajectory buffer to check the trajectory shape change.
  m_trajectory_buffer.push_back(m_current_trajectory);
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

AckermannLateralCommand MpcLateralController::getStopControlCommand() const
{
  AckermannLateralCommand cmd;
  cmd.steering_tire_angle = static_cast<decltype(cmd.steering_tire_angle)>(m_steer_cmd_prev);
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

AckermannLateralCommand MpcLateralController::getInitialControlCommand() const
{
  AckermannLateralCommand cmd;
  cmd.steering_tire_angle = m_current_steering.steering_tire_angle;
  cmd.steering_tire_rotation_rate = 0.0;
  return cmd;
}

bool MpcLateralController::isStoppedState() const
{
  // If the nearest index is not found, return false
  if (m_current_trajectory.points.empty()) {
    return false;
  }

  // Note: This function used to take into account the distance to the stop line
  // for the stop state judgement. However, it has been removed since the steering
  // control was turned off when approaching/exceeding the stop line on a curve or
  // emergency stop situation and it caused large tracking error.
  const size_t nearest = motion_utils::findFirstNearestIndexWithSoftConstraints(
    m_current_trajectory.points, m_current_kinematic_state.pose.pose, m_ego_nearest_dist_threshold,
    m_ego_nearest_yaw_threshold);

  const double current_vel = m_current_kinematic_state.twist.twist.linear.x;
  const double target_vel = m_current_trajectory.points.at(nearest).longitudinal_velocity_mps;

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

AckermannLateralCommand MpcLateralController::createCtrlCmdMsg(
  const AckermannLateralCommand & ctrl_cmd)
{
  auto out = ctrl_cmd;
  out.stamp = node_->now();
  m_steer_cmd_prev = out.steering_tire_angle;
  return out;
}

void MpcLateralController::publishPredictedTraj(Trajectory & predicted_traj) const
{
  predicted_traj.header.stamp = node_->now();
  predicted_traj.header.frame_id = m_current_trajectory.header.frame_id;
  m_pub_predicted_traj->publish(predicted_traj);
}

void MpcLateralController::publishDebugValues(Float32MultiArrayStamped & debug_values) const
{
  debug_values.stamp = node_->now();
  m_pub_debug_values->publish(debug_values);

  Float32Stamped offset;
  offset.stamp = node_->now();
  offset.data = steering_offset_->getOffset();
  m_pub_steer_offset->publish(offset);
}

void MpcLateralController::setSteeringToHistory(const AckermannLateralCommand & steering)
{
  const auto time = node_->now();
  if (m_mpc_steering_history.empty()) {
    m_mpc_steering_history.emplace_back(steering, time);
    m_is_mpc_history_filled = false;
    return;
  }

  m_mpc_steering_history.emplace_back(steering, time);

  // Check the history is filled or not.
  if (rclcpp::Duration(time - m_mpc_steering_history.begin()->second).seconds() >= 1.0) {
    m_is_mpc_history_filled = true;
    // remove old data that is older than 1 sec
    for (auto itr = m_mpc_steering_history.begin(); itr != m_mpc_steering_history.end(); ++itr) {
      if (rclcpp::Duration(time - itr->second).seconds() > 1.0) {
        m_mpc_steering_history.erase(m_mpc_steering_history.begin());
      } else {
        break;
      }
    }
  } else {
    m_is_mpc_history_filled = false;
  }
}

bool MpcLateralController::isMpcConverged()
{
  // If the number of variable below the 2, there is no enough data so MPC is not converged.
  if (m_mpc_steering_history.size() < 2) {
    return false;
  }

  // If the history is not filled, return false.

  if (!m_is_mpc_history_filled) {
    return false;
  }

  // Find the maximum and minimum values of the steering angle in the past 1 second.
  double min_steering_value = m_mpc_steering_history[0].first.steering_tire_angle;
  double max_steering_value = m_mpc_steering_history[0].first.steering_tire_angle;
  for (size_t i = 1; i < m_mpc_steering_history.size(); i++) {
    if (m_mpc_steering_history.at(i).first.steering_tire_angle < min_steering_value) {
      min_steering_value = m_mpc_steering_history.at(i).first.steering_tire_angle;
    }
    if (m_mpc_steering_history.at(i).first.steering_tire_angle > max_steering_value) {
      max_steering_value = m_mpc_steering_history.at(i).first.steering_tire_angle;
    }
  }
  return (max_steering_value - min_steering_value) < m_mpc_converged_threshold_rps;
}

void MpcLateralController::declareMPCparameters()
{
  m_mpc.m_param.prediction_horizon = node_->declare_parameter<int>("mpc_prediction_horizon");
  m_mpc.m_param.prediction_dt = node_->declare_parameter<double>("mpc_prediction_dt");

  const auto dp = [&](const auto & param) { return node_->declare_parameter<double>(param); };

  auto & nw = m_mpc.m_param.nominal_weight;
  nw.lat_error = dp("mpc_weight_lat_error");
  nw.heading_error = dp("mpc_weight_heading_error");
  nw.heading_error_squared_vel = dp("mpc_weight_heading_error_squared_vel");
  nw.steering_input = dp("mpc_weight_steering_input");
  nw.steering_input_squared_vel = dp("mpc_weight_steering_input_squared_vel");
  nw.lat_jerk = dp("mpc_weight_lat_jerk");
  nw.steer_rate = dp("mpc_weight_steer_rate");
  nw.steer_acc = dp("mpc_weight_steer_acc");
  nw.terminal_lat_error = dp("mpc_weight_terminal_lat_error");
  nw.terminal_heading_error = dp("mpc_weight_terminal_heading_error");

  auto & lcw = m_mpc.m_param.low_curvature_weight;
  lcw.lat_error = dp("mpc_low_curvature_weight_lat_error");
  lcw.heading_error = dp("mpc_low_curvature_weight_heading_error");
  lcw.heading_error_squared_vel = dp("mpc_low_curvature_weight_heading_error_squared_vel");
  lcw.steering_input = dp("mpc_low_curvature_weight_steering_input");
  lcw.steering_input_squared_vel = dp("mpc_low_curvature_weight_steering_input_squared_vel");
  lcw.lat_jerk = dp("mpc_low_curvature_weight_lat_jerk");
  lcw.steer_rate = dp("mpc_low_curvature_weight_steer_rate");
  lcw.steer_acc = dp("mpc_low_curvature_weight_steer_acc");
  m_mpc.m_param.low_curvature_thresh_curvature = dp("mpc_low_curvature_thresh_curvature");

  m_mpc.m_param.zero_ff_steer_deg = dp("mpc_zero_ff_steer_deg");
  m_mpc.m_param.acceleration_limit = dp("mpc_acceleration_limit");
  m_mpc.m_param.velocity_time_constant = dp("mpc_velocity_time_constant");
  m_mpc.m_param.min_prediction_length = dp("mpc_min_prediction_length");
}

rcl_interfaces::msg::SetParametersResult MpcLateralController::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  MPCParam param = m_mpc.m_param;
  auto & nw = param.nominal_weight;
  auto & lcw = param.low_curvature_weight;

  using MPCUtils::update_param;
  try {
    update_param(parameters, "mpc_prediction_horizon", param.prediction_horizon);
    update_param(parameters, "mpc_prediction_dt", param.prediction_dt);

    const std::string ns_nw = "mpc_weight_";
    update_param(parameters, ns_nw + "lat_error", nw.lat_error);
    update_param(parameters, ns_nw + "heading_error", nw.heading_error);
    update_param(parameters, ns_nw + "heading_error_squared_vel", nw.heading_error_squared_vel);
    update_param(parameters, ns_nw + "steering_input", nw.steering_input);
    update_param(parameters, ns_nw + "steering_input_squared_vel", nw.steering_input_squared_vel);
    update_param(parameters, ns_nw + "lat_jerk", nw.lat_jerk);
    update_param(parameters, ns_nw + "steer_rate", nw.steer_rate);
    update_param(parameters, ns_nw + "steer_acc", nw.steer_acc);
    update_param(parameters, ns_nw + "terminal_lat_error", nw.terminal_lat_error);
    update_param(parameters, ns_nw + "terminal_heading_error", nw.terminal_heading_error);

    const std::string ns_lcw = "mpc_low_curvature_weight_";
    update_param(parameters, ns_lcw + "lat_error", lcw.lat_error);
    update_param(parameters, ns_lcw + "heading_error", lcw.heading_error);
    update_param(parameters, ns_lcw + "heading_error_squared_vel", lcw.heading_error_squared_vel);
    update_param(parameters, ns_lcw + "steering_input", lcw.steering_input);
    update_param(parameters, ns_lcw + "steering_input_squared_vel", lcw.steering_input_squared_vel);
    update_param(parameters, ns_lcw + "lat_jerk", lcw.lat_jerk);
    update_param(parameters, ns_lcw + "steer_rate", lcw.steer_rate);
    update_param(parameters, ns_lcw + "steer_acc", lcw.steer_acc);

    update_param(
      parameters, "mpc_low_curvature_thresh_curvature", param.low_curvature_thresh_curvature);

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
    const auto change_distance = tier4_autoware_utils::calcDistance2d(
      trajectory.points.back().pose, m_current_trajectory.points.back().pose);
    if (change_distance > m_new_traj_end_dist) {
      return true;
    }
  }
  return false;
}

bool MpcLateralController::isValidTrajectory(const Trajectory & traj) const
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

}  // namespace autoware::motion::control::mpc_lateral_controller
