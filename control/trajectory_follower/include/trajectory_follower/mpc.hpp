// Copyright 2018-2021 The Autoware Foundation
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

#ifndef TRAJECTORY_FOLLOWER__MPC_HPP_
#define TRAJECTORY_FOLLOWER__MPC_HPP_

#include "geometry/common_2d.hpp"
#include "helper_functions/angle_utils.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower/interpolate.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "trajectory_follower/mpc_trajectory.hpp"
#include "trajectory_follower/mpc_utils.hpp"
#include "trajectory_follower/qp_solver/qp_solver_osqp.hpp"
#include "trajectory_follower/qp_solver/qp_solver_unconstr_fast.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"
#include "trajectory_follower/visibility_control.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{

struct MPCParam
{
  //!< @brief prediction horizon step
  int prediction_horizon;
  //!< @brief prediction horizon sampling time
  double prediction_dt;
  //!< @brief threshold that feed-forward angle becomes zero
  double zero_ff_steer_deg;
  //!< @brief delay time for steering input to be compensated
  double input_delay;
  //!< @brief for trajectory velocity calculation
  double acceleration_limit;
  //!< @brief for trajectory velocity calculation
  double velocity_time_constant;
  //!< @brief minimum prediction dist for low velocity
  double min_prediction_length;
  //!< @brief time constant for steer model
  double steer_tau;
  // for weight matrix Q
  //!< @brief lateral error weight
  double weight_lat_error;
  //!< @brief heading error weight
  double weight_heading_error;
  //!< @brief heading error * velocity weight
  double weight_heading_error_squared_vel;
  //!< @brief terminal lateral error weight
  double weight_terminal_lat_error;
  //!< @brief terminal heading error weight
  double weight_terminal_heading_error;
  //!< @brief lateral error weight in matrix Q in low curvature point
  double low_curvature_weight_lat_error;
  //!< @brief heading error weight in matrix Q in low curvature point
  double low_curvature_weight_heading_error;
  //!< @brief heading error * velocity weight in matrix Q in low curvature point
  double low_curvature_weight_heading_error_squared_vel;
  // for weight matrix R
  //!< @brief steering error weight
  double weight_steering_input;
  //!< @brief steering error * velocity weight
  double weight_steering_input_squared_vel;
  //!< @brief lateral jerk weight
  double weight_lat_jerk;
  //!< @brief steering rate weight
  double weight_steer_rate;
  //!< @brief steering angle acceleration weight
  double weight_steer_acc;
  //!< @brief steering error weight in matrix R in low curvature point
  double low_curvature_weight_steering_input;
  //!< @brief steering error * velocity weight in matrix R in low curvature point
  double low_curvature_weight_steering_input_squared_vel;
  //!< @brief lateral jerk weight in matrix R in low curvature point
  double low_curvature_weight_lat_jerk;
  //!< @brief steering rate weight in matrix R in low curvature point
  double low_curvature_weight_steer_rate;
  //!< @brief steering angle acceleration weight in matrix R in low curvature
  double low_curvature_weight_steer_acc;
  //!< @brief threshold of curvature to use "low curvature" parameter
  double low_curvature_thresh_curvature;
};
/**
 * MPC problem data
 */
struct MPCData
{
  int nearest_idx;
  double nearest_time;
  geometry_msgs::msg::Pose nearest_pose;
  double steer;
  double predicted_steer;
  double lateral_err;
  double yaw_err;
};
/**
 * Matrices used for MPC optimization
 */
struct MPCMatrix
{
  Eigen::MatrixXd Aex;
  Eigen::MatrixXd Bex;
  Eigen::MatrixXd Wex;
  Eigen::MatrixXd Cex;
  Eigen::MatrixXd Qex;
  Eigen::MatrixXd R1ex;
  Eigen::MatrixXd R2ex;
  Eigen::MatrixXd Uref_ex;
};
/**
 * MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */
class TRAJECTORY_FOLLOWER_PUBLIC MPC
{
private:
  //!< @brief ROS logger used for debug logging
  rclcpp::Logger m_logger = rclcpp::get_logger("mpc_logger");
  //!< @brief ROS clock
  rclcpp::Clock::SharedPtr m_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  //!< @brief vehicle model type for MPC
  std::string m_vehicle_model_type;
  //!< @brief vehicle model for MPC
  std::shared_ptr<trajectory_follower::VehicleModelInterface> m_vehicle_model_ptr;
  //!< @brief qp solver for MPC
  std::shared_ptr<trajectory_follower::QPSolverInterface> m_qpsolver_ptr;
  //!< @brief lowpass filter for steering command
  trajectory_follower::Butterworth2dFilter m_lpf_steering_cmd;
  //!< @brief lowpass filter for lateral error
  trajectory_follower::Butterworth2dFilter m_lpf_lateral_error;
  //!< @brief lowpass filter for heading error
  trajectory_follower::Butterworth2dFilter m_lpf_yaw_error;

  //!< @brief raw output computed two iterations ago
  double m_raw_steer_cmd_pprev = 0.0;
  //!< @brief previous lateral error for derivative
  double m_lateral_error_prev = 0.0;
  //!< @brief previous lateral error for derivative
  double m_yaw_error_prev = 0.0;
  //!< @brief previous predicted steering
  std::shared_ptr<double> m_steer_prediction_prev;
  //!< @brief previous computation time
  rclcpp::Time m_time_prev = rclcpp::Time(0, 0, RCL_ROS_TIME);
  //!< @brief shift is forward or not.
  bool m_is_forward_shift = true;
  //!< @brief buffer of sent command
  std::vector<autoware_auto_control_msgs::msg::AckermannLateralCommand> m_ctrl_cmd_vec;
  //!< @brief minimum prediction distance
  double m_min_prediction_length = 5.0;

  /**
   * @brief get variables for mpc calculation
   */
  bool getData(
    const trajectory_follower::MPCTrajectory & traj,
    const autoware_auto_vehicle_msgs::msg::SteeringReport & current_steer,
    const geometry_msgs::msg::Pose & current_pose, MPCData * data);
  /**
   * @brief calculate predicted steering
   */
  double calcSteerPrediction();
  /**
   * @brief get the sum of all steering commands over the given time range
   */
  double getSteerCmdSum(
    const rclcpp::Time & t_start, const rclcpp::Time & t_end, const double time_constant) const;
  /**
   * @brief set the reference trajectory to follow
   */
  void storeSteerCmd(const double steer);
  /**
   * @brief reset previous result of MPC
   */
  void resetPrevResult(const autoware_auto_vehicle_msgs::msg::SteeringReport & current_steer);
  /**
   * @brief set initial condition for mpc
   * @param [in] data mpc data
   */
  Eigen::VectorXd getInitialState(const MPCData & data);
  /**
   * @brief update status for delay compensation
   * @param [in] traj MPCTrajectory to follow
   * @param [in] start_time start time for update
   * @param [out] x updated state at delayed_time
   */
  bool updateStateForDelayCompensation(
    const trajectory_follower::MPCTrajectory & traj, const double & start_time,
    Eigen::VectorXd * x);
  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [in] reference_trajectory used for linearization around reference trajectory
   */
  MPCMatrix generateMPCMatrix(
    const trajectory_follower::MPCTrajectory & reference_trajectory, const double prediction_dt);
  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [in] mpc_matrix parameters matrix to use for optimization
   * @param [in] x0 initial state vector
   * @param [in] prediction_dt prediction delta time
   * @param [out] Uex optimized input vector
   */
  bool executeOptimization(
    const MPCMatrix & mpc_matrix, const Eigen::VectorXd & x0, const double prediction_dt,
    Eigen::VectorXd * Uex);
  /**
   * @brief resample trajectory with mpc resampling time
   */
  bool resampleMPCTrajectoryByTime(
    const double start_time, const double prediction_dt,
    const trajectory_follower::MPCTrajectory & input,
    trajectory_follower::MPCTrajectory * output) const;
  /**
   * @brief apply velocity dynamics filter with v0 from closest index
   */
  trajectory_follower::MPCTrajectory applyVelocityDynamicsFilter(
    const trajectory_follower::MPCTrajectory & trajectory,
    const geometry_msgs::msg::Pose & current_pose, const double v0) const;
  /**
   * @brief get prediction delta time of mpc.
   * If trajectory length is shorter than min_prediction length, adjust delta time.
   */
  double getPredictionDeltaTime(
    const double start_time, const trajectory_follower::MPCTrajectory & input,
    const geometry_msgs::msg::Pose & current_pose) const;
  /**
   * @brief add weights related to lateral_jerk, steering_rate, steering_acc into R
   */
  void addSteerWeightR(const double prediction_dt, Eigen::MatrixXd * R) const;
  /**
   * @brief add weights related to lateral_jerk, steering_rate, steering_acc into f
   */
  void addSteerWeightF(const double prediction_dt, Eigen::MatrixXd * f) const;

  /**
   * @brief calculate desired steering rate.
   */
  double calcDesiredSteeringRate(
    const MPCMatrix & m, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
    const double u_filtered, const float current_steer, const double predict_dt) const;

  /**
   * @brief check if the matrix has invalid value
   */
  bool isValid(const MPCMatrix & m) const;
  /**
   * @brief return true if the given curvature is considered low
   */
  inline bool isLowCurvature(const double curvature)
  {
    return std::fabs(curvature) < m_param.low_curvature_thresh_curvature;
  }
  /**
   * @brief return the weight of the lateral error for the given curvature
   */
  inline double getWeightLatError(const double curvature)
  {
    return isLowCurvature(curvature) ? m_param.low_curvature_weight_lat_error
                                     : m_param.weight_lat_error;
  }
  /**
   * @brief return the weight of the heading error for the given curvature
   */
  inline double getWeightHeadingError(const double curvature)
  {
    return isLowCurvature(curvature) ? m_param.low_curvature_weight_heading_error
                                     : m_param.weight_heading_error;
  }
  /**
   * @brief return the squared velocity weight of the heading error for the given curvature
   */
  inline double getWeightHeadingErrorSqVel(const double curvature)
  {
    return isLowCurvature(curvature) ? m_param.low_curvature_weight_heading_error_squared_vel
                                     : m_param.weight_heading_error_squared_vel;
  }
  /**
   * @brief return the weight of the steer input for the given curvature
   */
  inline double getWeightSteerInput(const double curvature)
  {
    return isLowCurvature(curvature) ? m_param.low_curvature_weight_steering_input
                                     : m_param.weight_steering_input;
  }
  /**
   * @brief return the squared velocity weight of the steer input for the given curvature
   */
  inline double getWeightSteerInputSqVel(const double curvature)
  {
    return isLowCurvature(curvature) ? m_param.low_curvature_weight_steering_input_squared_vel
                                     : m_param.weight_steering_input_squared_vel;
  }
  /**
   * @brief return the weight of the lateral jerk for the given curvature
   */
  inline double getWeightLatJerk(const double curvature)
  {
    return isLowCurvature(curvature) ? m_param.low_curvature_weight_lat_jerk
                                     : m_param.weight_lat_jerk;
  }
  /**
   * @brief return the weight of the steering rate for the given curvature
   */
  inline double getWeightSteerRate(const double curvature)
  {
    return isLowCurvature(curvature) ? m_param.low_curvature_weight_steer_rate
                                     : m_param.weight_steer_rate;
  }
  /**
   * @brief return the weight of the steering acceleration for the given curvature
   */
  inline double getWeightSteerAcc(const double curvature)
  {
    return isLowCurvature(curvature) ? m_param.low_curvature_weight_steer_acc
                                     : m_param.weight_steer_acc;
  }

public:
  //!< @brief reference trajectory to be followed
  trajectory_follower::MPCTrajectory m_ref_traj;
  //!< @brief MPC design parameter
  MPCParam m_param;
  //!< @brief mpc_output buffer for delay time compensation
  std::deque<double> m_input_buffer;
  //!< @brief mpc raw output in previous period
  double m_raw_steer_cmd_prev = 0.0;
  /* parameters for control*/
  //!< @brief use stop cmd when lateral error exceeds this [m]
  double m_admissible_position_error;
  //!< @brief use stop cmd when yaw error exceeds this [rad]
  double m_admissible_yaw_error_rad;
  //!< @brief steering command limit [rad]
  double m_steer_lim;
  //!< @brief steering rate limit [rad/s]
  double m_steer_rate_lim;
  //!< @brief control frequency [s]
  double m_ctrl_period;
  /* parameters for path smoothing */
  //!< @brief flag to use predicted steer, not measured steer.
  bool m_use_steer_prediction;
  //!< @brief parameters for nearest index search
  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;

  /**
   * @brief constructor
   */
  MPC() = default;
  /**
   * @brief calculate control command by MPC algorithm
   * @param [in] current_steer current steering of the vehicle
   * @param [in] current_velocity current velocity of the vehicle [m/s]
   * @param [in] current_pose current pose of the vehicle
   * @param [out] ctrl_cmd control command calculated with mpc algorithm
   * @param [out] predicted_traj predicted MPC trajectory
   * @param [out] diagnostic diagnostic msg to be filled-out
   */
  bool calculateMPC(
    const autoware_auto_vehicle_msgs::msg::SteeringReport & current_steer,
    const double current_velocity, const geometry_msgs::msg::Pose & current_pose,
    autoware_auto_control_msgs::msg::AckermannLateralCommand & ctrl_cmd,
    autoware_auto_planning_msgs::msg::Trajectory & predicted_traj,
    tier4_debug_msgs::msg::Float32MultiArrayStamped & diagnostic);
  /**
   * @brief set the reference trajectory to follow
   */
  void setReferenceTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory & trajectory_msg,
    const double traj_resample_dist, const bool enable_path_smoothing,
    const int path_filter_moving_ave_num, const int curvature_smoothing_num_traj,
    const int curvature_smoothing_num_ref_steer);
  /**
   * @brief set the vehicle model of this MPC
   */
  inline void setVehicleModel(
    std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr,
    const std::string & vehicle_model_type)
  {
    m_vehicle_model_ptr = vehicle_model_ptr;
    m_vehicle_model_type = vehicle_model_type;
  }
  /**
   * @brief set the QP solver of this MPC
   */
  inline void setQPSolver(std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr)
  {
    m_qpsolver_ptr = qpsolver_ptr;
  }
  /**
   * @brief initialize low pass filters
   */
  inline void initializeLowPassFilters(
    const double steering_lpf_cutoff_hz, const double error_deriv_lpf_cutoff_hz)
  {
    m_lpf_steering_cmd.initialize(m_ctrl_period, steering_lpf_cutoff_hz);
    m_lpf_lateral_error.initialize(m_ctrl_period, error_deriv_lpf_cutoff_hz);
    m_lpf_yaw_error.initialize(m_ctrl_period, error_deriv_lpf_cutoff_hz);
  }
  /**
   * @brief return true if the vehicle model of this MPC is set
   */
  inline bool hasVehicleModel() const { return m_vehicle_model_ptr != nullptr; }
  /**
   * @brief return true if the QP solver of this MPC is set
   */
  inline bool hasQPSolver() const { return m_qpsolver_ptr != nullptr; }
  /**
   * @brief set the RCLCPP logger to use for logging
   */
  inline void setLogger(rclcpp::Logger logger) { m_logger = logger; }
  /**
   * @brief set the RCLCPP clock to use for time keeping
   */
  inline void setClock(rclcpp::Clock::SharedPtr clock) { m_clock = clock; }
};  // class MPC
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__MPC_HPP_
