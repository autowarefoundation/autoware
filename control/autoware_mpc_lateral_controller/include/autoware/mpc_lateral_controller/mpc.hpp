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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_HPP_

#include "autoware/mpc_lateral_controller/lowpass_filter.hpp"
#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/qp_solver/qp_solver_interface.hpp"
#include "autoware/mpc_lateral_controller/steering_predictor.hpp"
#include "autoware/mpc_lateral_controller/vehicle_model/vehicle_model_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

using autoware_control_msgs::msg::Lateral;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::SteeringReport;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Weight factors used in Model Predictive Control
struct MPCWeight
{
  // Weight for lateral tracking error. A larger weight leads to less lateral tracking error.
  double lat_error;

  // Weight for heading tracking error. A larger weight reduces heading tracking error.
  double heading_error;

  // Weight combining heading error and velocity. Adjusts the influence of heading error based on
  // velocity.
  double heading_error_squared_vel;

  // Weight for lateral tracking error at the terminal state of the trajectory. This improves the
  // stability of MPC.
  double terminal_lat_error;

  // Weight for heading tracking error at the terminal state of the trajectory. This improves the
  // stability of MPC.
  double terminal_heading_error;

  // Weight for the steering input. This surpress the deviation between the steering command and
  // reference steering angle calculated from curvature.
  double steering_input;

  // Adjusts the influence of steering_input weight based on velocity.
  double steering_input_squared_vel;

  // Weight for lateral jerk. Penalizes sudden changes in lateral acceleration.
  double lat_jerk;

  // Weight for steering rate. Penalizes the speed of steering angle change.
  double steer_rate;

  // Weight for steering angle acceleration. Regulates the rate of change of steering rate.
  double steer_acc;
};

struct MPCParam
{
  // Number of steps in the prediction horizon.
  int prediction_horizon;

  // Sampling time for the prediction horizon.
  double prediction_dt;

  // Threshold at which the feed-forward steering angle becomes zero.
  double zero_ff_steer_deg;

  // Time delay for compensating the steering input.
  double input_delay;

  // Limit for calculating trajectory velocity.
  double acceleration_limit;

  // Time constant for calculating trajectory velocity.
  double velocity_time_constant;

  // Minimum prediction distance used for low velocity case.
  double min_prediction_length;

  // Time constant for the steer model.
  double steer_tau;

  // Weight parameters for the MPC in nominal conditions.
  MPCWeight nominal_weight;

  // Weight parameters for the MPC in low curvature path conditions.
  MPCWeight low_curvature_weight;

  // Curvature threshold to determine when to use "low curvature" parameter settings.
  double low_curvature_thresh_curvature;
};

struct TrajectoryFilteringParam
{
  // path resampling interval [m]
  double traj_resample_dist;

  // flag of traj extending for terminal yaw
  bool extend_trajectory_for_end_yaw_control;

  // flag for path smoothing
  bool enable_path_smoothing;

  // param of moving average filter for path smoothing
  int path_filter_moving_ave_num;

  // point-to-point index distance for curvature calculation for trajectory
  int curvature_smoothing_num_traj;

  // point-to-point index distance for curvature calculation for reference steer command
  int curvature_smoothing_num_ref_steer;
};

struct MPCData
{
  // Index of the nearest point in the trajectory.
  size_t nearest_idx{};

  // Time stamp of the nearest point in the trajectory.
  double nearest_time{};

  // Pose (position and orientation) of the nearest point in the trajectory.
  Pose nearest_pose{};

  // Current steering angle.
  double steer{};

  // Predicted steering angle based on the vehicle model.
  double predicted_steer{};

  // Lateral tracking error.
  double lateral_err{};

  // Yaw (heading) tracking error.
  double yaw_err{};

  MPCData() = default;
};

/**
 * MPC matrix with the following format:
 * Xex = Aex * X0 + Bex * Uex * Wex
 * Yex = Cex * Xex
 * Cost = Xex' * Qex * Xex + (Uex - Uref_ex)' * R1ex * (Uex - Uref_ex) +  Uex' * R2ex * Uex
 */
struct MPCMatrix
{
  MatrixXd Aex;
  MatrixXd Bex;
  MatrixXd Wex;
  MatrixXd Cex;
  MatrixXd Qex;
  MatrixXd R1ex;
  MatrixXd R2ex;
  MatrixXd Uref_ex;

  MPCMatrix() = default;
};

/**
 * MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */
class MPC
{
private:
  rclcpp::Logger m_logger = rclcpp::get_logger("mpc_logger");  // ROS logger used for debug logging.
  rclcpp::Clock::SharedPtr m_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);  // ROS clock.

  // Vehicle model used for MPC.
  std::shared_ptr<VehicleModelInterface> m_vehicle_model_ptr;

  // QP solver used for MPC.
  std::shared_ptr<QPSolverInterface> m_qpsolver_ptr;

  // Calculate predicted steering angle based on the steering dynamics. The predicted value should
  // fit to the actual steering angle if the vehicle model is accurate enough.
  std::shared_ptr<SteeringPredictor> m_steering_predictor;

  Butterworth2dFilter m_lpf_steering_cmd;   // Low-pass filter for smoothing the steering command.
  Butterworth2dFilter m_lpf_lateral_error;  // Low-pass filter for smoothing the lateral error.
  Butterworth2dFilter m_lpf_yaw_error;      // Low-pass filter for smoothing the heading error.

  double m_raw_steer_cmd_pprev = 0.0;  // Raw output computed two iterations ago.
  double m_lateral_error_prev = 0.0;   // Previous lateral error for derivative calculation.
  double m_yaw_error_prev = 0.0;       // Previous heading error for derivative calculation.

  bool m_is_forward_shift = true;  // Flag indicating if the shift is in the forward direction.

  double m_min_prediction_length = 5.0;  // Minimum prediction distance.

  rclcpp::Publisher<Trajectory>::SharedPtr m_debug_frenet_predicted_trajectory_pub;
  rclcpp::Publisher<Trajectory>::SharedPtr m_debug_resampled_reference_trajectory_pub;
  /**
   * @brief Get variables for MPC calculation.
   * @param trajectory The reference trajectory.
   * @param current_steer The current steering report.
   * @param current_kinematics The current vehicle kinematics.
   * @return A pair of a boolean flag indicating success and the MPC data.
   */
  std::pair<bool, MPCData> getData(
    const MPCTrajectory & trajectory, const SteeringReport & current_steer,
    const Odometry & current_kinematics);

  /**
   * @brief Get the initial state for MPC.
   * @param data The MPC data.
   * @return The initial state as a vector.
   */
  VectorXd getInitialState(const MPCData & data);

  /**
   * @brief Update the state for delay compensation.
   * @param traj The reference trajectory to follow.
   * @param start_time The time where x0_orig is defined.
   * @param x0_orig The original initial state vector.
   * @return A pair of a boolean flag indicating success and the updated state at delayed_time.
   */
  std::pair<bool, VectorXd> updateStateForDelayCompensation(
    const MPCTrajectory & traj, const double & start_time, const VectorXd & x0_orig);

  /**
   * @brief Generate the MPC matrix using the reference trajectory and vehicle model.
   * @param reference_trajectory The reference trajectory used for linearization.
   * @param prediction_dt The prediction time step.
   * @return The generated MPC matrix.
   */
  MPCMatrix generateMPCMatrix(
    const MPCTrajectory & reference_trajectory, const double prediction_dt);

  /**
   * @brief Execute the optimization using the provided MPC matrix, initial state, and prediction
   * time step.
   * @param mpc_matrix The parameters matrix used for optimization.
   * @param x0 The initial state vector.
   * @param prediction_dt The prediction time step.
   * @param [in] trajectory mpc reference trajectory
   * @param [in] current_velocity current ego velocity
   * @return A pair of a boolean flag indicating success and the optimized input vector.
   */
  std::pair<bool, VectorXd> executeOptimization(
    const MPCMatrix & mpc_matrix, const VectorXd & x0, const double prediction_dt,
    const MPCTrajectory & trajectory, const double current_velocity);

  /**
   * @brief Resample the trajectory with the MPC resampling time.
   * @param start_time The start time for resampling.
   * @param prediction_dt The prediction time step.
   * @param input The input trajectory.
   * @return A pair of a boolean flag indicating success and the resampled trajectory.
   */
  std::pair<bool, MPCTrajectory> resampleMPCTrajectoryByTime(
    const double start_time, const double prediction_dt, const MPCTrajectory & input) const;

  /**
   * @brief Apply the velocity dynamics filter to the trajectory using the current kinematics.
   * @param trajectory The input trajectory.
   * @param current_kinematics The current vehicle kinematics.
   * @return The filtered trajectory.
   */
  MPCTrajectory applyVelocityDynamicsFilter(
    const MPCTrajectory & trajectory, const Odometry & current_kinematics) const;

  /**
   * @brief Get the prediction time step for MPC. If the trajectory length is shorter than
   * min_prediction_length, adjust the time step.
   * @param start_time The start time of the trajectory.
   * @param input The input trajectory.
   * @param current_kinematics The current vehicle kinematics.
   * @return The prediction time step.
   */
  double getPredictionDeltaTime(
    const double start_time, const MPCTrajectory & input,
    const Odometry & current_kinematics) const;

  /**
   * @brief Add weights related to lateral jerk, steering rate, and steering acceleration to the R
   * matrix.
   * @param prediction_dt The prediction time step.
   * @param R The R matrix to modify.
   */
  void addSteerWeightR(const double prediction_dt, MatrixXd & R) const;

  /**
   * @brief Add weights related to lateral jerk, steering rate, and steering acceleration to the f
   * matrix.
   * @param prediction_dt The prediction time step.
   * @param f The f matrix to modify.
   */
  void addSteerWeightF(const double prediction_dt, MatrixXd & f) const;

  /**
   * @brief Calculate the desired steering rate for the steering_rate command.
   * @param m The MPC matrix used for optimization.
   * @param x0 The initial state matrix.
   * @param Uex The input matrix.
   * @param u_filtered The filtered input.
   * @param current_steer The current steering angle.
   * @param predict_dt The prediction time step.
   * @return The desired steering rate.
   */
  double calcDesiredSteeringRate(
    const MPCMatrix & m, const MatrixXd & x0, const MatrixXd & Uex, const double u_filtered,
    const float current_steer, const double predict_dt) const;

  /**
   * @brief calculate predicted trajectory
   * @param mpc_matrix The MPC matrix used in the mpc problem.
   * @param x0 initial state used in the mpc problem.
   * @param Uex optimized input.
   * @param mpc_resampled_ref_traj reference trajectory resampled in the mpc time-step
   * @param dt delta time used in the mpc problem.
   * @param coordinate String specifying the coordinate system ("world" or "frenet", default is
   * "world")
   * @return predicted path
   */
  Trajectory calculatePredictedTrajectory(
    const MPCMatrix & mpc_matrix, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
    const MPCTrajectory & reference_trajectory, const double dt,
    const std::string & coordinate = "world") const;

  /**
   * @brief Check if the MPC matrix has any invalid values.
   * @param m The MPC matrix to check.
   * @return True if the matrix is valid, false otherwise.
   */
  bool isValid(const MPCMatrix & m) const;

  /**
   * @brief Get the weight for the MPC optimization based on the curvature.
   * @param curvature The curvature value.
   * @return The weight for the MPC optimization.
   */
  inline MPCWeight getWeight(const double curvature)
  {
    return std::fabs(curvature) < m_param.low_curvature_thresh_curvature
             ? m_param.low_curvature_weight
             : m_param.nominal_weight;
  }

  /**
   * @brief Generate diagnostic data for debugging purposes.
   * @param reference_trajectory The reference trajectory.
   * @param mpc_data The MPC data.
   * @param mpc_matrix The MPC matrix.
   * @param ctrl_cmd The control command.
   * @param Uex The optimized input vector.
   * @param current_kinematics The current vehicle kinematics.
   * @return The generated diagnostic data.
   */
  Float32MultiArrayStamped generateDiagData(
    const MPCTrajectory & reference_trajectory, const MPCData & mpc_data,
    const MPCMatrix & mpc_matrix, const Lateral & ctrl_cmd, const VectorXd & Uex,
    const Odometry & current_kinematics) const;

  /**
   * @brief calculate steering rate limit along with the target trajectory
   * @param reference_trajectory The reference trajectory.
   * @param current_velocity current velocity of ego.
   */
  VectorXd calcSteerRateLimitOnTrajectory(
    const MPCTrajectory & trajectory, const double current_velocity) const;

  //!< @brief logging with warn and return false
  template <typename... Args>
  inline bool fail_warn_throttle(Args &&... args) const
  {
    RCLCPP_WARN_THROTTLE(m_logger, *m_clock, 3000, args...);
    return false;
  }

  //!< @brief logging with warn
  template <typename... Args>
  inline void warn_throttle(Args &&... args) const
  {
    RCLCPP_WARN_THROTTLE(m_logger, *m_clock, 3000, args...);
  }

public:
  MPCTrajectory m_reference_trajectory;  // Reference trajectory to be followed.
  MPCParam m_param;                      // MPC design parameters.
  std::deque<double> m_input_buffer;     // MPC output buffer for delay time compensation.
  double m_raw_steer_cmd_prev = 0.0;     // Previous MPC raw output.

  /* Parameters for control */
  double m_admissible_position_error;  // Threshold for lateral error to trigger stop command [m].
  double m_admissible_yaw_error_rad;   // Threshold for yaw error to trigger stop command [rad].
  double m_steer_lim;                  // Steering command limit [rad].
  double m_ctrl_period;                // Control frequency [s].

  //!< @brief steering rate limit list depending on curvature [/m], [rad/s]
  std::vector<std::pair<double, double>> m_steer_rate_lim_map_by_curvature{};

  //!< @brief steering rate limit list depending on velocity [m/s], [rad/s]
  std::vector<std::pair<double, double>> m_steer_rate_lim_map_by_velocity{};

  bool m_use_steer_prediction;  // Flag to use predicted steer instead of measured steer.
  double ego_nearest_dist_threshold = 3.0;  // Threshold for nearest index search based on distance.
  double ego_nearest_yaw_threshold = M_PI_2;  // Threshold for nearest index search based on yaw.

  bool m_use_delayed_initial_state =
    true;  // Flag to use x0_delayed as initial state for predicted trajectory

  bool m_publish_debug_trajectories = false;  // Flag to publish predicted trajectory and
                                              // resampled reference trajectory for debug purpose

  //!< Constructor.
  explicit MPC(rclcpp::Node & node);

  /**
   * @brief Calculate control command using the MPC algorithm.
   * @param current_steer Current steering report.
   * @param current_kinematics Current vehicle kinematics.
   * @param ctrl_cmd Computed lateral control command.
   * @param predicted_trajectory Predicted trajectory based on MPC result.
   * @param diagnostic Diagnostic data for debugging purposes.
   * @return True if the MPC calculation is successful, false otherwise.
   */
  bool calculateMPC(
    const SteeringReport & current_steer, const Odometry & current_kinematics, Lateral & ctrl_cmd,
    Trajectory & predicted_trajectory, Float32MultiArrayStamped & diagnostic);

  /**
   * @brief Set the reference trajectory to be followed.
   * @param trajectory_msg The reference trajectory message.
   * @param param Trajectory filtering parameters.
   */
  void setReferenceTrajectory(
    const Trajectory & trajectory_msg, const TrajectoryFilteringParam & param,
    const Odometry & current_kinematics);

  /**
   * @brief Reset the previous result of MPC.
   * @param current_steer Current steering report.
   */
  void resetPrevResult(const SteeringReport & current_steer);

  /**
   * @brief Set the vehicle model for this MPC.
   * @param vehicle_model_ptr Pointer to the vehicle model.
   */
  inline void setVehicleModel(std::shared_ptr<VehicleModelInterface> vehicle_model_ptr)
  {
    m_vehicle_model_ptr = vehicle_model_ptr;
  }

  /**
   * @brief Set the QP solver for this MPC.
   * @param qpsolver_ptr Pointer to the QP solver.
   */
  inline void setQPSolver(std::shared_ptr<QPSolverInterface> qpsolver_ptr)
  {
    m_qpsolver_ptr = qpsolver_ptr;
  }

  /**
   * @brief Initialize the steering predictor for this MPC.
   */
  inline void initializeSteeringPredictor()
  {
    m_steering_predictor =
      std::make_shared<SteeringPredictor>(m_param.steer_tau, m_param.input_delay);
  }

  /**
   * @brief Initialize the low-pass filters.
   * @param steering_lpf_cutoff_hz Cutoff frequency for the steering command low-pass filter.
   * @param error_deriv_lpf_cutoff_hz Cutoff frequency for the error derivative low-pass filter.
   */
  inline void initializeLowPassFilters(
    const double steering_lpf_cutoff_hz, const double error_deriv_lpf_cutoff_hz)
  {
    m_lpf_steering_cmd.initialize(m_ctrl_period, steering_lpf_cutoff_hz);
    m_lpf_lateral_error.initialize(m_ctrl_period, error_deriv_lpf_cutoff_hz);
    m_lpf_yaw_error.initialize(m_ctrl_period, error_deriv_lpf_cutoff_hz);
  }

  /**
   * @brief Check if the MPC has a vehicle model set.
   * @return True if the vehicle model is set, false otherwise.
   */
  inline bool hasVehicleModel() const { return m_vehicle_model_ptr != nullptr; }

  /**
   * @brief Check if the MPC has a QP solver set.
   * @return True if the QP solver is set, false otherwise.
   */
  inline bool hasQPSolver() const { return m_qpsolver_ptr != nullptr; }

  /**
   * @brief Set the RCLCPP logger to be used for logging.
   * @param logger The RCLCPP logger object.
   */
  inline void setLogger(rclcpp::Logger logger) { m_logger = logger; }

  /**
   * @brief Set the RCLCPP clock to be used for time keeping.
   * @param clock The shared pointer to the RCLCPP clock.
   */
  inline void setClock(rclcpp::Clock::SharedPtr clock) { m_clock = clock; }
};  // class MPC
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_HPP_
