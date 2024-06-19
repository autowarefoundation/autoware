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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_HPP_

#include "autoware/mpc_lateral_controller/mpc.hpp"
#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "autoware/mpc_lateral_controller/steering_offset/steering_offset.hpp"
#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_control_msgs/msg/lateral.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control::mpc_lateral_controller
{

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
using autoware_control_msgs::msg::Lateral;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::SteeringReport;
using nav_msgs::msg::Odometry;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_debug_msgs::msg::Float32Stamped;

class MpcLateralController : public trajectory_follower::LateralControllerBase
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit MpcLateralController(
    rclcpp::Node & node, std::shared_ptr<diagnostic_updater::Updater> diag_updater);
  virtual ~MpcLateralController();

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  rclcpp::Publisher<Trajectory>::SharedPtr m_pub_predicted_traj;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr m_pub_debug_values;
  rclcpp::Publisher<Float32Stamped>::SharedPtr m_pub_steer_offset;

  std::shared_ptr<diagnostic_updater::Updater>
    diag_updater_{};  // Diagnostic updater for publishing diagnostic data.

  //!< @brief parameters for path smoothing
  TrajectoryFilteringParam m_trajectory_filtering_param;

  // Ego vehicle speed threshold to enter the stop state.
  double m_stop_state_entry_ego_speed;

  // Target vehicle speed threshold to enter the stop state.
  double m_stop_state_entry_target_speed;

  // Convergence threshold for steering control.
  double m_converged_steer_rad;

  // max mpc output change threshold for 1 sec
  double m_mpc_converged_threshold_rps;

  // Time duration threshold to check if the trajectory shape has changed.
  double m_new_traj_duration_time;

  // Distance threshold to check if the trajectory shape has changed.
  double m_new_traj_end_dist;

  // Flag indicating whether to keep the steering control until it converges.
  bool m_keep_steer_control_until_converged;

  // MPC solver checker.
  bool m_is_mpc_solved{true};

  // trajectory buffer for detecting new trajectory
  std::deque<Trajectory> m_trajectory_buffer;

  void setStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void setupDiag();

  std::unique_ptr<MPC> m_mpc;  // MPC object for trajectory following.

  // Check is mpc output converged
  bool m_is_mpc_history_filled{false};

  // store the last mpc outputs for 1 sec
  std::vector<std::pair<Lateral, rclcpp::Time>> m_mpc_steering_history{};

  // set the mpc steering output to history
  void setSteeringToHistory(const Lateral & steering);

  // check if the mpc steering output is converged
  bool isMpcConverged();

  // measured kinematic state
  Odometry m_current_kinematic_state;

  SteeringReport m_current_steering;  // Measured steering information.

  Trajectory m_current_trajectory;  // Current reference trajectory for path following.

  double m_steer_cmd_prev = 0.0;  // MPC output in the previous period.

  // Flag indicating whether the previous control command is initialized.
  bool m_is_ctrl_cmd_prev_initialized = false;

  // Previous control command for path following.
  Lateral m_ctrl_cmd_prev;

  //  Flag indicating whether the first trajectory has been received.
  bool m_has_received_first_trajectory = false;

  // Threshold distance for the ego vehicle in nearest index search.
  double m_ego_nearest_dist_threshold;

  // Threshold yaw for the ego vehicle in nearest index search.
  double m_ego_nearest_yaw_threshold;

  // Flag indicating whether auto steering offset removal is enabled.
  bool enable_auto_steering_offset_removal_;

  // Steering offset estimator for offset compensation.
  std::shared_ptr<SteeringOffsetEstimator> steering_offset_;

  /**
   * @brief Initialize the timer
   * @param period_s Control period in seconds.
   */
  void initTimer(double period_s);

  /**
   * @brief Create the vehicle model based on the provided parameters.
   * @param wheelbase Vehicle's wheelbase.
   * @param steer_lim Steering command limit.
   * @param steer_tau Steering time constant.
   * @param node Reference to the node.
   * @return Pointer to the created vehicle model.
   */
  std::shared_ptr<VehicleModelInterface> createVehicleModel(
    const double wheelbase, const double steer_lim, const double steer_tau, rclcpp::Node & node);

  /**
   * @brief Create the quadratic problem solver interface.
   * @param node Reference to the node.
   * @return Pointer to the created QP solver interface.
   */
  std::shared_ptr<QPSolverInterface> createQPSolverInterface(rclcpp::Node & node);

  /**
   * @brief Create the steering offset estimator for offset compensation.
   * @param wheelbase Vehicle's wheelbase.
   * @param node Reference to the node.
   * @return Pointer to the created steering offset estimator.
   */
  std::shared_ptr<SteeringOffsetEstimator> createSteerOffsetEstimator(
    const double wheelbase, rclcpp::Node & node);

  /**
   * @brief Check if all necessary data is received and ready to run the control.
   * @param input_data Input data required for control calculation.
   * @return True if the data is ready, false otherwise.
   */
  bool isReady(const trajectory_follower::InputData & input_data) override;

  /**
   * @brief Compute the control command for path following with a constant control period.
   * @param input_data Input data required for control calculation.
   * @return Lateral output control command.
   */
  trajectory_follower::LateralOutput run(
    trajectory_follower::InputData const & input_data) override;

  /**
   * @brief Set the current trajectory using the received message.
   * @param msg Received trajectory message.
   */
  void setTrajectory(const Trajectory & msg, const Odometry & current_kinematics);

  /**
   * @brief Check if the received data is valid.
   * @return True if the data is valid, false otherwise.
   */
  bool checkData() const;

  /**
   * @brief Create the control command.
   * @param ctrl_cmd Control command to be created.
   * @return Created control command.
   */
  Lateral createCtrlCmdMsg(const Lateral & ctrl_cmd);

  /**
   * @brief Publish the predicted future trajectory.
   * @param predicted_traj Predicted future trajectory to be published.
   */
  void publishPredictedTraj(Trajectory & predicted_traj) const;

  /**
   * @brief Publish diagnostic message.
   * @param diagnostic Diagnostic message to be published.
   */
  void publishDebugValues(Float32MultiArrayStamped & diagnostic) const;

  /**
   * @brief Get the stop control command.
   * @return Stop control command.
   */
  Lateral getStopControlCommand() const;

  /**
   * @brief Get the control command applied before initialization.
   * @return Initial control command.
   */
  Lateral getInitialControlCommand() const;

  /**
   * @brief Check if the ego car is in a stopped state.
   * @return True if the ego car is stopped, false otherwise.
   */
  bool isStoppedState() const;

  /**
   * @brief Check if the trajectory has a valid value.
   * @param traj Trajectory to be checked.
   * @return True if the trajectory is valid, false otherwise.
   */
  bool isValidTrajectory(const Trajectory & traj) const;

  /**
   * @brief Check if the trajectory shape has changed.
   * @return True if the trajectory shape has changed, false otherwise.
   */
  bool isTrajectoryShapeChanged() const;

  /**
   * @brief Check if the steering control is converged and stable now.
   * @param cmd Steering control command to be checked.
   * @return True if the steering control is converged and stable, false otherwise.
   */
  bool isSteerConverged(const Lateral & cmd) const;

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  /**
   * @brief Declare MPC parameters as ROS parameters to allow tuning on the fly.
   * @param node Reference to the node.
   */
  void declareMPCparameters(rclcpp::Node & node);

  /**
   * @brief Callback function called when parameters are changed outside of the node.
   * @param parameters Vector of changed parameters.
   * @return Result of the parameter callback.
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  template <typename... Args>
  inline void info_throttle(Args &&... args)
  {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 5000, args...);
  }

  template <typename... Args>
  inline void warn_throttle(Args &&... args)
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, args...);
  }
};
}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__MPC_LATERAL_CONTROLLER_HPP_
