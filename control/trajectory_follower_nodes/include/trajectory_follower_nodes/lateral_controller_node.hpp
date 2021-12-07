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

#ifndef TRAJECTORY_FOLLOWER_NODES__LATERAL_CONTROLLER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODES__LATERAL_CONTROLLER_NODE_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "trajectory_follower_nodes/visibility_control.hpp"
#include "trajectory_follower/interpolate.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "trajectory_follower/mpc.hpp"
#include "trajectory_follower/mpc_trajectory.hpp"
#include "trajectory_follower/mpc_utils.hpp"
#include "trajectory_follower/qp_solver/qp_solver_osqp.hpp"
#include "trajectory_follower/qp_solver/qp_solver_unconstr_fast.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

#include "autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_system_msgs/msg/float32_multi_array_diagnostic.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "common/types.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "osqp_interface/osqp_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "vehicle_info_util/vehicle_info_util.hpp"


namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

class TRAJECTORY_FOLLOWER_PUBLIC LateralController : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit LateralController(const rclcpp::NodeOptions & node_options);

  /**
   * @brief destructor
   */
  virtual ~LateralController();

private:
  //!< @brief topic publisher for control command
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannLateralCommand>::SharedPtr
    m_pub_ctrl_cmd;
  //!< @brief topic publisher for predicted trajectory
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr m_pub_predicted_traj;
  //!< @brief topic publisher for control diagnostic
  rclcpp::Publisher<autoware_auto_system_msgs::msg::Float32MultiArrayDiagnostic>::SharedPtr
    m_pub_diagnostic;
  //!< @brief topic subscription for reference waypoints
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr m_sub_ref_path;
  //!< @brief subscription for current velocity
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_odometry;
  //!< @brief subscription for current steering
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr m_sub_steering;
  //!< @brief timer to update after a given interval
  rclcpp::TimerBase::SharedPtr m_timer;
  //!< @brief subscription for transform messages
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_sub;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_static_sub;

  /* parameters for path smoothing */
  //!< @brief flag for path smoothing
  bool8_t m_enable_path_smoothing;
  //!< @brief param of moving average filter for path smoothing
  int64_t m_path_filter_moving_ave_num;
  //!< @brief point-to-point index distance for curvature calculation for trajectory  //NOLINT
  int64_t m_curvature_smoothing_num_traj;
  //!< @brief point-to-point index distance for curvature calculation for reference steer command  //NOLINT
  int64_t m_curvature_smoothing_num_ref_steer;
  //!< @brief path resampling interval [m]
  float64_t m_traj_resample_dist;

  /* parameters for stop state */
  float64_t m_stop_state_entry_ego_speed;
  float64_t m_stop_state_entry_target_speed;

  // MPC object
  trajectory_follower::MPC m_mpc;

  //!< @brief measured pose
  geometry_msgs::msg::PoseStamped::SharedPtr m_current_pose_ptr;
  //!< @brief measured velocity
  nav_msgs::msg::Odometry::SharedPtr m_current_odometry_ptr;
  //!< @brief measured steering
  autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr m_current_steering_ptr;
  //!< @brief reference trajectory
  autoware_auto_planning_msgs::msg::Trajectory::SharedPtr
    m_current_trajectory_ptr;

  //!< @brief mpc filtered output in previous period
  float64_t m_steer_cmd_prev = 0.0;

  //!< @brief flag of m_ctrl_cmd_prev initialization
  bool8_t m_is_ctrl_cmd_prev_initialized = false;
  //!< @brief previous control command
  autoware_auto_control_msgs::msg::AckermannLateralCommand m_ctrl_cmd_prev;

  //!< @brief buffer for transforms
  tf2::BufferCore m_tf_buffer{tf2::BUFFER_CORE_DEFAULT_CACHE_TIME};
  tf2_ros::TransformListener m_tf_listener{m_tf_buffer};

  //!< initialize timer to work in real, simulation, and replay
  void initTimer(float64_t period_s);
  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void onTimer();

  /**
   * @brief set m_current_trajectory with received message
   */
  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr);

  /**
   * @brief update current_pose from tf
   * @return true if the current pose was updated, false otherwise
   */
  bool8_t updateCurrentPose();

  /**
   * @brief check if the received data is valid.
   */
  bool8_t checkData() const;

  /**
   * @brief set current_velocity with received message
   */
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief set current_steering with received message
   */
  void onSteering(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);

  /**
   * @brief publish control command
   * @param [in] cmd published control command
   */
  void publishCtrlCmd(autoware_auto_control_msgs::msg::AckermannLateralCommand cmd);

  /**
   * @brief publish predicted future trajectory
   * @param [in] predicted_traj published predicted trajectory
   */
  void publishPredictedTraj(autoware_auto_planning_msgs::msg::Trajectory & predicted_traj) const;

  /**
   * @brief publish diagnostic message
   * @param [in] diagnostic published diagnostic
   */
  void publishDiagnostic(autoware_auto_system_msgs::msg::Float32MultiArrayDiagnostic & diagnostic)
  const;

  /**
   * @brief get stop command
   */
  autoware_auto_control_msgs::msg::AckermannLateralCommand getStopControlCommand() const;

  /**
   * @brief get initial command
   */
  autoware_auto_control_msgs::msg::AckermannLateralCommand getInitialControlCommand() const;

  /**
   * @brief check ego car is in stopped state
   */
  bool8_t isStoppedState() const;

  /**
   * @brief check if the trajectory has valid value
   */
  bool8_t isValidTrajectory(const autoware_auto_planning_msgs::msg::Trajectory & traj) const;

  OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  /**
   * @brief Declare MPC parameters as ROS parameters to allow tuning on the fly
   */
  void declareMPCparameters();

  /**
   * @brief Called when parameters are changed outside of node
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
};
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER_NODES__LATERAL_CONTROLLER_NODE_HPP_
