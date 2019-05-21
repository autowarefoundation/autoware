/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file moc_follower.h
 * @brief mpc follower class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#pragma once
#include <vector>
#include <iostream>
#include <limits>
#include <chrono>

#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleStatus.h>

#include "mpc_follower/mpc_utils.h"
#include "mpc_follower/mpc_trajectory.h"
#include "mpc_follower/lowpass_filter.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_dynamics.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.h"
#include "mpc_follower/qp_solver/qp_solver_unconstr.h"
#include "mpc_follower/qp_solver/qp_solver_unconstr_fast.h"

/** 
 * @class MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */
class MPCFollower
{
public:
  /**
   * @brief constructor
   */
  MPCFollower();

  /**
   * @brief destructor
   */
  ~MPCFollower();

private:
  ros::NodeHandle nh_;                    //!< @brief ros node handle
  ros::NodeHandle pnh_;                   //!< @brief private ros node handle
  ros::Publisher pub_steer_vel_ctrl_cmd_; //!< @brief topic publisher for control command
  ros::Publisher pub_twist_cmd_;          //!< @brief topic publisher for twist command
  ros::Subscriber sub_ref_path_;          //!< @brief topic subscriber for reference waypoints
  ros::Subscriber sub_pose_;              //!< @brief subscriber for current pose
  ros::Subscriber sub_vehicle_status_;    //!< @brief subscriber for currrent vehicle status
  ros::Timer timer_control_;              //!< @brief timer for control command computation

  MPCTrajectory ref_traj_;                                   //!< @brief reference trajectory to be followed
  Butterworth2dFilter lpf_steering_cmd_;                     //!< @brief lowpass filter for steering command
  Butterworth2dFilter lpf_lateral_error_;                    //!< @brief lowpass filter for lateral error to calculate derivatie
  Butterworth2dFilter lpf_yaw_error_;                        //!< @brief lowpass filter for heading error to calculate derivatie
  autoware_msgs::Lane current_waypoints_;                    //!< @brief current waypoints to be followed
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_; //!< @brief vehicle model for MPC
  std::string vehicle_model_type_;                           //!< @brief vehicle model type for MPC
  std::shared_ptr<QPSolverInterface> qpsolver_ptr_;          //!< @brief qp solver for MPC
  std::string output_interface_;                             //!< @brief output command type

  /* parameters for control*/
  double ctrl_period_;              //!< @brief control frequency [s]
  double steering_lpf_cutoff_hz_;   //!< @brief cutoff frequency of lowpass filter for steering command [Hz]
  double admisible_position_error_; //!< @brief stop MPC calculation when lateral error is large than this value [m]
  double admisible_yaw_error_deg_;  //!< @brief stop MPC calculation when heading error is large than this value [deg]
  double steer_lim_deg_;            //!< @brief steering command limit [rad]
  double wheelbase_;                //!< @brief vehicle wheelbase length [m] to convert steering angle to angular velocity
  double steering_gear_ratio_;      //!< @brief steering gear ratio to convert steering to steering_wheel

  /* parameters for path smoothing */
  bool enable_path_smoothing_;     //< @brief flag for path smoothing
  bool enable_yaw_recalculation_;  //< @brief flag for recalculation of yaw angle after resampling
  int path_filter_moving_ave_num_; //< @brief param of moving average filter for path smoothing
  int path_smoothing_times_;       //< @brief number of times of applying path smoothing filter
  int curvature_smoothing_num_;    //< @brief point-to-point index distance used in curvature calculation
  double traj_resample_dist_;      //< @brief path resampling interval [m]

  struct MPCParam
  {
    int prediction_horizon;                         //< @brief prediction horizon step
    double prediction_sampling_time;                //< @brief prediction horizon period
    double weight_lat_error;                        //< @brief lateral error weight in matrix Q
    double weight_heading_error;                    //< @brief heading error weight in matrix Q
    double weight_heading_error_squared_vel_coeff;  //< @brief heading error * velocity weight in matrix Q
    double weight_steering_input;                   //< @brief steering error weight in matrix R
    double weight_steering_input_squared_vel_coeff; //< @brief steering error * velocity weight in matrix R
    double weight_lat_jerk;                         //< @brief lateral jerk weight in matrix R
    double weight_terminal_lat_error;               //< @brief terminal lateral error weight in matrix Q
    double weight_terminal_heading_error;           //< @brief terminal heading error weight in matrix Q
    double zero_ff_steer_deg;                       //< @brief threshold that feed-forward angle becomes zero
  };
  MPCParam mpc_param_; // for mpc design parameter

  struct VehicleStatus
  {
    std_msgs::Header header;    //< @brief header
    geometry_msgs::Pose pose;   //< @brief vehicle pose
    geometry_msgs::Twist twist; //< @brief vehicle velocity
    double tire_angle_rad;      //< @brief vehicle tire angle
  };
  VehicleStatus vehicle_status_; //< @brief vehicle status

  double steer_cmd_prev_;     //< @brief steering command calculated in previous period
  double lateral_error_prev_; //< @brief previous lateral error for derivative
  double yaw_error_prev_;     //< @brief previous lateral error for derivative

  /* flags */
  bool my_position_ok_; //< @brief flag for validity of current pose
  bool my_velocity_ok_; //< @brief flag for validity of current velocity
  bool my_steering_ok_; //< @brief flag for validity of steering angle

  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void timerCallback(const ros::TimerEvent &);

  /**
   * @brief set current_waypoints_ with receved message
   */
  void callbackRefPath(const autoware_msgs::Lane::ConstPtr &);

  /**
   * @brief set vehicle_status_.pose with receved message 
   */
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr &);

  /**
   * @brief set vehicle_status_.twist and vehicle_status_.tire_angle_rad with receved message
   */
  void callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg);

  /**
   * @brief publish control command calculated by MPC
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] acc_cmd acceleration command [m/s2] for vehicle control
   * @param [in] steer_cmd steering angle command [rad] for vehicle control
   * @param [in] steer_vel_cmd steering angle speed [rad/s] for vehicle control
   */
  void publishControlCommands(const double &vel_cmd, const double &acc_cmd,
                              const double &steer_cmd, const double &steer_vel_cmd);

  /**
   * @brief publish control command as geometry_msgs/TwistStamped type
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] omega_cmd angular velocity command [rad/s] for vehicle control
   */
  void publishTwist(const double &vel_cmd, const double &omega_cmd);

  /**
   * @brief publish control command as autoware_msgs/ControlCommand type
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] acc_cmd acceleration command [m/s2] for vehicle control
   * @param [in] steer_cmd steering angle command [rad] for vehicle control
   */
  void publishCtrlCmd(const double &vel_cmd, const double &acc_cmd, const double &steer_cmd);

  /**
   * @brief calculate control command by MPC algorithm
   * @param [out] vel_cmd velocity command
   * @param [out] acc_cmd acceleration command
   * @param [out] steer_cmd steering command
   * @param [out] steer_vel_cmd steering rotation speed command
   */
  bool calculateMPC(double &vel_cmd, double &acc_cmd, double &steer_cmd, double &steer_vel_cmd);

  /* debug */
  bool show_debug_info_;      //!< @brief flag to display debug info
  bool publish_debug_values_; //!< @brief flag to publish debug info

  ros::Publisher pub_debug_filtered_traj_;        //!< @brief publisher for debug info
  ros::Publisher pub_debug_predicted_traj_;       //!< @brief publisher for debug info
  ros::Publisher pub_debug_values_;               //!< @brief publisher for debug info
  ros::Publisher pub_debug_mpc_calc_time_;        //!< @brief publisher for debug info
  ros::Publisher pub_debug_steer_cmd_;            //!< @brief publisher for debug info
  ros::Publisher pub_debug_steer_cmd_raw_;        //!< @brief publisher for debug info
  ros::Publisher pub_debug_steer_cmd_ff_;         //!< @brief publisher for debug info
  ros::Publisher pub_debug_steer_;                //!< @brief publisher for debug info
  ros::Publisher pub_debug_current_vel_;          //!< @brief publisher for debug info
  ros::Publisher pub_debug_vel_cmd_;              //!< @brief publisher for debug info
  ros::Publisher pub_debug_laterr_;               //!< @brief publisher for debug info
  ros::Publisher pub_debug_yawerr_;               //!< @brief publisher for debug info
  ros::Publisher pub_debug_angvel_cmd_;           //!< @brief publisher for debug info
  ros::Publisher pub_debug_angvel_steer_;         //!< @brief publisher for debug info
  ros::Publisher pub_debug_angvel_cmd_ff_;        //!< @brief publisher for debug info
  ros::Publisher pub_debug_angvel_estimatetwist_; //!< @brief publisher for debug info

  ros::Subscriber sub_estimate_twist_;         //!< @brief subscriber for /estimate_twist for debug
  geometry_msgs::TwistStamped estimate_twist_; //!< @brief received /estimate_twist for debug

  /**
   * @brief convert MPCTraj to visualizaton marker for visualization
   */
  void convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::Marker &markers,
                           std::string ns, double r, double g, double b, double z);

  /**
   * @brief callback for estimate twist for debug
   */
  void callbackEstimateTwist(const geometry_msgs::TwistStamped &msg) { estimate_twist_ = msg; }
};
