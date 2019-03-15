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

#pragma once
#include <vector>
#include <iostream>
#include <limits>
#include <chrono>

#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleStatus.h>

#include "mpc_follower/mpc_utils.h"
#include "mpc_follower/mpc_trajectory.h"
#include "mpc_follower/lowpass_filter.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics.h"
#include "mpc_follower/qp_solver/qp_solver.h"

class MPCFollower
{
public:
  MPCFollower();
  ~MPCFollower();

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_steer_vel_ctrl_cmd_, pub_twist_cmd_;
  ros::Subscriber sub_ref_path_, sub_twist_, sub_pose_, sub_vehicle_status_;
  ros::Timer timer_control_;

  MPCTrajectory ref_traj_;                // reference trajectory for mpc
  Butterworth2dFilter lpf_steering_cmd_;  // steering command lowpass filter
  KinematicsBicycleModel vehicle_model_;  // vehicle model
  autoware_msgs::Lane current_waypoints_; // current received waypoints

  /* set vehicle control command interface */
  enum CtrlCmdInterface
  {
    TWIST = 0,
    STEER = 1,
    STEER_AND_VEL = 2,
    ALL = 3,
  };
  CtrlCmdInterface ctrl_cmd_interface_; // currentlly, twist or steer_and_vel

  /* parameters */
  double ctrl_period_;              // deside control frequency
  bool enable_path_smoothing_;      // flag for path smoothing
  bool enable_yaw_recalculation_;   // recalculate yaw angle after resampling
  int path_filter_moving_ave_num_;  // path smoothing moving average number
  int path_smoothing_times_;        // number of times of applying smoothing filter
  int curvature_smoothing_num_;     // for smoothing curvature calculation
  double traj_resample_dist_;       // path resample distance span
  double steering_lpf_cutoff_hz_;   // for steering command smoothing
  double admisible_position_error_; // stop mpc calculation when lateral error is large than this value.
  double admisible_yaw_error_deg_;  // stop mpc calculation when yaw error is large than this value.
  double steer_lim_deg_;            // steering command limit [rad]
  double wheelbase_;                // only used to convert steering to twist

  struct MPCParam
  {
    int n;                                  // prediction horizon step
    double dt;                              // prediction horizon period
    double weight_lat_error;                // for weight matrix Q
    double weight_heading_error;            // for weight matrix Q
    double weight_steering_input;           // for weight matrix R
    double weight_steering_input_vel_coeff; // for weight matrix R coeff of velocity
    double delay_compensation_time;         // use interpolation for time delay compensation
    double zero_curvature_range;     // set reference curvature to zero when the value is smaller than this.
  };
  MPCParam mpc_param_; // for mpc design

  struct VehicleStatus
  {
    std_msgs::Header header;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    double tire_angle_rad;
  };
  VehicleStatus vehicle_status_; // updated by topic callback

  /* flags */
  bool my_position_ok_;
  bool my_velocity_ok_;
  bool my_steering_ok_;

  void timerCallback(const ros::TimerEvent &);
  void callbackRefPath(const autoware_msgs::Lane::ConstPtr &);
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr &);
  void callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg);

  void publishControlCommands(const double &vel_cmd, const double &steer_cmd);
  void publishAsTwist(const double &vel_cmd, const double &omega_cmd);
  void publishSteerAndVel(const double &vel_cmd, const double &steer_cmd);

  bool calculateMPC(double &vel_cmd, double &steer_cmd);

  /* debug */
  bool show_debug_info_;
  ros::Publisher pub_debug_filtered_traj_, pub_debug_predicted_traj_, pub_debug_values_;
  ros::Subscriber sub_ndt_twist_;
  void convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::Marker &markers, std::string ns, double r, double g, double b);

  geometry_msgs::TwistStamped estimate_twist_;
  ros::Subscriber sub_estimate_twist_;
  void callbackEstimateTwist(const geometry_msgs::TwistStamped &msg) {
    estimate_twist_ = msg;
  }
};
