// Copyright 2020 Tier IV, Inc.
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

#ifndef OBSTACLE_STOP_PLANNER__ADAPTIVE_CRUISE_CONTROL_HPP_
#define OBSTACLE_STOP_PLANNER__ADAPTIVE_CRUISE_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#include <vector>

namespace motion_planning
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_auto_perception_msgs::msg::PredictedObject;
class AdaptiveCruiseController
{
public:
  AdaptiveCruiseController(
    rclcpp::Node * node, const double vehicle_width, const double vehicle_length,
    const double baselink2front);

  void insertAdaptiveCruiseVelocity(
    const TrajectoryPoints & trajectory, const int nearest_collision_point_idx,
    const geometry_msgs::msg::Pose self_pose, const pcl::PointXYZ & nearest_collision_point,
    const rclcpp::Time nearest_collision_point_time,
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr object_ptr,
    const nav_msgs::msg::Odometry::ConstSharedPtr current_velocity_ptr, bool * need_to_stop,
    TrajectoryPoints * output_trajectory, const std_msgs::msg::Header trajectory_header);

  void insertAdaptiveCruiseVelocity(
    const TrajectoryPoints & trajectory, const int nearest_collision_point_idx,
    const geometry_msgs::msg::Pose self_pose, const pcl::PointXYZ & nearest_collision_point,
    const rclcpp::Time nearest_collision_point_time,
    const nav_msgs::msg::Odometry::ConstSharedPtr current_velocity_ptr, bool * need_to_stop,
    TrajectoryPoints * output_trajectory, const std_msgs::msg::Header trajectory_header,
    const PredictedObject & target_object);

private:
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr pub_debug_;

  rclcpp::Node * node_;
  /*
   * Parameter
   */
  double vehicle_width_;
  double vehicle_length_;
  double baselink2front_;

  rclcpp::Time prev_collision_point_time_;
  pcl::PointXYZ prev_collision_point_;
  double prev_target_vehicle_time_ = 0.0;
  double prev_target_vehicle_dist_ = 0.0;
  double prev_target_velocity_ = 0.0;
  bool prev_collision_point_valid_ = false;
  bool prev_obstacle_velocity_judge_to_start_acc_ = false;
  std::vector<nav_msgs::msg::Odometry> est_vel_que_;
  double prev_upper_velocity_ = 0.0;

  struct Param
  {
    //!< @brief use tracking objects for estimating object velocity or not
    bool use_object_to_est_vel;

    //!< @brief use pcl for estimating object velocity or not
    bool use_pcl_to_est_vel;

    //!< @brief consider forward vehicle velocity to self upper velocity or not
    bool consider_obj_velocity;

    //!< @brief The distance to extend the polygon length the object in pointcloud-object matching
    double object_polygon_length_margin;

    //!< @brief The distance to extend the polygon width the object in pointcloud-object matching
    double object_polygon_width_margin;

    //!< @brief Maximum time difference treated as continuous points in speed estimation using a
    // point cloud
    double valid_est_vel_diff_time;

    //!< @brief Time width of information used for speed estimation in speed estimation using a
    // point cloud
    double valid_vel_que_time;

    //!< @brief Maximum value of valid speed estimation results in speed estimation using a point
    // cloud
    double valid_est_vel_max;

    //!< @brief Minimum value of valid speed estimation results in speed estimation using a point
    // cloud
    double valid_est_vel_min;

    //!< @brief Embed a stop line if the maximum speed calculated by ACC is lower than this speed
    double thresh_vel_to_stop;

    /* parameter for acc */
    //!< @brief start acc when the velocity of the forward obstacle exceeds this value
    double obstacle_velocity_thresh_to_start_acc;

    //!< @brief stop acc when the velocity of the forward obstacle falls below this value
    double obstacle_velocity_thresh_to_stop_acc;

    //!< @brief supposed minimum acceleration in emergency stop
    double emergency_stop_acceleration;

    //!< @brief supposed minimum acceleration of forward vehicle in emergency stop
    double obstacle_emergency_stop_acceleration;

    //!< @brief supposed idling time to start emergency stop
    double emergency_stop_idling_time;

    //!< @brief minimum distance of emergency stop
    double min_dist_stop;

    //!< @brief supposed maximum acceleration in active cruise control
    double max_standard_acceleration;

    //!< @brief supposed minimum acceleration(deceleration) in active cruise control
    double min_standard_acceleration;

    //!< @brief supposed idling time to react object in active cruise control
    double standard_idling_time;

    //!< @brief minimum distance in active cruise control
    double min_dist_standard;

    //!< @brief supposed minimum acceleration of forward obstacle
    double obstacle_min_standard_acceleration;

    //!< @brief margin to insert upper velocity
    double margin_rate_to_change_vel;

    //!< @brief use time-compensation to calculate distance to forward vehicle
    bool use_time_compensation_to_dist;

    //!< @brief gain of lowpass filter of upper velocity
    double lowpass_gain_;

    //!< @brief when failed to estimate velocity, use rough velocity estimation or not
    bool use_rough_est_vel;

    //!< @brief in rough velocity estimation, front car velocity is
    //!< estimated as self current velocity * this value
    double rough_velocity_rate;

    /* parameter for pid used in acc */
    //!< @brief coefficient P in PID control (used when target dist -current_dist >=0)
    double p_coeff_pos;

    //!< @brief coefficient P in PID control (used when target dist -current_dist <0)
    double p_coeff_neg;

    //!< @brief coefficient D in PID control (used when delta_dist >=0)
    double d_coeff_pos;

    //!< @brief coefficient D in PID control (used when delta_dist <0)
    double d_coeff_neg;

    static constexpr double d_coeff_valid_time = 1.0;
    static constexpr double d_coeff_valid_diff_vel = 20.0;
    static constexpr double d_max_vel_norm = 3.0;
  };
  Param param_;

  double getMedianVel(const std::vector<nav_msgs::msg::Odometry> vel_que);
  double lowpass_filter(const double current_value, const double prev_value, const double gain);
  void calcDistanceToNearestPointOnPath(
    const TrajectoryPoints & trajectory, const int nearest_point_idx,
    const geometry_msgs::msg::Pose & self_pose, const pcl::PointXYZ & nearest_collision_point,
    const rclcpp::Time & nearest_collision_point_time, double * distance,
    const std_msgs::msg::Header & trajectory_header);
  double calcTrajYaw(const TrajectoryPoints & trajectory, const int collision_point_idx);
  bool estimatePointVelocityFromObject(
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr object_ptr,
    const double traj_yaw, const pcl::PointXYZ & nearest_collision_point, double * velocity);
  bool estimatePointVelocityFromPcl(
    const double traj_yaw, const pcl::PointXYZ & nearest_collision_point,
    const rclcpp::Time & nearest_collision_point_time, double * velocity);
  void calculateProjectedVelocityFromObject(
    const PredictedObject & object, const double traj_yaw, double * velocity);
  double estimateRoughPointVelocity(double current_vel);
  bool isObstacleVelocityHigh(const double obj_vel);
  double calcUpperVelocity(const double dist_to_col, const double obj_vel, const double self_vel);
  double calcThreshDistToForwardObstacle(const double current_vel, const double obj_vel);
  double calcBaseDistToForwardObstacle(const double current_vel, const double obj_vel);
  double calcTargetVelocity_P(const double target_dist, const double current_dist);
  double calcTargetVelocity_I(const double target_dist, const double current_dist);
  double calcTargetVelocity_D(const double target_dist, const double current_dist);
  double calcTargetVelocityByPID(
    const double current_vel, const double current_dist, const double obj_vel);

  void insertMaxVelocityToPath(
    const geometry_msgs::msg::Pose self_pose, const double current_vel, const double target_vel,
    const double dist_to_collision_point, TrajectoryPoints * output_trajectory);
  void registerQueToVelocity(const double vel, const rclcpp::Time & vel_time);

  /* Debug */
  mutable tier4_debug_msgs::msg::Float32MultiArrayStamped debug_values_;
  enum DBGVAL {
    ESTIMATED_VEL_PCL = 0,
    ESTIMATED_VEL_OBJ = 1,
    ESTIMATED_VEL_FINAL = 2,
    FORWARD_OBJ_DISTANCE = 3,
    CURRENT_VEL = 4,
    UPPER_VEL_P = 5,
    UPPER_VEL_I = 6,
    UPPER_VEL_D = 7,
    UPPER_VEL_RAW = 8,
    UPPER_VEL = 9
  };
  static constexpr unsigned int num_debug_values_ = 10;
};

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__ADAPTIVE_CRUISE_CONTROL_HPP_
