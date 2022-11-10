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

#include "control_performance_analysis/control_performance_analysis_core.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace control_performance_analysis
{
using geometry_msgs::msg::Quaternion;

ControlPerformanceAnalysisCore::ControlPerformanceAnalysisCore()
{
  prev_target_vars_ = std::make_unique<msg::ErrorStamped>();
  prev_driving_vars_ = std::make_unique<msg::DrivingMonitorStamped>();
  odom_history_ptr_ = std::make_shared<std::vector<Odometry>>();
  p_.odom_interval_ = 0;
  p_.curvature_interval_length_ = 10.0;
  p_.acceptable_max_distance_to_waypoint_ = 1.5;
  p_.acceptable_max_yaw_difference_rad_ = 1.0472;
  p_.prevent_zero_division_value_ = 0.001;
  p_.lpf_gain_ = 0.8;
  p_.wheelbase_ = 2.74;
}

ControlPerformanceAnalysisCore::ControlPerformanceAnalysisCore(Params & p) : p_{p}
{
  // prepare control performance struct
  prev_target_vars_ = std::make_unique<msg::ErrorStamped>();
  prev_driving_vars_ = std::make_unique<msg::DrivingMonitorStamped>();
  odom_history_ptr_ = std::make_shared<std::vector<Odometry>>();
}

void ControlPerformanceAnalysisCore::setCurrentWaypoints(const Trajectory & trajectory)
{
  current_waypoints_ptr_ = std::make_shared<PoseArray>();
  current_waypoints_vel_ptr_ = std::make_shared<std::vector<double>>();

  for (const auto & point : trajectory.points) {
    current_waypoints_ptr_->poses.emplace_back(point.pose);
    current_waypoints_vel_ptr_->emplace_back(point.longitudinal_velocity_mps);
  }
}

void ControlPerformanceAnalysisCore::setOdomHistory(const Odometry & odom)
{
  // We need to take the odometry history to calculate jerk and acceleration
  if (!odom_history_ptr_->empty() && odom.header.stamp == odom_history_ptr_->back().header.stamp) {
    return;
  }
  if (odom_history_ptr_->size() >= (3 + p_.odom_interval_ * 2)) {
    // If higher, remove the first element of vector
    odom_history_ptr_->erase(odom_history_ptr_->begin());
    odom_history_ptr_->push_back(odom);
  } else {
    odom_history_ptr_->push_back(odom);
  }
}

void ControlPerformanceAnalysisCore::setCurrentPose(const Pose & msg)
{
  current_vec_pose_ptr_ = std::make_shared<Pose>(msg);
}

void ControlPerformanceAnalysisCore::setCurrentControlValue(const AckermannControlCommand & msg)
{
  current_control_ptr_ = std::make_shared<AckermannControlCommand>(msg);
}

std::pair<bool, int32_t> ControlPerformanceAnalysisCore::findClosestPrevWayPointIdx_path_direction()
{
  if (!isDataReady()) {
    return std::make_pair(false, std::numeric_limits<int32_t>::quiet_NaN());
  }

  auto closest_idx = motion_utils::findNearestIndex(
    current_waypoints_ptr_->poses, *current_vec_pose_ptr_, p_.acceptable_max_distance_to_waypoint_,
    p_.acceptable_max_yaw_difference_rad_);

  // find the prev and next waypoint

  if (*closest_idx != 0 && (*closest_idx + 1) != current_waypoints_ptr_->poses.size()) {
    const double dist_to_prev = std::hypot(
      current_vec_pose_ptr_->position.x -
        current_waypoints_ptr_->poses.at(*closest_idx - 1).position.x,
      current_vec_pose_ptr_->position.y -
        current_waypoints_ptr_->poses.at(*closest_idx - 1).position.y);
    const double dist_to_next = std::hypot(
      current_vec_pose_ptr_->position.x -
        current_waypoints_ptr_->poses.at(*closest_idx + 1).position.x,
      current_vec_pose_ptr_->position.y -
        current_waypoints_ptr_->poses.at(*closest_idx + 1).position.y);
    if (dist_to_next > dist_to_prev) {
      idx_prev_wp_ = std::make_unique<int32_t>(*closest_idx - 1);
      idx_next_wp_ = std::make_unique<int32_t>(*closest_idx);
    } else {
      idx_prev_wp_ = std::make_unique<int32_t>(*closest_idx);
      idx_next_wp_ = std::make_unique<int32_t>(*closest_idx + 1);
    }
  } else if (*closest_idx == 0) {
    idx_prev_wp_ = std::make_unique<int32_t>(*closest_idx);
    idx_next_wp_ = std::make_unique<int32_t>(*closest_idx + 1);
  } else {
    idx_prev_wp_ = std::make_unique<int32_t>(*closest_idx - 1);
    idx_next_wp_ = std::make_unique<int32_t>(*closest_idx);
  }
  return (idx_prev_wp_ && idx_next_wp_)
           ? std::make_pair(true, *idx_prev_wp_)
           : std::make_pair(false, std::numeric_limits<int32_t>::quiet_NaN());
}

bool ControlPerformanceAnalysisCore::isDataReady() const
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  if (!current_vec_pose_ptr_) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock, 1000, "cannot get current pose into control_performance algorithm");
    return false;
  }

  if (current_waypoints_ptr_->poses.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, clock, 1000, "cannot get current trajectory waypoints ...");
    return false;
  }

  if (odom_history_ptr_->size() < 2) {
    RCLCPP_WARN_THROTTLE(logger_, clock, 1000, "waiting for odometry data ...");
    return false;
  }

  if (!current_control_ptr_) {
    RCLCPP_WARN_THROTTLE(logger_, clock, 1000, "waiting for current_control_cmd ...");
    return false;
  }

  if (!current_vec_steering_msg_ptr_) {
    RCLCPP_WARN_THROTTLE(logger_, clock, 1000, "waiting for current_steering ...");
    return false;
  }

  return true;
}

bool ControlPerformanceAnalysisCore::calculateErrorVars()
{
  // Check if data is ready.
  if (!isDataReady() || !idx_prev_wp_) {
    return false;
  }

  // Get the interpolated pose
  std::pair<bool, Pose> pair_pose_interp_wp_ = calculateClosestPose();

  // Find and set the waypoint L-wheelbase meters ahead of the current waypoint.
  findCurveRefIdx();

  if (
    !pair_pose_interp_wp_.first || !interpolated_pose_ptr_ || !interpolated_velocity_ptr_ ||
    !interpolated_acceleration_ptr_ || !interpolated_steering_angle_ptr_) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000,
      "Cannot get interpolated pose, velocity, acceleration, and steering into control_performance "
      "algorithm");
    return false;
  }

  const auto pose_interp_wp_ = pair_pose_interp_wp_.second;

  // Create interpolated waypoint vector
  const std::vector<double> interp_waypoint_xy{
    pose_interp_wp_.position.x, pose_interp_wp_.position.y};

  // Create vehicle position vector
  const std::vector<double> vehicle_position_xy{
    current_vec_pose_ptr_->position.x, current_vec_pose_ptr_->position.y};

  // Get Yaw angles of the reference waypoint and the vehicle
  const double & target_yaw = tf2::getYaw(pose_interp_wp_.orientation);
  const double & vehicle_yaw_angle = tf2::getYaw(current_vec_pose_ptr_->orientation);

  // Compute Curvature at the point where the front axle might follow
  // get the waypoint corresponds to the front_axle center

  if (!idx_curve_ref_wp_) {
    RCLCPP_WARN(logger_, "Cannot find index of curvature reference waypoint ");
    return false;
  }

  const double & curvature_est = estimateCurvature();                // three point curvature
  const double & curvature_est_pp = estimatePurePursuitCurvature();  // pure pursuit curvature

  // Compute lateral, longitudinal, heading error w.r.t. frenet frame

  std::vector<double> lateral_longitudinal_error =
    utils::computeLateralLongitudinalError(interp_waypoint_xy, vehicle_position_xy, target_yaw);
  const double & lateral_error = lateral_longitudinal_error[0];
  const double & longitudinal_error = lateral_longitudinal_error[1];

  // Compute the yaw angle error.
  const double & heading_yaw_error = utils::angleDistance(vehicle_yaw_angle, target_yaw);

  // Set the values of ErrorMsgVars.

  error_vars.error.lateral_error = lateral_error;
  error_vars.error.longitudinal_error = longitudinal_error;
  error_vars.error.heading_error = heading_yaw_error;

  // odom history contains k + 1, k, k - 1 ... steps. We are in kth step
  const uint & odom_size = odom_history_ptr_->size();

  error_vars.header.stamp = odom_history_ptr_->at(odom_size - 2).header.stamp;  // we are in step k

  const double & Vx = odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x;
  // Current acceleration calculation
  const double & d_x = odom_history_ptr_->at(odom_size - 1).pose.pose.position.x -
                       odom_history_ptr_->at(odom_size - 2).pose.pose.position.x;
  const double & d_y = odom_history_ptr_->at(odom_size - 1).pose.pose.position.y -
                       odom_history_ptr_->at(odom_size - 2).pose.pose.position.y;
  const double & ds = std::hypot(d_x, d_y);

  const double & vel_mean = (odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x +
                             odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x) /
                            2.0;
  const double & dv = odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x -
                      odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x;
  const double & dt = ds / std::max(vel_mean, p_.prevent_zero_division_value_);
  const double & Ax = dv / std::max(dt, p_.prevent_zero_division_value_);  // current acceleration

  const double & longitudinal_error_velocity =
    Vx * cos(heading_yaw_error) - *interpolated_velocity_ptr_ * (1 - curvature_est * lateral_error);
  const double & lateral_error_velocity =
    Vx * sin(heading_yaw_error) - *interpolated_velocity_ptr_ * curvature_est * longitudinal_error;

  const double & steering_cmd = current_control_ptr_->lateral.steering_tire_angle;
  const double & current_steering_val = current_vec_steering_msg_ptr_->steering_tire_angle;
  error_vars.error.control_effort_energy = contR * steering_cmd * steering_cmd;  // u*R*u';

  const double & heading_velocity_error = (Vx * tan(current_steering_val)) / p_.wheelbase_ -
                                          *this->interpolated_velocity_ptr_ * curvature_est;

  const double & lateral_acceleration_error =
    -curvature_est * *interpolated_acceleration_ptr_ * longitudinal_error -
    curvature_est * *interpolated_velocity_ptr_ * longitudinal_error_velocity +
    Vx * heading_velocity_error * cos(heading_yaw_error) + Ax * sin(heading_yaw_error);

  const double & longitudinal_acceleration_error =
    curvature_est * *interpolated_acceleration_ptr_ * lateral_error +
    curvature_est * *interpolated_velocity_ptr_ * lateral_error_velocity -
    Vx * heading_velocity_error * sin(heading_yaw_error) + Ax * cos(heading_yaw_error) -
    *interpolated_acceleration_ptr_;

  error_vars.error.lateral_error_velocity = lateral_error_velocity;
  error_vars.error.lateral_error_acceleration = lateral_acceleration_error;
  error_vars.error.longitudinal_error_velocity = longitudinal_error_velocity;
  error_vars.error.longitudinal_error_acceleration = longitudinal_acceleration_error;
  error_vars.error.heading_error_velocity = heading_velocity_error;

  Eigen::Vector2d error_vec;
  error_vec << lateral_error, heading_yaw_error;

  error_vars.error.error_energy = error_vec.dot(error_vec);
  error_vars.error.value_approximation = error_vec.transpose() * lyap_P_ * error_vec;  // x'Px

  error_vars.error.curvature_estimate = curvature_est;
  error_vars.error.curvature_estimate_pp = curvature_est_pp;

  error_vars.error.vehicle_velocity_error = Vx - *this->interpolated_velocity_ptr_;
  error_vars.error.tracking_curvature_discontinuity_ability =
    (std::fabs(curvature_est - prev_target_vars_->error.curvature_estimate)) /
    (1 + std::fabs(lateral_error - prev_target_vars_->error.lateral_error));

  if (prev_target_vars_) {
    // LPF for error vars

    error_vars.error.curvature_estimate =
      p_.lpf_gain_ * prev_target_vars_->error.curvature_estimate +
      (1 - p_.lpf_gain_) * error_vars.error.curvature_estimate;

    error_vars.error.curvature_estimate_pp =
      p_.lpf_gain_ * prev_target_vars_->error.curvature_estimate_pp +
      (1 - p_.lpf_gain_) * error_vars.error.curvature_estimate_pp;

    error_vars.error.lateral_error = p_.lpf_gain_ * prev_target_vars_->error.lateral_error +
                                     (1 - p_.lpf_gain_) * error_vars.error.lateral_error;

    error_vars.error.lateral_error_velocity =
      p_.lpf_gain_ * prev_target_vars_->error.lateral_error_velocity +
      (1 - p_.lpf_gain_) * error_vars.error.lateral_error_velocity;

    error_vars.error.lateral_error_acceleration =
      p_.lpf_gain_ * prev_target_vars_->error.lateral_error_acceleration +
      (1 - p_.lpf_gain_) * error_vars.error.lateral_error_acceleration;

    error_vars.error.longitudinal_error =
      p_.lpf_gain_ * prev_target_vars_->error.longitudinal_error +
      (1 - p_.lpf_gain_) * error_vars.error.longitudinal_error;

    error_vars.error.longitudinal_error_velocity =
      p_.lpf_gain_ * prev_target_vars_->error.longitudinal_error_velocity +
      (1 - p_.lpf_gain_) * error_vars.error.longitudinal_error_velocity;

    error_vars.error.longitudinal_error_acceleration =
      p_.lpf_gain_ * prev_target_vars_->error.longitudinal_error_acceleration +
      (1 - p_.lpf_gain_) * error_vars.error.longitudinal_error_acceleration;

    error_vars.error.heading_error = p_.lpf_gain_ * prev_target_vars_->error.heading_error +
                                     (1 - p_.lpf_gain_) * error_vars.error.heading_error;

    error_vars.error.heading_error_velocity =
      p_.lpf_gain_ * prev_target_vars_->error.heading_error_velocity +
      (1 - p_.lpf_gain_) * error_vars.error.heading_error_velocity;

    error_vars.error.control_effort_energy =
      p_.lpf_gain_ * prev_target_vars_->error.control_effort_energy +
      (1 - p_.lpf_gain_) * error_vars.error.control_effort_energy;

    error_vars.error.error_energy = p_.lpf_gain_ * prev_target_vars_->error.error_energy +
                                    (1 - p_.lpf_gain_) * error_vars.error.error_energy;
  }

  prev_target_vars_ = std::make_unique<msg::ErrorStamped>(error_vars);

  return true;
}

bool ControlPerformanceAnalysisCore::calculateDrivingVars()
{
  if (!odom_history_ptr_->empty()) {
    const uint odom_size = odom_history_ptr_->size();

    if (odom_history_ptr_->at(odom_size - 1).header.stamp != last_odom_header.stamp) {
      //  Add desired steering angle

      if (interpolated_steering_angle_ptr_) {
        driving_status_vars.desired_steering_angle.header =
          odom_history_ptr_->at(odom_size - 1).header;
        driving_status_vars.desired_steering_angle.data = *interpolated_steering_angle_ptr_;
      }

      //  Calculate lateral acceleration

      driving_status_vars.lateral_acceleration.header.set__stamp(
        odom_history_ptr_->at(odom_size - 1).header.stamp);
      driving_status_vars.lateral_acceleration.data =
        odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x *
        tan(current_vec_steering_msg_ptr_->steering_tire_angle) / p_.wheelbase_;

      if (odom_history_ptr_->size() >= p_.odom_interval_ + 2) {
        // Calculate longitudinal acceleration

        const double dv =
          odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x -
          odom_history_ptr_->at(odom_size - p_.odom_interval_ - 2).twist.twist.linear.x;

        const auto odom_duration =
          (rclcpp::Time(odom_history_ptr_->at(odom_size - 1).header.stamp) -
           rclcpp::Time(odom_history_ptr_->at(odom_size - p_.odom_interval_ - 2).header.stamp));

        const double dt_odom = odom_duration.seconds();

        driving_status_vars.longitudinal_acceleration.data = dv / dt_odom;
        driving_status_vars.longitudinal_acceleration.header.set__stamp(
          rclcpp::Time(odom_history_ptr_->at(odom_size - p_.odom_interval_ - 2).header.stamp) +
          odom_duration * 0.5);  // Time stamp of acceleration data

        //  Calculate lateral jerk

        const double d_lateral_jerk = driving_status_vars.lateral_acceleration.data -
                                      prev_driving_vars_->lateral_acceleration.data;

        //  We already know the delta time from above. same as longitudinal acceleration

        driving_status_vars.lateral_jerk.data = d_lateral_jerk / dt_odom;
        driving_status_vars.lateral_jerk.header =
          driving_status_vars.longitudinal_acceleration.header;
      }

      if (odom_history_ptr_->size() == 2 * p_.odom_interval_ + 3) {
        // calculate longitudinal jerk
        const double d_a = driving_status_vars.longitudinal_acceleration.data -
                           prev_driving_vars_->longitudinal_acceleration.data;
        const auto duration =
          (rclcpp::Time(driving_status_vars.longitudinal_acceleration.header.stamp) -
           rclcpp::Time(prev_driving_vars_->longitudinal_acceleration.header.stamp));
        const double dt = duration.seconds();
        driving_status_vars.longitudinal_jerk.data = d_a / dt;
        driving_status_vars.longitudinal_jerk.header.set__stamp(
          rclcpp::Time(prev_driving_vars_->longitudinal_acceleration.header.stamp) +
          duration * 0.5);  // Time stamp of jerk data
      }
      if (prev_driving_vars_) {
        // LPF for driving status vars

        driving_status_vars.longitudinal_acceleration.data =
          p_.lpf_gain_ * prev_driving_vars_->longitudinal_acceleration.data +
          (1 - p_.lpf_gain_) * driving_status_vars.longitudinal_acceleration.data;

        driving_status_vars.lateral_acceleration.data =
          p_.lpf_gain_ * prev_driving_vars_->lateral_acceleration.data +
          (1 - p_.lpf_gain_) * driving_status_vars.lateral_acceleration.data;

        driving_status_vars.lateral_jerk.data =
          p_.lpf_gain_ * prev_driving_vars_->lateral_jerk.data +
          (1 - p_.lpf_gain_) * driving_status_vars.lateral_jerk.data;

        driving_status_vars.longitudinal_jerk.data =
          p_.lpf_gain_ * prev_driving_vars_->longitudinal_jerk.data +
          (1 - p_.lpf_gain_) * driving_status_vars.longitudinal_jerk.data;

        driving_status_vars.controller_processing_time.data =
          p_.lpf_gain_ * prev_driving_vars_->controller_processing_time.data +
          (1 - p_.lpf_gain_) * driving_status_vars.controller_processing_time.data;

        driving_status_vars.desired_steering_angle.data =
          p_.lpf_gain_ * prev_driving_vars_->desired_steering_angle.data +
          (1 - p_.lpf_gain_) * driving_status_vars.desired_steering_angle.data;
      }

      prev_driving_vars_ =
        std::move(std::make_unique<msg::DrivingMonitorStamped>(driving_status_vars));

      last_odom_header.stamp = odom_history_ptr_->at(odom_size - 1).header.stamp;
      last_steering_report.stamp = current_vec_steering_msg_ptr_->stamp;

    } else if (last_steering_report.stamp != current_vec_steering_msg_ptr_->stamp) {
      driving_status_vars.lateral_acceleration.header.set__stamp(
        current_vec_steering_msg_ptr_->stamp);
      driving_status_vars.lateral_acceleration.data =
        odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x *
        tan(current_vec_steering_msg_ptr_->steering_tire_angle) / p_.wheelbase_;
      last_steering_report.stamp = current_vec_steering_msg_ptr_->stamp;
    }
    return true;

  } else {
    RCLCPP_ERROR(logger_, "Can not get odometry data! ");
    return false;
  }
}

void ControlPerformanceAnalysisCore::setSteeringStatus(const SteeringReport & steering)
{
  current_vec_steering_msg_ptr_ = std::make_shared<SteeringReport>(steering);
}

void ControlPerformanceAnalysisCore::findCurveRefIdx()
{
  // Get the previous waypoint as the reference
  if (!interpolated_pose_ptr_) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000, "Cannot set the curvature_idx, no valid interpolated pose ...");

    return;
  }

  auto fun_distance_cond = [this](auto pose_t) {
    const double dist = std::hypot(
      pose_t.position.x - this->interpolated_pose_ptr_->position.x,
      pose_t.position.y - this->interpolated_pose_ptr_->position.y);

    return dist > p_.wheelbase_;
  };

  auto it = std::find_if(
    current_waypoints_ptr_->poses.cbegin() + *idx_prev_wp_, current_waypoints_ptr_->poses.cend(),
    fun_distance_cond);

  if (it == current_waypoints_ptr_->poses.cend()) {
    it = std::prev(it);
  }

  const int32_t & temp_idx_curve_ref_wp = std::distance(current_waypoints_ptr_->poses.cbegin(), it);
  idx_curve_ref_wp_ = std::make_unique<int32_t>(temp_idx_curve_ref_wp);
}

std::pair<bool, Pose> ControlPerformanceAnalysisCore::calculateClosestPose()
{
  Pose temp_interpolated_pose;

  // Get index of prev waypoint and sanity check.
  if (!idx_prev_wp_ && !idx_next_wp_) {
    RCLCPP_ERROR(logger_, "Cannot find the previous and next waypoints.");
    return std::make_pair(false, temp_interpolated_pose);
  }

  // Define the next waypoint - so that we can define an interval in which the car follow a line.
  double next_wp_acc = 0.0;
  double prev_wp_acc = 0.0;

  /*
   *  Create two vectors originating from the previous waypoints to the next waypoint and the
   *  vehicle position and find projection of vehicle vector on the the trajectory section,
   *
   * */

  // First get te yaw angles of all three poses.
  const double & prev_yaw =
    tf2::getYaw(current_waypoints_ptr_->poses.at(*idx_prev_wp_).orientation);
  const double & prev_velocity = current_waypoints_vel_ptr_->at(*idx_prev_wp_);
  const double & next_velocity = current_waypoints_vel_ptr_->at(*idx_next_wp_);

  // Previous waypoint to next waypoint.
  const double & dx_prev2next = current_waypoints_ptr_->poses.at(*idx_next_wp_).position.x -
                                current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.x;

  const double & dy_prev2next = current_waypoints_ptr_->poses.at(*idx_next_wp_).position.y -
                                current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.y;

  const double & delta_psi_prev2next = tf2::getYaw(
    current_waypoints_ptr_->poses.at(*idx_next_wp_).orientation -
    current_waypoints_ptr_->poses.at(*idx_prev_wp_).orientation);
  const double & d_vel_prev2next = next_velocity - prev_velocity;

  // Create a vector from p0 (prev) --> p1 (to next wp)
  const std::vector<double> v_prev2next_wp{dx_prev2next, dy_prev2next};

  // Previous waypoint to the vehicle pose
  /*
   *   p0:previous waypoint ----> p1 next waypoint
   *   vector = p1 - p0. We project vehicle vector on this interval to
   *
   * */

  const double & dx_prev2vehicle =
    current_vec_pose_ptr_->position.x - current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.x;

  const double & dy_prev2vehicle =
    current_vec_pose_ptr_->position.y - current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.y;

  // Vector from p0 to p_vehicle
  const std::vector<double> v_prev2vehicle{dx_prev2vehicle, dy_prev2vehicle};

  // Compute the length of v_prev2next_wp : vector from p0 --> p1
  const double & distance_p02p1 = std::hypot(dx_prev2next, dy_prev2next);

  // Compute how far the car is away from p0 in p1 direction. p_interp is the location of the
  // interpolated waypoint. This is the dot product normalized by the length of the interval.
  // a.b = |a|.|b|.cos(alpha) -- > |a|.cos(alpha) = a.b / |b| where b is the path interval,

  const double & distance_p02p_interp =
    (dx_prev2next * dx_prev2vehicle + dy_prev2next * dy_prev2vehicle) / distance_p02p1;

  //  const double & distance_p_interp2p1 = distance_p02p1 - distance_p02p_interp;
  /*
   * We use the following linear interpolation
   *  pi = p0 + ratio_t * (p1 - p0)
   * */

  const double & ratio_t = distance_p02p_interp / distance_p02p1;

  // Interpolate pose.position and pose.orientation
  temp_interpolated_pose.position.x =
    current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.x + ratio_t * dx_prev2next;

  temp_interpolated_pose.position.y =
    current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.y + ratio_t * dy_prev2next;

  temp_interpolated_pose.position.z = 0.0;

  // Interpolate the yaw angle of pi : interpolated waypoint
  const double & interp_yaw_angle = prev_yaw + ratio_t * delta_psi_prev2next;
  const double & interp_velocity = prev_velocity + ratio_t * d_vel_prev2next;

  const Quaternion & orient_msg = utils::createOrientationMsgFromYaw(interp_yaw_angle);
  temp_interpolated_pose.orientation = orient_msg;

  /* interpolated acceleration calculation */

  if (
    *idx_prev_wp_ == 0 ||
    static_cast<size_t>(*idx_prev_wp_ + 1) == current_waypoints_ptr_->poses.size()) {
    prev_wp_acc = 0.0;
  } else {
    const double & d_x = current_waypoints_ptr_->poses.at(*idx_next_wp_).position.x -
                         current_waypoints_ptr_->poses.at(*idx_prev_wp_ - 1).position.x;
    const double & d_y = current_waypoints_ptr_->poses.at(*idx_next_wp_).position.y -
                         current_waypoints_ptr_->poses.at(*idx_prev_wp_ - 1).position.y;
    const double & ds = std::hypot(d_x, d_y);
    const double & vel_mean = (current_waypoints_vel_ptr_->at(*idx_next_wp_) +
                               current_waypoints_vel_ptr_->at(*idx_prev_wp_ - 1)) /
                              2.0;
    const double & dv = current_waypoints_vel_ptr_->at(*idx_next_wp_) -
                        current_waypoints_vel_ptr_->at(*idx_prev_wp_ - 1);
    const double & dt = ds / std::max(vel_mean, p_.prevent_zero_division_value_);
    prev_wp_acc = dv / std::max(dt, p_.prevent_zero_division_value_);
  }

  if (static_cast<size_t>(*idx_next_wp_ + 1) == current_waypoints_ptr_->poses.size()) {
    next_wp_acc = 0.0;
  } else {
    const double & d_x = current_waypoints_ptr_->poses.at(*idx_next_wp_ + 1).position.x -
                         current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.x;
    const double & d_y = current_waypoints_ptr_->poses.at(*idx_next_wp_ + 1).position.y -
                         current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.y;
    const double & ds = std::hypot(d_x, d_y);
    const double & vel_mean = (current_waypoints_vel_ptr_->at(*idx_next_wp_ + 1) +
                               current_waypoints_vel_ptr_->at(*idx_prev_wp_)) /
                              2.0;
    const double & dv = current_waypoints_vel_ptr_->at(*idx_next_wp_ + 1) -
                        current_waypoints_vel_ptr_->at(*idx_prev_wp_);
    const double & dt = ds / std::max(vel_mean, p_.prevent_zero_division_value_);
    next_wp_acc = dv / std::max(dt, p_.prevent_zero_division_value_);
  }
  const double & d_acc_prev2next = next_wp_acc - prev_wp_acc;
  const double & interp_acceleration = prev_wp_acc + ratio_t * d_acc_prev2next;

  const Pose interpolated_pose = temp_interpolated_pose;

  /* desired steering calculation */

  const double interp_steering_angle = std::atan(p_.wheelbase_ * estimateCurvature());

  setInterpolatedVars(
    interpolated_pose, interp_velocity, interp_acceleration, interp_steering_angle);

  return std::make_pair(true, interpolated_pose);
}

// Sets interpolated waypoint_ptr_.
void ControlPerformanceAnalysisCore::setInterpolatedVars(
  const Pose & interpolated_pose, const double & interpolated_velocity,
  const double & interpolated_acceleration, const double & interpolated_steering_angle)
{
  interpolated_pose_ptr_ = std::make_shared<Pose>(interpolated_pose);
  interpolated_velocity_ptr_ = std::make_shared<double>(interpolated_velocity);
  interpolated_acceleration_ptr_ = std::make_shared<double>(interpolated_acceleration);
  interpolated_steering_angle_ptr_ = std::make_shared<double>(interpolated_steering_angle);
}

double ControlPerformanceAnalysisCore::estimateCurvature()
{
  // Get idx of front-axle center reference point on the trajectory.
  // get the waypoint corresponds to the front_axle center.
  if (!idx_curve_ref_wp_) {
    RCLCPP_WARN(logger_, "Cannot find index of curvature reference waypoint ");
    return 0;
  }

  const Pose front_axleWP_pose = current_waypoints_ptr_->poses.at(*idx_curve_ref_wp_);

  // for guarding -1 in finding previous waypoint for the front axle
  const int32_t idx_prev_waypoint =
    *idx_curve_ref_wp_ >= 1 ? *idx_curve_ref_wp_ - 1 : *idx_curve_ref_wp_;

  const Pose front_axleWP_pose_prev = current_waypoints_ptr_->poses.at(idx_prev_waypoint);

  // Compute arc-length ds between 2 points.
  const double ds_arc_length = std::hypot(
    front_axleWP_pose_prev.position.x - front_axleWP_pose.position.x,
    front_axleWP_pose_prev.position.y - front_axleWP_pose.position.y);

  // Define waypoints 10 meters behind the rear axle if exist.
  // If not exist, we will take the first point of the
  // curvature triangle as the start point of the trajectory.
  const auto & num_of_back_indices = std::round(p_.curvature_interval_length_ / ds_arc_length);
  const int32_t loc_of_back_idx =
    (*idx_curve_ref_wp_ - num_of_back_indices < 0) ? 0 : *idx_curve_ref_wp_ - num_of_back_indices;

  // Define location of forward point 10 meters ahead of the front axle on curve.
  const uint32_t max_idx =
    std::distance(current_waypoints_ptr_->poses.cbegin(), current_waypoints_ptr_->poses.cend());

  const auto num_of_forward_indices = num_of_back_indices;
  const int32_t loc_of_forward_idx = (*idx_curve_ref_wp_ + num_of_forward_indices > max_idx)
                                       ? max_idx - 1
                                       : *idx_curve_ref_wp_ + num_of_forward_indices - 1;

  // We have three indices of the three trajectory poses.
  // We compute a curvature estimate from these points.

  const std::array<double, 2> a_coord{
    current_waypoints_ptr_->poses.at(loc_of_back_idx).position.x,
    current_waypoints_ptr_->poses.at(loc_of_back_idx).position.y};

  const std::array<double, 2> b_coord{
    current_waypoints_ptr_->poses.at(*idx_curve_ref_wp_).position.x,
    current_waypoints_ptr_->poses.at(*idx_curve_ref_wp_).position.y};

  const std::array<double, 2> c_coord{
    current_waypoints_ptr_->poses.at(loc_of_forward_idx).position.x,
    current_waypoints_ptr_->poses.at(loc_of_forward_idx).position.y};

  const double estimated_curvature = utils::curvatureFromThreePoints(a_coord, b_coord, c_coord);

  return estimated_curvature;
}

double ControlPerformanceAnalysisCore::estimatePurePursuitCurvature()
{
  if (!interpolated_pose_ptr_) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000,
      "Cannot set pure pursuit_target_point_idx, no valid interpolated pose ...");

    return 0;
  }

  const uint32_t & odom_size = odom_history_ptr_->size();
  const double & Vx = odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x;
  const double look_ahead_distance_pp = std::max(p_.wheelbase_, 2 * Vx);

  auto fun_distance_cond = [this, &look_ahead_distance_pp](auto pose_t) {
    const double dist = std::hypot(
      pose_t.position.x - this->interpolated_pose_ptr_->position.x,
      pose_t.position.y - this->interpolated_pose_ptr_->position.y);

    return dist > look_ahead_distance_pp;
  };

  auto it = std::find_if(
    current_waypoints_ptr_->poses.cbegin() + *idx_prev_wp_, current_waypoints_ptr_->poses.cend(),
    fun_distance_cond);

  Pose target_pose_pp;

  // If there is no waypoint left on the trajectory, interpolate one.
  if (it == current_waypoints_ptr_->poses.cend()) {
    // Interpolate a waypoint.
    it = std::prev(it);
    int32_t && temp_idx_pp = std::distance(current_waypoints_ptr_->poses.cbegin(), it);
    Pose const & last_pose_on_traj = current_waypoints_ptr_->poses.at(temp_idx_pp);

    // get the yaw angle of the last traj point.
    const double & yaw_pp = tf2::getYaw(last_pose_on_traj.orientation);

    // get unit tangent in this direction.
    const std::vector<double> unit_tangent = utils::getTangentVector(yaw_pp);

    target_pose_pp.position.z = 0;
    target_pose_pp.position.x =
      last_pose_on_traj.position.x + look_ahead_distance_pp * unit_tangent[0];
    target_pose_pp.position.y =
      last_pose_on_traj.position.y + look_ahead_distance_pp * unit_tangent[1];

    target_pose_pp.orientation = last_pose_on_traj.orientation;
  } else {
    // idx of the last waypoint on the trajectory is
    const int32_t & temp_idx_pp = std::distance(current_waypoints_ptr_->poses.cbegin(), it);
    const Pose & last_pose_on_traj = current_waypoints_ptr_->poses.at(temp_idx_pp);

    target_pose_pp.position.z = last_pose_on_traj.position.z;
    target_pose_pp.position.x = last_pose_on_traj.position.x;
    target_pose_pp.position.y = last_pose_on_traj.position.y;
    target_pose_pp.orientation = last_pose_on_traj.orientation;
  }

  // We have target pose for the pure pursuit.
  // Find projection of target vector from vehicle.

  const std::vector<double> vec_to_target{
    target_pose_pp.position.x - current_vec_pose_ptr_->position.x,
    target_pose_pp.position.y - current_vec_pose_ptr_->position.y};

  const double & current_vec_yaw = tf2::getYaw(current_vec_pose_ptr_->orientation);
  const std::vector<double> normal_vec = utils::getNormalVector(current_vec_yaw);  // ClockWise

  // Project this vector on the vehicle normal vector.
  const double x_pure_pursuit = vec_to_target[0] * normal_vec[0] + vec_to_target[1] * normal_vec[1];

  // Pure pursuit curvature.
  const double curvature_pure_pursuit =
    2 * x_pure_pursuit / (look_ahead_distance_pp * look_ahead_distance_pp);

  return curvature_pure_pursuit;
}
}  // namespace control_performance_analysis
