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

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace control_performance_analysis
{
using geometry_msgs::msg::Quaternion;

ControlPerformanceAnalysisCore::ControlPerformanceAnalysisCore() : wheelbase_{2.74}
{
  prev_target_vars_ = std::make_unique<msg::ErrorStamped>();
  prev_driving_vars_ = std::make_unique<msg::DrivingMonitorStamped>();
  odom_history_ptr_ = std::make_shared<std::vector<Odometry>>();
  odom_interval_ = 0;
  curvature_interval_length_ = 10.0;
  acceptable_min_waypoint_distance_ = 2.0;
  prevent_zero_division_value_ = 0.001;
  lpf_gain_ = 0.8;
}

ControlPerformanceAnalysisCore::ControlPerformanceAnalysisCore(
  double wheelbase, double curvature_interval_length, uint odom_interval,
  double acceptable_min_waypoint_distance, double prevent_zero_division_value, double lpf_gain_val)
: wheelbase_{wheelbase},
  curvature_interval_length_{curvature_interval_length},
  odom_interval_{odom_interval},
  acceptable_min_waypoint_distance_{acceptable_min_waypoint_distance},
  prevent_zero_division_value_{prevent_zero_division_value},
  lpf_gain_{lpf_gain_val}
{
  // prepare control performance struct
  prev_target_vars_ = std::make_unique<msg::ErrorStamped>();
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
  if (odom_history_ptr_->size() >= (3 + odom_interval_ * 2)) {
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

  /*
   *   Create Vectors of Path Directions for each interval
   *   interval_vector_xy = {waypoint_1 - waypoint_0}_xy
   * */

  // Prepare vector of projection distance values; projection of vehicle vectors onto the intervals
  std::vector<double> projection_distances_ds;

  auto f_projection_dist = [this](auto pose_1, auto pose_0) {
    // Vector of intervals.
    std::vector<double> int_vec{
      pose_1.position.x - pose_0.position.x, pose_1.position.y - pose_0.position.y};

    // Compute the magnitude of path interval vector
    double ds_mag = std::hypot(int_vec[0], int_vec[1]);

    // Vector to vehicle from the origin waypoints.
    std::vector<double> vehicle_vec{
      this->current_vec_pose_ptr_->position.x - pose_0.position.x,
      this->current_vec_pose_ptr_->position.y - pose_0.position.y};

    double projection_distance_onto_interval =
      (int_vec[0] * vehicle_vec[0] + int_vec[1] * vehicle_vec[1]) / ds_mag;

    return projection_distance_onto_interval;
  };

  // Fill the projection_distances vector.
  std::transform(
    current_waypoints_ptr_->poses.cbegin() + 1, current_waypoints_ptr_->poses.cend(),
    current_waypoints_ptr_->poses.cbegin(), std::back_inserter(projection_distances_ds),
    f_projection_dist);

  // Lambda function to replace negative numbers with a large number.
  auto fnc_check_if_negative = [](auto x) {
    return x < 0 ? std::numeric_limits<double>::max() : x;
  };

  std::vector<double> projections_distances_all_positive;
  std::transform(
    projection_distances_ds.cbegin(), projection_distances_ds.cend(),
    std::back_inserter(projections_distances_all_positive), fnc_check_if_negative);

  // Minimum of all positive distances and the index of the next waypoint.
  auto it = std::min_element(
    projections_distances_all_positive.cbegin(), projections_distances_all_positive.cend());

  // Extract the location of iterator idx and store in the class.
  int32_t && temp_idx_prev_wp_ = std::distance(projections_distances_all_positive.cbegin(), it);
  idx_prev_wp_ = std::make_unique<int32_t>(temp_idx_prev_wp_);

  // Distance of next waypoint to the vehicle, for anomaly detection.
  double min_distance_ds = projections_distances_all_positive[*idx_prev_wp_];
  int32_t length_of_trajectory =
    std::distance(current_waypoints_ptr_->poses.cbegin(), current_waypoints_ptr_->poses.cend());

  return ((min_distance_ds <= acceptable_min_waypoint_distance_) && (*idx_prev_wp_ >= 0) &&
          (*idx_prev_wp_ <= length_of_trajectory))
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

  auto && pose_interp_wp_ = pair_pose_interp_wp_.second;

  // Create interpolated waypoint vector
  std::vector<double> interp_waypoint_xy{pose_interp_wp_.position.x, pose_interp_wp_.position.y};

  // Create vehicle position vector
  std::vector<double> vehicle_position_xy{
    current_vec_pose_ptr_->position.x, current_vec_pose_ptr_->position.y};

  // Get Yaw angles of the reference waypoint and the vehicle
  double target_yaw = tf2::getYaw(pose_interp_wp_.orientation);
  double vehicle_yaw_angle = tf2::getYaw(current_vec_pose_ptr_->orientation);

  // Compute Curvature at the point where the front axle might follow
  // get the waypoint corresponds to the front_axle center

  if (!idx_curve_ref_wp_) {
    RCLCPP_WARN(logger_, "Cannot find index of curvature reference waypoint ");
    return false;
  }

  double curvature_est = estimateCurvature();                // three point curvature
  double curvature_est_pp = estimatePurePursuitCurvature();  // pure pursuit curvature

  // Compute lateral, longitudinal, heading error w.r.t. frenet frame

  std::vector<double> lateral_longitudinal_error =
    utils::computeLateralLongitudinalError(interp_waypoint_xy, vehicle_position_xy, target_yaw);
  double & lateral_error = lateral_longitudinal_error[0];
  double & longitudinal_error = lateral_longitudinal_error[1];

  // Compute the yaw angle error.
  double && heading_yaw_error = utils::angleDistance(vehicle_yaw_angle, target_yaw);

  // Set the values of ErrorMsgVars.

  error_vars.error.lateral_error = lateral_error;
  error_vars.error.longitudinal_error = longitudinal_error;
  error_vars.error.heading_error = heading_yaw_error;

  // odom history contains k + 1, k, k - 1 ... steps. We are in kth step
  uint && odom_size = odom_history_ptr_->size();

  error_vars.header.stamp = odom_history_ptr_->at(odom_size - 2).header.stamp;  // we are in step k

  double & Vx = odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x;
  // Current acceleration calculation
  double && d_x = odom_history_ptr_->at(odom_size - 1).pose.pose.position.x -
                  odom_history_ptr_->at(odom_size - 2).pose.pose.position.x;
  double && d_y = odom_history_ptr_->at(odom_size - 1).pose.pose.position.y -
                  odom_history_ptr_->at(odom_size - 2).pose.pose.position.y;
  double && ds = std::hypot(d_x, d_y);

  double && vel_mean = (odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x +
                        odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x) /
                       2.0;
  double && dv = odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x -
                 odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x;
  double && dt = ds / std::max(vel_mean, prevent_zero_division_value_);
  double && Ax = dv / std::max(dt, prevent_zero_division_value_);  // current acceleration

  double && longitudinal_error_velocity =
    Vx * cos(heading_yaw_error) - *interpolated_velocity_ptr_ * (1 - curvature_est * lateral_error);
  double && lateral_error_velocity =
    Vx * sin(heading_yaw_error) - *interpolated_velocity_ptr_ * curvature_est * longitudinal_error;

  double && steering_cmd = current_control_ptr_->lateral.steering_tire_angle;
  double && current_steering_val = current_vec_steering_msg_ptr_->steering_tire_angle;
  error_vars.error.control_effort_energy = contR * steering_cmd * steering_cmd;  // u*R*u';

  double && heading_velocity_error = (Vx * tan(current_steering_val)) / wheelbase_ -
                                     *this->interpolated_velocity_ptr_ * curvature_est;

  double && lateral_acceleration_error =
    -curvature_est * *interpolated_acceleration_ptr_ * longitudinal_error -
    curvature_est * *interpolated_velocity_ptr_ * longitudinal_error_velocity +
    Vx * heading_velocity_error * cos(heading_yaw_error) + Ax * sin(heading_yaw_error);

  double && longitudinal_acceleration_error =
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

    error_vars.error.curvature_estimate = lpf_gain_ * prev_target_vars_->error.curvature_estimate +
                                          (1 - lpf_gain_) * error_vars.error.curvature_estimate;

    error_vars.error.curvature_estimate_pp =
      lpf_gain_ * prev_target_vars_->error.curvature_estimate_pp +
      (1 - lpf_gain_) * error_vars.error.curvature_estimate_pp;

    error_vars.error.lateral_error = lpf_gain_ * prev_target_vars_->error.lateral_error +
                                     (1 - lpf_gain_) * error_vars.error.lateral_error;

    error_vars.error.lateral_error_velocity =
      lpf_gain_ * prev_target_vars_->error.lateral_error_velocity +
      (1 - lpf_gain_) * error_vars.error.lateral_error_velocity;

    error_vars.error.lateral_error_acceleration =
      lpf_gain_ * prev_target_vars_->error.lateral_error_acceleration +
      (1 - lpf_gain_) * error_vars.error.lateral_error_acceleration;

    error_vars.error.longitudinal_error = lpf_gain_ * prev_target_vars_->error.longitudinal_error +
                                          (1 - lpf_gain_) * error_vars.error.longitudinal_error;

    error_vars.error.longitudinal_error_velocity =
      lpf_gain_ * prev_target_vars_->error.longitudinal_error_velocity +
      (1 - lpf_gain_) * error_vars.error.longitudinal_error_velocity;

    error_vars.error.longitudinal_error_acceleration =
      lpf_gain_ * prev_target_vars_->error.longitudinal_error_acceleration +
      (1 - lpf_gain_) * error_vars.error.longitudinal_error_acceleration;

    error_vars.error.heading_error = lpf_gain_ * prev_target_vars_->error.heading_error +
                                     (1 - lpf_gain_) * error_vars.error.heading_error;

    error_vars.error.heading_error_velocity =
      lpf_gain_ * prev_target_vars_->error.heading_error_velocity +
      (1 - lpf_gain_) * error_vars.error.heading_error_velocity;

    error_vars.error.control_effort_energy =
      lpf_gain_ * prev_target_vars_->error.control_effort_energy +
      (1 - lpf_gain_) * error_vars.error.control_effort_energy;

    error_vars.error.error_energy = lpf_gain_ * prev_target_vars_->error.error_energy +
                                    (1 - lpf_gain_) * error_vars.error.error_energy;
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
        tan(current_vec_steering_msg_ptr_->steering_tire_angle) / wheelbase_;

      if (odom_history_ptr_->size() >= odom_interval_ + 2) {
        // Calculate longitudinal acceleration

        double dv = odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x -
                    odom_history_ptr_->at(odom_size - odom_interval_ - 2).twist.twist.linear.x;

        auto duration =
          (rclcpp::Time(odom_history_ptr_->at(odom_size - 1).header.stamp) -
           rclcpp::Time(odom_history_ptr_->at(odom_size - odom_interval_ - 2).header.stamp));

        double dt = duration.seconds();

        driving_status_vars.longitudinal_acceleration.data = dv / dt;
        driving_status_vars.longitudinal_acceleration.header.set__stamp(
          rclcpp::Time(odom_history_ptr_->at(odom_size - odom_interval_ - 2).header.stamp) +
          duration * 0.5);  // Time stamp of acceleration data

        //  Calculate lateral jerk

        double d_lateral_jerk = driving_status_vars.lateral_acceleration.data -
                                prev_driving_vars_->lateral_acceleration.data;

        //  We already know the delta time from above. same as longitudinal acceleration

        driving_status_vars.lateral_jerk.data = d_lateral_jerk / dt;
        driving_status_vars.lateral_jerk.header =
          driving_status_vars.longitudinal_acceleration.header;
      }

      if (odom_history_ptr_->size() == 2 * odom_interval_ + 3) {
        // calculate longitudinal jerk
        double d_a = driving_status_vars.longitudinal_acceleration.data -
                     prev_driving_vars_->longitudinal_acceleration.data;
        auto duration =
          (rclcpp::Time(driving_status_vars.longitudinal_acceleration.header.stamp) -
           rclcpp::Time(prev_driving_vars_->longitudinal_acceleration.header.stamp));
        double dt = duration.seconds();
        driving_status_vars.longitudinal_jerk.data = d_a / dt;
        driving_status_vars.longitudinal_jerk.header.set__stamp(
          rclcpp::Time(prev_driving_vars_->longitudinal_acceleration.header.stamp) +
          duration * 0.5);  // Time stamp of jerk data
      }
      if (prev_driving_vars_) {
        // LPF for driving status vars

        driving_status_vars.longitudinal_acceleration.data =
          lpf_gain_ * prev_driving_vars_->longitudinal_acceleration.data +
          (1 - lpf_gain_) * driving_status_vars.longitudinal_acceleration.data;

        driving_status_vars.lateral_acceleration.data =
          lpf_gain_ * prev_driving_vars_->lateral_acceleration.data +
          (1 - lpf_gain_) * driving_status_vars.lateral_acceleration.data;

        driving_status_vars.lateral_jerk.data =
          lpf_gain_ * prev_driving_vars_->lateral_jerk.data +
          (1 - lpf_gain_) * driving_status_vars.lateral_jerk.data;

        driving_status_vars.longitudinal_jerk.data =
          lpf_gain_ * prev_driving_vars_->longitudinal_jerk.data +
          (1 - lpf_gain_) * driving_status_vars.longitudinal_jerk.data;

        driving_status_vars.controller_processing_time.data =
          lpf_gain_ * prev_driving_vars_->controller_processing_time.data +
          (1 - lpf_gain_) * driving_status_vars.controller_processing_time.data;

        driving_status_vars.desired_steering_angle.data =
          lpf_gain_ * prev_driving_vars_->desired_steering_angle.data +
          (1 - lpf_gain_) * driving_status_vars.desired_steering_angle.data;
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
        tan(current_vec_steering_msg_ptr_->steering_tire_angle) / wheelbase_;
      last_steering_report.stamp = current_vec_steering_msg_ptr_->stamp;
    }
    return true;

  } else {
    RCLCPP_ERROR(logger_, "Can not get odometry data! ");
    return false;
  }
}

Pose ControlPerformanceAnalysisCore::getPrevWPPose() const
{
  Pose pose_ref_waypoint_ = current_waypoints_ptr_->poses.at(*idx_prev_wp_);
  return pose_ref_waypoint_;
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
    double dist = std::hypot(
      pose_t.position.x - this->interpolated_pose_ptr_->position.x,
      pose_t.position.y - this->interpolated_pose_ptr_->position.y);

    return dist > wheelbase_;
  };

  auto it = std::find_if(
    current_waypoints_ptr_->poses.cbegin() + *idx_prev_wp_, current_waypoints_ptr_->poses.cend(),
    fun_distance_cond);

  if (it == current_waypoints_ptr_->poses.cend()) {
    it = std::prev(it);
  }

  int32_t && temp_idx_curve_ref_wp = std::distance(current_waypoints_ptr_->poses.cbegin(), it);
  idx_curve_ref_wp_ = std::make_unique<int32_t>(temp_idx_curve_ref_wp);
}

std::pair<bool, Pose> ControlPerformanceAnalysisCore::calculateClosestPose()
{
  Pose interpolated_pose;

  // Get index of prev waypoint and sanity check.
  if (!idx_prev_wp_) {
    RCLCPP_ERROR(logger_, "Cannot find the next waypoint.");
    return std::make_pair(false, interpolated_pose);
  }

  // Define the next waypoint - so that we can define an interval in which the car follow a line.
  int32_t idx_next_wp_temp;
  double next_wp_acc = 0.0;
  double prev_wp_acc = 0.0;

  int32_t total_num_of_waypoints_in_traj =
    std::distance(current_waypoints_ptr_->poses.cbegin(), current_waypoints_ptr_->poses.cend());

  if (*idx_prev_wp_ < total_num_of_waypoints_in_traj - 1) {
    idx_next_wp_temp = *idx_prev_wp_ + 1;

  } else {
    idx_next_wp_temp = total_num_of_waypoints_in_traj - 1;
  }

  idx_next_wp_ = std::make_unique<int32_t>(idx_next_wp_temp);

  /*
   *  Create two vectors originating from the previous waypoints to the next waypoint and the
   *  vehicle position and find projection of vehicle vector on the the trajectory section,
   *
   * */

  // First get te yaw angles of all three poses.
  double && prev_yaw = tf2::getYaw(current_waypoints_ptr_->poses.at(*idx_prev_wp_).orientation);
  double && next_yaw = tf2::getYaw(current_waypoints_ptr_->poses.at(*idx_next_wp_).orientation);

  double & prev_velocity = current_waypoints_vel_ptr_->at(*idx_prev_wp_);
  double & next_velocity = current_waypoints_vel_ptr_->at(*idx_next_wp_);

  // Previous waypoint to next waypoint.
  double && dx_prev2next = current_waypoints_ptr_->poses.at(*idx_next_wp_).position.x -
                           current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.x;

  double && dy_prev2next = current_waypoints_ptr_->poses.at(*idx_next_wp_).position.y -
                           current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.y;

  double && delta_psi_prev2next = utils::angleDistance(next_yaw, prev_yaw);
  double && d_vel_prev2next = next_velocity - prev_velocity;

  // Create a vector from p0 (prev) --> p1 (to next wp)
  std::vector<double> v_prev2next_wp{dx_prev2next, dy_prev2next};

  // Previous waypoint to the vehicle pose
  /*
   *   p0:previous waypoint ----> p1 next waypoint
   *   vector = p1 - p0. We project vehicle vector on this interval to
   *
   * */

  double && dx_prev2vehicle =
    current_vec_pose_ptr_->position.x - current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.x;

  double && dy_prev2vehicle =
    current_vec_pose_ptr_->position.y - current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.y;

  // Vector from p0 to p_vehicle
  std::vector<double> v_prev2vehicle{dx_prev2vehicle, dy_prev2vehicle};

  // Compute the length of v_prev2next_wp : vector from p0 --> p1
  double && distance_p02p1 = std::hypot(dx_prev2next, dy_prev2next);

  // Compute how far the car is away from p0 in p1 direction. p_interp is the location of the
  // interpolated waypoint. This is the dot product normalized by the length of the interval.
  // a.b = |a|.|b|.cos(alpha) -- > |a|.cos(alpha) = a.b / |b| where b is the path interval,

  double && distance_p02p_interp =
    (dx_prev2next * dx_prev2vehicle + dy_prev2next * dy_prev2vehicle) / distance_p02p1;

  double && distance_p_interp2p1 = distance_p02p1 - distance_p02p_interp;
  /*
   * We use the following linear interpolation
   *  pi = p0 + ratio_t * (p1 - p0)
   * */

  double && ratio_t = distance_p02p_interp / distance_p02p1;

  // Interpolate pose.position and pose.orientation
  interpolated_pose.position.x =
    current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.x + ratio_t * dx_prev2next;

  interpolated_pose.position.y =
    current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.y + ratio_t * dy_prev2next;

  interpolated_pose.position.z = 0.0;

  // Interpolate the yaw angle of pi : interpolated waypoint
  double && interp_yaw_angle = prev_yaw + ratio_t * delta_psi_prev2next;
  double && interp_velocity = prev_velocity + ratio_t * d_vel_prev2next;

  Quaternion && orient_msg = utils::createOrientationMsgFromYaw(interp_yaw_angle);
  interpolated_pose.orientation = orient_msg;

  /* interpolated steering calculation */

  double interp_steering_angle = 0.0;

  if (static_cast<size_t>(*idx_next_wp_ + 1) < current_waypoints_ptr_->poses.size()) {
    double && dx_p1_p2 = current_waypoints_ptr_->poses.at(*idx_next_wp_ + 1).position.x -
                         current_waypoints_ptr_->poses.at(*idx_next_wp_).position.x;
    double && dy_p1_p2 = current_waypoints_ptr_->poses.at(*idx_next_wp_ + 1).position.y -
                         current_waypoints_ptr_->poses.at(*idx_next_wp_).position.y;
    double && distance_p1_p2 = std::hypot(dx_p1_p2, dy_p1_p2);
    double && prev_steering =
      static_cast<float>(std::atan((next_yaw - prev_yaw) * wheelbase_ / (distance_p02p1)));
    double && next_steering = static_cast<float>(std::atan(
      (tf2::getYaw(current_waypoints_ptr_->poses.at(*idx_next_wp_ + 1).orientation) - next_yaw) *
      wheelbase_ / (distance_p1_p2)));
    interp_steering_angle = prev_steering + ratio_t * (next_steering - prev_steering);
  } else {
    interp_steering_angle = static_cast<float>(std::atan(
      (next_yaw - tf2::getYaw(interpolated_pose.orientation)) * wheelbase_ /
      (distance_p_interp2p1)));
  }

  /* interpolated acceleration calculation */

  if (*idx_prev_wp_ == 0 || *idx_prev_wp_ == total_num_of_waypoints_in_traj - 1) {
    prev_wp_acc = 0.0;
  } else {
    double && d_x = current_waypoints_ptr_->poses.at(*idx_next_wp_).position.x -
                    current_waypoints_ptr_->poses.at(*idx_prev_wp_ - 1).position.x;
    double && d_y = current_waypoints_ptr_->poses.at(*idx_next_wp_).position.y -
                    current_waypoints_ptr_->poses.at(*idx_prev_wp_ - 1).position.y;
    double && ds = std::hypot(d_x, d_y);
    double && vel_mean = (current_waypoints_vel_ptr_->at(*idx_next_wp_) +
                          current_waypoints_vel_ptr_->at(*idx_prev_wp_ - 1)) /
                         2.0;
    double && dv = current_waypoints_vel_ptr_->at(*idx_next_wp_) -
                   current_waypoints_vel_ptr_->at(*idx_prev_wp_ - 1);
    double && dt = ds / std::max(vel_mean, prevent_zero_division_value_);
    prev_wp_acc = dv / std::max(dt, prevent_zero_division_value_);
  }

  if (*idx_next_wp_ == total_num_of_waypoints_in_traj - 1) {
    next_wp_acc = 0.0;
  } else {
    double && d_x = current_waypoints_ptr_->poses.at(*idx_next_wp_ + 1).position.x -
                    current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.x;
    double && d_y = current_waypoints_ptr_->poses.at(*idx_next_wp_ + 1).position.y -
                    current_waypoints_ptr_->poses.at(*idx_prev_wp_).position.y;
    double && ds = std::hypot(d_x, d_y);
    double && vel_mean = (current_waypoints_vel_ptr_->at(*idx_next_wp_ + 1) +
                          current_waypoints_vel_ptr_->at(*idx_prev_wp_)) /
                         2.0;
    double && dv = current_waypoints_vel_ptr_->at(*idx_next_wp_ + 1) -
                   current_waypoints_vel_ptr_->at(*idx_prev_wp_);
    double && dt = ds / std::max(vel_mean, prevent_zero_division_value_);
    next_wp_acc = dv / std::max(dt, prevent_zero_division_value_);
  }
  double && d_acc_prev2next = next_wp_acc - prev_wp_acc;
  double && interp_acceleration = prev_wp_acc + ratio_t * d_acc_prev2next;

  setInterpolatedVars(
    interpolated_pose, interp_velocity, interp_acceleration, interp_steering_angle);

  return std::make_pair(true, interpolated_pose);
}

// Sets interpolated waypoint_ptr_.
void ControlPerformanceAnalysisCore::setInterpolatedVars(
  Pose & interpolated_pose, double & interpolated_velocity, double & interpolated_acceleration,
  double & interpolated_steering_angle)
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
  Pose front_axleWP_pose = current_waypoints_ptr_->poses.at(*idx_curve_ref_wp_);

  // for guarding -1 in finding previous waypoint for the front axle
  int32_t idx_prev_waypoint = *idx_curve_ref_wp_ >= 1 ? *idx_curve_ref_wp_ - 1 : *idx_curve_ref_wp_;

  Pose front_axleWP_pose_prev = current_waypoints_ptr_->poses.at(idx_prev_waypoint);

  // Compute arc-length ds between 2 points.
  double ds_arc_length = std::hypot(
    front_axleWP_pose_prev.position.x - front_axleWP_pose.position.x,
    front_axleWP_pose_prev.position.y - front_axleWP_pose.position.y);

  // Define waypoints 10 meters behind the rear axle if exist.
  // If not exist, we will take the first point of the
  // curvature triangle as the start point of the trajectory.
  auto && num_of_back_indices = std::round(curvature_interval_length_ / ds_arc_length);
  int32_t loc_of_back_idx =
    (*idx_curve_ref_wp_ - num_of_back_indices < 0) ? 0 : *idx_curve_ref_wp_ - num_of_back_indices;

  // Define location of forward point 10 meters ahead of the front axle on curve.
  uint32_t max_idx =
    std::distance(current_waypoints_ptr_->poses.cbegin(), current_waypoints_ptr_->poses.cend());

  auto num_of_forward_indices = num_of_back_indices;
  int32_t loc_of_forward_idx = (*idx_curve_ref_wp_ + num_of_forward_indices > max_idx)
                                 ? max_idx - 1
                                 : *idx_curve_ref_wp_ + num_of_forward_indices - 1;

  // We have three indices of the three trajectory poses.
  // We compute a curvature estimate from these points.

  std::array<double, 2> a_coord{
    current_waypoints_ptr_->poses.at(loc_of_back_idx).position.x,
    current_waypoints_ptr_->poses.at(loc_of_back_idx).position.y};

  std::array<double, 2> b_coord{
    current_waypoints_ptr_->poses.at(*idx_curve_ref_wp_).position.x,
    current_waypoints_ptr_->poses.at(*idx_curve_ref_wp_).position.y};

  std::array<double, 2> c_coord{
    current_waypoints_ptr_->poses.at(loc_of_forward_idx).position.x,
    current_waypoints_ptr_->poses.at(loc_of_forward_idx).position.y};

  double estimated_curvature = utils::curvatureFromThreePoints(a_coord, b_coord, c_coord);

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
  double & Vx = odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x;
  double look_ahead_distance_pp = std::max(wheelbase_, 2 * Vx);

  auto fun_distance_cond = [this, &look_ahead_distance_pp](auto pose_t) {
    double dist = std::hypot(
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
    double const && yaw_pp = tf2::getYaw(last_pose_on_traj.orientation);

    // get unit tangent in this direction.
    std::vector<double> unit_tangent = utils::getTangentVector(yaw_pp);

    target_pose_pp.position.z = 0;
    target_pose_pp.position.x =
      last_pose_on_traj.position.x + look_ahead_distance_pp * unit_tangent[0];
    target_pose_pp.position.y =
      last_pose_on_traj.position.y + look_ahead_distance_pp * unit_tangent[1];

    target_pose_pp.orientation = last_pose_on_traj.orientation;
  } else {
    // idx of the last waypoint on the trajectory is
    int32_t && temp_idx_pp = std::distance(current_waypoints_ptr_->poses.cbegin(), it);
    Pose const & last_pose_on_traj = current_waypoints_ptr_->poses.at(temp_idx_pp);

    target_pose_pp.position.z = last_pose_on_traj.position.z;
    target_pose_pp.position.x = last_pose_on_traj.position.x;
    target_pose_pp.position.y = last_pose_on_traj.position.y;
    target_pose_pp.orientation = last_pose_on_traj.orientation;
  }

  // We have target pose for the pure pursuit.
  // Find projection of target vector from vehicle.

  std::vector<double> vec_to_target{
    target_pose_pp.position.x - current_vec_pose_ptr_->position.x,
    target_pose_pp.position.y - current_vec_pose_ptr_->position.y};

  double const && current_vec_yaw = tf2::getYaw(current_vec_pose_ptr_->orientation);
  std::vector<double> normal_vec = utils::getNormalVector(current_vec_yaw);  // ClockWise

  // Project this vector on the vehicle normal vector.
  double x_pure_pursuit = vec_to_target[0] * normal_vec[0] + vec_to_target[1] * normal_vec[1];

  // Pure pursuit curvature.
  double curvature_pure_pursuit =
    2 * x_pure_pursuit / (look_ahead_distance_pp * look_ahead_distance_pp);

  return curvature_pure_pursuit;
}
}  // namespace control_performance_analysis
