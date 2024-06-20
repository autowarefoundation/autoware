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

#include "autoware/motion_utils/trajectory/interpolation.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
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
  current_trajectory_ptr_ = std::make_shared<Trajectory>(trajectory);
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

void ControlPerformanceAnalysisCore::setCurrentControlValue(const Control & msg)
{
  current_control_ptr_ = std::make_shared<Control>(msg);
}

std::pair<bool, int32_t> ControlPerformanceAnalysisCore::findClosestPrevWayPointIdx_path_direction()
{
  if (!isDataReady()) {
    return std::make_pair(false, std::numeric_limits<int32_t>::quiet_NaN());
  }

  const auto closest_segment = autoware::motion_utils::findNearestSegmentIndex(
    current_trajectory_ptr_->points, *current_vec_pose_ptr_,
    p_.acceptable_max_distance_to_waypoint_, p_.acceptable_max_yaw_difference_rad_);

  if (!closest_segment) {  // fail to find closest idx
    return std::make_pair(false, std::numeric_limits<int32_t>::quiet_NaN());
  }

  idx_prev_wp_ = std::make_unique<int32_t>(closest_segment.value());
  idx_next_wp_ = std::make_unique<int32_t>(closest_segment.value() + 1);
  return std::make_pair(true, *idx_prev_wp_);
}

bool ControlPerformanceAnalysisCore::isDataReady() const
{
  const auto waiting = [this](const std::string & name) {
    rclcpp::Clock clock{RCL_ROS_TIME};
    RCLCPP_INFO_SKIPFIRST_THROTTLE(logger_, clock, 1000, "waiting for %s data ...", name.c_str());
    return false;
  };

  if (!current_vec_pose_ptr_) {
    return waiting("current pose");
  }

  if (current_trajectory_ptr_->points.empty()) {
    return waiting("current trajectory");
  }

  if (odom_history_ptr_->size() < 2) {
    return waiting("odometry");
  }

  if (!current_control_ptr_) {
    return waiting("current_control_cmd");
  }

  if (!current_vec_steering_msg_ptr_) {
    return waiting("current_steering");
  }

  return true;
}

bool ControlPerformanceAnalysisCore::calculateErrorVars()
{
  // Check if data is ready.
  if (!isDataReady()) {
    return false;
  }

  // update closest index
  findClosestPrevWayPointIdx_path_direction();

  // Get the interpolated pose
  const auto [success, pose_interp_wp] = calculateClosestPose();

  if (!success) {
    RCLCPP_WARN_THROTTLE(logger_, clock_, 1000, "Cannot get interpolated variables.");
    return false;
  }

  const double curvature_est = estimateCurvature();                // three point curvature
  const double curvature_est_pp = estimatePurePursuitCurvature();  // pure pursuit curvature

  // Compute lateral, longitudinal, heading error w.r.t. frenet frame
  const double target_yaw = tf2::getYaw(pose_interp_wp.orientation);
  const auto [lateral_error, longitudinal_error] = utils::computeLateralLongitudinalError(
    pose_interp_wp.position, current_vec_pose_ptr_->position, target_yaw);

  // Compute the yaw angle error.
  const double heading_yaw_error =
    autoware::universe_utils::calcYawDeviation(pose_interp_wp, *current_vec_pose_ptr_);

  // Set the values of ErrorMsgVars.

  error_vars.error.lateral_error = lateral_error;
  error_vars.error.longitudinal_error = longitudinal_error;
  error_vars.error.heading_error = heading_yaw_error;

  // odom history contains k + 1, k, k - 1 ... steps. We are in kth step
  const uint odom_size = odom_history_ptr_->size();

  error_vars.header.stamp = odom_history_ptr_->at(odom_size - 2).header.stamp;  // we are in step k

  const double Vx = odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x;

  // Current acceleration calculation
  const auto ds = autoware::universe_utils::calcDistance2d(
    odom_history_ptr_->at(odom_size - 1).pose.pose, odom_history_ptr_->at(odom_size - 2).pose.pose);

  const double vel_mean = (odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x +
                           odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x) /
                          2.0;
  const double dv = odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x -
                    odom_history_ptr_->at(odom_size - 2).twist.twist.linear.x;
  const double dt = ds / std::max(vel_mean, p_.prevent_zero_division_value_);
  const double Ax = dv / std::max(dt, p_.prevent_zero_division_value_);  // current acceleration

  const double longitudinal_error_velocity =
    Vx * cos(heading_yaw_error) - *interpolated_velocity_ptr_ * (1 - curvature_est * lateral_error);
  const double lateral_error_velocity =
    Vx * sin(heading_yaw_error) - *interpolated_velocity_ptr_ * curvature_est * longitudinal_error;

  const double steering_cmd = current_control_ptr_->lateral.steering_tire_angle;
  const double current_steering_val = current_vec_steering_msg_ptr_->steering_tire_angle;
  error_vars.error.control_effort_energy = contR * steering_cmd * steering_cmd;  // u*R*u';

  const double heading_velocity_error = (Vx * tan(current_steering_val)) / p_.wheelbase_ -
                                        *this->interpolated_velocity_ptr_ * curvature_est;

  const double lateral_acceleration_error =
    -curvature_est * *interpolated_acceleration_ptr_ * longitudinal_error -
    curvature_est * *interpolated_velocity_ptr_ * longitudinal_error_velocity +
    Vx * heading_velocity_error * cos(heading_yaw_error) + Ax * sin(heading_yaw_error);

  const double longitudinal_acceleration_error =
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

    const auto lpf = [&](auto & var, const auto & prev_var) {
      var = p_.lpf_gain_ * prev_var + (1 - p_.lpf_gain_) * var;
    };
    auto & error = error_vars.error;
    const auto & prev_error = prev_target_vars_->error;
    lpf(error.curvature_estimate, prev_error.curvature_estimate);
    lpf(error.curvature_estimate_pp, prev_error.curvature_estimate_pp);
    lpf(error.lateral_error, prev_error.lateral_error);
    lpf(error.lateral_error_velocity, prev_error.lateral_error_velocity);
    lpf(error.lateral_error_acceleration, prev_error.lateral_error_acceleration);
    lpf(error.longitudinal_error, prev_error.longitudinal_error);
    lpf(error.longitudinal_error_velocity, prev_error.longitudinal_error_velocity);
    lpf(error.longitudinal_error_acceleration, prev_error.longitudinal_error_acceleration);
    lpf(error.heading_error, prev_error.heading_error);
    lpf(error.heading_error_velocity, prev_error.heading_error_velocity);
    lpf(error.control_effort_energy, prev_error.control_effort_energy);
    lpf(error.error_energy, prev_error.error_energy);
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
        const auto lpf = [&](auto & var, const auto & prev_var) {
          var = p_.lpf_gain_ * prev_var + (1 - p_.lpf_gain_) * var;
        };
        auto & curr = driving_status_vars;
        const auto & prev = prev_driving_vars_;
        lpf(curr.longitudinal_acceleration.data, prev->longitudinal_acceleration.data);
        lpf(curr.lateral_acceleration.data, prev->lateral_acceleration.data);
        lpf(curr.lateral_jerk.data, prev->lateral_jerk.data);
        lpf(curr.longitudinal_jerk.data, prev->longitudinal_jerk.data);
        lpf(curr.controller_processing_time.data, prev->controller_processing_time.data);
        lpf(curr.desired_steering_angle.data, prev->desired_steering_angle.data);
      }

      prev_driving_vars_ =
        std::move(std::make_unique<msg::DrivingMonitorStamped>(driving_status_vars));

      last_odom_header.stamp = odom_history_ptr_->at(odom_size - 1).header.stamp;
      last_steering_report.stamp = current_vec_steering_msg_ptr_->stamp;

    } else if (last_steering_report.stamp != current_vec_steering_msg_ptr_->stamp) {
      driving_status_vars.lateral_acceleration.header.stamp = current_vec_steering_msg_ptr_->stamp;
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

std::optional<int32_t> ControlPerformanceAnalysisCore::findCurveRefIdx()
{
  // Get the previous waypoint as the reference
  if (!interpolated_pose_ptr_) {
    RCLCPP_WARN_THROTTLE(
      logger_, clock_, 1000, "Cannot set the curvature_idx, no valid interpolated pose ...");
    return std::nullopt;
  }

  auto fun_distance_cond = [this](auto point_t) {
    const double dist =
      autoware::universe_utils::calcDistance2d(point_t.pose, *interpolated_pose_ptr_);
    return dist > p_.wheelbase_;
  };

  auto it = std::find_if(
    current_trajectory_ptr_->points.cbegin() + *idx_prev_wp_,
    current_trajectory_ptr_->points.cend(), fun_distance_cond);

  if (it == current_trajectory_ptr_->points.cend()) {
    it = std::prev(it);
  }

  const auto idx_curve_ref_wp = std::distance(current_trajectory_ptr_->points.cbegin(), it);
  return idx_curve_ref_wp;
}

std::pair<bool, Pose> ControlPerformanceAnalysisCore::calculateClosestPose()
{
  const auto interp_point =
    autoware::motion_utils::calcInterpolatedPoint(*current_trajectory_ptr_, *current_vec_pose_ptr_);

  const double interp_steering_angle = std::atan(p_.wheelbase_ * estimateCurvature());

  setInterpolatedVars(
    interp_point.pose, interp_point.longitudinal_velocity_mps, interp_point.acceleration_mps2,
    interp_steering_angle);

  if (
    !interpolated_pose_ptr_ || !interpolated_velocity_ptr_ || !interpolated_acceleration_ptr_ ||
    !interpolated_steering_angle_ptr_) {
    return std::make_pair(false, interp_point.pose);
  }

  return std::make_pair(true, interp_point.pose);
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
  // Find and set the waypoint L-wheelbase meters ahead of the current waypoint.
  const auto idx_curve_ref_wp_optional = findCurveRefIdx();

  // Get idx of front-axle center reference point on the trajectory.
  // get the waypoint corresponds to the front_axle center.
  if (!idx_curve_ref_wp_optional) {
    RCLCPP_WARN(logger_, "Cannot find index of curvature reference waypoint ");
    return 0;
  }
  const auto idx_curve_ref_wp = idx_curve_ref_wp_optional.value();

  const auto & points = current_trajectory_ptr_->points;

  const Pose front_axleWP_pose = points.at(idx_curve_ref_wp).pose;

  // for guarding -1 in finding previous waypoint for the front axle
  const int32_t idx_prev_waypoint = std::max(idx_curve_ref_wp - 1, 0);

  const Pose front_axleWP_pose_prev = points.at(idx_prev_waypoint).pose;

  // Compute arc-length ds between 2 points.
  const double ds_arc_length =
    autoware::universe_utils::calcDistance2d(front_axleWP_pose_prev, front_axleWP_pose);

  // Define waypoints 10 meters behind the rear axle if exist.
  // If not exist, we will take the first point of the
  // curvature triangle as the start point of the trajectory.
  const auto num_of_indices =
    std::max(static_cast<int32_t>(std::round(p_.curvature_interval_length_ / ds_arc_length)), 1);

  const int32_t loc_of_back_idx = std::max(idx_curve_ref_wp - num_of_indices, 0);

  // Define location of forward point 10 meters ahead of the front axle on curve.
  const int32_t max_idx = points.size() - 1;

  const int32_t loc_of_forward_idx = std::min(idx_curve_ref_wp + num_of_indices, max_idx);

  // We have three indices of the three trajectory poses.
  // We compute a curvature estimate from these points.
  double estimated_curvature = 0.0;
  try {
    estimated_curvature = autoware::universe_utils::calcCurvature(
      points.at(loc_of_back_idx).pose.position, points.at(idx_curve_ref_wp).pose.position,
      points.at(loc_of_forward_idx).pose.position);
  } catch (...) {
    estimated_curvature = 0.0;
  }

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

  const uint32_t odom_size = odom_history_ptr_->size();
  const double Vx = odom_history_ptr_->at(odom_size - 1).twist.twist.linear.x;
  const double look_ahead_distance_pp = std::max(p_.wheelbase_, 2 * Vx);

  auto fun_distance_cond = [this, &look_ahead_distance_pp](auto point_t) {
    const double dist = autoware::universe_utils::calcDistance2d(point_t, *interpolated_pose_ptr_);
    return dist > look_ahead_distance_pp;
  };

  auto it = std::find_if(
    current_trajectory_ptr_->points.cbegin() + *idx_prev_wp_,
    current_trajectory_ptr_->points.cend(), fun_distance_cond);

  Pose target_pose_pp;

  // If there is no waypoint left on the trajectory, interpolate one.
  if (it == current_trajectory_ptr_->points.cend()) {
    // Interpolate a waypoint.
    it = std::prev(it);
    int32_t && temp_idx_pp = std::distance(current_trajectory_ptr_->points.cbegin(), it);
    const Pose & last_pose_on_traj = current_trajectory_ptr_->points.at(temp_idx_pp).pose;

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
    const int32_t temp_idx_pp = std::distance(current_trajectory_ptr_->points.cbegin(), it);
    const Pose last_pose_on_traj = current_trajectory_ptr_->points.at(temp_idx_pp).pose;

    target_pose_pp = last_pose_on_traj;
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
