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

#include "awapi_awiv_adapter/awapi_vehicle_state_publisher.hpp"

#include "tier4_auto_msgs_converter/tier4_auto_msgs_converter.hpp"

#include <limits>
#include <memory>

namespace autoware_api
{
AutowareIvVehicleStatePublisher::AutowareIvVehicleStatePublisher(rclcpp::Node & node)
: logger_(node.get_logger().get_child("awapi_awiv_vehicle_state_publisher")),
  clock_(node.get_clock()),
  prev_accel_(0.0)
{
  // publisher
  pub_state_ =
    node.create_publisher<tier4_api_msgs::msg::AwapiVehicleStatus>("output/vehicle_status", 1);
}

void AutowareIvVehicleStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  tier4_api_msgs::msg::AwapiVehicleStatus status = initVehicleStatus();

  // input header
  status.header.frame_id = "base_link";
  status.header.stamp = clock_->now();

  // get all info
  getPoseInfo(aw_info.current_pose_ptr, &status);
  getSteerInfo(aw_info.steer_ptr, &status);
  getVehicleCmdInfo(aw_info.vehicle_cmd_ptr, &status);
  getTurnSignalInfo(aw_info.turn_indicators_ptr, aw_info.hazard_lights_ptr, &status);
  getTwistInfo(aw_info.odometry_ptr, &status);
  getGearInfo(aw_info.gear_ptr, &status);
  getBatteryInfo(aw_info.battery_ptr, &status);
  getGpsInfo(aw_info.nav_sat_ptr, &status);

  // publish info
  pub_state_->publish(status);
}

tier4_api_msgs::msg::AwapiVehicleStatus AutowareIvVehicleStatePublisher::initVehicleStatus()
{
  tier4_api_msgs::msg::AwapiVehicleStatus status;
  // set default value
  if (std::numeric_limits<float>::has_quiet_NaN) {
    status.energy_level = std::numeric_limits<float>::quiet_NaN();
  }
  return status;
}

void AutowareIvVehicleStatePublisher::getPoseInfo(
  const std::shared_ptr<geometry_msgs::msg::PoseStamped> & pose_ptr,
  tier4_api_msgs::msg::AwapiVehicleStatus * status)
{
  if (!pose_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "current pose is nullptr");
    return;
  }

  // get pose
  status->pose = pose_ptr->pose;

  // convert quaternion to euler
  double yaw, pitch, roll;
  tf2::getEulerYPR(pose_ptr->pose.orientation, yaw, pitch, roll);
  status->eulerangle.yaw = yaw;
  status->eulerangle.pitch = pitch;
  status->eulerangle.roll = roll;
}

void AutowareIvVehicleStatePublisher::getSteerInfo(
  const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr & steer_ptr,
  tier4_api_msgs::msg::AwapiVehicleStatus * status)
{
  if (!steer_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "steer is nullptr");
    return;
  }

  // get steer
  using tier4_auto_msgs_converter::convert;
  status->steering = convert(*steer_ptr).data;

  // get steer vel
  if (previous_steer_ptr_) {
    // calculate steer vel from steer
    const double ds = steer_ptr->steering_tire_angle - previous_steer_ptr_->steering_tire_angle;
    const double dt = std::max(
      (rclcpp::Time(steer_ptr->stamp) - rclcpp::Time(previous_steer_ptr_->stamp)).seconds(), 1e-03);
    const double steer_vel = ds / dt;

    // apply lowpass filter
    const double lowpass_steer =
      lowpass_filter(steer_vel, prev_steer_vel_, steer_vel_lowpass_gain_);
    prev_steer_vel_ = lowpass_steer;
    status->steering_velocity = lowpass_steer;
  }
  previous_steer_ptr_ = steer_ptr;
}
void AutowareIvVehicleStatePublisher::getVehicleCmdInfo(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr & vehicle_cmd_ptr,
  tier4_api_msgs::msg::AwapiVehicleStatus * status)
{
  if (!vehicle_cmd_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "vehicle cmd is nullptr");
    return;
  }

  // get command
  status->target_acceleration = vehicle_cmd_ptr->longitudinal.acceleration;
  status->target_velocity = vehicle_cmd_ptr->longitudinal.speed;
  status->target_steering = vehicle_cmd_ptr->lateral.steering_tire_angle;
  status->target_steering_velocity = vehicle_cmd_ptr->lateral.steering_tire_rotation_rate;
}

void AutowareIvVehicleStatePublisher::getTurnSignalInfo(
  const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr & turn_indicators_ptr,
  const autoware_auto_vehicle_msgs::msg::HazardLightsReport::ConstSharedPtr & hazard_lights_ptr,
  tier4_api_msgs::msg::AwapiVehicleStatus * status)
{
  if (!turn_indicators_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "turn indicators is nullptr");
    return;
  }

  if (!hazard_lights_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "hazard lights is nullptr");
    return;
  }

  // get turn signal
  using tier4_auto_msgs_converter::convert;
  status->turn_signal = convert(*turn_indicators_ptr, *hazard_lights_ptr).data;
}

void AutowareIvVehicleStatePublisher::getTwistInfo(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_ptr,
  tier4_api_msgs::msg::AwapiVehicleStatus * status)
{
  if (!odometry_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "odometry is nullptr");
    return;
  }

  // get twist
  status->velocity = odometry_ptr->twist.twist.linear.x;
  status->angular_velocity = odometry_ptr->twist.twist.angular.z;

  // get accel
  if (previous_odometry_ptr_) {
    // calculate acceleration from velocity
    const double dv =
      odometry_ptr->twist.twist.linear.x - previous_odometry_ptr_->twist.twist.linear.x;
    const double dt = std::max(
      (rclcpp::Time(odometry_ptr->header.stamp) -
       rclcpp::Time(previous_odometry_ptr_->header.stamp))
        .seconds(),
      1e-03);
    const double accel = dv / dt;

    // apply lowpass filter
    const double lowpass_accel = lowpass_filter(accel, prev_accel_, accel_lowpass_gain_);
    prev_accel_ = lowpass_accel;
    status->acceleration = lowpass_accel;
  }
  previous_odometry_ptr_ = odometry_ptr;
}

void AutowareIvVehicleStatePublisher::getGearInfo(
  const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr & gear_ptr,
  tier4_api_msgs::msg::AwapiVehicleStatus * status)
{
  if (!gear_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "gear is nullptr");
    return;
  }

  // get gear (shift)
  using tier4_auto_msgs_converter::convert;
  status->gear = convert(*gear_ptr).shift.data;
}

void AutowareIvVehicleStatePublisher::getBatteryInfo(
  const tier4_vehicle_msgs::msg::BatteryStatus::ConstSharedPtr & battery_ptr,
  tier4_api_msgs::msg::AwapiVehicleStatus * status)
{
  if (!battery_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "battery is nullptr");
    return;
  }

  // get battery
  status->energy_level = battery_ptr->energy_level;
}

void AutowareIvVehicleStatePublisher::getGpsInfo(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr & nav_sat_ptr,
  tier4_api_msgs::msg::AwapiVehicleStatus * status)
{
  if (!nav_sat_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "nav_sat(gps) is nullptr");
    return;
  }

  // get geo_point
  status->geo_point.latitude = nav_sat_ptr->latitude;
  status->geo_point.longitude = nav_sat_ptr->longitude;
  status->geo_point.altitude = nav_sat_ptr->altitude;
}

}  // namespace autoware_api
