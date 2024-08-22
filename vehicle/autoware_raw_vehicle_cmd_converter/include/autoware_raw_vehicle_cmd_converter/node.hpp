//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__NODE_HPP_
#define AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__NODE_HPP_

#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "autoware_raw_vehicle_cmd_converter/accel_map.hpp"
#include "autoware_raw_vehicle_cmd_converter/brake_map.hpp"
#include "autoware_raw_vehicle_cmd_converter/pid.hpp"
#include "autoware_raw_vehicle_cmd_converter/steer_map.hpp"
#include "autoware_raw_vehicle_cmd_converter/vgr.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::raw_vehicle_cmd_converter
{
using Control = autoware_control_msgs::msg::Control;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_vehicle_msgs::msg::ActuationCommandStamped;
using tier4_vehicle_msgs::msg::ActuationStatusStamped;
using TwistStamped = geometry_msgs::msg::TwistStamped;
using Odometry = nav_msgs::msg::Odometry;
using Steering = autoware_vehicle_msgs::msg::SteeringReport;

class DebugValues
{
public:
  enum class TYPE {
    CURR_TIME = 0,
    P = 1,
    I = 2,
    D = 3,
    FF = 4,
    FB = 5,
    STEER = 6,
    ERROR_P = 7,
    ERROR_I = 8,
    ERROR_D = 9,
    SIZE  // this is the number of enum elements
  };
  std::array<double, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }
  void setValues(TYPE type, double val) { values_.at(static_cast<int>(type)) = val; }
  void setValues(int type, double val) { values_.at(type) = val; }

private:
  std::array<double, static_cast<int>(TYPE::SIZE)> values_;
};

class RawVehicleCommandConverterNode : public rclcpp::Node
{
public:
  explicit RawVehicleCommandConverterNode(const rclcpp::NodeOptions & node_options);

  //!< @brief topic publisher for low level vehicle command
  rclcpp::Publisher<ActuationCommandStamped>::SharedPtr pub_actuation_cmd_;
  rclcpp::Publisher<Steering>::SharedPtr pub_steering_status_;
  //!< @brief subscriber for vehicle command
  rclcpp::Subscription<Control>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<ActuationStatusStamped>::SharedPtr sub_actuation_status_;
  rclcpp::Subscription<Steering>::SharedPtr sub_steering_;
  // polling subscribers
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};

  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<TwistStamped> current_twist_ptr_;  // [m/s]
  std::unique_ptr<double> current_steer_ptr_;
  ActuationStatusStamped::ConstSharedPtr actuation_status_ptr_;
  Control::ConstSharedPtr control_cmd_ptr_;
  AccelMap accel_map_;
  BrakeMap brake_map_;
  SteerMap steer_map_;
  VGR vgr_;
  // TODO(tanaka): consider accel/brake pid too
  PIDController steer_pid_;
  bool ff_map_initialized_;
  double max_accel_cmd_;
  double max_brake_cmd_;
  double max_steer_cmd_;
  double min_steer_cmd_;
  // ros parameter
  bool use_steer_ff_;
  bool use_steer_fb_;
  bool is_debugging_;
  bool convert_accel_cmd_;                                             //!< @brief use accel or not
  bool convert_brake_cmd_;                                             //!< @brief use brake or not
  std::optional<std::string> convert_steer_cmd_method_{std::nullopt};  //!< @brief method to convert
  bool need_to_subscribe_actuation_status_{false};
  rclcpp::Time prev_time_steer_calculation_{0, 0, RCL_ROS_TIME};

  // Whether to subscribe to actuation_status and calculate and publish steering_status
  // For example, receive the steering wheel angle and calculate the steering wheel angle based on
  // the gear ratio. If false, the vehicle interface must publish steering_status.
  bool convert_actuation_to_steering_status_{false};  // !< @brief use actuation_status or not

  double calculateAccelMap(
    const double current_velocity, const double desired_acc, bool & accel_cmd_is_zero);
  double calculateBrakeMap(const double current_velocity, const double desired_acc);
  double calculateSteerFromMap(const double vel, const double steering, const double steer_rate);
  void onControlCmd(const Control::ConstSharedPtr msg);
  void onSteering(const Steering::ConstSharedPtr msg);
  void onActuationStatus(const ActuationStatusStamped::ConstSharedPtr msg);
  void publishActuationCmd();
  // for debugging
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr debug_pub_steer_pid_;
  DebugValues debug_steer_;

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace autoware::raw_vehicle_cmd_converter

#endif  // AUTOWARE_RAW_VEHICLE_CMD_CONVERTER__NODE_HPP_
