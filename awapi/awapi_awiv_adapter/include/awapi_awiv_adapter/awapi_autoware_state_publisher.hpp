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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_STATE_PUBLISHER_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_STATE_PUBLISHER_HPP_

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_api_msgs/msg/awapi_autoware_status.hpp>

#include <set>
#include <string>
#include <vector>

namespace autoware_api
{
class AutowareIvAutowareStatePublisher
{
public:
  explicit AutowareIvAutowareStatePublisher(rclcpp::Node & node);
  void statePublisher(const AutowareInfo & aw_info);

private:
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  // publisher
  rclcpp::Publisher<tier4_api_msgs::msg::AwapiAutowareStatus>::SharedPtr pub_state_;

  // parameter

  /* parameter for judging goal now */
  bool arrived_goal_;
  autoware_auto_system_msgs::msg::AutowareState::_state_type prev_state_;

  void getAutowareStateInfo(
    const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr & autoware_state_ptr,
    tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getControlModeInfo(
    const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr & control_mode_ptr,
    tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getGateModeInfo(
    const tier4_control_msgs::msg::GateMode::ConstSharedPtr & gate_mode_ptr,
    tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getEmergencyStateInfo(
    const autoware_auto_system_msgs::msg::EmergencyState::ConstSharedPtr & emergency_state_ptr,
    tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getCurrentMaxVelInfo(
    const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr & current_max_velocity_ptr,
    tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getHazardStatusInfo(
    const AutowareInfo & aw_info, tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getStopReasonInfo(
    const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr & stop_reason_ptr,
    tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getDiagInfo(const AutowareInfo & aw_info, tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getErrorDiagInfo(
    const AutowareInfo & aw_info, tier4_api_msgs::msg::AwapiAutowareStatus * status);
  void getGlobalRptInfo(
    const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr & global_rpt_ptr,
    tier4_api_msgs::msg::AwapiAutowareStatus * status);

  bool isGoal(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr & autoware_state);
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_AUTOWARE_STATE_PUBLISHER_HPP_
