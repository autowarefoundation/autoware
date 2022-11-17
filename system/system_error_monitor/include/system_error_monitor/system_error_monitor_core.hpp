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

#ifndef SYSTEM_ERROR_MONITOR__SYSTEM_ERROR_MONITOR_CORE_HPP_
#define SYSTEM_ERROR_MONITOR__SYSTEM_ERROR_MONITOR_CORE_HPP_

#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_system_msgs/msg/hazard_status_stamped.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>

#include <boost/optional.hpp>

#include <deque>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

struct DiagStamped
{
  std_msgs::msg::Header header;
  diagnostic_msgs::msg::DiagnosticStatus status;
};

using DiagBuffer = std::deque<DiagStamped>;

struct DiagConfig
{
  std::string name;
  std::string sf_at;
  std::string lf_at;
  std::string spf_at;
  bool auto_recovery;
};

using RequiredModules = std::vector<DiagConfig>;

struct KeyName
{
  static constexpr const char * autonomous_driving = "autonomous_driving";
  static constexpr const char * external_control = "external_control";
};

class AutowareErrorMonitor : public rclcpp::Node
{
public:
  AutowareErrorMonitor();

private:
  // Parameter
  struct Parameters
  {
    int update_rate;
    bool ignore_missing_diagnostics;
    bool add_leaf_diagnostics;
    double data_ready_timeout;
    double data_heartbeat_timeout;
    double diag_timeout_sec;
    double hazard_recovery_timeout;
    int emergency_hazard_level;
    bool use_emergency_hold;
    bool use_emergency_hold_in_manual_driving;
  };

  Parameters params_{};

  rclcpp::Time emergency_state_switch_time_;
  rclcpp::Time initialized_time_;
  autoware_auto_system_msgs::msg::HazardStatus hazard_status_{};
  std::unordered_map<std::string, RequiredModules> required_modules_map_;
  std::string current_mode_;

  void loadRequiredModules(const std::string & key);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  bool isDataHeartbeatTimeout();
  void onTimer();

  // Subscriber
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diag_array_;
  rclcpp::Subscription<autoware_auto_system_msgs::msg::AutowareState>::SharedPtr
    sub_autoware_state_;
  rclcpp::Subscription<tier4_control_msgs::msg::GateMode>::SharedPtr sub_current_gate_mode_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    sub_control_mode_;
  void onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr msg);
  void onCurrentGateMode(const tier4_control_msgs::msg::GateMode::ConstSharedPtr msg);
  void onControlMode(const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg);
  void onDiagArray(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);

  const size_t diag_buffer_size_ = 100;
  std::unordered_map<std::string, DiagBuffer> diag_buffer_map_;
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_array_;
  autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr autoware_state_;
  tier4_control_msgs::msg::GateMode::ConstSharedPtr current_gate_mode_;
  autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr control_mode_;

  // Publisher
  rclcpp::Publisher<autoware_auto_system_msgs::msg::HazardStatusStamped>::SharedPtr
    pub_hazard_status_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_err_;
  void publishHazardStatus(const autoware_auto_system_msgs::msg::HazardStatus & hazard_status);

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_clear_emergency_;
  bool onClearEmergencyService(
    [[maybe_unused]] std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  // for Heartbeat
  rclcpp::Time diag_array_stamp_;
  rclcpp::Time autoware_state_stamp_;
  rclcpp::Time current_gate_mode_stamp_;
  rclcpp::Time control_mode_stamp_;

  // Algorithm
  boost::optional<DiagStamped> getLatestDiag(const std::string & diag_name) const;
  uint8_t getHazardLevel(const DiagConfig & required_module, const int diag_level) const;
  void appendHazardDiag(
    const DiagConfig & required_module, const diagnostic_msgs::msg::DiagnosticStatus & diag,
    autoware_auto_system_msgs::msg::HazardStatus * hazard_status) const;
  autoware_auto_system_msgs::msg::HazardStatus judgeHazardStatus() const;
  void updateHazardStatus();
  bool canAutoRecovery() const;
  bool isEmergencyHoldingRequired() const;
};

#endif  // SYSTEM_ERROR_MONITOR__SYSTEM_ERROR_MONITOR_CORE_HPP_
