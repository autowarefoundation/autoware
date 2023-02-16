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

#ifndef DUMMY_DIAG_PUBLISHER__DUMMY_DIAG_PUBLISHER_CORE_HPP_
#define DUMMY_DIAG_PUBLISHER__DUMMY_DIAG_PUBLISHER_CORE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>
#include <vector>

struct DiagConfig
{
  std::string hardware_id;
  std::string msg_ok;
  std::string msg_warn;
  std::string msg_error;
  std::string msg_stale;
};

class DummyDiagPublisher : public rclcpp::Node
{
public:
  DummyDiagPublisher();

private:
  enum Status {
    OK,
    WARN,
    ERROR,
    STALE,
  };

  struct DummyDiagConfig
  {
    std::string name;
    bool is_active;
    Status status;
  };

  using RequiredDiags = std::vector<DummyDiagConfig>;

  // Parameter
  double update_rate_;
  DiagConfig diag_config_;
  DummyDiagConfig config_;

  RequiredDiags required_diags_;
  void loadRequiredDiags();

  std::optional<Status> convertStrToStatus(std::string & status_str);
  std::string convertStatusToStr(const Status & status);

  // Dynamic Reconfigure
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  rcl_interfaces::msg::SetParametersResult updateDiag(
    const std::string diag_name, DummyDiagConfig & config, bool is_active, const Status status);

  // Diagnostic Updater
  // Set long period to reduce automatic update
  diagnostic_updater::Updater updater_{this, 1000.0 /* sec */};

  void produceOKDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void produceErrorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void produceWarnDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void produceStaleDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void addDiagByStatus(const std::string & diag_name, const Status status);
  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DUMMY_DIAG_PUBLISHER__DUMMY_DIAG_PUBLISHER_CORE_HPP_
