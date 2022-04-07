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

#ifndef DUMMY_DIAG_PUBLISHER__DUMMY_DIAG_PUBLISHER_NODE_HPP_
#define DUMMY_DIAG_PUBLISHER__DUMMY_DIAG_PUBLISHER_NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

struct DiagConfig
{
  std::string name;
  std::string hardware_id;
  std::string msg_ok;
  std::string msg_warn;
  std::string msg_error;
  std::string msg_stale;
};

class DummyDiagPublisherNode : public rclcpp::Node
{
public:
  explicit DummyDiagPublisherNode(const rclcpp::NodeOptions & node_options);

private:
  enum Status {
    OK,
    WARN,
    ERROR,
    STALE,
  };

  struct DummyDiagPublisherConfig
  {
    Status status;
    bool is_active;
  };

  // Parameter
  double update_rate_;
  DiagConfig diag_config_;
  DummyDiagPublisherConfig config_;

  // Dynamic Reconfigure
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  // Diagnostic Updater
  // Set long period to reduce automatic update
  diagnostic_updater::Updater updater_{this, 1000.0 /* sec */};

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Timer
  void onTimer();
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // DUMMY_DIAG_PUBLISHER__DUMMY_DIAG_PUBLISHER_NODE_HPP_
