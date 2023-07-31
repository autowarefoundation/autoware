// Copyright 2023 TIER IV
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

#ifndef YABLOC_MONITOR_CORE_HPP_
#define YABLOC_MONITOR_CORE_HPP_

#include "availability_module.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

class YabLocMonitor : public rclcpp::Node
{
public:
  YabLocMonitor();

private:
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void on_timer();

  diagnostic_updater::Updater updater_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Evaluation modules
  std::unique_ptr<AvailabilityModule> availability_module_;
};
#endif  // YABLOC_MONITOR_CORE_HPP_
