// Copyright 2024 TIER IV, Inc.
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

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef FAULT_INJECTION__FAULT_INJECTION_DIAG_UPDATER_HPP_
#define FAULT_INJECTION__FAULT_INJECTION_DIAG_UPDATER_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fault_injection
{
class FaultInjectionDiagUpdater : public diagnostic_updater::DiagnosticTaskVector
{
public:
  template <class NodeT>
  explicit FaultInjectionDiagUpdater(NodeT node, double period = 1.0)
  : FaultInjectionDiagUpdater(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_timers_interface(),
      node->get_node_topics_interface(), period)
  {
  }

  FaultInjectionDiagUpdater(
    const std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> & base_interface,
    const std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> & clock_interface,
    const std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> & logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface,
    double period = 1.0)
  : base_interface_(base_interface),
    timers_interface_(std::move(timers_interface)),
    clock_(clock_interface->get_clock()),
    period_(rclcpp::Duration::from_nanoseconds(static_cast<rcl_duration_value_t>(period * 1e9))),
    publisher_(rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      topics_interface, "/diagnostics", 1)),
    logger_(logging_interface->get_logger()),
    node_name_(base_interface->get_name())
  {
    reset_timer();
  }

  /**
   * \brief Returns the interval between updates.
   */
  [[nodiscard]] auto get_period() const { return period_; }

  /**
   * \brief Sets the period as a rclcpp::Duration
   */
  void set_period(const rclcpp::Duration & period)
  {
    period_ = period;
    reset_timer();
  }

  /**
   * \brief Sets the period given a value in seconds
   */
  void set_period(double period)
  {
    set_period(rclcpp::Duration::from_nanoseconds(static_cast<rcl_duration_value_t>(period * 1e9)));
  }

  /**
   * \brief Forces to send out an update for all known DiagnosticStatus.
   */
  void force_update() { update(); }

  /**
   * \brief Output a message on all the known DiagnosticStatus.
   *
   * Useful if something drastic is happening such as shutdown or a
   * self-test.
   *
   * \param lvl Level of the diagnostic being output.
   *
   * \param msg Status message to output.
   */
  void broadcast(int lvl, const std::string & msg)
  {
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

    const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
    for (const auto & task : tasks) {
      diagnostic_updater::DiagnosticStatusWrapper status;

      status.name = task.getName();
      status.summary(lvl, msg);

      status_vec.push_back(status);
    }

    publish(status_vec);
  }

  void set_hardware_id_f(const char * format, ...)
  {
    va_list va;
    const int k_buffer_size = 1000;
    char buff[1000];  // @todo This could be done more elegantly.
    va_start(va, format);
    if (vsnprintf(buff, k_buffer_size, format, va) >= k_buffer_size) {
      RCLCPP_DEBUG(logger_, "Really long string in diagnostic_updater::setHardwareIDf.");
    }
    hardware_id_ = std::string(buff);
    va_end(va);
  }

  void set_hardware_id(const std::string & hardware_id) { hardware_id_ = hardware_id; }

private:
  void reset_timer()
  {
    update_timer_ = rclcpp::create_timer(
      base_interface_, timers_interface_, clock_, period_,
      std::bind(&FaultInjectionDiagUpdater::update, this));
  }

  /**
   * \brief Causes the diagnostics to update if the inter-update interval
   * has been exceeded.
   */
  void update()
  {
    if (rclcpp::ok()) {
      std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;

      std::unique_lock<std::mutex> lock(
        lock_);  // Make sure no adds happen while we are processing here.
      const std::vector<DiagnosticTaskInternal> & tasks = getTasks();
      for (const auto & task : tasks) {
        diagnostic_updater::DiagnosticStatusWrapper status;

        status.name = task.getName();
        status.level = 2;
        status.message = "No message was set";
        status.hardware_id = hardware_id_;

        task.run(status);

        status_vec.push_back(status);
      }

      publish(status_vec);
    }
  }

  /**
   * Publishes a single diagnostic status.
   */
  void publish(diagnostic_msgs::msg::DiagnosticStatus & stat)
  {
    std::vector<diagnostic_msgs::msg::DiagnosticStatus> status_vec;
    status_vec.push_back(stat);
    publish(status_vec);
  }

  /**
   * Publishes a vector of diagnostic statuses.
   */
  void publish(std::vector<diagnostic_msgs::msg::DiagnosticStatus> & status_vec)
  {
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.status = status_vec;
    msg.header.stamp = clock_->now();
    publisher_->publish(msg);
  }

  /**
   * Causes a placeholder DiagnosticStatus to be published as soon as a
   * diagnostic task is added to the Updater.
   */
  void addedTaskCallback(DiagnosticTaskInternal & task) override
  {
    diagnostic_updater::DiagnosticStatusWrapper stat;
    stat.name = task.getName();
    stat.summary(0, "Node starting up");
    publish(stat);
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers_interface_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Duration period_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::Logger logger_;

  std::string hardware_id_;
  std::string node_name_;
};
}  // namespace fault_injection

#endif  // FAULT_INJECTION__FAULT_INJECTION_DIAG_UPDATER_HPP_
