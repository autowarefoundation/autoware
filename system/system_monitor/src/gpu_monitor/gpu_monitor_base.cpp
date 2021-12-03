// Copyright 2020 Autoware Foundation
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

/**
 * @file nvml_gpu_monitor.cpp
 * @brief GPU monitor class
 */

#include "system_monitor/gpu_monitor/gpu_monitor_base.hpp"

#include <unistd.h>

#include <string>

GPUMonitorBase::GPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  hostname_{""},
  temp_warn_(declare_parameter<float>("temp_warn", 90.0)),
  temp_error_(declare_parameter<float>("temp_error", 95.0)),
  gpu_usage_warn_(declare_parameter<float>("gpu_usage_warn", 0.90)),
  gpu_usage_error_(declare_parameter<float>("gpu_usage_error", 1.00)),
  memory_usage_warn_(declare_parameter<float>("memory_usage_warn", 0.95)),
  memory_usage_error_(declare_parameter<float>("memory_usage_error", 0.99))
{
  gethostname(hostname_, sizeof(hostname_));

  updater_.setHardwareID(hostname_);
  updater_.add("GPU Temperature", this, &GPUMonitorBase::checkTemp);
  updater_.add("GPU Usage", this, &GPUMonitorBase::checkUsage);
  updater_.add("GPU Memory Usage", this, &GPUMonitorBase::checkMemoryUsage);
  updater_.add("GPU Thermal Throttling", this, &GPUMonitorBase::checkThrottling);
  updater_.add("GPU Frequency", this, &GPUMonitorBase::checkFrequency);
}

void GPUMonitorBase::update() { updater_.force_update(); }

void GPUMonitorBase::shut_down()
{ /*NOOP by default.*/
}

void GPUMonitorBase::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & /* stat */)
{
  RCLCPP_INFO_ONCE(get_logger(), "GPUMonitorBase::checkTemp not implemented.");
}

void GPUMonitorBase::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & /* stat */)
{
  RCLCPP_INFO_ONCE(get_logger(), "GPUMonitorBase::checkUsage not implemented.");
}

void GPUMonitorBase::checkMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper & /* stat */)
{
  RCLCPP_INFO_ONCE(get_logger(), "GPUMonitorBase::checkMemoryUsage not implemented.");
}

void GPUMonitorBase::checkThrottling(diagnostic_updater::DiagnosticStatusWrapper & /* stat */)
{
  RCLCPP_INFO_ONCE(get_logger(), "GPUMonitorBase::checkThrottling not implemented.");
}

void GPUMonitorBase::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & /* stat */)
{
  RCLCPP_INFO_ONCE(get_logger(), "GPUMonitorBase::checkFrequency not implemented.");
}
