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
 * @file tegra_gpu_monitor.cpp
 * @brief Tegra GPU monitor class
 */

#include "system_monitor/gpu_monitor/tegra_gpu_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/filesystem.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <regex>
#include <string>
#include <vector>

namespace fs = boost::filesystem;

GPUMonitor::GPUMonitor(const rclcpp::NodeOptions & options) : GPUMonitorBase("gpu_monitor", options)
{
  getTempNames();
  getLoadNames();
  getFreqNames();

  // There is no separate gpu memory in tegra. Both cpu and gpu uses cpu memory. thus remove.
  updater_.removeByName("GPU Memory Usage");
  // There is no event record for thermal throttling.
  // Need to manually monitor temperature to figure out if thermal limits crossed or not.
  updater_.removeByName("GPU Thermal Throttling");
}

void GPUMonitor::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (temps_.empty()) {
    stat.summary(DiagStatus::ERROR, "temperature files not found");
    return;
  }

  int level = DiagStatus::OK;
  std::string error_str;

  for (const auto & itr : temps_) {
    // Read temperature file
    const fs::path path(itr.path_);
    fs::ifstream ifs(path, std::ios::in);
    if (!ifs) {
      stat.add("file open error", itr.path_);
      error_str = "file open error";
      continue;
    }

    float temp{};
    ifs >> temp;
    ifs.close();
    temp /= 1000;
    stat.addf(itr.label_, "%.1f DegC", temp);

    level = DiagStatus::OK;
    if (temp >= temp_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (temp >= temp_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(level, temp_dict_.at(level));
  }
}

void GPUMonitor::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (loads_.empty()) {
    stat.summary(DiagStatus::ERROR, "load files not found");
    return;
  }

  int level = DiagStatus::OK;
  std::string error_str;

  for (const auto & itr : loads_) {
    // Read load file
    const fs::path path(itr.path_);
    fs::ifstream ifs(path, std::ios::in);
    if (!ifs) {
      stat.add("file open error", itr.path_);
      error_str = "file open error";
      continue;
    }

    float load{};
    ifs >> load;
    ifs.close();
    stat.addf(itr.label_, "%.1f%%", load / 10);

    level = DiagStatus::OK;
    load /= 1000;
    if (load >= gpu_usage_error_) {
      level = std::max(level, static_cast<int>(DiagStatus::ERROR));
    } else if (load >= gpu_usage_warn_) {
      level = std::max(level, static_cast<int>(DiagStatus::WARN));
    }
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(level, load_dict_.at(level));
  }
}

void GPUMonitor::checkThrottling(
  [[maybe_unused]] diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Please remove the [[maybe_unused]] tag after implementation, it's a temp build fix
  // TODO(Fumihito Ito): implement me
}

void GPUMonitor::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (freqs_.empty()) {
    stat.summary(DiagStatus::ERROR, "frequency files not found");
    return;
  }

  for (const auto & freq : freqs_) {
    // Read cur_freq file
    const fs::path path(freq.path_);
    fs::ifstream ifs(path, std::ios::in);
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        stat.addf(fmt::format("GPU {}: clock", freq.label_), "%d MHz", std::stoi(line) / 1000000);
      }
    }
    ifs.close();
  }

  stat.summary(DiagStatus::OK, "OK");
}

void GPUMonitor::getTempNames()
{
  // Jetson TX1 TX2 Nano: thermal_zone1, Xavier: thermal_zone0
  std::vector<thermal_zone> therms;
  SystemMonitorUtility::getThermalZone("GPU-therm", &therms);

  for (const auto & therm : therms) {
    temps_.emplace_back(therm.label_, therm.path_);
  }
}

void GPUMonitor::getLoadNames()
{
  const fs::path root("/sys/devices");

  for (const fs::path & path :
       boost::make_iterator_range(fs::directory_iterator(root), fs::directory_iterator())) {
    if (!fs::is_directory(path)) {
      continue;
    }

    std::cmatch match;
    const char * str_path = path.generic_string().c_str();

    // /sys/devices/gpu.[0-9] ?
    if (!std::regex_match(str_path, match, std::regex(".*gpu\\.(\\d+)"))) {
      continue;
    }

    // /sys/devices/gpu.[0-9]/load
    const fs::path load_path = path / "load";
    loads_.emplace_back(path.filename().generic_string(), load_path.generic_string());
  }
}

void GPUMonitor::getFreqNames()
{
  const fs::path root("/sys/class/devfreq");

  for (const fs::path & path :
       boost::make_iterator_range(fs::directory_iterator(root), fs::directory_iterator())) {
    // /sys/class/devfreq/?????/cur_freq ?
    if (!fs::is_directory(path)) {
      continue;
    }

    const fs::path freq_path = path / "cur_freq";
    freqs_.emplace_back(path.filename().generic_string(), freq_path.generic_string());
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GPUMonitor)
