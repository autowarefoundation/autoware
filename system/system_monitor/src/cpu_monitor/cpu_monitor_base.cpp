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
 * @file cpu_monitor_base.cpp
 * @brief CPU monitor base class
 */

#include "system_monitor/cpu_monitor/cpu_monitor_base.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread.hpp>

#include <fmt/format.h>

#include <algorithm>
#include <regex>
#include <string>

namespace bp = boost::process;
namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

CPUMonitorBase::CPUMonitorBase(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  updater_(this),
  hostname_(),
  num_cores_(0),
  temps_(),
  freqs_(),
  mpstat_exists_(false),
  usage_warn_(declare_parameter<float>("usage_warn", 0.96)),
  usage_error_(declare_parameter<float>("usage_error", 0.96)),
  usage_warn_count_(declare_parameter<int>("usage_warn_count", 1)),
  usage_error_count_(declare_parameter<int>("usage_error_count", 2)),
  usage_avg_(declare_parameter<bool>("usage_avg", true))
{
  gethostname(hostname_, sizeof(hostname_));
  num_cores_ = boost::thread::hardware_concurrency();
  usage_warn_check_cnt_.resize(num_cores_ + 2);   // 2 = all + dummy
  usage_error_check_cnt_.resize(num_cores_ + 2);  // 2 = all + dummy

  // Check if command exists
  fs::path p = bp::search_path("mpstat");
  mpstat_exists_ = (p.empty()) ? false : true;

  updater_.setHardwareID(hostname_);
  updater_.add("CPU Temperature", this, &CPUMonitorBase::checkTemp);
  updater_.add("CPU Usage", this, &CPUMonitorBase::checkUsage);
  updater_.add("CPU Load Average", this, &CPUMonitorBase::checkLoad);
  updater_.add("CPU Thermal Throttling", this, &CPUMonitorBase::checkThrottling);
  updater_.add("CPU Frequency", this, &CPUMonitorBase::checkFrequency);

  // Publisher
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  pub_cpu_usage_ =
    this->create_publisher<tier4_external_api_msgs::msg::CpuUsage>("~/cpu_usage", durable_qos);
}

void CPUMonitorBase::update()
{
  updater_.force_update();
}

void CPUMonitorBase::checkTemp(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (temps_.empty()) {
    stat.summary(DiagStatus::ERROR, "temperature files not found");
    return;
  }

  int level = DiagStatus::OK;
  std::string error_str = "";

  for (auto itr = temps_.begin(); itr != temps_.end(); ++itr) {
    // Read temperature file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (!ifs) {
      stat.add("file open error", itr->path_);
      error_str = "file open error";
      continue;
    }

    float temp;
    ifs >> temp;
    ifs.close();
    temp /= 1000;
    stat.addf(itr->label_, "%.1f DegC", temp);
  }

  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, error_str);
  } else {
    stat.summary(level, temp_dict_.at(level));
  }

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::checkUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  tier4_external_api_msgs::msg::CpuUsage cpu_usage;
  using CpuStatus = tier4_external_api_msgs::msg::CpuStatus;

  if (!mpstat_exists_) {
    stat.summary(DiagStatus::ERROR, "mpstat error");
    stat.add(
      "mpstat", "Command 'mpstat' not found, but can be installed with: sudo apt install sysstat");
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  // Get CPU Usage

  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c("mpstat -P ALL 1 1 -o JSON", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_err >> os.rdbuf();
    stat.summary(DiagStatus::ERROR, "mpstat error");
    stat.add("mpstat", os.str().c_str());
    cpu_usage.all.status = CpuStatus::STALE;
    publishCpuUsage(cpu_usage);
    return;
  }

  std::string cpu_name;
  float usr{0.0};
  float nice{0.0};
  float sys{0.0};
  float iowait{0.0};
  float idle{0.0};
  float usage{0.0};
  float total{0.0};
  int level = DiagStatus::OK;
  int whole_level = DiagStatus::OK;

  pt::ptree pt;
  try {
    // Analyze JSON output
    read_json(is_out, pt);

    for (const pt::ptree::value_type & child1 : pt.get_child("sysstat.hosts")) {
      const pt::ptree & hosts = child1.second;

      for (const pt::ptree::value_type & child2 : hosts.get_child("statistics")) {
        const pt::ptree & statistics = child2.second;

        for (const pt::ptree::value_type & child3 : statistics.get_child("cpu-load")) {
          const pt::ptree & cpu_load = child3.second;
          bool get_cpu_name = false;

          CpuStatus cpu_status;

          if (boost::optional<std::string> v = cpu_load.get_optional<std::string>("cpu")) {
            cpu_name = v.get();
            get_cpu_name = true;
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("usr")) {
            usr = v.get();
            cpu_status.usr = usr;
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("nice")) {
            nice = v.get();
            cpu_status.nice = nice;
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("sys")) {
            sys = v.get();
            cpu_status.sys = sys;
          }
          if (boost::optional<float> v = cpu_load.get_optional<float>("idle")) {
            idle = v.get();
            cpu_status.idle = idle;
          }

          total = 100.0 - iowait - idle;
          usage = total * 1e-2;
          if (get_cpu_name) {
            level = CpuUsageToLevel(cpu_name, usage);
          } else {
            level = CpuUsageToLevel(std::string("err"), usage);
          }

          cpu_status.total = total;
          cpu_status.status = level;

          stat.add(fmt::format("CPU {}: status", cpu_name), load_dict_.at(level));
          stat.addf(fmt::format("CPU {}: total", cpu_name), "%.2f%%", total);
          stat.addf(fmt::format("CPU {}: usr", cpu_name), "%.2f%%", usr);
          stat.addf(fmt::format("CPU {}: nice", cpu_name), "%.2f%%", nice);
          stat.addf(fmt::format("CPU {}: sys", cpu_name), "%.2f%%", sys);
          stat.addf(fmt::format("CPU {}: idle", cpu_name), "%.2f%%", idle);

          if (usage_avg_ == true) {
            if (cpu_name == "all") {
              whole_level = level;
            }
          } else {
            whole_level = std::max(whole_level, level);
          }

          if (cpu_name == "all") {
            cpu_usage.all = cpu_status;
          } else {
            cpu_usage.cpus.push_back(cpu_status);
          }
        }
      }
    }
  } catch (const std::exception & e) {
    stat.summary(DiagStatus::ERROR, "mpstat exception");
    stat.add("mpstat", e.what());
    std::fill(usage_warn_check_cnt_.begin(), usage_warn_check_cnt_.end(), 0);
    std::fill(usage_error_check_cnt_.begin(), usage_error_check_cnt_.end(), 0);
    cpu_usage.all.status = CpuStatus::STALE;
    cpu_usage.cpus.clear();
    publishCpuUsage(cpu_usage);
    return;
  }

  stat.summary(whole_level, load_dict_.at(whole_level));

  // Publish msg
  publishCpuUsage(cpu_usage);

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

int CPUMonitorBase::CpuUsageToLevel(const std::string & cpu_name, float usage)
{
  // cpu name to counter index
  int idx;
  try {
    int num = std::stoi(cpu_name);
    if (num > num_cores_ || num < 0) {
      num = num_cores_;
    }
    idx = num + 1;
  } catch (std::exception &) {
    if (cpu_name == std::string("all")) {  // mpstat output "all"
      idx = 0;
    } else {
      idx = num_cores_ + 1;
    }
  }

  // convert CPU usage to level
  int level = DiagStatus::OK;
  if (usage >= usage_warn_) {
    if (usage_warn_check_cnt_[idx] < usage_warn_count_) {
      usage_warn_check_cnt_[idx]++;
    }
    if (usage_warn_check_cnt_[idx] >= usage_warn_count_) {
      level = DiagStatus::WARN;
    }
  } else {
    usage_warn_check_cnt_[idx] = 0;
  }
  if (usage >= usage_error_) {
    if (usage_error_check_cnt_[idx] < usage_error_count_) {
      usage_error_check_cnt_[idx]++;
    }
    if (usage_error_check_cnt_[idx] >= usage_error_count_) {
      level = DiagStatus::ERROR;
    }
  } else {
    usage_error_check_cnt_[idx] = 0;
  }

  return level;
}

void CPUMonitorBase::checkLoad(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  double loadavg[3];

  std::ifstream ifs("/proc/loadavg", std::ios::in);

  if (!ifs) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", strerror(errno));
    return;
  }

  std::string line;

  if (!std::getline(ifs, line)) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  if (sscanf(line.c_str(), "%lf %lf %lf", &loadavg[0], &loadavg[1], &loadavg[2]) != 3) {
    stat.summary(DiagStatus::ERROR, "uptime error");
    stat.add("uptime", "format error");
    return;
  }

  loadavg[0] /= num_cores_;
  loadavg[1] /= num_cores_;
  loadavg[2] /= num_cores_;

  stat.summary(DiagStatus::OK, "OK");
  stat.addf("1min", "%.2f%%", loadavg[0] * 1e2);
  stat.addf("5min", "%.2f%%", loadavg[1] * 1e2);
  stat.addf("15min", "%.2f%%", loadavg[2] * 1e2);

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::checkThrottling(
  [[maybe_unused]] diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::checkThrottling not implemented.");
}

void CPUMonitorBase::checkFrequency(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Remember start time to measure elapsed time
  const auto t_start = SystemMonitorUtility::startMeasurement();

  if (freqs_.empty()) {
    stat.summary(DiagStatus::ERROR, "frequency files not found");
    return;
  }

  for (auto itr = freqs_.begin(); itr != freqs_.end(); ++itr) {
    // Read scaling_cur_freq file
    const fs::path path(itr->path_);
    fs::ifstream ifs(path, std::ios::in);
    if (ifs) {
      std::string line;
      if (std::getline(ifs, line)) {
        stat.addf(fmt::format("CPU {}: clock", itr->index_), "%d MHz", std::stoi(line) / 1000);
      }
    }
    ifs.close();
  }

  stat.summary(DiagStatus::OK, "OK");

  // Measure elapsed time since start time and report
  SystemMonitorUtility::stopMeasurement(t_start, stat);
}

void CPUMonitorBase::getTempNames()
{
  RCLCPP_INFO(this->get_logger(), "CPUMonitorBase::getTempNames not implemented.");
}

void CPUMonitorBase::getFreqNames()
{
  const fs::path root("/sys/devices/system/cpu");

  for (const fs::path & path :
       boost::make_iterator_range(fs::directory_iterator(root), fs::directory_iterator())) {
    if (!fs::is_directory(path)) {
      continue;
    }

    std::cmatch match;
    const char * cpu_dir = path.generic_string().c_str();

    // /sys/devices/system/cpu[0-9] ?
    if (!std::regex_match(cpu_dir, match, std::regex(".*cpu(\\d+)"))) {
      continue;
    }

    // /sys/devices/system/cpu[0-9]/cpufreq/scaling_cur_freq
    cpu_freq_info freq;
    const fs::path freq_path = path / "cpufreq/scaling_cur_freq";
    freq.index_ = std::stoi(match[1].str());
    freq.path_ = freq_path.generic_string();
    freqs_.push_back(freq);
  }

  std::sort(freqs_.begin(), freqs_.end(), [](const cpu_freq_info & c1, const cpu_freq_info & c2) {
    return c1.index_ < c2.index_;
  });  // NOLINT
}

void CPUMonitorBase::publishCpuUsage(tier4_external_api_msgs::msg::CpuUsage usage)
{
  // Create timestamp
  const auto stamp = this->now();

  usage.stamp = stamp;
  pub_cpu_usage_->publish(usage);
}
