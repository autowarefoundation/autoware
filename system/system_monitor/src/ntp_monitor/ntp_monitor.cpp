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
 * @file ntp_monitor.cpp
 * @brief NTP monitor class
 */

#include "system_monitor/ntp_monitor/ntp_monitor.hpp"

#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#include <fmt/format.h>

#include <map>
#include <regex>
#include <string>

namespace bp = boost::process;
namespace fs = boost::filesystem;

NTPMonitor::NTPMonitor(const rclcpp::NodeOptions & options)
: Node("ntp_monitor", options),
  updater_(this),
  offset_warn_(declare_parameter<float>("offset_warn", 0.1)),
  offset_error_(declare_parameter<float>("offset_error", 5.0)),
  timeout_(declare_parameter<int>("timeout", 5)),
  timeout_expired_(false)
{
  using namespace std::literals::chrono_literals;

  gethostname(hostname_, sizeof(hostname_));

  // Check if command exists
  fs::path p = bp::search_path("chronyc");
  chronyc_exists_ = (p.empty()) ? false : true;

  updater_.setHardwareID(hostname_);
  updater_.add("NTP Offset", this, &NTPMonitor::checkOffset);

  // Start timer to execute top command
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&NTPMonitor::onTimer, this), timer_callback_group_);
}

void NTPMonitor::checkOffset(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!chronyc_exists_) {
    stat.summary(DiagStatus::ERROR, "chronyc error");
    stat.add(
      "chronyc", "Command 'chronyc' not found, but can be installed with: sudo apt install chrony");
    return;
  }

  std::string error_str;
  std::string pipe2_err_str;
  float offset = 0.0f;
  std::map<std::string, std::string> tracking_map;
  double elapsed_ms;

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_str = error_str_;
    pipe2_err_str = pipe2_err_str_;
    offset = offset_;
    tracking_map = tracking_map_;
    elapsed_ms = elapsed_ms_;
  }

  if (!pipe2_err_str.empty()) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", pipe2_err_str);
    return;
  }
  if (!error_str.empty()) {
    stat.summary(DiagStatus::ERROR, "chronyc error");
    stat.add("chronyc", error_str);
    return;
  }

  int level = DiagStatus::OK;

  // Check an earlier offset as well
  float abs = std::abs(offset);
  if (abs >= offset_error_) {
    level = DiagStatus::ERROR;
  } else if (abs >= offset_warn_) {
    level = DiagStatus::WARN;
  }
  for (auto itr = tracking_map.begin(); itr != tracking_map.end(); ++itr) {
    stat.add(itr->first, itr->second);
  }

  // Check timeout has expired regarding executing chronyc
  bool timeout_expired = false;
  {
    std::lock_guard<std::mutex> lock(timeout_mutex_);
    timeout_expired = timeout_expired_;
  }

  if (!timeout_expired) {
    stat.summary(level, offset_dict_.at(level));
  } else {
    stat.summary(DiagStatus::WARN, "chronyc timeout expired");
  }

  stat.addf("execution time", "%f ms", elapsed_ms);
}

void NTPMonitor::onTimer()
{
  // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  std::string error_str;
  std::string pipe2_err_str;
  float offset = 0.0f;
  std::map<std::string, std::string> tracking_map;

  // Start timeout timer for executing chronyc
  {
    std::lock_guard<std::mutex> lock(timeout_mutex_);
    timeout_expired_ = false;
  }
  timeout_timer_ = rclcpp::create_timer(
    this, get_clock(), std::chrono::seconds(timeout_), std::bind(&NTPMonitor::onTimeout, this));

  error_str = executeChronyc(offset, tracking_map, pipe2_err_str);

  // Returning from chronyc, stop timeout timer
  timeout_timer_->cancel();

  const double elapsed_ms = stop_watch.toc("execution_time");

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(mutex_);
    error_str_ = error_str;
    pipe2_err_str_ = pipe2_err_str;
    offset_ = offset;
    tracking_map_ = tracking_map;
    elapsed_ms_ = elapsed_ms;
  }
}

void NTPMonitor::onTimeout()
{
  RCLCPP_WARN(get_logger(), "Timeout occurred.");
  std::lock_guard<std::mutex> lock(timeout_mutex_);
  timeout_expired_ = true;
}

std::string NTPMonitor::executeChronyc(
  float & out_offset, std::map<std::string, std::string> & out_tracking_map,
  std::string & pipe2_err_str)
{
  std::string result;

  // Tracking chrony status

  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    pipe2_err_str = std::string(strerror(errno));
    return result;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  bp::child c("chronyc tracking", bp::std_out > is_out);
  c.wait();
  if (c.exit_code() != 0) {
    std::ostringstream os;
    is_out >> os.rdbuf();
    result = os.str().c_str();
    return result;
  }

  std::string line;
  std::cmatch match;
  const std::regex filter("^(.+[A-Za-z()]) *: (.*)");
  const std::regex filter_system_time("([0-9.]*) seconds (slow|fast).*");

  while (std::getline(is_out, line) && !line.empty()) {
    if (std::regex_match(line.c_str(), match, filter)) {
      out_tracking_map[match[1].str()] = match[2].str();
    }
  }

  // System time : conversion string to float
  std::string str_system_time = out_tracking_map["System time"];
  if (std::regex_match(str_system_time.c_str(), match, filter_system_time)) {
    out_offset = std::atof(match[1].str().c_str());

    if (match[2].str() == "fast") {
      // "fast" is - value(match to ntpdate)
      out_offset *= -1;
    } else {
      // "slow" is + value(match to ntpdate)
    }
  } else {
    RCLCPP_WARN(get_logger(), "regex_match: illegal result. str = %s", str_system_time.c_str());
  }
  return result;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(NTPMonitor)
