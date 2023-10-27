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
 * @file process_monitor.cpp
 * @brief Process monitor class
 */

#include "system_monitor/process_monitor/process_monitor.hpp"

#include "system_monitor/system_monitor_utility.hpp"

#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <fmt/format.h>

#include <memory>
#include <regex>
#include <string>
#include <vector>

ProcessMonitor::ProcessMonitor(const rclcpp::NodeOptions & options)
: Node("process_monitor", options),
  updater_(this),
  num_of_procs_(declare_parameter<int>("num_of_procs", 5)),
  is_top_error_(false),
  is_pipe2_error_(false)
{
  using namespace std::literals::chrono_literals;

  int index;

  gethostname(hostname_, sizeof(hostname_));

  updater_.setHardwareID(hostname_);
  updater_.add("Tasks Summary", this, &ProcessMonitor::monitorProcesses);

  for (index = 0; index < num_of_procs_; ++index) {
    auto task = std::make_shared<DiagTask>(fmt::format("High-load Proc[{}]", index));
    load_tasks_.push_back(task);
    updater_.add(*task);
  }
  for (index = 0; index < num_of_procs_; ++index) {
    auto task = std::make_shared<DiagTask>(fmt::format("High-mem Proc[{}]", index));
    memory_tasks_.push_back(task);
    updater_.add(*task);
  }

  // Start timer to execute top command
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&ProcessMonitor::onTimer, this), timer_callback_group_);
}

void ProcessMonitor::monitorProcesses(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // thread-safe read
  std::string str;
  bool is_top_error;
  bool is_pipe2_error;
  double elapsed_ms;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    str = top_output_;
    is_top_error = is_top_error_;
    is_pipe2_error = is_pipe2_error_;
    elapsed_ms = elapsed_ms_;
  }

  if (is_pipe2_error) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", str);
    setErrorContent(&load_tasks_, "pipe2 error", "pipe2", str);
    setErrorContent(&memory_tasks_, "pipe2 error", "pipe2", str);
    return;
  }
  if (is_top_error) {
    stat.summary(DiagStatus::ERROR, "top error");
    stat.add("top", str);
    setErrorContent(&load_tasks_, "top error", "top", str);
    setErrorContent(&memory_tasks_, "top error", "top", str);
    return;
  }

  // If top still not running
  if (str.empty()) {
    // Send OK tentatively
    stat.summary(DiagStatus::OK, "starting up");
    return;
  }

  // Get task summary
  getTasksSummary(stat, str);
  // Remove header
  removeHeader(stat, str);

  // Get high load processes
  getHighLoadProcesses(str);

  // Get high memory processes
  getHighMemoryProcesses(str);

  stat.addf("execution time", "%f ms", elapsed_ms);
}

void ProcessMonitor::getTasksSummary(
  diagnostic_updater::DiagnosticStatusWrapper & stat, const std::string & output)
{
  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int p_fd[2];
  if (pipe2(p_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe p{p_fd[0], p_fd[1]};

  std::string line;

  // Echo output for grep
  {
    int out_fd[2];
    if (pipe2(out_fd, O_CLOEXEC) != 0) {
      stat.summary(DiagStatus::ERROR, "pipe2 error");
      stat.add("pipe2", strerror(errno));
      return;
    }
    bp::pipe out_pipe{out_fd[0], out_fd[1]};
    bp::ipstream is_out{std::move(out_pipe)};

    int err_fd[2];
    if (pipe2(err_fd, O_CLOEXEC) != 0) {
      stat.summary(DiagStatus::ERROR, "pipe2 error");
      stat.add("pipe2", strerror(errno));
      return;
    }
    bp::pipe err_pipe{err_fd[0], err_fd[1]};
    bp::ipstream is_err{std::move(err_pipe)};

    bp::child c(fmt::format("echo {}", output), bp::std_out > p, bp::std_err > is_err);
    c.wait();
    if (c.exit_code() != 0) {
      std::ostringstream os;
      is_err >> os.rdbuf();
      stat.summary(DiagStatus::ERROR, "echo error");
      stat.add("echo", os.str().c_str());
      return;
    }
  }
  // Find matching pattern of summary
  {
    int out_fd[2];
    if (pipe2(out_fd, O_CLOEXEC) != 0) {
      stat.summary(DiagStatus::ERROR, "pipe2 error");
      stat.add("pipe2", strerror(errno));
      return;
    }
    bp::pipe out_pipe{out_fd[0], out_fd[1]};
    bp::ipstream is_out{std::move(out_pipe)};

    bp::child c("grep Tasks:", bp::std_out > is_out, bp::std_in < p);
    c.wait();
    // no matching line
    if (c.exit_code() != 0) {
      stat.summary(DiagStatus::ERROR, "matching pattern not found");
      stat.add("name", "Tasks:");
      return;
    }

    std::getline(is_out, line);
    std::cmatch match;
    const std::regex filter(
      "^Tasks: (\\d+) total,\\s+(\\d+) running,\\s+(\\d+) sleeping,\\s+(\\d+) stopped,\\s+(\\d+) "
      "zombie");

    if (std::regex_match(line.c_str(), match, filter)) {
      stat.add("total", match[1].str());
      stat.add("running", match[2].str());
      stat.add("sleeping", match[3].str());
      stat.add("stopped", match[4].str());
      stat.add("zombie", match[5].str());
      stat.summary(DiagStatus::OK, "OK");
    } else {
      stat.summary(DiagStatus::ERROR, "invalid format");
    }
  }
}

void ProcessMonitor::removeHeader(
  diagnostic_updater::DiagnosticStatusWrapper & stat, std::string & output)
{
  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int p1_fd[2];
  if (pipe2(p1_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe p1{p1_fd[0], p1_fd[1]};

  int p2_fd[2];
  if (pipe2(p2_fd, O_CLOEXEC) != 0) {
    stat.summary(DiagStatus::ERROR, "pipe2 error");
    stat.add("pipe2", strerror(errno));
    return;
  }
  bp::pipe p2{p2_fd[0], p2_fd[1]};

  std::ostringstream os;

  // Echo output for sed
  {
    int err_fd[2];
    if (pipe2(err_fd, O_CLOEXEC) != 0) {
      stat.summary(DiagStatus::ERROR, "pipe2 error");
      stat.add("pipe2", strerror(errno));
      return;
    }
    bp::pipe err_pipe{err_fd[0], err_fd[1]};
    bp::ipstream is_err{std::move(err_pipe)};

    bp::child c(fmt::format("echo {}", output), bp::std_out > p1, bp::std_err > is_err);
    c.wait();
    if (c.exit_code() != 0) {
      is_err >> os.rdbuf();
      stat.summary(DiagStatus::ERROR, "echo error");
      stat.add("echo", os.str().c_str());
      return;
    }
  }
  // Remove %Cpu section
  {
    int err_fd[2];
    if (pipe2(err_fd, O_CLOEXEC) != 0) {
      stat.summary(DiagStatus::ERROR, "pipe2 error");
      stat.add("pipe2", strerror(errno));
      return;
    }
    bp::pipe err_pipe{err_fd[0], err_fd[1]};
    bp::ipstream is_err{std::move(err_pipe)};

    bp::child c("sed \"/^%Cpu/d\"", bp::std_out > p2, bp::std_err > is_err, bp::std_in < p1);
    c.wait();
    // no matching line
    if (c.exit_code() != 0) {
      stat.summary(DiagStatus::ERROR, "sed error");
      stat.add("sed", "Failed to remove header");
      return;
    }
  }
  // Remove header
  {
    int out_fd[2];
    if (pipe2(out_fd, O_CLOEXEC) != 0) {
      stat.summary(DiagStatus::ERROR, "pipe2 error");
      stat.add("pipe2", strerror(errno));
      return;
    }
    bp::pipe out_pipe{out_fd[0], out_fd[1]};
    bp::ipstream is_out{std::move(out_pipe)};

    int err_fd[2];
    if (pipe2(err_fd, O_CLOEXEC) != 0) {
      stat.summary(DiagStatus::ERROR, "pipe2 error");
      stat.add("pipe2", strerror(errno));
      return;
    }
    bp::pipe err_pipe{err_fd[0], err_fd[1]};
    bp::ipstream is_err{std::move(err_pipe)};

    bp::child c("sed \"1,6d\"", bp::std_out > is_out, bp::std_err > is_err, bp::std_in < p2);
    c.wait();
    // no matching line
    if (c.exit_code() != 0) {
      stat.summary(DiagStatus::ERROR, "sed error");
      stat.add("sed", "Failed to remove header");
      return;
    }
    // overwrite
    is_out >> os.rdbuf();
    output = os.str();
  }
}

void ProcessMonitor::getHighLoadProcesses(const std::string & output)
{
  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int p_fd[2];
  if (pipe2(p_fd, O_CLOEXEC) != 0) {
    setErrorContent(&load_tasks_, "pipe2 error", "pipe2", strerror(errno));
    return;
  }
  bp::pipe p{p_fd[0], p_fd[1]};

  std::ostringstream os;

  // Echo output for sed
  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    setErrorContent(&load_tasks_, "pipe2 error", "pipe2", strerror(errno));
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    setErrorContent(&load_tasks_, "pipe2 error", "pipe2", strerror(errno));
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  bp::child c(fmt::format("echo {}", output), bp::std_out > p, bp::std_err > is_err);
  c.wait();
  if (c.exit_code() != 0) {
    is_err >> os.rdbuf();
    setErrorContent(&load_tasks_, "echo error", "echo", os.str().c_str());
    return;
  }

  // Get top-rated
  getTopratedProcesses(&load_tasks_, &p);
}

void ProcessMonitor::getHighMemoryProcesses(const std::string & output)
{
  // boost::process create file descriptor without O_CLOEXEC required for multithreading.
  // So create file descriptor with O_CLOEXEC and pass it to boost::process.
  int p1_fd[2];
  if (pipe2(p1_fd, O_CLOEXEC) != 0) {
    setErrorContent(&memory_tasks_, "pipe2 error", "pipe2", strerror(errno));
    return;
  }
  bp::pipe p1{p1_fd[0], p1_fd[1]};

  int p2_fd[2];
  if (pipe2(p2_fd, O_CLOEXEC) != 0) {
    setErrorContent(&memory_tasks_, "pipe2 error", "pipe2", strerror(errno));
    return;
  }
  bp::pipe p2{p2_fd[0], p2_fd[1]};

  std::ostringstream os;

  // Echo output for sed
  {
    int out_fd[2];
    if (pipe2(out_fd, O_CLOEXEC) != 0) {
      setErrorContent(&memory_tasks_, "pipe2 error", "pipe2", strerror(errno));
      return;
    }
    bp::pipe out_pipe{out_fd[0], out_fd[1]};
    bp::ipstream is_out{std::move(out_pipe)};

    int err_fd[2];
    if (pipe2(err_fd, O_CLOEXEC) != 0) {
      setErrorContent(&memory_tasks_, "pipe2 error", "pipe2", strerror(errno));
      return;
    }
    bp::pipe err_pipe{err_fd[0], err_fd[1]};
    bp::ipstream is_err{std::move(err_pipe)};

    bp::child c(fmt::format("echo {}", output), bp::std_out > p1, bp::std_err > is_err);
    c.wait();
    if (c.exit_code() != 0) {
      is_err >> os.rdbuf();
      setErrorContent(&memory_tasks_, "echo error", "echo", os.str().c_str());
      return;
    }
  }
  // Sort by memory usage
  {
    int out_fd[2];
    if (pipe2(out_fd, O_CLOEXEC) != 0) {
      setErrorContent(&memory_tasks_, "pipe2 error", "pipe2", strerror(errno));
      return;
    }
    bp::pipe out_pipe{out_fd[0], out_fd[1]};
    bp::ipstream is_out{std::move(out_pipe)};

    int err_fd[2];
    if (pipe2(err_fd, O_CLOEXEC) != 0) {
      setErrorContent(&memory_tasks_, "pipe2 error", "pipe2", strerror(errno));
      return;
    }
    bp::pipe err_pipe{err_fd[0], err_fd[1]};
    bp::ipstream is_err{std::move(err_pipe)};

    bp::child c("sort -r -k 10 -n", bp::std_out > p2, bp::std_err > is_err, bp::std_in < p1);
    c.wait();
    if (c.exit_code() != 0) {
      is_err >> os.rdbuf();
      setErrorContent(&memory_tasks_, "sort error", "sort", os.str().c_str());
      return;
    }
  }

  // Get top-rated
  getTopratedProcesses(&memory_tasks_, &p2);
}

bool ProcessMonitor::getCommandLineFromPiD(const std::string & pid, std::string & command)
{
  std::string commandLineFilePath = "/proc/" + pid + "/cmdline";
  std::ifstream commandFile(commandLineFilePath, std::ios::in | std::ios::binary);

  if (commandFile.is_open()) {
    std::vector<uint8_t> buffer;
    std::copy(
      std::istream_iterator<uint8_t>(commandFile), std::istream_iterator<uint8_t>(),
      std::back_inserter(buffer));
    commandFile.close();
    std::replace(
      buffer.begin(), buffer.end(), '\0',
      ' ');  // 0x00 is used as delimiter in /cmdline instead of 0x20 (space)
    command = std::string(buffer.begin(), buffer.end());
    return (buffer.size() > 0) ? true : false;  // cmdline is empty if it is kernel process
  } else {
    return false;
  }
}

void ProcessMonitor::getTopratedProcesses(
  std::vector<std::shared_ptr<DiagTask>> * tasks, bp::pipe * p)
{
  if (tasks == nullptr || p == nullptr) {
    return;
  }

  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    setErrorContent(tasks, "pipe2 error", "pipe2", strerror(errno));
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    setErrorContent(tasks, "pipe2 error", "pipe2", strerror(errno));
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  std::ostringstream os;

  bp::child c(
    fmt::format("sed -n \"1,{} p\"", num_of_procs_), bp::std_out > is_out, bp::std_err > is_err,
    bp::std_in < *p);

  c.wait();
  // Failed to modify line
  if (c.exit_code() != 0) {
    is_err >> os.rdbuf();
    setErrorContent(tasks, "sed error", "sed", os.str().c_str());
    return;
  }

  std::string line;
  int index = 0;

  while (std::getline(is_out, line) && !line.empty()) {
    std::istringstream stream(line);

    ProcessInfo info;
    stream >> info.processId >> info.userName >> info.priority >> info.niceValue >>
      info.virtualImage >> info.residentSize >> info.sharedMemSize >> info.processStatus >>
      info.cpuUsage >> info.memoryUsage >> info.cpuTime;

    std::string program_name;
    std::getline(stream, program_name);

    bool flag_find_command_line = getCommandLineFromPiD(info.processId, info.commandName);

    if (!flag_find_command_line) {
      info.commandName = program_name;  // if command line is not found, use program name instead
    }

    tasks->at(index)->setDiagnosticsStatus(DiagStatus::OK, "OK");
    tasks->at(index)->setProcessInformation(info);
    ++index;
  }
}

void ProcessMonitor::setErrorContent(
  std::vector<std::shared_ptr<DiagTask>> * tasks, const std::string & message,
  const std::string & error_command, const std::string & content)
{
  if (tasks == nullptr) {
    return;
  }

  for (auto itr = tasks->begin(); itr != tasks->end(); ++itr) {
    (*itr)->setDiagnosticsStatus(DiagStatus::ERROR, message);
    (*itr)->setErrorContent(error_command, content);
  }
}

void ProcessMonitor::onTimer()
{
  bool is_top_error = false;

  // Start to measure elapsed time
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("execution_time");

  int out_fd[2];
  if (pipe2(out_fd, O_CLOEXEC) != 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    top_output_ = std::string(strerror(errno));
    is_pipe2_error_ = true;
    return;
  }
  bp::pipe out_pipe{out_fd[0], out_fd[1]};
  bp::ipstream is_out{std::move(out_pipe)};

  int err_fd[2];
  if (pipe2(err_fd, O_CLOEXEC) != 0) {
    std::lock_guard<std::mutex> lock(mutex_);
    top_output_ = std::string(strerror(errno));
    is_pipe2_error_ = true;
    return;
  }
  bp::pipe err_pipe{err_fd[0], err_fd[1]};
  bp::ipstream is_err{std::move(err_pipe)};

  std::ostringstream os;

  // Get processes
  bp::child c("top -bn1 -o %CPU -w 128", bp::std_out > is_out, bp::std_err > is_err);
  c.wait();

  if (c.exit_code() != 0) {
    is_top_error = true;
    is_err >> os.rdbuf();
  } else {
    is_out >> os.rdbuf();
  }

  const double elapsed_ms = stop_watch.toc("execution_time");

  // thread-safe copy
  {
    std::lock_guard<std::mutex> lock(mutex_);
    top_output_ = os.str();
    is_top_error_ = is_top_error;
    is_pipe2_error_ = false;
    elapsed_ms_ = elapsed_ms;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ProcessMonitor)
