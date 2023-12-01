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

/**
 * @file diag_task.h
 * @brief diagnostics task for high load/memory procs
 */

#ifndef SYSTEM_MONITOR__PROCESS_MONITOR__DIAG_TASK_HPP_
#define SYSTEM_MONITOR__PROCESS_MONITOR__DIAG_TASK_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <string>

/**
 * @brief Struct for storing process information
 */
struct ProcessInfo
{
  std::string processId;
  std::string userName;
  std::string priority;
  std::string niceValue;
  std::string virtualImage;
  std::string residentSize;
  std::string sharedMemSize;
  std::string processStatus;
  std::string cpuUsage;
  std::string memoryUsage;
  std::string cpuTime;
  std::string commandName;
};

class DiagTask : public diagnostic_updater::DiagnosticTask
{
public:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief constructor
   * @param [in] name diagnostics status name
   */
  explicit DiagTask(const std::string & name) : DiagnosticTask(name) { level_ = DiagStatus::STALE; }

  /**
   * @brief main loop
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    if (level_ != DiagStatus::OK) {
      stat.add("content", content_);
    } else {
      stat.add("COMMAND", info_.commandName);
      stat.add("%CPU", info_.cpuUsage);
      stat.add("%MEM", info_.memoryUsage);
      stat.add("PID", info_.processId);
      stat.add("USER", info_.userName);
      stat.add("PR", info_.priority);
      stat.add("NI", info_.niceValue);
      stat.add("VIRT", info_.virtualImage);
      stat.add("RES", info_.residentSize);
      stat.add("SHR", info_.sharedMemSize);
      stat.add("S", info_.processStatus);
      stat.add("TIME+", info_.cpuTime);
    }
    stat.summary(level_, message_);
  }

  /**
   * @brief set diagnostics status
   * @param [in] status Diagnostics error level
   * @param [in] message Diagnostics status message
   */
  void setDiagnosticsStatus(int level, const std::string & message)
  {
    level_ = level;
    message_ = message;
  }

  /**
   * @brief set error content
   * @param [in] error_command Error command
   * @param [in] content Error content
   */
  void setErrorContent(const std::string & error_command, const std::string & content)
  {
    error_command_ = error_command;
    content_ = content;
  }

  /**
   * @brief Set process information
   * @param [in] info Process information
   */
  void setProcessInformation(const struct ProcessInfo & info) { info_ = info; }

private:
  int level_;                  //!< @brief Diagnostics error level
  std::string message_;        //!< @brief Diagnostics status message
  std::string error_command_;  //!< @brief Error command
  std::string content_;        //!< @brief Error content
  struct ProcessInfo info_;    //!< @brief Struct for storing process information
};

#endif  // SYSTEM_MONITOR__PROCESS_MONITOR__DIAG_TASK_HPP_
