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

class DiagTask : public diagnostic_updater::DiagnosticTask
{
public:
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  /**
   * @brief constructor
   * @param [in] name diagnostics status name
   */
  explicit DiagTask(const std::string & name) : DiagnosticTask(name) {}

  /**
   * @brief main loop
   * @param [out] stat diagnostic message passed directly to diagnostic publish calls
   */
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    stat.summary(level_, message_);

    if (level_ != DiagStatus::OK) {
      stat.add("content", content_);
    } else {
      stat.add("COMMAND", command_);
      stat.add("%CPU", cpu_);
      stat.add("%MEM", mem_);
      stat.add("PID", pid_);
      stat.add("USER", user_);
      stat.add("PR", pr_);
      stat.add("NI", ni_);
      stat.add("VIRT", virt_);
      stat.add("RES", res_);
      stat.add("SHR", shr_);
      stat.add("S", s_);
      stat.add("TIME+", time_);
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
   * @brief set process id
   * @param [in] pid process id
   */
  void setProcessId(const std::string & pid) { pid_ = pid; }

  /**
   * @brief set user name
   * @param [in] user user name
   */
  void setUserName(const std::string & user) { user_ = user; }

  /**
   * @brief set priority
   * @param [in] pr priority
   */
  void setPriority(const std::string & pr) { pr_ = pr; }

  /**
   * @brief set nice value
   * @param [in] ni nice value
   */
  void setNiceValue(const std::string & ni) { ni_ = ni; }

  /**
   * @brief set virtual image
   * @param [in] virt virtual image
   */
  void setVirtualImage(const std::string & virt) { virt_ = virt; }

  /**
   * @brief set resident size
   * @param [in] res resident size
   */
  void setResidentSize(const std::string & res) { res_ = res; }

  /**
   * @brief set shared mem size
   * @param [in] shr shared mem size
   */
  void setSharedMemSize(const std::string & shr) { shr_ = shr; }

  /**
   * @brief set process status
   * @param [in] s process status
   */
  void setProcessStatus(const std::string & s) { s_ = s; }

  /**
   * @brief set CPU usage
   * @param [in] cpu CPU usage
   */
  void setCPUUsage(const std::string & cpu) { cpu_ = cpu; }

  /**
   * @brief set memory usage
   * @param [in] mem memory usage
   */
  void setMemoryUsage(const std::string & mem) { mem_ = mem; }

  /**
   * @brief set CPU time
   * @param [in] time CPU time
   */
  void setCPUTime(const std::string & time) { time_ = time; }

  /**
   * @brief set Command name/line
   * @param [in] command Command name/line
   */
  void setCommandName(const std::string & command) { command_ = command; }

private:
  int level_;                  //!< @brief Diagnostics error level
  std::string message_;        //!< @brief Diagnostics status message
  std::string error_command_;  //!< @brief Error command
  std::string content_;        //!< @brief Error content

  std::string pid_;      //!< @brief Process Id
  std::string user_;     //!< @brief User Name
  std::string pr_;       //!< @brief Priority
  std::string ni_;       //!< @brief Nice value
  std::string virt_;     //!< @brief Virtual Image (kb)
  std::string res_;      //!< @brief Resident size (kb)
  std::string shr_;      //!< @brief Shared Mem size (kb)
  std::string s_;        //!< @brief Process Status
  std::string cpu_;      //!< @brief CPU usage
  std::string mem_;      //!< @brief Memory usage
  std::string time_;     //!< @brief CPU Time
  std::string command_;  //!< @brief Command name/line
};

#endif  // SYSTEM_MONITOR__PROCESS_MONITOR__DIAG_TASK_HPP_
