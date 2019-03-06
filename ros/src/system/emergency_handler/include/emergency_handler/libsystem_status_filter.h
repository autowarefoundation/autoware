#ifndef __SYSTEM_STATUS_FILTER_H__
#define __SYSTEM_STATUS_FILTER_H__
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <autoware_system_msgs/SystemStatus.h>
#include <set>
#include <map>
#include <emergency_handler/libvital_monitor.h>

typedef autoware_system_msgs::SystemStatus SystemStatus;
typedef autoware_system_msgs::DiagnosticStatusArray DiagnosticStatusArray;
typedef autoware_system_msgs::DiagnosticStatus DiagnosticStatus;
typedef autoware_system_msgs::NodeStatus NodeStatus;
typedef autoware_system_msgs::HardwareStatus HardwareStatus;

enum StatusType
{
  NONE,
  NOT_READY,
  OK,
  ERROR
};

struct FactorStatusArray
{
  std::set<std::string> keyset_;
  DiagnosticStatusArray dataset_;
  void add(const DiagnosticStatus& status);
  void reset();
};

class SystemStatusFilter
{
public:
  SystemStatusFilter();
  virtual int selectPriority(const SystemStatus& status);
  static const DiagnosticStatusArray& getFactorStatusArray();
  static void resetFactorStatusArray();
  const std::function<int(const SystemStatus&)>& getFunc() const;
  static VitalMonitor vital_monitor_;

protected:
  std::function<int(const SystemStatus&)> callback_;
  static FactorStatusArray factor_status_array_;
  static const int normal_behavior_;

  StatusType calcStatus(const DiagnosticStatus& status, int level_th);
  StatusType calcStatus(const DiagnosticStatusArray& st_array, int level_th);
  StatusType calcStatus(const NodeStatus& node_status, int level_th);
  StatusType calcStatus(const HardwareStatus& hw_status, int level_th);
  StatusType calcStatus(std::string node_name, const std::vector<NodeStatus>& array, int level_th);

  bool checkAllNodeSimplly(const std::vector<NodeStatus>& array, int level_th);
  bool checkAllHardwareSimplly(const std::vector<HardwareStatus>& array, int level_th);
  template<typename T> bool checkAllSimplly(const std::vector<T>& array, int level_th);
};


#endif
