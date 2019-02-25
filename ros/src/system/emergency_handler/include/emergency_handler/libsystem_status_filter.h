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

class SystemStatusFilter
{
public:
  SystemStatusFilter();
  virtual int selectPriority(const SystemStatus& status);
  const std::function<int(const SystemStatus&)>& getFunc() const;
  static void initMonitoredNodeList(ros::NodeHandle& pnh);
  static void updateNodeStatus(const SystemStatus& status);
  static const std::map<std::string, std::string>& getDeadNodes();

protected:
  std::function<int(const SystemStatus&)> callback_;
  static std::map<std::string, std::string> monitored_nodes_, dead_nodes_;
  static const int normal_behavior_;

  StatusType getStatus(const DiagnosticStatusArray& st_array, int level_th) const;
  StatusType getStatus(const NodeStatus& node_status, int level_th) const;
  StatusType getStatus(const HardwareStatus& hw_status, int level_th) const;
  StatusType getStatus(std::string node_name, const std::vector<NodeStatus>& array, int level_th) const;

  bool checkAllNodeSimplly(const std::vector<NodeStatus>& array, int level_th) const;
  bool checkAllHardwareSimplly(const std::vector<HardwareStatus>& array, int level_th) const;
  template<typename T> bool checkAllSimplly(const std::vector<T>& array, int level_th) const;
};


#endif
