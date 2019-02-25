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
#include <emergency_handler/libsystem_status_filter.h>

SystemStatusFilter::SystemStatusFilter() :
  callback_(std::bind(&SystemStatusFilter::selectPriority, this, std::placeholders::_1)){}

int SystemStatusFilter::selectPriority(const SystemStatus& status)
{
  return normal_behavior_;
}

const std::function<int(const SystemStatus&)>& SystemStatusFilter::getFunc() const
{
  return callback_;
}

const std::map<std::string, std::string>& SystemStatusFilter::getDeadNodes()
{
  return dead_nodes_;
}

void SystemStatusFilter::updateNodeStatus(const SystemStatus& status)
{
  for (const auto& node : status.available_nodes)
  {
    if (dead_nodes_.count(node) != 0)
    {
      ROS_INFO("%s is switched to be available", node.c_str());
      dead_nodes_.erase(node);
    }
  }
  const auto& current_node = status.available_nodes;
  for (const auto& node : monitored_nodes_)
  {
    const std::string node_name = "/" + node.first;
    const auto& found = std::find(current_node.begin(), current_node.end(), node_name);
    if (found == current_node.end() && dead_nodes_.count(node_name) == 0)
    {
      ROS_INFO("%s is not available", node_name.c_str());
      dead_nodes_.emplace(node_name, node.second);
    }
  }
}

void SystemStatusFilter::initMonitoredNodeList(ros::NodeHandle& pnh)
{
  pnh.getParam("vital_monitor", monitored_nodes_);
  dead_nodes_.clear();
}

std::map<std::string, std::string> SystemStatusFilter::monitored_nodes_;
std::map<std::string, std::string> SystemStatusFilter::dead_nodes_;
const int SystemStatusFilter::normal_behavior_ = INT_MAX;

StatusType SystemStatusFilter::getStatus(const DiagnosticStatusArray& st_array, int level_th) const
{
  const auto found = find_if(st_array.status.begin(), st_array.status.end(),
    [=](const DiagnosticStatus& s){return s.level >= level_th;});
  if (found != st_array.status.end())
  {
    return StatusType::ERROR;
  }
  return StatusType::OK;
}

StatusType SystemStatusFilter::getStatus(const NodeStatus& node_status, int level_th) const
{
  if (!node_status.node_activated)
  {
    return StatusType::NOT_READY;
  }
  for(const auto& st_array : node_status.status)
  {
    if (getStatus(st_array, level_th) == StatusType::ERROR)
    {
      return StatusType::ERROR;
    }
  }
  return StatusType::OK;
}

StatusType SystemStatusFilter::getStatus(const HardwareStatus& hw_status, int level_th) const
{
  for(const auto& st_array : hw_status.status)
  {
    if (getStatus(st_array, level_th) == StatusType::ERROR)
    {
      return StatusType::ERROR;
    }
  }
  return StatusType::OK;
}

StatusType SystemStatusFilter::getStatus(std::string node_name, const std::vector<NodeStatus>& array, int level_th) const
{
  const auto found = find_if(array.begin(), array.end(),
    [=](const NodeStatus& s){return s.node_name == node_name;});
  if (found != array.end())
  {
    return getStatus(*found, level_th);
  }
  return StatusType::NONE;
}

bool SystemStatusFilter::checkAllNodeSimplly(const std::vector<NodeStatus>& array, int level_th) const
{
  return checkAllSimplly<NodeStatus>(array, level_th);
}

bool SystemStatusFilter::checkAllHardwareSimplly(const std::vector<HardwareStatus>& array, int level_th) const
{
  return checkAllSimplly<HardwareStatus>(array, level_th);
}

template<typename T> bool SystemStatusFilter::checkAllSimplly(const std::vector<T>& array, int level_th) const
{
  for (const auto& st : array)
  {
    const StatusType status = getStatus(st, level_th);
    if (status == StatusType::NOT_READY)
    {
      continue;
    }
    else if (status == StatusType::ERROR)
    {
      return false;
    }
  }
  return true;
}
