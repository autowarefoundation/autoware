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
#include <emergency_handler/libvital_monitor.h>

const std::map<std::string, int>& VitalMonitor::getDeadNodes()
{
  return dead_nodes_;
}

void VitalMonitor::updateNodeStatus(const std::vector<std::string>& available_nodes)
{
  const ros::Time current = ros::Time::now();
  static ros::Time previous = current;
  const double diff = (current - previous).toSec();
  previous = current;

  const auto& cur_nodes = available_nodes;
  for (const auto& node : cur_nodes)
  {
    if (dead_nodes_.count(node) != 0)
    {
      ROS_INFO("%s is switched to be available", node.c_str());
      required_nodes_.at(node).reset();
      dead_nodes_.erase(node);
    }
  }
  for (auto& node : required_nodes_)
  {
    const std::string node_name = node.first;
    const auto& found = std::find(cur_nodes.begin(), cur_nodes.end(), node_name);
    if (found == cur_nodes.end())
    {
      node.second.spend(diff);
    }
  }
  for (const auto& node : required_nodes_)
  {
    const std::string node_name = node.first;
    if (dead_nodes_.count(node_name) == 0 && node.second.isDead())
    {
      ROS_INFO("%s is not available", node_name.c_str());
      dead_nodes_.emplace(node_name, node.second.level_);
    }
  }
}

void VitalMonitor::initMonitoredNodeList(ros::NodeHandle& pnh)
{
  XmlRpc::XmlRpcValue params;
  pnh.getParam("vital_monitor", params);
  for (const auto& param : params)
  {
    std::string node_name = "/" + param.first;
    auto val = param.second;
    const double timeout_sec = val.hasMember("timeout") ? double(val["timeout"]) : 0.1;
    const int level = val.hasMember("level") ? int(val["level"]) : 1;
    required_nodes_.emplace(node_name, LifeTime(timeout_sec, level));
  }
  dead_nodes_.clear();
}

autoware_system_msgs::DiagnosticStatusArray
  VitalMonitor::createDiagnosticStatusArray(std::string dead_node_name, std_msgs::Header& header, int level) const
{
  autoware_system_msgs::DiagnosticStatus ds;
  std::string name(dead_node_name);
  if (name[0] == '/')
  {
    name.erase(name.begin());
  }
  ds.header = header;
  ds.key = "node_" + name + "_dead";
  ds.description = "node " + name + " is dead";
  ds.type = autoware_system_msgs::DiagnosticStatus::PROCESS_HAS_DIED;
  ds.level = level;
  autoware_system_msgs::DiagnosticStatusArray array;
  array.status.emplace_back(ds);
  return array;
}

autoware_system_msgs::NodeStatus
  VitalMonitor::createNodeStatus(std::string dead_node_name, std_msgs::Header& header, int level) const
{
  autoware_system_msgs::NodeStatus ns;
  ns.header = header;
  ns.node_name = dead_node_name;
  ns.node_activated = true;
  ns.status.emplace_back(createDiagnosticStatusArray(dead_node_name, header, level));
  return ns;
}

autoware_system_msgs::SystemStatus
  VitalMonitor::addDeadNodes(const autoware_system_msgs::SystemStatus& status) const
{
  autoware_system_msgs::SystemStatus updated_status(status);
  for (auto& node : dead_nodes_)
  {
    const std::string name = node.first;
    const int level = node.second;
    auto& array = updated_status.node_status;
    auto found = std::find_if(array.begin(), array.end(),
      [&](autoware_system_msgs::NodeStatus& stat){return (name == stat.node_name);});
    if (found == array.end())
    {
      array.emplace_back(createNodeStatus(name, updated_status.header, level));
    }
    else
    {
      found->node_activated = true;
      found->status.emplace_back(createDiagnosticStatusArray(name, updated_status.header, level));
    }
  }
  return updated_status;
}
