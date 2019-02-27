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

const std::map<std::string, bool>& VitalMonitor::getDeadNodes()
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
      dead_nodes_.emplace(node_name, node.second.respawn_);
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
    const bool can_respawn = val.hasMember("respawn") ? bool(val["respawn"]) : false;
    required_nodes_.emplace(node_name, LifeTime(timeout_sec, can_respawn));
  }
  dead_nodes_.clear();
}
