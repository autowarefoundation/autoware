/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Masaya Kataoka
 */

#include <autoware_health_checker/health_analyzer.h>

HealthAnalyzer::HealthAnalyzer(ros::NodeHandle nh, ros::NodeHandle pnh) {
  nh_ = nh;
  pnh_ = pnh;
  pnh_.param<int>("warn_nodes_count_threshold", warn_nodes_count_threshold_, 30);
  system_status_summary_pub_ =
      nh_.advertise<autoware_system_msgs::SystemStatus>(
          "/system_status/summary", 1);
  system_status_sub_ = nh_.subscribe(
      "/system_status", 1, &HealthAnalyzer::systemStatusCallback, this);
}

HealthAnalyzer::~HealthAnalyzer() { writeDot(); }

int HealthAnalyzer::countWarn(autoware_system_msgs::SystemStatus msg) {
  int count = 0;
  for (auto node_status_itr = msg.node_status.begin();
       node_status_itr != msg.node_status.end(); node_status_itr++) {
    for (auto status_itr = node_status_itr->status.begin();
         status_itr != node_status_itr->status.end(); status_itr++) {
      for (auto itr = status_itr->status.begin();
           itr != status_itr->status.end(); itr++) {
        if (itr->level == autoware_health_checker::LEVEL_WARN) {
          count++;
        }
      }
    }
  }
  for (auto hardware_status_itr = msg.hardware_status.begin();
       hardware_status_itr != msg.hardware_status.end();
       hardware_status_itr++) {
    for (auto status_itr = hardware_status_itr->status.begin();
         status_itr != hardware_status_itr->status.end(); status_itr++) {
      for (auto itr = status_itr->status.begin();
           itr != status_itr->status.end(); itr++) {
        if (itr->level == autoware_health_checker::LEVEL_WARN) {
          count++;
        }
      }
    }
  }
  return count;
}

std::vector<std::string>
HealthAnalyzer::findWarningNodes(autoware_system_msgs::SystemStatus status) {
  std::vector<std::string> ret;
  for (auto node_status_itr = status.node_status.begin();
       node_status_itr != status.node_status.end(); node_status_itr++) {
    for (auto status_itr = node_status_itr->status.begin();
         status_itr != node_status_itr->status.end(); status_itr++) {
      for (auto itr = status_itr->status.begin();
           itr != status_itr->status.end(); itr++) {
        if ((itr->level == autoware_health_checker::LEVEL_WARN) &&
            (isAlreadyExist(ret, node_status_itr->node_name) == false)) {
          ret.push_back(node_status_itr->node_name);
        } else if ((itr->level == autoware_health_checker::LEVEL_ERROR) &&
                   (isAlreadyExist(ret, node_status_itr->node_name) == false)) {
          ret.push_back(node_status_itr->node_name);
        } else if ((itr->level == autoware_health_checker::LEVEL_FATAL) &&
                   (isAlreadyExist(ret, node_status_itr->node_name) == false)) {
          ret.push_back(node_status_itr->node_name);
        }
      }
    }
  }
  return ret;
}

std::vector<std::string>
HealthAnalyzer::findErrorNodes(autoware_system_msgs::SystemStatus status) {
  std::vector<std::string> ret;
  for (auto node_status_itr = status.node_status.begin();
       node_status_itr != status.node_status.end(); node_status_itr++) {
    for (auto status_itr = node_status_itr->status.begin();
         status_itr != node_status_itr->status.end(); status_itr++) {
      for (auto itr = status_itr->status.begin();
           itr != status_itr->status.end(); itr++) {
        if ((itr->level == autoware_health_checker::LEVEL_ERROR) &&
            (isAlreadyExist(ret, node_status_itr->node_name) == false)) {
          if (isAlreadyExist(status.available_nodes,
                             node_status_itr->node_name)) {
            ret.push_back(node_status_itr->node_name);
          }
        } else if ((itr->level == autoware_health_checker::LEVEL_FATAL) &&
                   (isAlreadyExist(ret, node_status_itr->node_name) == false)) {
          if (isAlreadyExist(status.available_nodes,
                             node_status_itr->node_name)) {
            ret.push_back(node_status_itr->node_name);
          }
        }
      }
    }
  }
  return ret;
}

std::vector<std::string>
HealthAnalyzer::findRootNodes(std::vector<std::string> target_nodes) {
  std::vector<std::string> ret;
  for (auto itr = target_nodes.begin(); itr != target_nodes.end(); itr++) {
    std::vector<std::string> pub_nodes;
    adjacency_iterator_t vi;
    adjacency_iterator_t vi_end;
    vertex_t target_node = getTargetNode(*itr);
    bool depend_found = false;
    for (boost::tie(vi, vi_end) = adjacent_vertices(target_node, depend_graph_);
         vi != vi_end; ++vi) {
      if (isAlreadyExist(target_nodes, depend_graph_[*vi].node_name)) {
        depend_found = true;
      }
    }
    if (!depend_found) {
      ret.push_back(*itr);
      // ROS_ERROR_STREAM("Autoware Health Analyzer detects something wrong in "
      // << *itr << " node.");
    }
  }
  return ret;
}

autoware_system_msgs::SystemStatus
HealthAnalyzer::filterSystemStatus(autoware_system_msgs::SystemStatus status) {
  int warn_count = countWarn(status);
  autoware_system_msgs::SystemStatus filtered_status;
  std::vector<std::string> target_nodes;
  if (warn_count >= warn_nodes_count_threshold_) {
    filtered_status.detect_too_match_warning = true;
    target_nodes = findWarningNodes(status);
  } else {
    filtered_status.detect_too_match_warning = false;
    target_nodes = findErrorNodes(status);
  }
  std::vector<std::string> root_nodes = findRootNodes(target_nodes);
  for (auto node_status_itr = status.node_status.begin();
       node_status_itr != status.node_status.end(); node_status_itr++) {
    if (isAlreadyExist(root_nodes, node_status_itr->node_name)) {
      filtered_status.node_status.push_back(*node_status_itr);
    }
  }
  filtered_status.header = status.header;
  filtered_status.available_nodes = status.available_nodes;
  filtered_status.hardware_status = status.hardware_status;
  filtered_status.topic_statistics = status.topic_statistics;
  return filtered_status;
}

void HealthAnalyzer::systemStatusCallback(
    const autoware_system_msgs::SystemStatus::ConstPtr msg) {
  generateDependGraph(*msg);
  system_status_summary_pub_.publish(filterSystemStatus(*msg));
  return;
}

void HealthAnalyzer::generateDependGraph(
    autoware_system_msgs::SystemStatus status) {
  depend_graph_ = graph_t();
  for (auto itr = status.topic_statistics.begin();
       itr != status.topic_statistics.end(); itr++) {
    addDepend(*itr);
  }
  return;
}

void HealthAnalyzer::writeDot() {
  std::string path = ros::package::getPath("autoware_health_checker") +
                     "/data/node_depends.dot";
  std::ofstream f(path.c_str());
  boost::write_graphviz(
      f, depend_graph_,
      boost::make_label_writer(get(&node_property::node_name, depend_graph_)));
  return;
}

bool HealthAnalyzer::nodeAlreadyExistsInGraph(std::string target_node) {
  auto vertex_range = boost::vertices(depend_graph_);
  for (auto first = vertex_range.first, last = vertex_range.second;
       first != last; ++first) {
    vertex_t v = *first;
    if (depend_graph_[v].node_name == target_node) {
      return true;
    }
  }
  return false;
}

vertex_t HealthAnalyzer::getTargetNode(std::string target_node) {
  vertex_t ret;
  auto vertex_range = boost::vertices(depend_graph_);
  for (auto first = vertex_range.first, last = vertex_range.second;
       first != last; ++first) {
    vertex_t v = *first;
    if (depend_graph_[v].node_name == target_node) {
      ret = v;
      return ret;
    }
  }
  return ret;
}

void HealthAnalyzer::addDepend(rosgraph_msgs::TopicStatistics statistics) {
  if (statistics.node_pub == statistics.node_sub) {
    return;
  }
  vertex_t node_sub;
  vertex_t node_pub;
  edge_t topic;
  if (!nodeAlreadyExistsInGraph(statistics.node_pub)) {
    vertex_t v = boost::add_vertex(depend_graph_);
    depend_graph_[v].node_name = statistics.node_pub;
    node_pub = v;
  } else {
    vertex_t v = getTargetNode(statistics.node_pub);
    node_pub = v;
  }
  if (!nodeAlreadyExistsInGraph(statistics.node_sub)) {
    vertex_t v = boost::add_vertex(depend_graph_);
    depend_graph_[v].node_name = statistics.node_sub;
    node_sub = v;
  } else {
    vertex_t v = getTargetNode(statistics.node_sub);
    node_sub = v;
  }
  bool inserted = false;
  boost::tie(topic, inserted) =
      boost::add_edge(node_sub, node_pub, depend_graph_);
  depend_graph_[topic].node_sub = statistics.node_sub;
  depend_graph_[topic].node_pub = statistics.node_pub;
  return;
}