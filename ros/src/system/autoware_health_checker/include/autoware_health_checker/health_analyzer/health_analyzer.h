#ifndef HEALTH_ANALYZER_H_INCLUDED
#define HEALTH_ANALYZER_H_INCLUDED

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

// headers in ROS
#include <ros/package.h>
#include <ros/ros.h>

// headers in STL
#include <map>

// headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/SystemStatus.h>

// headers in boost
#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

struct topic_property {
  std::string node_pub;
  std::string node_sub;
};

struct node_property {
  std::string node_name;
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS,
                              node_property, topic_property>
    graph_t;
typedef graph_t::vertex_descriptor vertex_t;
typedef graph_t::edge_descriptor edge_t;
typedef boost::graph_traits<graph_t>::adjacency_iterator adjacency_iterator_t;
typedef boost::graph_traits<graph_t>::out_edge_iterator out_edge_iterator_t;

class HealthAnalyzer {
public:
  HealthAnalyzer(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~HealthAnalyzer();

private:
  ros::Subscriber system_status_sub_;
  ros::Subscriber topic_statistics_sub_;
  ros::Publisher system_status_summary_pub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  void
  systemStatusCallback(const autoware_system_msgs::SystemStatus::ConstPtr msg);
  void generateDependGraph(autoware_system_msgs::SystemStatus status);
  void addDepend(rosgraph_msgs::TopicStatistics statistics);
  autoware_system_msgs::SystemStatus
  filterSystemStatus(autoware_system_msgs::SystemStatus status);
  std::vector<std::string>
  findWarningNodes(autoware_system_msgs::SystemStatus status);
  std::vector<std::string>
  findErrorNodes(autoware_system_msgs::SystemStatus status);
  std::vector<std::string> findRootNodes(std::vector<std::string> target_nodes);
  bool nodeAlreadyExistsInGraph(std::string target_node);
  vertex_t getTargetNode(std::string target_node);
  int countWarn(autoware_system_msgs::SystemStatus msg);
  void writeDot();
  graph_t depend_graph_;
  int warn_nodes_count_threshold_;
  template <typename T> bool isAlreadyExist(std::vector<T> vector, T target) {
    for (auto itr = vector.begin(); itr != vector.end(); itr++) {
      if (target == *itr) {
        return true;
      }
    }
    return false;
  };
};

#endif // HEALTH_ANALYZER_H_INCLUDED