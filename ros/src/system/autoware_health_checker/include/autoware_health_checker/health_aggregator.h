#ifndef HEALTH_AGGREGATOR_H_INCLUDED
#define HEALTH_AGGREGATOR_H_INCLUDED

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
#include <diagnostic_msgs/DiagnosticArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>

// headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/NodeStatus.h>
#include <autoware_system_msgs/SystemStatus.h>
#include <rosgraph_msgs/TopicStatistics.h>

// headers in boost
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread.hpp>

// headers in STL
#include <map>
#include <mutex>

class HealthAggregator {
public:
  HealthAggregator(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~HealthAggregator();
  void run();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher system_status_pub_;
  std::map<uint8_t, ros::Publisher> text_pub_;
  ros::Subscriber node_status_sub_;
  ros::Subscriber diagnostic_array_sub_;
  ros::Subscriber topic_statistics_sub_;
  void publishSystemStatus();
  void nodeStatusCallback(const autoware_system_msgs::NodeStatus::ConstPtr msg);
  void
  diagnosticArrayCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr msg);
  void
  topicStatisticsCallback(const rosgraph_msgs::TopicStatistics::ConstPtr msg);
  std::string
  generateText(std::vector<autoware_system_msgs::DiagnosticStatus> status);
  jsk_rviz_plugins::OverlayText
  generateOverlayText(autoware_system_msgs::SystemStatus status, uint8_t level);
  std::vector<autoware_system_msgs::DiagnosticStatus>
  filterNodeStatus(autoware_system_msgs::SystemStatus status, uint8_t level);
  boost::optional<autoware_system_msgs::HardwareStatus>
  convert(const diagnostic_msgs::DiagnosticArray::ConstPtr msg);
  autoware_system_msgs::SystemStatus system_status_;
  std::mutex mtx_;
  void updateConnectionStatus();
  // key topic_name,publisher_node,subscriber_node
  std::map<std::array<std::string, 3>, rosgraph_msgs::TopicStatistics>
      topic_status_;
  void updateTopicStatus();
};
#endif // HEALTH_AGGREGATOR_H_INCLUDED