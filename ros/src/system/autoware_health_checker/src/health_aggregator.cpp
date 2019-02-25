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

#include <autoware_health_checker/health_aggregator.h>

HealthAggregator::HealthAggregator(ros::NodeHandle nh, ros::NodeHandle pnh) {
  nh_ = nh;
  pnh_ = pnh;
}

HealthAggregator::~HealthAggregator() {}

void HealthAggregator::run() {
  system_status_pub_ =
      nh_.advertise<autoware_system_msgs::SystemStatus>("/system_status", 10);
  text_pub_[autoware_health_checker::LEVEL_OK] =
      pnh_.advertise<jsk_rviz_plugins::OverlayText>("ok_text", 1);
  text_pub_[autoware_health_checker::LEVEL_WARN] =
      pnh_.advertise<jsk_rviz_plugins::OverlayText>("warn_text", 1);
  text_pub_[autoware_health_checker::LEVEL_ERROR] =
      pnh_.advertise<jsk_rviz_plugins::OverlayText>("error_text", 1);
  text_pub_[autoware_health_checker::LEVEL_FATAL] =
      pnh_.advertise<jsk_rviz_plugins::OverlayText>("fatal_text", 1);
  node_status_sub_ = nh_.subscribe("/node_status", 10,
                                   &HealthAggregator::nodeStatusCallback, this);
  diagnostic_array_sub_ = nh_.subscribe(
      "/diagnostic_agg", 10, &HealthAggregator::diagnosticArrayCallback, this);
  topic_statistics_sub_ = nh_.subscribe(
      "/statistics", 1, &HealthAggregator::topicStatisticsCallback, this);
  boost::thread publish_thread(
      boost::bind(&HealthAggregator::publishSystemStatus, this));
  return;
}

void HealthAggregator::topicStatisticsCallback(
    const rosgraph_msgs::TopicStatistics::ConstPtr msg) {
  std::array<std::string, 3> key = {msg->topic, msg->node_pub, msg->node_sub};
  topic_status_[key] = *msg;
  return;
}

void HealthAggregator::updateTopicStatus() {
  ros::Time now = ros::Time::now();
  for (std::pair<std::array<std::string, 3>, rosgraph_msgs::TopicStatistics>
           pair : topic_status_) {
    if ((now - ros::Duration(autoware_health_checker::BUFFER_LENGTH)) <
        pair.second.window_stop) {
      system_status_.topic_statistics.push_back(pair.second);
    }
  }
  return;
}

void HealthAggregator::publishSystemStatus() {
  ros::Rate rate = ros::Rate(autoware_health_checker::UPDATE_RATE);
  while (ros::ok()) {
    mtx_.lock();
    system_status_.header.stamp = ros::Time::now();
    updateConnectionStatus();
    updateTopicStatus();
    system_status_pub_.publish(system_status_);
    text_pub_[autoware_health_checker::LEVEL_OK].publish(
        generateOverlayText(system_status_, autoware_health_checker::LEVEL_OK));
    text_pub_[autoware_health_checker::LEVEL_WARN].publish(generateOverlayText(
        system_status_, autoware_health_checker::LEVEL_WARN));
    text_pub_[autoware_health_checker::LEVEL_ERROR].publish(generateOverlayText(
        system_status_, autoware_health_checker::LEVEL_ERROR));
    text_pub_[autoware_health_checker::LEVEL_FATAL].publish(generateOverlayText(
        system_status_, autoware_health_checker::LEVEL_FATAL));
    system_status_.topic_statistics.clear();
    system_status_.node_status.clear();
    system_status_.hardware_status.clear();
    mtx_.unlock();
    rate.sleep();
  }
  return;
}

void HealthAggregator::updateConnectionStatus() {
  std::vector<std::string> detected_nodes;
  ros::master::getNodes(detected_nodes);
  system_status_.available_nodes = detected_nodes;
  return;
}

void HealthAggregator::nodeStatusCallback(
    const autoware_system_msgs::NodeStatus::ConstPtr msg) {
  mtx_.lock();
  system_status_.node_status.push_back(*msg);
  mtx_.unlock();
  return;
}

void HealthAggregator::diagnosticArrayCallback(
    const diagnostic_msgs::DiagnosticArray::ConstPtr msg) {
  mtx_.lock();
  boost::optional<autoware_system_msgs::HardwareStatus> status = convert(msg);
  if (status) {
    system_status_.hardware_status.push_back(*status);
  }
  mtx_.unlock();
  return;
}

std::string HealthAggregator::generateText(
    std::vector<autoware_system_msgs::DiagnosticStatus> status) {
  std::string text;
  for (auto itr = status.begin(); itr != status.end(); itr++) {
    text = text + itr->description + "\n";
    // text = itr->key + " : " + itr->description + "\n";
  }
  return text;
}

jsk_rviz_plugins::OverlayText
HealthAggregator::generateOverlayText(autoware_system_msgs::SystemStatus status,
                                      uint8_t level) {
  jsk_rviz_plugins::OverlayText text;
  text.action = text.ADD;
  text.width = 640;
  text.height = 640;
  text.top = 0;
  text.bg_color.r = 0;
  text.bg_color.g = 0;
  text.bg_color.b = 0;
  text.bg_color.a = 0.7;
  text.text_size = 20.0;
  if (level == autoware_health_checker::LEVEL_OK) {
    text.left = 0;
    text.fg_color.r = 0.0;
    text.fg_color.g = 0.0;
    text.fg_color.b = 1.0;
    text.fg_color.a = 1.0;
    text.text = generateText(filterNodeStatus(status, level));
  } else if (level == autoware_health_checker::LEVEL_WARN) {
    text.left = 640 * 1;
    text.fg_color.r = 1.0;
    text.fg_color.g = 1.0;
    text.fg_color.b = 0.0;
    text.fg_color.a = 1.0;
    text.text = generateText(filterNodeStatus(status, level));
  } else if (level == autoware_health_checker::LEVEL_ERROR) {
    text.left = 640 * 2;
    text.fg_color.r = 1.0;
    text.fg_color.g = 0.0;
    text.fg_color.b = 0.0;
    text.fg_color.a = 1.0;
    text.text = generateText(filterNodeStatus(status, level));
  } else if (level == autoware_health_checker::LEVEL_FATAL) {
    text.left = 640 * 3;
    text.fg_color.r = 1.0;
    text.fg_color.g = 1.0;
    text.fg_color.b = 1.0;
    text.fg_color.a = 1.0;
    text.text = generateText(filterNodeStatus(status, level));
  }
  return text;
}

std::vector<autoware_system_msgs::DiagnosticStatus>
HealthAggregator::filterNodeStatus(autoware_system_msgs::SystemStatus status,
                                   uint8_t level) {
  std::vector<autoware_system_msgs::DiagnosticStatus> ret;
  for (auto node_status_itr = status.node_status.begin();
       node_status_itr != status.node_status.end(); node_status_itr++) {
    if (node_status_itr->node_activated) {
      for (auto array_itr = node_status_itr->status.begin();
           array_itr != node_status_itr->status.end(); array_itr++) {
        for (auto itr = array_itr->status.begin();
             itr != array_itr->status.end(); itr++) {
          if (itr->level == level) {
            ret.push_back(*itr);
          }
        }
      }
    }
  }
  return ret;
}

boost::optional<autoware_system_msgs::HardwareStatus> HealthAggregator::convert(
    const diagnostic_msgs::DiagnosticArray::ConstPtr msg) {
  autoware_system_msgs::HardwareStatus status;
  if (msg->status.size() == 0) {
    return boost::none;
  }
  status.header = msg->header;
  for (auto diag_itr = msg->status.begin(); diag_itr != msg->status.end();
       diag_itr++) {
    status.hardware_name = diag_itr->hardware_id;
    autoware_system_msgs::DiagnosticStatus diag;
    autoware_system_msgs::DiagnosticStatusArray diag_array;
    diag.header = msg->header;
    diag.key = diag_itr->hardware_id;
    diag.description = diag_itr->message;
    diag.type = autoware_system_msgs::DiagnosticStatus::HARDWARE;
    if (diag_itr->level == diagnostic_msgs::DiagnosticStatus::OK) {
      diag.level = autoware_health_checker::LEVEL_OK;
    } else if (diag_itr->level == diagnostic_msgs::DiagnosticStatus::WARN) {
      diag.level = autoware_health_checker::LEVEL_WARN;
    } else if (diag_itr->level == diagnostic_msgs::DiagnosticStatus::ERROR) {
      diag.level = autoware_health_checker::LEVEL_ERROR;
    } else if (diag_itr->level == diagnostic_msgs::DiagnosticStatus::STALE) {
      diag.level = autoware_health_checker::LEVEL_FATAL;
    }
    using namespace boost::property_tree;
    std::stringstream ss;
    ptree pt;
    for (auto value_itr = diag_itr->values.begin();
         value_itr != diag_itr->values.end(); value_itr++) {
      pt.put(value_itr->key + ".string", value_itr->value);
    }
    write_json(ss, pt);
    diag.value = ss.str();
    diag_array.status.push_back(diag);
    status.status.push_back(diag_array);
  }
  return status;
}