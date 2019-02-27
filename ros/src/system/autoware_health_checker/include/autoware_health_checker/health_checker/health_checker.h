#ifndef HEALTH_CHECKER_H_INCLUDED
#define HEALTH_CHECKER_H_INCLUDED

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
#include <ros/ros.h>

// headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/NodeStatus.h>
#include <autoware_health_checker/health_checker/diag_buffer.h>
#include <autoware_health_checker/health_checker/rate_checker.h>
#include <autoware_health_checker/health_checker/value_manager.h>

// headers in STL
#include <functional>
#include <map>
#include <memory>
#include <sstream>

// headers in boost
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread.hpp>

namespace autoware_health_checker {
class HealthChecker {
public:
  HealthChecker(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~HealthChecker();
  void ENABLE();
  uint8_t CHECK_MIN_VALUE(std::string key, double value, double warn_value,
                          double error_value, double fatal_value,
                          std::string description);
  uint8_t CHECK_MAX_VALUE(std::string key, double value, double warn_value,
                          double error_value, double fatal_value,
                          std::string description);
  // std::pair<double,double> first value is min value and second value is max
  // value
  uint8_t CHECK_RANGE(std::string key, double value,
                      std::pair<double, double> warn_value,
                      std::pair<double, double> error_value,
                      std::pair<double, double> fatal_value,
                      std::string description);
  template <class T>
  uint8_t CHECK_VALUE(
      std::string key, T value, std::function<uint8_t(T value)> check_func,
      std::function<boost::property_tree::ptree(T value)> value_json_func,
      std::string description) {
    addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::INVALID_VALUE,
                 description);
    uint8_t check_result = check_func(value);
    boost::property_tree::ptree pt = value_json_func(value);
    std::stringstream ss;
    write_json(ss, pt);
    autoware_system_msgs::DiagnosticStatus new_status;
    new_status.type = autoware_system_msgs::DiagnosticStatus::INVALID_VALUE;
    new_status.level = check_result;
    new_status.description = description;
    new_status.value = ss.str();
    new_status.header.stamp = ros::Time::now();
    diag_buffers_[key]->addDiag(new_status);
    return new_status.level;
  }
  void CHECK_RATE(std::string key, double warn_rate, double error_rate,
                  double fatal_rate, std::string description);
  uint8_t CHECK_TRUE(std::string key, bool value, uint8_t level,
                     std::string description);
  uint8_t SET_DIAG_STATUS(autoware_system_msgs::DiagnosticStatus status);
  void NODE_ACTIVATE() {
    std::lock_guard<std::mutex> lock(mtx_);
    node_activated_ = true;
  };
  void NODE_DEACTIVATE() {
    std::lock_guard<std::mutex> lock(mtx_);
    node_activated_ = false;
  };
  bool getNodeStatus() { return node_activated_; };

private:
  ValueManager value_manager_;
  std::vector<std::string> getKeys();
  std::vector<std::string> getRateCheckerKeys();
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::map<std::string, std::unique_ptr<DiagBuffer>> diag_buffers_;
  std::map<std::string, std::unique_ptr<RateChecker>> rate_checkers_;
  ros::Publisher status_pub_;
  bool keyExist(std::string key);
  bool addNewBuffer(std::string key, uint8_t type, std::string description);
  template <typename T> std::string valueToJson(T value) {
    using namespace boost::property_tree;
    std::stringstream ss;
    ptree pt;
    pt.put("value", value);
    write_json(ss, pt);
    return ss.str();
  }
  void publishStatus();
  bool node_activated_;
  bool ros_ok_;
  std::mutex mtx_;
};
}
#endif // HEALTH_CHECKER_H_INCLUDED