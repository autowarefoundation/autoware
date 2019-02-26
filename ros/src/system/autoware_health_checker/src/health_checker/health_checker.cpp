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

#include <health_checker/health_checker.h>

namespace autoware_health_checker {
HealthChecker::HealthChecker(ros::NodeHandle nh, ros::NodeHandle pnh)
    : value_manager_(nh, pnh) {
  node_activated_ = false;
  ros_ok_ = true;
  nh_ = nh;
  pnh_ = pnh;
  value_manager_.run();
  status_pub_ =
      nh_.advertise<autoware_system_msgs::NodeStatus>("node_status", 10);
}

HealthChecker::~HealthChecker() { ros_ok_ = false; }

void HealthChecker::publishStatus() {
  ros::Rate rate = ros::Rate(autoware_health_checker::UPDATE_RATE);
  while (ros_ok_) {
    mtx_.lock();
    autoware_system_msgs::NodeStatus status;
    status.node_activated = node_activated_;
    ros::Time now = ros::Time::now();
    status.header.stamp = now;
    status.node_name = ros::this_node::getName();
    std::vector<std::string> checker_keys = getRateCheckerKeys();
    // iterate Rate checker and publish rate_check result
    for (auto key_itr = checker_keys.begin(); key_itr != checker_keys.end();
         key_itr++) {
      autoware_system_msgs::DiagnosticStatusArray diag_array;
      autoware_system_msgs::DiagnosticStatus diag;
      diag.type = autoware_system_msgs::DiagnosticStatus::LOOP_RATE_IS_SLOW;
      std::pair<uint8_t, double> result =
          rate_checkers_[*key_itr]->getErrorLevelAndRate();
      diag.level = result.first;
      diag.key = *key_itr;
      diag.value = valueToJson(result.second);
      diag.description = rate_checkers_[*key_itr]->description;
      diag.header.stamp = now;
      diag_array.status.push_back(diag);
      status.status.push_back(diag_array);
    }
    // iterate Diagnostic Buffer and publish all diagnostic data
    std::vector<std::string> keys = getKeys();
    for (auto key_itr = keys.begin(); key_itr != keys.end(); key_itr++) {
      status.status.push_back(diag_buffers_[*key_itr]->getAndClearData());
    }
    status_pub_.publish(status);
    mtx_.unlock();
    rate.sleep();
  }
  return;
}

void HealthChecker::SET_DIAG_STATUS(
    autoware_system_msgs::DiagnosticStatus status) {
  addNewBuffer(status.key, status.type, status.description);
  diag_buffers_[status.key]->addDiag(status);
  return;
}

void HealthChecker::ENABLE() {
  boost::thread publish_thread(
      boost::bind(&HealthChecker::publishStatus, this));
  return;
}

std::vector<std::string> HealthChecker::getKeys() {
  std::vector<std::string> keys;
  std::vector<std::string> checker_keys = getRateCheckerKeys();
  std::pair<std::string, std::shared_ptr<DiagBuffer>> buf_itr;
  BOOST_FOREACH (buf_itr, diag_buffers_) {
    bool matched = false;
    for (auto checker_key_itr = checker_keys.begin();
         checker_key_itr != checker_keys.end(); checker_key_itr++) {
      if (*checker_key_itr == buf_itr.first) {
        matched = true;
      }
    }
    if (!matched) {
      keys.push_back(buf_itr.first);
    }
  }
  return keys;
}

std::vector<std::string> HealthChecker::getRateCheckerKeys() {
  std::vector<std::string> keys;
  std::pair<std::string, std::shared_ptr<RateChecker>> checker_itr;
  BOOST_FOREACH (checker_itr, rate_checkers_) {
    keys.push_back(checker_itr.first);
  }
  return keys;
}

bool HealthChecker::keyExist(std::string key) {
  if (diag_buffers_.count(key) == 0) {
    return false;
  }
  return true;
}

// add New Diagnostic Buffer if the key does not exist
bool HealthChecker::addNewBuffer(std::string key, uint8_t type,
                                 std::string description) {
  mtx_.lock();
  if (!keyExist(key)) {
    std::shared_ptr<DiagBuffer> buf_ptr = std::make_shared<DiagBuffer>(
        key, type, description, autoware_health_checker::BUFFER_LENGTH);
    diag_buffers_[key] = buf_ptr;
    mtx_.unlock();
    return true;
  }
  mtx_.unlock();
  return false;
}

uint8_t HealthChecker::CHECK_MIN_VALUE(std::string key, double value,
                                       double warn_value, double error_value,
                                       double fatal_value,
                                       std::string description) {
  value_manager_.setDefaultValue(key, "min", warn_value, error_value,
                                 fatal_value);
  addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE,
               description);
  autoware_system_msgs::DiagnosticStatus new_status;
  new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
  if (value < value_manager_.getValue(key, "min",
                                      autoware_health_checker::LEVEL_FATAL)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
  } else if (value < value_manager_.getValue(
                         key, "min", autoware_health_checker::LEVEL_ERROR)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
  } else if (value < value_manager_.getValue(
                         key, "min", autoware_health_checker::LEVEL_WARN)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
  } else {
    new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
  }
  new_status.key = key;
  new_status.description = description;
  new_status.value = valueToJson(value);
  diag_buffers_[key]->addDiag(new_status);
  return new_status.level;
}

uint8_t HealthChecker::CHECK_MAX_VALUE(std::string key, double value,
                                       double warn_value, double error_value,
                                       double fatal_value,
                                       std::string description) {
  value_manager_.setDefaultValue(key, "max", warn_value, error_value,
                                 fatal_value);
  addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE,
               description);
  autoware_system_msgs::DiagnosticStatus new_status;
  new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
  if (value > value_manager_.getValue(key, "max",
                                      autoware_health_checker::LEVEL_FATAL)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
  } else if (value > value_manager_.getValue(
                         key, "max", autoware_health_checker::LEVEL_ERROR)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
  } else if (value > value_manager_.getValue(
                         key, "max", autoware_health_checker::LEVEL_WARN)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
  } else {
    new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
  }
  new_status.key = key;
  new_status.description = description;
  new_status.value = valueToJson(value);
  new_status.header.stamp = ros::Time::now();
  diag_buffers_[key]->addDiag(new_status);
  return new_status.level;
}

uint8_t HealthChecker::CHECK_RANGE(std::string key, double value,
                                   std::pair<double, double> warn_value,
                                   std::pair<double, double> error_value,
                                   std::pair<double, double> fatal_value,
                                   std::string description) {
  value_manager_.setDefaultValue(key, "min", warn_value.first,
                                 error_value.first, fatal_value.first);
  value_manager_.setDefaultValue(key, "max", warn_value.second,
                                 error_value.second, fatal_value.second);
  addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE,
               description);
  autoware_system_msgs::DiagnosticStatus new_status;
  new_status.type = autoware_system_msgs::DiagnosticStatus::OUT_OF_RANGE;
  if (value < value_manager_.getValue(key, "min",
                                      autoware_health_checker::LEVEL_FATAL) ||
      value > value_manager_.getValue(key, "max",
                                      autoware_health_checker::LEVEL_FATAL)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::FATAL;
  } else if (value < value_manager_.getValue(
                         key, "min", autoware_health_checker::LEVEL_ERROR) ||
             value > value_manager_.getValue(
                         key, "max", autoware_health_checker::LEVEL_ERROR)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::ERROR;
  } else if (value < value_manager_.getValue(
                         key, "min", autoware_health_checker::LEVEL_WARN) ||
             value > value_manager_.getValue(
                         key, "max", autoware_health_checker::LEVEL_WARN)) {
    new_status.level = autoware_system_msgs::DiagnosticStatus::WARN;
  } else {
    new_status.level = autoware_system_msgs::DiagnosticStatus::OK;
  }
  new_status.key = key;
  new_status.value = valueToJson(value);
  new_status.description = description;
  new_status.header.stamp = ros::Time::now();
  diag_buffers_[key]->addDiag(new_status);
  return new_status.level;
}

void HealthChecker::CHECK_RATE(std::string key, double warn_rate,
                               double error_rate, double fatal_rate,
                               std::string description) {
  if (!keyExist(key)) {
    value_manager_.setDefaultValue(key, "rate", warn_rate, error_rate,
                                   fatal_rate);
    std::shared_ptr<RateChecker> checker_ptr = std::make_shared<RateChecker>(
        autoware_health_checker::BUFFER_LENGTH, warn_rate, error_rate,
        fatal_rate, description);
    rate_checkers_[key] = checker_ptr;
  }
  addNewBuffer(key, autoware_system_msgs::DiagnosticStatus::LOOP_RATE_IS_SLOW,
               description);
  rate_checkers_[key]->setRate(
      value_manager_.getValue(key, "rate", autoware_health_checker::LEVEL_WARN),
      value_manager_.getValue(key, "rate",
                              autoware_health_checker::LEVEL_ERROR),
      value_manager_.getValue(key, "rate",
                              autoware_health_checker::LEVEL_FATAL));
  rate_checkers_[key]->check();
  return;
}
}