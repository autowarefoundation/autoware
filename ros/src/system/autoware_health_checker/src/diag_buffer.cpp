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

#include <autoware_health_checker/diag_buffer.h>

namespace autoware_health_checker {
DiagBuffer::DiagBuffer(std::string key, uint8_t type, std::string description,
                       double buffer_length)
    : type(type), description(description) {
  key_ = key;
  buffer_length_ = ros::Duration(buffer_length);
}

DiagBuffer::~DiagBuffer() {}

void DiagBuffer::addDiag(autoware_system_msgs::DiagnosticStatus status) {
  std::lock_guard<std::mutex> lock(mtx_);
  buffer_[status.level].status.emplace_back(status);
  updateBuffer();
  return;
}

autoware_system_msgs::DiagnosticStatusArray DiagBuffer::getAndClearData() {
  std::lock_guard<std::mutex> lock(mtx_);
  autoware_system_msgs::DiagnosticStatusArray data;
  data = buffer_[autoware_health_checker::LEVEL_FATAL];
  data.status.insert(
      data.status.end(),
      buffer_[autoware_health_checker::LEVEL_ERROR].status.begin(),
      buffer_[autoware_health_checker::LEVEL_ERROR].status.end());
  data.status.insert(
      data.status.end(),
      buffer_[autoware_health_checker::LEVEL_WARN].status.begin(),
      buffer_[autoware_health_checker::LEVEL_WARN].status.end());
  data.status.insert(data.status.end(),
                     buffer_[autoware_health_checker::LEVEL_OK].status.begin(),
                     buffer_[autoware_health_checker::LEVEL_OK].status.end());
  data.status.insert(
      data.status.end(),
      buffer_[autoware_health_checker::LEVEL_UNDEFINED].status.begin(),
      buffer_[autoware_health_checker::LEVEL_UNDEFINED].status.end());
  std::sort(data.status.begin(), data.status.end(),
            std::bind(&DiagBuffer::isOlderTimestamp, this,
                      std::placeholders::_1, std::placeholders::_2));
  buffer_.clear();
  return data;
}

uint8_t DiagBuffer::getErrorLevel() {
  std::lock_guard<std::mutex> lock(mtx_);
  updateBuffer();
  if (buffer_[autoware_health_checker::LEVEL_FATAL].status.size() != 0) {
    return autoware_health_checker::LEVEL_FATAL;
  } else if (buffer_[autoware_health_checker::LEVEL_ERROR].status.size() != 0) {
    return autoware_health_checker::LEVEL_ERROR;
  } else if (buffer_[autoware_health_checker::LEVEL_WARN].status.size() != 0) {
    return autoware_health_checker::LEVEL_WARN;
  } else {
    return autoware_health_checker::LEVEL_OK;
  }
}

// filter data from timestamp and level
autoware_system_msgs::DiagnosticStatusArray
DiagBuffer::filterBuffer(ros::Time now, uint8_t level) {
  autoware_system_msgs::DiagnosticStatusArray filterd_data;
  autoware_system_msgs::DiagnosticStatusArray ret;
  if (buffer_.count(level) != 0) {
    filterd_data = buffer_[level];
  }
  for (auto data_itr = filterd_data.status.begin();
       data_itr != filterd_data.status.end(); data_itr++) {
    if (data_itr->header.stamp > (now - buffer_length_)) {
      ret.status.push_back(*data_itr);
    }
  }
  return ret;
}

void DiagBuffer::updateBuffer() {
  ros::Time now = ros::Time::now();
  buffer_[autoware_health_checker::LEVEL_FATAL] =
      filterBuffer(now, autoware_health_checker::LEVEL_FATAL);
  buffer_[autoware_health_checker::LEVEL_ERROR] =
      filterBuffer(now, autoware_health_checker::LEVEL_ERROR);
  buffer_[autoware_health_checker::LEVEL_WARN] =
      filterBuffer(now, autoware_health_checker::LEVEL_WARN);
  buffer_[autoware_health_checker::LEVEL_OK] =
      filterBuffer(now, autoware_health_checker::LEVEL_OK);
  buffer_[autoware_health_checker::LEVEL_UNDEFINED] =
      filterBuffer(now, autoware_health_checker::LEVEL_UNDEFINED);
  return;
}

bool DiagBuffer::isOlderTimestamp(
    const autoware_system_msgs::DiagnosticStatus &a,
    const autoware_system_msgs::DiagnosticStatus &b) {
  return a.header.stamp < b.header.stamp;
}
}