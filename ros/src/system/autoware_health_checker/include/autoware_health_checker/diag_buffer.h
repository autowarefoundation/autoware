#ifndef DIAG_BUFFER_H_INCLUDED
#define DIAG_BUFFER_H_INCLUDED

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

// headers in Autoare
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/DiagnosticStatusArray.h>

// headers in STL
#include <map>
#include <mutex>
#include <string>
#include <vector>

// headers in ROS
#include <ros/ros.h>

namespace autoware_health_checker {
class DiagBuffer {
public:
  DiagBuffer(std::string key, uint8_t type, std::string description,
             double buffer_length);
  ~DiagBuffer();
  void addDiag(autoware_system_msgs::DiagnosticStatus status);
  autoware_system_msgs::DiagnosticStatusArray getAndClearData();
  const uint8_t type;
  const std::string description;

private:
  std::mutex mtx_;
  uint8_t getErrorLevel();
  void updateBuffer();
  std::string key_;
  ros::Duration buffer_length_;
  std::map<uint8_t, autoware_system_msgs::DiagnosticStatusArray> buffer_;
  autoware_system_msgs::DiagnosticStatusArray filterBuffer(ros::Time now,
                                                           uint8_t level);
  ros::Publisher status_pub_;
  bool isOlderTimestamp(const autoware_system_msgs::DiagnosticStatus &a,
                        const autoware_system_msgs::DiagnosticStatus &b);
};
}

#endif // DIAG_BUFFER_H_INCLUDED