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

#include <autoware_health_checker/system_status_subscriber.h>

namespace autoware_health_checker {
SystemStatusSubscriber::SystemStatusSubscriber(ros::NodeHandle nh,
                                               ros::NodeHandle pnh) {
  nh_ = nh;
  pnh_ = pnh;
}

SystemStatusSubscriber::~SystemStatusSubscriber() {}

void SystemStatusSubscriber::enable() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(1);
  status_sub_ = nh_.subscribe(
      "system_status", 10, &SystemStatusSubscriber::systemStatusCallback, this);
  while (ros::ok()) {
    rate.sleep();
  }
  spinner.stop();
  return;
}

void SystemStatusSubscriber::systemStatusCallback(
    const autoware_system_msgs::SystemStatus::ConstPtr msg) {
  for (auto function_itr = functions_.begin(); function_itr != functions_.end();
       function_itr++) {
    std::function<void(autoware_system_msgs::SystemStatus)> func =
        *function_itr;
    func(*msg);
  }
  return;
}

void SystemStatusSubscriber::addCallback(
    std::function<void(autoware_system_msgs::SystemStatus)> func) {
  functions_.push_back(func);
  return;
}
}