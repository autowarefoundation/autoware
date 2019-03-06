#ifndef SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED
#define SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED

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

// headers in Autoware
#include <autoware_health_checker/constants.h>
#include <autoware_system_msgs/SystemStatus.h>

// headers in ROS
#include <ros/ros.h>

// headers in STL
#include <functional>
#include <mutex>

namespace autoware_health_checker {
class SystemStatusSubscriber {
public:
  SystemStatusSubscriber(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~SystemStatusSubscriber();
  void enable();
  void
  addCallback(std::function<void(autoware_system_msgs::SystemStatus)> func);

private:
  void
  systemStatusCallback(const autoware_system_msgs::SystemStatus::ConstPtr msg);
  std::mutex mtx_;
  ros::Subscriber status_sub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::vector<std::function<void(autoware_system_msgs::SystemStatus)>>
      functions_;
};
}

#endif // SYSTEM_STATUS_SUBSCRIBER_H_INCLUDED