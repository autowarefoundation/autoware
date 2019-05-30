/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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
 */

#ifndef G30ESLI_INTERFACE_H
#define G30ESLI_INTERFACE_H

#include <cmath>
#include <thread>
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "g30esli_ros.h"

class G30esliInterface
{
private:
  // ros nodehandle
  ros::NodeHandle nh_, private_nh_;

  // ros subscriber
  ros::Subscriber vehicle_cmd_sub_;
  ros::Subscriber ds4_sub_;
  ros::Subscriber engage_sub_;

  // ros publisher
  ros::Publisher vehicle_status_pub_;
  ros::Publisher current_twist_pub_;
  ros::Publisher battery_pub_;

  // ros param
  std::string device_;
  bool use_ds4_;
  double steering_offset_deg_;
  double command_timeout_;
  double brake_threshold_;

  // variables
  bool engage_;
  bool terminate_thread_;
  ros::Time engage_start_;
  std::thread* thread_read_status_;
  std::thread* thread_read_keyboard_;
  std::thread* thread_publish_status_;
  std::mutex engage_mutex_;

  // ymc g30esli ros driver
  MODE mode_;
  G30esliROS g30esli_ros_;

public:
  G30esliInterface();
  ~G30esliInterface();

  // callbacks
  void vehicleCmdCallback(const autoware_msgs::VehicleCmdConstPtr& msg);
  void engageCallback(const std_msgs::BoolConstPtr& msg);
  void ds4Callback(const ds4_msgs::DS4ConstPtr& msg);

  // thread functions
  void readKeyboard();
  void readStatus();
  void publishStatus();

  // fucntions
  void run();
  bool kbhit();
};

#endif
