#ifndef VALUE_MANAGER_H_INCLUDED
#define VALUE_MANAGER_H_INCLUDED

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

// headers in STL
#include <map>
#include <mutex>

// headers in Boost
#include <boost/optional.hpp>
#include <boost/thread.hpp>

class ValueManager {
public:
  ValueManager(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~ValueManager();
  void run();
  void stop() { ros_ok_ = false; };
  void setDefaultValue(std::string key, std::string type, double warn_value,
                       double error_value, double fatal_value);
  double getValue(std::string key, std::string type, uint8_t level);

private:
  std::map<std::pair<std::pair<std::string, std::string>, uint8_t>, double>
      data_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  XmlRpc::XmlRpcValue diag_params_;
  void updateParams();
  bool ros_ok_;
  bool foundParamKey(std::string key, std::string type, uint8_t level,
                     double &value);
  std::mutex mtx_;
};
#endif // VALUE_MANAGER_H_INCLUDED