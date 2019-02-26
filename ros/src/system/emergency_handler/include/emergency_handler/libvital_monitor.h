#ifndef __LIBVITAL_MONITOR_H__
#define __LIBVITAL_MONITOR_H__
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
#include <ros/ros.h>
#include <map>
#include <set>

struct LifeTime
{
  LifeTime(double secs, bool respawn) :
    life_time_(secs), default_life_time_(secs), respawn_(respawn){}
  bool respawn_;
  const double default_life_time_;
  double life_time_;
  const bool isDead() const
  {
    return (life_time_ == 0.0);
  }
  void spend(double secs)
  {
    life_time_ = std::max(life_time_ - secs, 0.0);
  }
  void reset()
  {
    life_time_ = default_life_time_;
  }
};

class VitalMonitor
{
public:
  void initMonitoredNodeList(ros::NodeHandle& pnh);
  void updateNodeStatus(const std::vector<std::string>& available_nodes);
  const std::map<std::string, bool>& getDeadNodes();
private:
  std::map<std::string, LifeTime> required_nodes_;
  std::map<std::string, bool> dead_nodes_;
};

#endif
