#ifndef RATE_CHECKER_H_INCLUDED
#define RATE_CHECKER_H_INCLUDED

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

// headers in STL
#include <mutex>
#include <vector>

// headers in Boost
#include <boost/optional.hpp>

// headers in Autoware
#include <autoware_health_checker/constants.h>

namespace autoware_health_checker {
class RateChecker {
public:
  RateChecker(double buffer_length, double warn_rate, double error_rate,
              double fatal_rate, std::string description);
  ~RateChecker();
  void check();
  std::pair<uint8_t, double> getErrorLevelAndRate();
  uint8_t getErrorLevel();
  boost::optional<double> getRate();
  const std::string description;

private:
  ros::Time start_time_;
  void update();
  std::vector<ros::Time> data_;
  const double buffer_length_;
  const double warn_rate_;
  const double error_rate_;
  const double fatal_rate_;
  std::mutex mtx_;
};
}
#endif // RATE_CHECKER_H_INCLUDED