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

#include <autoware_health_checker/rate_checker.h>

namespace autoware_health_checker {
RateChecker::RateChecker(double buffer_length, double warn_rate,
                         double error_rate, double fatal_rate,
                         std::string description)
    : buffer_length_(buffer_length), warn_rate_(warn_rate),
      error_rate_(error_rate), fatal_rate_(fatal_rate),
      description(description) {
  start_time_ = ros::Time::now();
}

RateChecker::~RateChecker() {}

std::pair<uint8_t, double> RateChecker::getErrorLevelAndRate() {
  std::pair<uint8_t, double> ret;
  boost::optional<double> rate = getRate();
  if (!rate) {
    ret = std::make_pair(autoware_health_checker::LEVEL_ERROR, 0);
  } else if (rate.get() < fatal_rate_) {
    ret = std::make_pair(autoware_health_checker::LEVEL_FATAL, rate.get());
  } else if (rate.get() < error_rate_) {
    ret = std::make_pair(autoware_health_checker::LEVEL_ERROR, rate.get());
  } else if (rate.get() < warn_rate_) {
    ret = std::make_pair(autoware_health_checker::LEVEL_WARN, rate.get());
  } else {
    ret = std::make_pair(autoware_health_checker::LEVEL_OK, rate.get());
  }
  return ret;
}

uint8_t RateChecker::getErrorLevel() {
  boost::optional<double> rate = getRate();
  if (!rate) {
    return autoware_health_checker::LEVEL_ERROR;
  }
  if (rate.get() < fatal_rate_) {
    return autoware_health_checker::LEVEL_FATAL;
  }
  if (rate.get() < error_rate_) {
    return autoware_health_checker::LEVEL_ERROR;
  }
  if (rate.get() < warn_rate_) {
    return autoware_health_checker::LEVEL_WARN;
  }
  return autoware_health_checker::LEVEL_OK;
}

void RateChecker::check() {
  update();
  mtx_.lock();
  data_.push_back(ros::Time::now());
  mtx_.unlock();
}

void RateChecker::update() {
  mtx_.lock();
  std::vector<ros::Time> buffer;
  for (auto data_itr = data_.begin(); data_itr != data_.end(); data_itr++) {
    if (*data_itr > ros::Time::now() - ros::Duration(buffer_length_)) {
      buffer.push_back(*data_itr);
    }
  }
  data_ = buffer;
  mtx_.unlock();
  return;
}

boost::optional<double> RateChecker::getRate() {
  boost::optional<double> rate;
  if (ros::Time::now() - start_time_ < ros::Duration(buffer_length_)) {
    return boost::none;
  }
  update();
  mtx_.lock();
  rate = data_.size() / buffer_length_;
  mtx_.unlock();
  return rate;
}
}