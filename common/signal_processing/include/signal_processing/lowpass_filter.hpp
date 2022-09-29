// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SIGNAL_PROCESSING__LOWPASS_FILTER_HPP_
#define SIGNAL_PROCESSING__LOWPASS_FILTER_HPP_

#include "signal_processing/lowpass_filter_1d.hpp"

#include "geometry_msgs/msg/twist.hpp"

/**
 * @class First-order low-pass filter
 * @brief filtering values
 */
template <typename T>
class LowpassFilterInterface
{
protected:
  boost::optional<T> x_;  //!< @brief current filtered value
  double gain_;           //!< @brief gain value of first-order low-pass filter

public:
  explicit LowpassFilterInterface(const double gain) : gain_(gain) {}

  void reset() { x_ = {}; }
  void reset(const T & x) { x_ = x; }

  boost::optional<T> getValue() const { return x_; }

  virtual T filter(const T & u) = 0;
};

class LowpassFilterTwist : public LowpassFilterInterface<geometry_msgs::msg::Twist>
{
public:
  explicit LowpassFilterTwist(const double gain)
  : LowpassFilterInterface<geometry_msgs::msg::Twist>(gain)
  {
  }

  geometry_msgs::msg::Twist filter(const geometry_msgs::msg::Twist & u) override;
};

#endif  // SIGNAL_PROCESSING__LOWPASS_FILTER_HPP_
