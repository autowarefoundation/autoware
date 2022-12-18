// Copyright 2022 TIER IV, Inc.
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

#ifndef OBSTACLE_CRUISE_PLANNER__STOP_PLANNING_DEBUG_INFO_HPP_
#define OBSTACLE_CRUISE_PLANNER__STOP_PLANNING_DEBUG_INFO_HPP_

#include <rclcpp/rclcpp.hpp>

#include "tier4_debug_msgs/msg/float32_multi_array_stamped.hpp"

#include <array>

using tier4_debug_msgs::msg::Float32MultiArrayStamped;

class StopPlanningDebugInfo
{
public:
  enum class TYPE {
    // ego
    EGO_VELOCITY = 0,  // index: 0
    EGO_ACCELERATION,
    EGO_JERK,  // no data

    // stop planning
    STOP_CURRENT_OBSTACLE_DISTANCE,  // index: 3
    STOP_CURRENT_OBSTACLE_VELOCITY,
    STOP_TARGET_OBSTACLE_DISTANCE,
    STOP_TARGET_VELOCITY,
    STOP_TARGET_ACCELERATION,

    SIZE
  };

  /**
   * @brief get the index corresponding to the given value TYPE
   * @param [in] type the TYPE enum for which to get the index
   * @return index of the type
   */
  int getIndex(const TYPE type) const { return static_cast<int>(type); }

  /**
   * @brief get all the debug values as an std::array
   * @return array of all debug values
   */
  std::array<double, static_cast<int>(TYPE::SIZE)> get() const { return info_; }

  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void set(const TYPE type, const double val) { info_.at(static_cast<int>(type)) = val; }

  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void set(const int type, const double val) { info_.at(type) = val; }

  void reset() { info_.fill(0.0); }

  Float32MultiArrayStamped convertToMessage(const rclcpp::Time & current_time) const
  {
    Float32MultiArrayStamped msg{};
    msg.stamp = current_time;
    for (const auto & v : get()) {
      msg.data.push_back(v);
    }
    return msg;
  }

private:
  std::array<double, static_cast<int>(TYPE::SIZE)> info_;
};

#endif  // OBSTACLE_CRUISE_PLANNER__STOP_PLANNING_DEBUG_INFO_HPP_
