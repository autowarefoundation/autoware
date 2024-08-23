// Copyright 2024 TIER IV, Inc.
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

#ifndef PROCESSOR__INPUT_MANAGER_HPP_
#define PROCESSOR__INPUT_MANAGER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
using ObjectsList = std::vector<std::pair<uint, DetectedObjects>>;

struct InputChannel
{
  std::string input_topic;  // topic name of the detection, e.g. "/detection/lidar"
  std::string long_name = "Detected Object";  // full name of the detection
  std::string short_name = "DET";             // abbreviation of the name
  bool is_spawn_enabled = true;               // enable spawn of the object
};

class InputStream
{
public:
  explicit InputStream(rclcpp::Node & node, uint & index);

  void init(const InputChannel & input_channel);

  void setTriggerFunction(std::function<void(const uint &)> func_trigger)
  {
    func_trigger_ = func_trigger;
  }

  void onMessage(const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg);
  void updateTimingStatus(const rclcpp::Time & now, const rclcpp::Time & objects_time);

  bool isTimeInitialized() const { return initial_count_ > 0; }
  uint getIndex() const { return index_; }
  void getObjectsOlderThan(
    const rclcpp::Time & object_latest_time, const rclcpp::Time & object_oldest_time,
    ObjectsList & objects_list);
  bool isSpawnEnabled() const { return is_spawn_enabled_; }

  void getTimeStatistics(
    double & latency_mean, double & latency_var, double & interval_mean,
    double & interval_var) const
  {
    latency_mean = latency_mean_;
    latency_var = latency_var_;
    interval_mean = interval_mean_;
    interval_var = interval_var_;
  }
  rclcpp::Time getLatestMeasurementTime() const { return latest_measurement_time_; }

private:
  rclcpp::Node & node_;
  uint index_;

  std::string input_topic_;
  std::string long_name_;
  std::string short_name_;
  bool is_spawn_enabled_{};

  size_t que_size_{30};
  std::deque<DetectedObjects> objects_que_;

  std::function<void(const uint &)> func_trigger_;

  // bool is_time_initialized_{false};
  int initial_count_{0};
  double expected_interval_{};
  double latency_mean_{};
  double latency_var_{};
  double interval_mean_{};
  double interval_var_{};

  rclcpp::Time latest_measurement_time_;
  rclcpp::Time latest_message_time_;
};

class InputManager
{
public:
  explicit InputManager(rclcpp::Node & node);
  void init(const std::vector<InputChannel> & input_channels);

  void setTriggerFunction(std::function<void()> func_trigger) { func_trigger_ = func_trigger; }
  void onTrigger(const uint & index) const;

  bool getObjects(const rclcpp::Time & now, ObjectsList & objects_list);

  bool isChannelSpawnEnabled(const uint & index) const
  {
    return input_streams_[index]->isSpawnEnabled();
  }

private:
  rclcpp::Node & node_;
  std::vector<rclcpp::Subscription<DetectedObjects>::SharedPtr> sub_objects_array_{};

  bool is_initialized_{false};
  rclcpp::Time latest_exported_object_time_;

  size_t input_size_{};
  std::vector<std::shared_ptr<InputStream>> input_streams_;

  std::function<void()> func_trigger_;
  uint target_stream_idx_{0};
  double target_stream_latency_{0.2};        // [s]
  double target_stream_latency_std_{0.04};   // [s]
  double target_stream_interval_{0.1};       // [s]
  double target_stream_interval_std_{0.02};  // [s]
  double target_latency_{0.2};               // [s]
  double target_latency_band_{1.0};          // [s]

private:
  void getObjectTimeInterval(
    const rclcpp::Time & now, rclcpp::Time & object_latest_time,
    rclcpp::Time & object_oldest_time) const;
  void optimizeTimings();
};

}  // namespace autoware::multi_object_tracker

#endif  // PROCESSOR__INPUT_MANAGER_HPP_
