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

#ifndef DEBUGGER__DEBUG_OBJECT_HPP_
#define DEBUGGER__DEBUG_OBJECT_HPP_

#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/universe_utils/ros/uuid_helper.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::multi_object_tracker
{

struct ObjectData
{
  rclcpp::Time time;

  // object uuid
  boost::uuids::uuid uuid;
  int uuid_int;
  std::string uuid_str;

  // association link, pair of coordinates
  // tracker to detection
  geometry_msgs::msg::Point tracker_point;
  geometry_msgs::msg::Point detection_point;
  bool is_associated{false};

  // existence probabilities
  std::vector<float> existence_vector;

  // detection channel id
  uint channel_id;
};

class TrackerObjectDebugger
{
public:
  explicit TrackerObjectDebugger(const std::string & frame_id);

private:
  bool is_initialized_{false};
  std::string frame_id_;
  visualization_msgs::msg::MarkerArray markers_;
  std::unordered_set<int> current_ids_;
  std::unordered_set<int> previous_ids_;
  rclcpp::Time message_time_;

  std::vector<ObjectData> object_data_list_;
  std::list<int32_t> unused_marker_ids_;
  int32_t marker_id_ = 0;
  std::vector<std::vector<ObjectData>> object_data_groups_;

  std::vector<std::string> channel_names_;

public:
  void setChannelNames(const std::vector<std::string> & channel_names)
  {
    channel_names_ = channel_names;
  }
  void collect(
    const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
    const uint & channel_index,
    const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
    const std::unordered_map<int, int> & direct_assignment,
    const std::unordered_map<int, int> & reverse_assignment);

  void reset();
  void draw(
    const std::vector<std::vector<ObjectData>> & object_data_groups,
    visualization_msgs::msg::MarkerArray & marker_array) const;
  void process();
  void getMessage(visualization_msgs::msg::MarkerArray & marker_array) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // DEBUGGER__DEBUG_OBJECT_HPP_
