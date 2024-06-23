// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
///

#ifndef MULTI_OBJECT_TRACKER__MULTI_OBJECT_TRACKER_CORE_HPP_
#define MULTI_OBJECT_TRACKER__MULTI_OBJECT_TRACKER_CORE_HPP_

#include "multi_object_tracker/data_association/data_association.hpp"
#include "multi_object_tracker/debugger/debugger.hpp"
#include "multi_object_tracker/processor/input_manager.hpp"
#include "multi_object_tracker/processor/processor.hpp"
#include "multi_object_tracker/tracker/model/tracker_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace multi_object_tracker
{

using DetectedObject = autoware_perception_msgs::msg::DetectedObject;
using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
using TrackedObjects = autoware_perception_msgs::msg::TrackedObjects;

class MultiObjectTracker : public rclcpp::Node
{
public:
  explicit MultiObjectTracker(const rclcpp::NodeOptions & node_options);

private:
  // ROS interface
  rclcpp::Publisher<TrackedObjects>::SharedPtr tracked_objects_pub_;
  rclcpp::Subscription<DetectedObjects>::SharedPtr detected_object_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // debugger
  std::unique_ptr<TrackerDebugger> debugger_;
  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;

  // publish timer
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Time last_published_time_;
  double publisher_period_;

  // internal states
  std::string world_frame_id_;  // tracking frame
  std::unique_ptr<DataAssociation> data_association_;
  std::unique_ptr<TrackerProcessor> processor_;

  // input manager
  std::unique_ptr<InputManager> input_manager_;

  std::vector<InputChannel> input_channels_{};
  size_t input_channel_size_{};

  // callback functions
  void onTimer();
  void onTrigger();
  void onMessage(const ObjectsList & objects_list);

  // publish processes
  void runProcess(const DetectedObjects & input_objects_msg, const uint & channel_index);
  void checkAndPublish(const rclcpp::Time & time);
  void publish(const rclcpp::Time & time) const;
  inline bool shouldTrackerPublish(const std::shared_ptr<const Tracker> tracker) const;
};

}  // namespace multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER__MULTI_OBJECT_TRACKER_CORE_HPP_
