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
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "delete_all_objects.hpp"

#include <tf2_ros/transform_listener.h>

#include <string>

namespace rviz_plugins
{
DeleteAllObjectsTool::DeleteAllObjectsTool()
{
  shortcut_key_ = 'd';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Pose Topic", "/simulation/dummy_perception_publisher/object_info",
    "The topic on which to publish dummy object info.", getPropertyContainer(), SLOT(updateTopic()),
    this);
}

void DeleteAllObjectsTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Delete All Objects");
  updateTopic();
}

void DeleteAllObjectsTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  dummy_object_info_pub_ = raw_node->create_publisher<tier4_simulation_msgs::msg::DummyObject>(
    topic_property_->getStdString(), 1);
  clock_ = raw_node->get_clock();
}

// cppcheck-suppress unusedFunction
void DeleteAllObjectsTool::onPoseSet(
  [[maybe_unused]] double x, [[maybe_unused]] double y, [[maybe_unused]] double theta)
{
  tier4_simulation_msgs::msg::DummyObject output_msg;
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  // header
  output_msg.header.frame_id = fixed_frame;
  output_msg.header.stamp = clock_->now();

  // action
  output_msg.action = tier4_simulation_msgs::msg::DummyObject::DELETEALL;

  dummy_object_info_pub_->publish(output_msg);
}

}  // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::DeleteAllObjectsTool, rviz_common::Tool)
