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

#include <mission_checkpoint/mission_checkpoint.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2_ros/transform_listener.h>

#include <string>

namespace rviz_plugins
{
MissionCheckpointTool::MissionCheckpointTool()
{
  shortcut_key_ = 'c';

  pose_topic_property_ = new rviz_common::properties::StringProperty(
    "Pose Topic", "mission_checkpoint", "The topic on which to publish checkpoint.",
    getPropertyContainer(), SLOT(updateTopic()), this);
  std_dev_x_ = new rviz_common::properties::FloatProperty(
    "X std deviation", 0.5, "X standard deviation for checkpoint pose [m]", getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty(
    "Y std deviation", 0.5, "Y standard deviation for checkpoint pose [m]", getPropertyContainer());
  std_dev_theta_ = new rviz_common::properties::FloatProperty(
    "Theta std deviation", M_PI / 12.0, "Theta standard deviation for checkpoint pose [rad]",
    getPropertyContainer());
  position_z_ = new rviz_common::properties::FloatProperty(
    "Z position", 0.0, "Z position for checkpoint pose [m]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_theta_->setMin(0);
  position_z_->setMin(0);
}

void MissionCheckpointTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("2D Checkpoint Pose");
  updateTopic();
}

void MissionCheckpointTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  pose_pub_ = raw_node->create_publisher<geometry_msgs::msg::PoseStamped>(
    pose_topic_property_->getStdString(), 1);
  clock_ = raw_node->get_clock();
}

void MissionCheckpointTool::onPoseSet(double x, double y, double theta)
{
  // pose
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = clock_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = position_z_->getFloat();

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  pose.pose.orientation = tf2::toMsg(quat);
  RCLCPP_INFO(
    rclcpp::get_logger("MissionCheckpointTool"), "Setting pose: %.3f %.3f %.3f %.3f [frame=%s]", x,
    y, position_z_->getFloat(), theta, fixed_frame.c_str());
  pose_pub_->publish(pose);
}

}  // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::MissionCheckpointTool, rviz_common::Tool)
