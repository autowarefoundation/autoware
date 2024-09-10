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

#include "pedestrian_pose.hpp"

#include "util.hpp"

#include <rviz_common/display_context.hpp>

#include <algorithm>
#include <random>
#include <string>

namespace rviz_plugins
{
PedestrianInitialPoseTool::PedestrianInitialPoseTool()
{
  shortcut_key_ = 'l';

  enable_interactive_property_ = new rviz_common::properties::BoolProperty(
    "Interactive", false, "Enable/Disable interactive action by manipulating mouse.",
    getPropertyContainer());
  property_frame_ = new rviz_common::properties::TfFrameProperty(
    "Target Frame", rviz_common::properties::TfFrameProperty::FIXED_FRAME_STRING,
    "The TF frame where the point cloud is output.", getPropertyContainer(), nullptr, true);
  topic_property_ = new rviz_common::properties::StringProperty(
    "Pose Topic", "/simulation/dummy_perception_publisher/object_info",
    "The topic on which to publish dummy object info.", getPropertyContainer(), SLOT(updateTopic()),
    this);
  std_dev_x_ = new rviz_common::properties::FloatProperty(
    "X std deviation", 0.03, "X standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_y_ = new rviz_common::properties::FloatProperty(
    "Y std deviation", 0.03, "Y standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_z_ = new rviz_common::properties::FloatProperty(
    "Z std deviation", 0.03, "Z standard deviation for initial pose [m]", getPropertyContainer());
  std_dev_theta_ = new rviz_common::properties::FloatProperty(
    "Theta std deviation", 5.0 * M_PI / 180.0, "Theta standard deviation for initial pose [rad]",
    getPropertyContainer());
  position_z_ = new rviz_common::properties::FloatProperty(
    "Z position", 0.0, "Z position for initial pose [m]", getPropertyContainer());
  velocity_ = new rviz_common::properties::FloatProperty(
    "Velocity", 0.0, "velocity [m/s]", getPropertyContainer());
  accel_ = new rviz_common::properties::FloatProperty(
    "Acceleration", 0.0, "acceleration [m/s^2]", getPropertyContainer());
  max_velocity_ = new rviz_common::properties::FloatProperty(
    "Max velocity", 33.3, "Max velocity [m/s]", getPropertyContainer());
  min_velocity_ = new rviz_common::properties::FloatProperty(
    "Min velocity", -33.3, "Min velocity [m/s]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_z_->setMin(0);
  std_dev_theta_->setMin(0);
}

void PedestrianInitialPoseTool::onInitialize()
{
  rviz_plugins::InteractiveObjectTool::onInitialize();
  setName("2D Dummy Pedestrian");
  updateTopic();
}

Object PedestrianInitialPoseTool::createObjectMsg() const
{
  Object object{};
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  // header
  object.header.frame_id = fixed_frame;
  object.header.stamp = clock_->now();

  // semantic
  object.classification.label = ObjectClassification::PEDESTRIAN;
  object.classification.probability = 1.0;

  // shape
  object.shape.type = Shape::CYLINDER;
  const double width = 0.6;
  const double length = 0.6;
  object.shape.dimensions.x = length;
  object.shape.dimensions.y = width;
  object.shape.dimensions.z = 2.0;

  // initial state
  object.initial_state.pose_covariance.pose.position.z = position_z_->getFloat();
  object.initial_state.pose_covariance.covariance[0] =
    std_dev_x_->getFloat() * std_dev_x_->getFloat();
  object.initial_state.pose_covariance.covariance[7] =
    std_dev_y_->getFloat() * std_dev_y_->getFloat();
  object.initial_state.pose_covariance.covariance[14] =
    std_dev_z_->getFloat() * std_dev_z_->getFloat();
  object.initial_state.pose_covariance.covariance[35] =
    std_dev_theta_->getFloat() * std_dev_theta_->getFloat();

  std::mt19937 gen(std::random_device{}());
  std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
  std::generate(object.id.uuid.begin(), object.id.uuid.end(), bit_eng);

  return object;
}
}  // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PedestrianInitialPoseTool, rviz_common::Tool)
