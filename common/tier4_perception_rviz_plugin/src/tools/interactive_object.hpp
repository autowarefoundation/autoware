// Copyright 2022 Tier IV, Inc.
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

#ifndef TOOLS__INTERACTIVE_OBJECT_HPP_
#define TOOLS__INTERACTIVE_OBJECT_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>
#include <rclcpp/node.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_default_plugins/tools/move/move_tool.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>
#endif

#include <dummy_perception_publisher/msg/object.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <boost/optional.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace rviz_plugins
{

using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::Shape;
using dummy_perception_publisher::msg::Object;

class InteractiveObject
{
public:
  explicit InteractiveObject(const Ogre::Vector3 & point);

  [[nodiscard]] std::array<uint8_t, 16> uuid() const;
  void twist(geometry_msgs::msg::Twist & twist) const;
  void transform(tf2::Transform & tf_map2object) const;
  void update(const Ogre::Vector3 & point);
  void reset();
  double distance(const Ogre::Vector3 & point);

private:
  std::array<uint8_t, 16> uuid_{};
  Ogre::Vector3 point_;
  Ogre::Vector3 velocity_;
  Ogre::Vector3 accel_;
  Ogre::Vector3 max_velocity_;
  Ogre::Vector3 min_velocity_;
  double theta_{0.0};
};

class InteractiveObjectCollection
{
public:
  InteractiveObjectCollection();

  void select(const Ogre::Vector3 & point);
  boost::optional<std::array<uint8_t, 16>> reset();
  boost::optional<std::array<uint8_t, 16>> create(const Ogre::Vector3 & point);
  boost::optional<std::array<uint8_t, 16>> remove(const Ogre::Vector3 & point);
  boost::optional<std::array<uint8_t, 16>> update(const Ogre::Vector3 & point);
  [[nodiscard]] boost::optional<geometry_msgs::msg::Twist> twist(
    const std::array<uint8_t, 16> & uuid) const;
  [[nodiscard]] boost::optional<tf2::Transform> transform(
    const std::array<uint8_t, 16> & uuid) const;

private:
  size_t nearest(const Ogre::Vector3 & point);
  InteractiveObject * target_;
  std::vector<std::unique_ptr<InteractiveObject>> objects_;
};

class InteractiveObjectTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  void onInitialize() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

  [[nodiscard]] virtual Object createObjectMsg() const = 0;

protected Q_SLOTS:
  virtual void updateTopic();

protected:  // NOLINT for Qt
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<Object>::SharedPtr dummy_object_info_pub_;

  rviz_default_plugins::tools::MoveTool move_tool_;

  rviz_common::properties::BoolProperty * enable_interactive_property_;
  rviz_common::properties::StringProperty * topic_property_;
  rviz_common::properties::FloatProperty * std_dev_x_;
  rviz_common::properties::FloatProperty * std_dev_y_;
  rviz_common::properties::FloatProperty * std_dev_z_;
  rviz_common::properties::FloatProperty * width_;
  rviz_common::properties::FloatProperty * length_;
  rviz_common::properties::FloatProperty * height_;
  rviz_common::properties::FloatProperty * std_dev_theta_;
  rviz_common::properties::FloatProperty * position_z_;
  rviz_common::properties::FloatProperty * velocity_;
  rviz_common::properties::FloatProperty * max_velocity_;
  rviz_common::properties::FloatProperty * min_velocity_;
  rviz_common::properties::FloatProperty * accel_;
  rviz_common::properties::TfFrameProperty * property_frame_;

private:
  void publishObjectMsg(const std::array<uint8_t, 16> & uuid, const uint32_t action);
  void onPoseSet(double x, double y, double theta) override;

  InteractiveObjectCollection objects_;
};
}  // namespace rviz_plugins
#endif  // TOOLS__INTERACTIVE_OBJECT_HPP_
