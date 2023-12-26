// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#include "tier4_planning_rviz_plugin/pose_with_uuid_stamped/display.hpp"

#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/axes.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

namespace
{
std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & u)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +u.uuid[i];
  }
  return ss.str();
}
}  // namespace

namespace rviz_plugins
{
AutowarePoseWithUuidStampedDisplay::AutowarePoseWithUuidStampedDisplay()
{
  length_property_ = new rviz_common::properties::FloatProperty(
    "Length", 1.5f, "Length of each axis, in meters.", this, SLOT(updateVisualization()));
  length_property_->setMin(0.0001f);

  radius_property_ = new rviz_common::properties::FloatProperty(
    "Radius", 0.5f, "Radius of each axis, in meters.", this, SLOT(updateVisualization()));
  radius_property_->setMin(0.0001f);

  uuid_text_view_property_ = new rviz_common::properties::BoolProperty(
    "UUID", false, "flag of visualizing uuid text", this, SLOT(updateVisualization()), this);
  uuid_text_scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 0.3f, "Scale of uuid text", uuid_text_view_property_, SLOT(updateVisualization()),
    this);
}

AutowarePoseWithUuidStampedDisplay::~AutowarePoseWithUuidStampedDisplay() = default;

void AutowarePoseWithUuidStampedDisplay::onInitialize()
{
  MFDClass::onInitialize();
  axes_ = std::make_unique<rviz_rendering::Axes>(
    scene_manager_, scene_node_, length_property_->getFloat(), radius_property_->getFloat());
  axes_->getSceneNode()->setVisible(isEnabled());

  uuid_node_.reset(scene_node_->createChildSceneNode());
  uuid_ = new rviz_rendering::MovableText("not initialized", "Liberation Sans", 0.1);
}

void AutowarePoseWithUuidStampedDisplay::onEnable()
{
  subscribe();
  axes_->getSceneNode()->setVisible(true);
}

void AutowarePoseWithUuidStampedDisplay::onDisable()
{
  unsubscribe();
  axes_->getSceneNode()->setVisible(false);
}

void AutowarePoseWithUuidStampedDisplay::subscribe()
{
  MFDClass::subscribe();
}

void AutowarePoseWithUuidStampedDisplay::unsubscribe()
{
  MFDClass::unsubscribe();
}

void AutowarePoseWithUuidStampedDisplay::updateVisualization()
{
  if (last_msg_ptr_ != nullptr) {
    processMessage(last_msg_ptr_);
  }
}

void AutowarePoseWithUuidStampedDisplay::processMessage(
  const autoware_planning_msgs::msg::PoseWithUuidStamped::ConstSharedPtr msg_ptr)
{
  uuid_node_->detachAllObjects();

  {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    if (!context_->getFrameManager()->getTransform(msg_ptr->header, position, orientation)) {
      const auto frame = msg_ptr->header.frame_id.c_str();
      RCLCPP_DEBUG(
        rclcpp::get_logger("AutowarePoseWithUuidStampedDisplay"),
        "Error transforming from frame '%s' to frame '%s'", frame, qPrintable(fixed_frame_));
      axes_->getSceneNode()->setVisible(false);
      uuid_->setVisible(false);
      setMissingTransformToFixedFrame(frame);
    } else {
      setTransformOk();
      axes_->getSceneNode()->setVisible(true);
      uuid_->setVisible(true);
      scene_node_->setPosition(position);
      scene_node_->setOrientation(orientation);
    }
  }

  {
    Ogre::Vector3 position;
    position.x = msg_ptr->pose.position.x;
    position.y = msg_ptr->pose.position.y;
    position.z = msg_ptr->pose.position.z;

    Ogre::Quaternion orientation;
    orientation.x = msg_ptr->pose.orientation.x;
    orientation.y = msg_ptr->pose.orientation.y;
    orientation.z = msg_ptr->pose.orientation.z;
    orientation.w = msg_ptr->pose.orientation.w;
    axes_->setPosition(position);
    axes_->setOrientation(orientation);
    axes_->set(length_property_->getFloat(), radius_property_->getFloat());

    if (uuid_text_view_property_->getBool()) {
      uuid_node_->setPosition(position);
      uuid_->setTextAlignment(
        rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);
      uuid_->setCaption(uuid_to_string(msg_ptr->uuid));
      uuid_->setCharacterHeight(uuid_text_scale_property_->getFloat());
      uuid_->setVisible(true);
      uuid_node_->attachObject(uuid_);
    }
  }
  last_msg_ptr_ = msg_ptr;
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowarePoseWithUuidStampedDisplay, rviz_common::Display)
