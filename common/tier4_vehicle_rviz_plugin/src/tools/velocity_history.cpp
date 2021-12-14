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

#include "velocity_history.hpp"

#include <OgreMaterialManager.h>

#include <algorithm>
#include <memory>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>

namespace rviz_plugins
{
std::unique_ptr<Ogre::ColourValue> VelocityHistoryDisplay::gradation(
  const QColor & color_min, const QColor & color_max, const double ratio)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g = color_max.greenF() * ratio + color_min.greenF() * (1.0 - ratio);
  color_ptr->r = color_max.redF() * ratio + color_min.redF() * (1.0 - ratio);
  color_ptr->b = color_max.blueF() * ratio + color_min.blueF() * (1.0 - ratio);

  return color_ptr;
}

std::unique_ptr<Ogre::ColourValue> VelocityHistoryDisplay::setColorDependsOnVelocity(
  const double vel_max, const double cmd_vel)
{
  const double cmd_vel_abs = std::fabs(cmd_vel);
  const double vel_min = 0.0;

  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue());
  if (vel_min < cmd_vel_abs && cmd_vel_abs <= (vel_max / 2.0)) {
    double ratio = (cmd_vel_abs - vel_min) / (vel_max / 2.0 - vel_min);
    color_ptr = gradation(Qt::red, Qt::yellow, ratio);
  } else if ((vel_max / 2.0) < cmd_vel_abs && cmd_vel_abs <= vel_max) {
    double ratio = (cmd_vel_abs - vel_max / 2.0) / (vel_max - vel_max / 2.0);
    color_ptr = gradation(Qt::yellow, Qt::green, ratio);
  } else if (vel_max < cmd_vel_abs) {
    *color_ptr = Ogre::ColourValue::Green;
  } else {
    *color_ptr = Ogre::ColourValue::Red;
  }

  return color_ptr;
}

VelocityHistoryDisplay::VelocityHistoryDisplay()
{
  property_velocity_timeout_ = new rviz_common::properties::FloatProperty(
    "Timeout", 10.0, "", this, SLOT(updateVisualization()), this);
  property_velocity_timeout_->setMin(0.0);
  property_velocity_timeout_->setMax(100000.0);
  property_velocity_alpha_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "", this, SLOT(updateVisualization()), this);
  property_velocity_alpha_->setMin(0.0);
  property_velocity_alpha_->setMax(1.0);
  property_velocity_scale_ = new rviz_common::properties::FloatProperty(
    "Scale", 0.3, "", this, SLOT(updateVisualization()), this);
  property_velocity_scale_->setMin(0.1);
  property_velocity_scale_->setMax(10.0);
  property_velocity_color_view_ = new rviz_common::properties::BoolProperty(
    "Constant Color", false, "", this, SLOT(updateVisualization()), this);
  property_velocity_color_ = new rviz_common::properties::ColorProperty(
    "Color", Qt::black, "", property_velocity_color_view_, SLOT(updateVisualization()), this);
  property_vel_max_ = new rviz_common::properties::FloatProperty(
    "Color Border Vel Max", 3.0, "[m/s]", this, SLOT(updateVisualization()), this);
  property_vel_max_->setMin(0.0);
}

VelocityHistoryDisplay::~VelocityHistoryDisplay()
{
  if (initialized()) {
    scene_manager_->destroyManualObject(velocity_manual_object_);
  }
}

void VelocityHistoryDisplay::onInitialize()
{
  RTDClass::onInitialize();

  velocity_manual_object_ = scene_manager_->createManualObject();
  velocity_manual_object_->setDynamic(true);
  scene_node_->attachObject(velocity_manual_object_);
}

void VelocityHistoryDisplay::reset()
{
  RTDClass::reset();
  velocity_manual_object_->clear();
}

bool VelocityHistoryDisplay::validateFloats(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr & msg_ptr)
{
  if (!rviz_common::validateFloats(msg_ptr->longitudinal_velocity)) {
    return false;
  }

  return true;
}

void VelocityHistoryDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  updateVisualization();
}

void VelocityHistoryDisplay::processMessage(
  const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) {
    return;
  }

  if (!validateFloats(msg_ptr)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  std_msgs::msg::Header header;
  header.stamp = msg_ptr->header.stamp;
  header.frame_id = "base_link";
  if (!context_->getFrameManager()->getTransform(header, position, orientation)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("VelocityHistoryDisplay"),
      "Error transforming from frame '%s' to frame '%s'", header.frame_id.c_str(),
      qPrintable(fixed_frame_));
  }

  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    histories_.emplace_back(msg_ptr, position);
  }
  queueRender();
}

void VelocityHistoryDisplay::updateVisualization()
{
  std::lock_guard<std::mutex> message_lock(mutex_);
  if (histories_.empty()) {
    return;
  }
  velocity_manual_object_->clear();

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);
  rclcpp::Time current_time = rviz_ros_node_.lock()->get_raw_node()->get_clock()->now();

  while (!histories_.empty()) {
    if (
      property_velocity_timeout_->getFloat() <
      (current_time - std::get<0>(histories_.front())->header.stamp).seconds()) {
      histories_.pop_front();
    } else {
      break;
    }
  }

  // std::cout << __LINE__ << ":" <<std::get<1>(histories_.front()) <<std::endl;
  velocity_manual_object_->estimateVertexCount(histories_.size());
  velocity_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_POINT_LIST);

  for (auto & history : histories_) {
    Ogre::ColourValue color;
    if (property_velocity_color_view_->getBool()) {
      color = rviz_common::properties::qtToOgre(property_velocity_color_->getColor());
    } else {
      /* color change depending on velocity */
      std::unique_ptr<Ogre::ColourValue> dynamic_color_ptr = setColorDependsOnVelocity(
        property_vel_max_->getFloat(), std::get<0>(history)->longitudinal_velocity);
      color = *dynamic_color_ptr;
    }
    color.a = 1.0 - (current_time - std::get<0>(history)->header.stamp).seconds() /
                      property_velocity_timeout_->getFloat();
    color.a = std::min(std::max(color.a, 0.0f), 1.0f);
    // std::cout << __LINE__ << ":" <<std::get<1>(histories_.front()) <<std::endl;

    // color.a = property_velocity_alpha_->getFloat();
    velocity_manual_object_->position(
      std::get<1>(history).x, std::get<1>(history).y,
      std::get<1>(history).z +
        std::get<0>(history)->longitudinal_velocity * property_velocity_scale_->getFloat());
    velocity_manual_object_->colour(color);
  }
  velocity_manual_object_->end();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::VelocityHistoryDisplay, rviz_common::Display)
