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

#include "pose_history_display.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

namespace rviz_plugins
{
PoseHistory::PoseHistory() : last_stamp_(0, 0, RCL_ROS_TIME)
{
  property_buffer_size_ = new rviz_common::properties::IntProperty("Buffer Size", 100, "", this);
  property_line_view_ = new rviz_common::properties::BoolProperty("Line", true, "", this);
  property_line_width_ =
    new rviz_common::properties::FloatProperty("Width", 0.1, "", property_line_view_);
  property_line_alpha_ =
    new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_line_view_);
  property_line_alpha_->setMin(0.0);
  property_line_alpha_->setMax(1.0);
  property_line_color_ =
    new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_line_view_);

  property_buffer_size_->setMin(0);
  property_buffer_size_->setMax(16000);
  property_line_width_->setMin(0.0);
}

PoseHistory::~PoseHistory() = default;  // Properties are deleted by Qt

void PoseHistory::onInitialize()
{
  MFDClass::onInitialize();
  lines_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
}

void PoseHistory::onEnable() { subscribe(); }

void PoseHistory::onDisable() { unsubscribe(); }

void PoseHistory::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  if (!history_.empty()) {
    lines_->clear();
    if (property_line_view_->getBool()) {
      updateLines();
    }
  }
}

void PoseHistory::subscribe() { MFDClass::subscribe(); }

void PoseHistory::unsubscribe()
{
  MFDClass::unsubscribe();

  history_.clear();
  lines_->clear();
}

void PoseHistory::processMessage(const geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
{
  if (!rviz_common::validateFloats(message->pose)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }
  if (target_frame_ != message->header.frame_id) {
    history_.clear();
    target_frame_ = message->header.frame_id;
  }
  history_.emplace_back(message);
  last_stamp_ = message->header.stamp;

  updateHistory();
}

void PoseHistory::updateHistory()
{
  const auto buffer_size = static_cast<size_t>(property_buffer_size_->getInt());
  while (buffer_size < history_.size()) {
    history_.pop_front();
  }
}

void PoseHistory::updateLines()
{
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(property_line_color_->getColor());
  color.a = property_line_alpha_->getFloat();
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  auto frame_manager = context_->getFrameManager();
  if (!frame_manager->getTransform(target_frame_, last_stamp_, position, orientation)) {
    setMissingTransformToFixedFrame(target_frame_);
    return;
  }

  setTransformOk();
  lines_->setMaxPointsPerLine(history_.size());
  lines_->setLineWidth(property_line_width_->getFloat());
  lines_->setPosition(position);
  lines_->setOrientation(orientation);
  lines_->setColor(color.r, color.g, color.b, color.a);

  for (const auto & message : history_) {
    Ogre::Vector3 point;
    point.x = message->pose.position.x;
    point.y = message->pose.position.y;
    point.z = message->pose.position.z;
    lines_->addPoint(point);
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PoseHistory, rviz_common::Display)
