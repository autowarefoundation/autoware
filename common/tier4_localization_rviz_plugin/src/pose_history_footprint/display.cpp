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

#include "display.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>

#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace rviz_plugins
{
PoseHistoryFootprint::PoseHistoryFootprint() : last_stamp_(0, 0, RCL_ROS_TIME)
{
  property_buffer_size_ = new rviz_common::properties::IntProperty("Buffer Size", 100, "", this);

  // trajectory footprint
  property_trajectory_footprint_view_ = new rviz_common::properties::BoolProperty(
    "View Trajectory Footprint", true, "", this, SLOT(updateVisualization()), this);
  property_trajectory_footprint_alpha_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "", property_trajectory_footprint_view_, SLOT(updateVisualization()), this);
  property_trajectory_footprint_alpha_->setMin(0.0);
  property_trajectory_footprint_alpha_->setMax(1.0);
  property_trajectory_footprint_color_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(230, 230, 50), "", property_trajectory_footprint_view_,
    SLOT(updateVisualization()), this);
  property_vehicle_length_ = new rviz_common::properties::FloatProperty(
    "Vehicle Length", 4.77, "", property_trajectory_footprint_view_, SLOT(updateVehicleInfo()),
    this);
  property_vehicle_width_ = new rviz_common::properties::FloatProperty(
    "Vehicle Width", 1.83, "", property_trajectory_footprint_view_, SLOT(updateVehicleInfo()),
    this);
  property_rear_overhang_ = new rviz_common::properties::FloatProperty(
    "Rear Overhang", 1.03, "", property_trajectory_footprint_view_, SLOT(updateVehicleInfo()),
    this);
  property_offset_ = new rviz_common::properties::FloatProperty(
    "Offset from BaseLink", 0.0, "", property_trajectory_footprint_view_, SLOT(updateVehicleInfo()),
    this);
  property_interval_ = new rviz_common::properties::FloatProperty(
    "Displayed Footprint Interval [m]", 1.0, "", property_trajectory_footprint_view_,
    SLOT(updateVehicleInfo()), this);
  property_vehicle_length_->setMin(0.0);
  property_vehicle_width_->setMin(0.0);
  property_rear_overhang_->setMin(0.0);
  property_interval_->setMin(0.0);

  updateVehicleInfo();
}

PoseHistoryFootprint::~PoseHistoryFootprint()
{
  if (initialized()) {
    scene_manager_->destroyManualObject(trajectory_footprint_manual_object_);
  }
}

void PoseHistoryFootprint::updateVehicleInfo()
{
  if (vehicle_info_) {
    vehicle_footprint_info_ = std::make_shared<VehicleFootprintInfo>(
      vehicle_info_->vehicle_length_m, vehicle_info_->vehicle_width_m,
      vehicle_info_->rear_overhang_m);
  } else {
    const float length{property_vehicle_length_->getFloat()};
    const float width{property_vehicle_width_->getFloat()};
    const float rear_overhang{property_rear_overhang_->getFloat()};

    vehicle_footprint_info_ = std::make_shared<VehicleFootprintInfo>(length, width, rear_overhang);
  }
}

void PoseHistoryFootprint::updateVisualization()
{
  if (last_msg_ptr_) {
    processMessage(last_msg_ptr_);
  }
}

void PoseHistoryFootprint::onInitialize()
{
  MFDClass::onInitialize();

  trajectory_footprint_manual_object_ = scene_manager_->createManualObject();
  trajectory_footprint_manual_object_->setDynamic(true);
  scene_node_->attachObject(trajectory_footprint_manual_object_);
}

void PoseHistoryFootprint::onEnable() { subscribe(); }

void PoseHistoryFootprint::onDisable() { unsubscribe(); }

void PoseHistoryFootprint::subscribe() { MFDClass::subscribe(); }

void PoseHistoryFootprint::unsubscribe()
{
  MFDClass::unsubscribe();

  history_.clear();
}

void PoseHistoryFootprint::processMessage(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
{
  if (!rviz_common::validateFloats(message->pose)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  updateHistory(message);
  updateFootprint();

  last_msg_ptr_ = message;
}

void PoseHistoryFootprint::updateHistory(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr message)
{
  if (history_.empty()) {
    history_.emplace_back(message);
  } else {
    const auto dx = message->pose.position.x - history_.back()->pose.position.x;
    const auto dy = message->pose.position.y - history_.back()->pose.position.y;
    if (std::hypot(dx, dy) > property_interval_->getFloat()) {
      history_.emplace_back(message);
    }
  }

  const auto buffer_size = static_cast<size_t>(property_buffer_size_->getInt());
  while (buffer_size < history_.size()) {
    history_.pop_front();
  }
}

void PoseHistoryFootprint::updateFootprint()
{
  // This doesn't work in the constructor.
  if (!vehicle_info_) {
    try {
      vehicle_info_ = std::make_shared<VehicleInfo>(
        VehicleInfoUtil(*rviz_ros_node_.lock()->get_raw_node()).getVehicleInfo());
      updateVehicleInfo();
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        rviz_ros_node_.lock()->get_raw_node()->get_logger(),
        *rviz_ros_node_.lock()->get_raw_node()->get_clock(), 5000, "Failed to get vehicle_info: %s",
        e.what());
    }
  }

  if (history_.empty()) return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(history_.back()->header, position, orientation)) {
    RCLCPP_DEBUG(
      rviz_ros_node_.lock()->get_raw_node()->get_logger(),
      "Error transforming from frame '%s' to frame '%s'", history_.back()->header.frame_id.c_str(),
      qPrintable(fixed_frame_));
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  trajectory_footprint_manual_object_->clear();

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);

  if (!history_.empty()) {
    trajectory_footprint_manual_object_->estimateVertexCount(history_.size() * 4 * 2);
    trajectory_footprint_manual_object_->begin(
      "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

    const float offset_from_baselink = property_offset_->getFloat();

    for (size_t point_idx = 0; point_idx < history_.size(); ++point_idx) {
      const auto & pose = history_.at(point_idx)->pose;
      /*
       * Footprint
       */
      if (property_trajectory_footprint_view_->getBool()) {
        Ogre::ColourValue color;
        color = rviz_common::properties::qtToOgre(property_trajectory_footprint_color_->getColor());
        color.a = property_trajectory_footprint_alpha_->getFloat();

        const auto info = vehicle_footprint_info_;
        const float top = info->length - info->rear_overhang - offset_from_baselink;
        const float bottom = -info->rear_overhang + offset_from_baselink;
        const float left = -info->width / 2.0;
        const float right = info->width / 2.0;

        const std::array<float, 4> lon_offset_vec{top, top, bottom, bottom};
        const std::array<float, 4> lat_offset_vec{left, right, right, left};

        for (int f_idx = 0; f_idx < 4; ++f_idx) {
          const auto & o = pose.orientation;
          const auto & p = pose.position;
          const Eigen::Quaternionf quat(o.w, o.x, o.y, o.z);
          {
            const Eigen::Vector3f offset_vec{
              lon_offset_vec.at(f_idx), lat_offset_vec.at(f_idx), 0.0};
            const auto offset_to_edge = quat * offset_vec;
            trajectory_footprint_manual_object_->position(
              p.x + offset_to_edge.x(), p.y + offset_to_edge.y(), p.z);
            trajectory_footprint_manual_object_->colour(color);
          }
          {
            const Eigen::Vector3f offset_vec{
              lon_offset_vec.at((f_idx + 1) % 4), lat_offset_vec.at((f_idx + 1) % 4), 0.0};
            const auto offset_to_edge = quat * offset_vec;
            trajectory_footprint_manual_object_->position(
              p.x + offset_to_edge.x(), p.y + offset_to_edge.y(), p.z);
            trajectory_footprint_manual_object_->colour(color);
          }
        }
      }
    }
    trajectory_footprint_manual_object_->end();
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PoseHistoryFootprint, rviz_common::Display)
