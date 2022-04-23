// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include <memory>

#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <trajectory_footprint/display.hpp>

#include <tf2/utils.h>

namespace rviz_plugins
{
AutowareTrajectoryFootprintDisplay::AutowareTrajectoryFootprintDisplay()
{
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
  property_vehicle_length_->setMin(0.0);
  property_vehicle_width_->setMin(0.0);
  property_rear_overhang_->setMin(0.0);

  // trajectory point
  property_trajectory_point_view_ = new rviz_common::properties::BoolProperty(
    "View Trajectory Point", false, "", this, SLOT(updateVisualization()), this);
  property_trajectory_point_alpha_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "", property_trajectory_point_view_, SLOT(updateVisualization()), this);
  property_trajectory_point_alpha_->setMin(0.0);
  property_trajectory_point_alpha_->setMax(1.0);
  property_trajectory_point_color_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(0, 60, 255), "", property_trajectory_point_view_, SLOT(updateVisualization()),
    this);
  property_trajectory_point_radius_ = new rviz_common::properties::FloatProperty(
    "Radius", 0.1, "", property_trajectory_point_view_, SLOT(updateVisualization()), this);
  property_trajectory_point_offset_ = new rviz_common::properties::FloatProperty(
    "Offset", 0.0, "", property_trajectory_point_view_, SLOT(updateVisualization()), this);

  updateVehicleInfo();
}

AutowareTrajectoryFootprintDisplay::~AutowareTrajectoryFootprintDisplay()
{
  if (initialized()) {
    scene_manager_->destroyManualObject(trajectory_footprint_manual_object_);
    scene_manager_->destroyManualObject(trajectory_point_manual_object_);
  }
}

void AutowareTrajectoryFootprintDisplay::onInitialize()
{
  MFDClass::onInitialize();

  trajectory_footprint_manual_object_ = scene_manager_->createManualObject();
  trajectory_footprint_manual_object_->setDynamic(true);
  scene_node_->attachObject(trajectory_footprint_manual_object_);

  trajectory_point_manual_object_ = scene_manager_->createManualObject();
  trajectory_point_manual_object_->setDynamic(true);
  scene_node_->attachObject(trajectory_point_manual_object_);
}

void AutowareTrajectoryFootprintDisplay::reset()
{
  MFDClass::reset();
  trajectory_footprint_manual_object_->clear();
  trajectory_point_manual_object_->clear();
}

bool AutowareTrajectoryFootprintDisplay::validateFloats(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & msg_ptr)
{
  for (auto && trajectory_point : msg_ptr->points) {
    if (!rviz_common::validateFloats(trajectory_point.pose)) {
      return false;
    }
  }
  return true;
}

void AutowareTrajectoryFootprintDisplay::processMessage(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg_ptr)
{
  if (!validateFloats(msg_ptr)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

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

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg_ptr->header, position, orientation)) {
    RCLCPP_DEBUG(
      rviz_ros_node_.lock()->get_raw_node()->get_logger(),
      "Error transforming from frame '%s' to frame '%s'", msg_ptr->header.frame_id.c_str(),
      qPrintable(fixed_frame_));
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  trajectory_footprint_manual_object_->clear();
  trajectory_point_manual_object_->clear();

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);

  if (!msg_ptr->points.empty()) {
    trajectory_footprint_manual_object_->estimateVertexCount(msg_ptr->points.size() * 4 * 2);
    trajectory_footprint_manual_object_->begin(
      "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

    trajectory_point_manual_object_->estimateVertexCount(msg_ptr->points.size() * 3 * 8);
    trajectory_point_manual_object_->begin(
      "BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (size_t point_idx = 0; point_idx < msg_ptr->points.size(); point_idx++) {
      const auto & path_point = msg_ptr->points.at(point_idx);
      /*
       * Footprint
       */
      if (property_trajectory_footprint_view_->getBool()) {
        Ogre::ColourValue color;
        color = rviz_common::properties::qtToOgre(property_trajectory_footprint_color_->getColor());
        color.a = property_trajectory_footprint_alpha_->getFloat();

        const auto info = vehicle_footprint_info_;
        const float top = info->length - info->rear_overhang;
        const float bottom = -info->rear_overhang;
        const float left = -info->width / 2.0;
        const float right = info->width / 2.0;

        const std::array<float, 4> lon_offset_vec{top, top, bottom, bottom};
        const std::array<float, 4> lat_offset_vec{left, right, right, left};

        for (int f_idx = 0; f_idx < 4; ++f_idx) {
          const Eigen::Quaternionf quat(
            path_point.pose.orientation.w, path_point.pose.orientation.x,
            path_point.pose.orientation.y, path_point.pose.orientation.z);

          {
            const Eigen::Vector3f offset_vec{
              lon_offset_vec.at(f_idx), lat_offset_vec.at(f_idx), 0.0};
            const auto offset_to_edge = quat * offset_vec;
            trajectory_footprint_manual_object_->position(
              path_point.pose.position.x + offset_to_edge.x(),
              path_point.pose.position.y + offset_to_edge.y(), path_point.pose.position.z);
            trajectory_footprint_manual_object_->colour(color);
          }
          {
            const Eigen::Vector3f offset_vec{
              lon_offset_vec.at((f_idx + 1) % 4), lat_offset_vec.at((f_idx + 1) % 4), 0.0};
            const auto offset_to_edge = quat * offset_vec;
            trajectory_footprint_manual_object_->position(
              path_point.pose.position.x + offset_to_edge.x(),
              path_point.pose.position.y + offset_to_edge.y(), path_point.pose.position.z);
            trajectory_footprint_manual_object_->colour(color);
          }
        }
      }

      /*
       * Point
       */
      if (property_trajectory_point_view_->getBool()) {
        Ogre::ColourValue color;
        color = rviz_common::properties::qtToOgre(property_trajectory_point_color_->getColor());
        color.a = property_trajectory_point_alpha_->getFloat();

        const double offset = property_trajectory_point_offset_->getFloat();
        const double yaw = tf2::getYaw(path_point.pose.orientation);
        const double base_x = path_point.pose.position.x + offset * std::cos(yaw);
        const double base_y = path_point.pose.position.y + offset * std::sin(yaw);
        const double base_z = path_point.pose.position.z;

        const double radius = property_trajectory_point_radius_->getFloat();
        for (size_t s_idx = 0; s_idx < 8; ++s_idx) {
          const double current_angle = static_cast<double>(s_idx) / 8.0 * 2.0 * M_PI;
          const double next_angle = static_cast<double>(s_idx + 1) / 8.0 * 2.0 * M_PI;
          trajectory_point_manual_object_->position(
            base_x + radius * std::cos(current_angle), base_y + radius * std::sin(current_angle),
            base_z);
          trajectory_point_manual_object_->colour(color);

          trajectory_point_manual_object_->position(
            base_x + radius * std::cos(next_angle), base_y + radius * std::sin(next_angle), base_z);
          trajectory_point_manual_object_->colour(color);

          trajectory_point_manual_object_->position(base_x, base_y, base_z);
          trajectory_point_manual_object_->colour(color);
        }
      }
    }

    trajectory_footprint_manual_object_->end();
    trajectory_point_manual_object_->end();
  }
  last_msg_ptr_ = msg_ptr;
}

void AutowareTrajectoryFootprintDisplay::updateVisualization()
{
  if (last_msg_ptr_ != nullptr) {
    processMessage(last_msg_ptr_);
  }
}

void AutowareTrajectoryFootprintDisplay::updateVehicleInfo()
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

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareTrajectoryFootprintDisplay, rviz_common::Display)
