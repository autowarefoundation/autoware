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

#ifndef PATH_FOOTPRINT__DISPLAY_BASE_HPP_
#define PATH_FOOTPRINT__DISPLAY_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <memory>

namespace rviz_plugins
{
using vehicle_info_util::VehicleInfo;
using vehicle_info_util::VehicleInfoUtil;

template <typename T>
class AutowarePathFootBaseprintDisplay : public rviz_common::MessageFilterDisplay<T>
{
public:
  AutowarePathFootBaseprintDisplay()
  :  // footprint
    property_footprint_view_{"View Footprint", true, "", this},
    property_footprint_alpha_{"Alpha", 1.0, "", &property_footprint_view_},
    property_footprint_color_{"Color", QColor(230, 230, 50), "", &property_footprint_view_},
    property_vehicle_length_{"Vehicle Length", 4.77, "", &property_footprint_view_},
    property_vehicle_width_{"Vehicle Width", 1.83, "", &property_footprint_view_},
    property_rear_overhang_{"Rear Overhang", 1.03, "", &property_footprint_view_},
    property_offset_{"Offset from BaseLink", 0.0, "", &property_footprint_view_},
    // point
    property_point_view_{"View Point", false, "", this},
    property_point_alpha_{"Alpha", 1.0, "", &property_point_view_},
    property_point_color_{"Color", QColor(0, 60, 255), "", &property_point_view_},
    property_point_radius_{"Radius", 0.1, "", &property_point_view_},
    property_point_offset_{"Offset", 0.0, "", &property_point_view_}
  {
    // initialize footprint
    property_footprint_alpha_.setMin(0.0);
    property_footprint_alpha_.setMax(1.0);
    property_vehicle_length_.setMin(0.0);
    property_vehicle_width_.setMin(0.0);
    property_rear_overhang_.setMin(0.0);
    // initialize point
    property_point_alpha_.setMin(0.0);
    property_point_alpha_.setMax(1.0);

    updateVehicleInfo();
  }

  virtual ~AutowarePathFootBaseprintDisplay()
  {
    if (this->initialized()) {
      this->scene_manager_->destroyManualObject(footprint_manual_object_);
      this->scene_manager_->destroyManualObject(point_manual_object_);
    }
  }

  void onInitialize() override
  {
    rviz_common::MessageFilterDisplay<T>::MFDClass::onInitialize();

    footprint_manual_object_ = this->scene_manager_->createManualObject();
    footprint_manual_object_->setDynamic(true);
    this->scene_node_->attachObject(footprint_manual_object_);

    point_manual_object_ = this->scene_manager_->createManualObject();
    point_manual_object_->setDynamic(true);
    this->scene_node_->attachObject(point_manual_object_);
  }

  void reset() override
  {
    rviz_common::MessageFilterDisplay<T>::MFDClass::reset();
    footprint_manual_object_->clear();
    point_manual_object_->clear();

    resetDetail();
  }

protected:
  virtual void resetDetail() {}
  virtual void preprocessMessageDetail([[maybe_unused]] const typename T::ConstSharedPtr msg_ptr) {}
  virtual void processMessageDetail(
    [[maybe_unused]] const typename T::ConstSharedPtr msg_ptr, [[maybe_unused]] const size_t p_idx)
  {
  }

  Ogre::ManualObject * footprint_manual_object_;
  rviz_common::properties::BoolProperty property_footprint_view_;
  rviz_common::properties::FloatProperty property_footprint_alpha_;
  rviz_common::properties::ColorProperty property_footprint_color_;
  rviz_common::properties::FloatProperty property_vehicle_length_;
  rviz_common::properties::FloatProperty property_vehicle_width_;
  rviz_common::properties::FloatProperty property_rear_overhang_;
  rviz_common::properties::FloatProperty property_offset_;

  Ogre::ManualObject * point_manual_object_;
  rviz_common::properties::BoolProperty property_point_view_;
  rviz_common::properties::FloatProperty property_point_alpha_;
  rviz_common::properties::ColorProperty property_point_color_;
  rviz_common::properties::FloatProperty property_point_radius_;
  rviz_common::properties::FloatProperty property_point_offset_;

  void updateVisualization()
  {
    if (last_msg_ptr_ != nullptr) {
      processMessage(last_msg_ptr_);
    }
  }

private:
  struct VehicleFootprintInfo
  {
    VehicleFootprintInfo(const float l, const float w, const float r)
    : length(l), width(w), rear_overhang(r)
    {
    }
    float length, width, rear_overhang;
  };

  std::shared_ptr<VehicleInfo> vehicle_info_;
  std::shared_ptr<VehicleFootprintInfo> vehicle_footprint_info_;

  void updateVehicleInfo()
  {
    if (vehicle_info_) {
      vehicle_footprint_info_ = std::make_shared<VehicleFootprintInfo>(
        vehicle_info_->vehicle_length_m, vehicle_info_->vehicle_width_m,
        vehicle_info_->rear_overhang_m);
    } else {
      const float length{property_vehicle_length_.getFloat()};
      const float width{property_vehicle_width_.getFloat()};
      const float rear_overhang{property_rear_overhang_.getFloat()};

      vehicle_footprint_info_ =
        std::make_shared<VehicleFootprintInfo>(length, width, rear_overhang);
    }
  }

  typename T::ConstSharedPtr last_msg_ptr_;
  bool validateFloats(const typename T::ConstSharedPtr & msg_ptr)
  {
    for (auto && point : msg_ptr->points) {
      if (!rviz_common::validateFloats(tier4_autoware_utils::getPose(point))) {
        return false;
      }
    }
    return true;
  }

  void processMessage(const typename T::ConstSharedPtr msg_ptr) override
  {
    if (!validateFloats(msg_ptr)) {
      this->setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message contained invalid floating point values (nans or infs)");
      return;
    }

    // This doesn't work in the constructor.
    if (!vehicle_info_) {
      try {
        vehicle_info_ = std::make_shared<VehicleInfo>(
          VehicleInfoUtil(*this->rviz_ros_node_.lock()->get_raw_node()).getVehicleInfo());
        updateVehicleInfo();
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          this->rviz_ros_node_.lock()->get_raw_node()->get_logger(),
          *this->rviz_ros_node_.lock()->get_raw_node()->get_clock(), 5000,
          "Failed to get vehicle_info: %s", e.what());
      }
    }

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!this->context_->getFrameManager()->getTransform(msg_ptr->header, position, orientation)) {
      RCLCPP_DEBUG(
        this->rviz_ros_node_.lock()->get_raw_node()->get_logger(),
        "Error transforming from frame '%s' to frame '%s'", msg_ptr->header.frame_id.c_str(),
        qPrintable(this->fixed_frame_));
    }

    this->scene_node_->setPosition(position);
    this->scene_node_->setOrientation(orientation);

    footprint_manual_object_->clear();
    point_manual_object_->clear();

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
      "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->setDepthWriteEnabled(false);

    if (!msg_ptr->points.empty()) {
      footprint_manual_object_->estimateVertexCount(msg_ptr->points.size() * 4 * 2);
      footprint_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

      point_manual_object_->estimateVertexCount(msg_ptr->points.size() * 3 * 8);
      point_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

      const float offset_from_baselink = property_offset_.getFloat();

      preprocessMessageDetail(msg_ptr);

      for (size_t p_idx = 0; p_idx < msg_ptr->points.size(); p_idx++) {
        const auto & point = msg_ptr->points.at(p_idx);
        const auto & pose = tier4_autoware_utils::getPose(point);
        // footprint
        if (property_footprint_view_.getBool()) {
          Ogre::ColourValue color;
          color = rviz_common::properties::qtToOgre(property_footprint_color_.getColor());
          color.a = property_footprint_alpha_.getFloat();

          const auto info = vehicle_footprint_info_;
          const float top = info->length - info->rear_overhang - offset_from_baselink;
          const float bottom = -info->rear_overhang + offset_from_baselink;
          const float left = -info->width / 2.0;
          const float right = info->width / 2.0;

          const std::array<float, 4> lon_offset_vec{top, top, bottom, bottom};
          const std::array<float, 4> lat_offset_vec{left, right, right, left};

          for (int f_idx = 0; f_idx < 4; ++f_idx) {
            const Eigen::Quaternionf quat(
              pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

            {
              const Eigen::Vector3f offset_vec{
                lon_offset_vec.at(f_idx), lat_offset_vec.at(f_idx), 0.0};
              const auto offset_to_edge = quat * offset_vec;
              footprint_manual_object_->position(
                pose.position.x + offset_to_edge.x(), pose.position.y + offset_to_edge.y(),
                pose.position.z);
              footprint_manual_object_->colour(color);
            }
            {
              const Eigen::Vector3f offset_vec{
                lon_offset_vec.at((f_idx + 1) % 4), lat_offset_vec.at((f_idx + 1) % 4), 0.0};
              const auto offset_to_edge = quat * offset_vec;
              footprint_manual_object_->position(
                pose.position.x + offset_to_edge.x(), pose.position.y + offset_to_edge.y(),
                pose.position.z);
              footprint_manual_object_->colour(color);
            }
          }
        }

        // point
        if (property_point_view_.getBool()) {
          Ogre::ColourValue color;
          color = rviz_common::properties::qtToOgre(property_point_color_.getColor());
          color.a = property_point_alpha_.getFloat();

          const double offset = property_point_offset_.getFloat();
          const double yaw = tf2::getYaw(pose.orientation);
          const double base_x = pose.position.x + offset * std::cos(yaw);
          const double base_y = pose.position.y + offset * std::sin(yaw);
          const double base_z = pose.position.z;

          const double radius = property_point_radius_.getFloat();
          for (size_t s_idx = 0; s_idx < 8; ++s_idx) {
            const double current_angle = static_cast<double>(s_idx) / 8.0 * 2.0 * M_PI;
            const double next_angle = static_cast<double>(s_idx + 1) / 8.0 * 2.0 * M_PI;
            point_manual_object_->position(
              base_x + radius * std::cos(current_angle), base_y + radius * std::sin(current_angle),
              base_z);
            point_manual_object_->colour(color);

            point_manual_object_->position(
              base_x + radius * std::cos(next_angle), base_y + radius * std::sin(next_angle),
              base_z);
            point_manual_object_->colour(color);

            point_manual_object_->position(base_x, base_y, base_z);
            point_manual_object_->colour(color);
          }
        }

        processMessageDetail(msg_ptr, p_idx);
      }

      footprint_manual_object_->end();
      point_manual_object_->end();
    }
    last_msg_ptr_ = msg_ptr;
  }
};
}  // namespace rviz_plugins

#endif  // PATH_FOOTPRINT__DISPLAY_BASE_HPP_
