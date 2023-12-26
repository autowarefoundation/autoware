// Copyright 2023 TIER IV, Inc.
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

#ifndef TIER4_PLANNING_RVIZ_PLUGIN__PATH__DISPLAY_BASE_HPP_
#define TIER4_PLANNING_RVIZ_PLUGIN__PATH__DISPLAY_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/movable_text.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <deque>
#include <memory>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "tier4_planning_rviz_plugin/utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace
{
std::unique_ptr<Ogre::ColourValue> gradation(
  const QColor & color_min, const QColor & color_max, const double ratio)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g =
    static_cast<float>(color_max.greenF() * ratio + color_min.greenF() * (1.0 - ratio));
  color_ptr->r = static_cast<float>(color_max.redF() * ratio + color_min.redF() * (1.0 - ratio));
  color_ptr->b = static_cast<float>(color_max.blueF() * ratio + color_min.blueF() * (1.0 - ratio));

  return color_ptr;
}

std::unique_ptr<Ogre::ColourValue> setColorDependsOnVelocity(
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

template <typename T>
bool validateFloats(const typename T::ConstSharedPtr & msg_ptr)
{
  for (auto && path_point : msg_ptr->points) {
    if (
      !rviz_common::validateFloats(tier4_autoware_utils::getPose(path_point)) &&
      !rviz_common::validateFloats(tier4_autoware_utils::getLongitudinalVelocity(path_point))) {
      return false;
    }
  }
  return true;
}
}  // namespace

namespace rviz_plugins
{
using vehicle_info_util::VehicleInfo;
using vehicle_info_util::VehicleInfoUtil;
template <typename T>
class AutowarePathBaseDisplay : public rviz_common::MessageFilterDisplay<T>
{
public:
  AutowarePathBaseDisplay()
  :  // path
    property_path_view_{"View Path", true, "", this},
    property_path_width_view_{"Constant Width", false, "", &property_path_view_},
    property_path_width_{"Width", 2.0, "", &property_path_view_},
    property_path_alpha_{"Alpha", 1.0, "", &property_path_view_},
    property_path_color_view_{"Constant Color", false, "", &property_path_view_},
    property_path_color_{"Color", Qt::black, "", &property_path_view_},
    property_vel_max_{"Color Border Vel Max", 3.0, "[m/s]", this},
    // velocity
    property_velocity_view_{"View Velocity", true, "", this},
    property_velocity_alpha_{"Alpha", 1.0, "", &property_velocity_view_},
    property_velocity_scale_{"Scale", 0.3, "", &property_velocity_view_},
    property_velocity_color_view_{"Constant Color", false, "", &property_velocity_view_},
    property_velocity_color_{"Color", Qt::black, "", &property_velocity_view_},
    // velocity text
    property_velocity_text_view_{"View Text Velocity", false, "", this},
    property_velocity_text_scale_{"Scale", 0.3, "", &property_velocity_text_view_},
    // footprint
    property_footprint_view_{"View Footprint", false, "", this},
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
    property_point_offset_{"Offset", 0.0, "", &property_point_view_},
    // slope
    property_slope_text_view_{"View Text Slope", false, "", this},
    property_slope_text_scale_{"Scale", 0.3, "", &property_slope_text_view_}
  {
    // path
    property_path_width_.setMin(0.0);
    property_path_alpha_.setMin(0.0);
    property_path_alpha_.setMax(1.0);
    property_vel_max_.setMin(0.0);
    // velocity
    property_velocity_alpha_.setMin(0.0);
    property_velocity_alpha_.setMax(1.0);
    // velocity text
    property_velocity_scale_.setMin(0.1);
    property_velocity_scale_.setMax(10.0);

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

  ~AutowarePathBaseDisplay() override
  {
    if (this->initialized()) {
      this->scene_manager_->destroyManualObject(path_manual_object_);
      this->scene_manager_->destroyManualObject(velocity_manual_object_);
      for (size_t i = 0; i < velocity_text_nodes_.size(); i++) {
        Ogre::SceneNode * node = velocity_text_nodes_.at(i);
        node->removeAndDestroyAllChildren();
        node->detachAllObjects();
        this->scene_manager_->destroySceneNode(node);
      }
      for (size_t i = 0; i < slope_text_nodes_.size(); i++) {
        Ogre::SceneNode * node = slope_text_nodes_.at(i);
        node->removeAndDestroyAllChildren();
        node->detachAllObjects();
        this->scene_manager_->destroySceneNode(node);
      }
      this->scene_manager_->destroyManualObject(footprint_manual_object_);
      this->scene_manager_->destroyManualObject(point_manual_object_);
    }
  }
  void onInitialize() override
  {
    rviz_common::MessageFilterDisplay<T>::MFDClass::onInitialize();

    path_manual_object_ = this->scene_manager_->createManualObject();
    velocity_manual_object_ = this->scene_manager_->createManualObject();
    footprint_manual_object_ = this->scene_manager_->createManualObject();
    point_manual_object_ = this->scene_manager_->createManualObject();
    path_manual_object_->setDynamic(true);
    velocity_manual_object_->setDynamic(true);
    footprint_manual_object_->setDynamic(true);
    point_manual_object_->setDynamic(true);
    this->scene_node_->attachObject(path_manual_object_);
    this->scene_node_->attachObject(velocity_manual_object_);
    this->scene_node_->attachObject(footprint_manual_object_);
    this->scene_node_->attachObject(point_manual_object_);
  }

  void reset() override
  {
    rviz_common::MessageFilterDisplay<T>::MFDClass::reset();
    path_manual_object_->clear();
    velocity_manual_object_->clear();

    for (size_t i = 0; i < velocity_texts_.size(); i++) {
      Ogre::SceneNode * node = velocity_text_nodes_.at(i);
      node->detachAllObjects();
      node->removeAndDestroyAllChildren();
      this->scene_manager_->destroySceneNode(node);
    }
    velocity_text_nodes_.clear();
    velocity_texts_.clear();

    for (size_t i = 0; i < slope_texts_.size(); i++) {
      Ogre::SceneNode * node = slope_text_nodes_.at(i);
      node->detachAllObjects();
      node->removeAndDestroyAllChildren();
      this->scene_manager_->destroySceneNode(node);
    }
    slope_text_nodes_.clear();
    slope_texts_.clear();

    footprint_manual_object_->clear();
    point_manual_object_->clear();
  }

protected:
  virtual void visualizeDrivableArea([[maybe_unused]] const typename T::ConstSharedPtr msg_ptr) {}
  virtual void preProcessMessageDetail() {}
  virtual void preVisualizePathFootprintDetail(
    [[maybe_unused]] const typename T::ConstSharedPtr msg_ptr)
  {
  }
  virtual void visualizePathFootprintDetail(
    [[maybe_unused]] const typename T::ConstSharedPtr msg_ptr, [[maybe_unused]] const size_t p_idx)
  {
  }

  void processMessage(const typename T::ConstSharedPtr msg_ptr) override
  {
    path_manual_object_->clear();
    velocity_manual_object_->clear();
    footprint_manual_object_->clear();
    point_manual_object_->clear();

    if (!validateFloats<T>(msg_ptr)) {
      this->setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        "Message contained invalid floating point values (nans or infs)");
      return;
    }

    preProcessMessageDetail();

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!this->context_->getFrameManager()->getTransform(msg_ptr->header, position, orientation)) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("AutowarePathBaseDisplay"),
        "Error transforming from frame '%s' to frame '%s'", msg_ptr->header.frame_id.c_str(),
        qPrintable(this->fixed_frame_));
    }
    this->scene_node_->setPosition(position);
    this->scene_node_->setOrientation(orientation);

    // visualize Path
    visualizePath(msg_ptr);

    // visualize drivable area
    visualizeDrivableArea(msg_ptr);

    // visualize PathFootprint
    visualizePathFootprint(msg_ptr);

    last_msg_ptr_ = msg_ptr;
  }

  void visualizePath(const typename T::ConstSharedPtr msg_ptr)
  {
    if (msg_ptr->points.empty()) {
      return;
    }

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
      "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->setDepthWriteEnabled(false);

    path_manual_object_->estimateVertexCount(msg_ptr->points.size() * 2);
    velocity_manual_object_->estimateVertexCount(msg_ptr->points.size());
    path_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    velocity_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    if (msg_ptr->points.size() > velocity_texts_.size()) {
      for (size_t i = velocity_texts_.size(); i < msg_ptr->points.size(); i++) {
        Ogre::SceneNode * node = this->scene_node_->createChildSceneNode();
        rviz_rendering::MovableText * text =
          new rviz_rendering::MovableText("not initialized", "Liberation Sans", 0.1);
        text->setVisible(false);
        text->setTextAlignment(
          rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);
        node->attachObject(text);
        velocity_texts_.push_back(text);
        velocity_text_nodes_.push_back(node);
      }
    } else if (msg_ptr->points.size() < velocity_texts_.size()) {
      for (size_t i = velocity_texts_.size() - 1; i >= msg_ptr->points.size(); i--) {
        Ogre::SceneNode * node = velocity_text_nodes_.at(i);
        node->detachAllObjects();
        node->removeAndDestroyAllChildren();
        this->scene_manager_->destroySceneNode(node);
      }
      velocity_texts_.resize(msg_ptr->points.size());
      velocity_text_nodes_.resize(msg_ptr->points.size());
    }

    if (msg_ptr->points.size() > slope_texts_.size()) {
      for (size_t i = slope_texts_.size(); i < msg_ptr->points.size(); i++) {
        Ogre::SceneNode * node = this->scene_node_->createChildSceneNode();
        rviz_rendering::MovableText * text =
          new rviz_rendering::MovableText("not initialized", "Liberation Sans", 0.1);
        text->setVisible(false);
        text->setTextAlignment(
          rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_ABOVE);
        node->attachObject(text);
        slope_texts_.push_back(text);
        slope_text_nodes_.push_back(node);
      }
    } else if (msg_ptr->points.size() < slope_texts_.size()) {
      for (size_t i = slope_texts_.size() - 1; i >= msg_ptr->points.size(); i--) {
        Ogre::SceneNode * node = slope_text_nodes_.at(i);
        node->detachAllObjects();
        node->removeAndDestroyAllChildren();
        this->scene_manager_->destroySceneNode(node);
      }
      slope_texts_.resize(msg_ptr->points.size());
      slope_text_nodes_.resize(msg_ptr->points.size());
    }

    const auto info = vehicle_footprint_info_;
    const float left = property_path_width_view_.getBool() ? -property_path_width_.getFloat() / 2.0
                                                           : -info->width / 2.0;
    const float right = property_path_width_view_.getBool() ? property_path_width_.getFloat() / 2.0
                                                            : info->width / 2.0;

    for (size_t point_idx = 0; point_idx < msg_ptr->points.size(); point_idx++) {
      const auto & path_point = msg_ptr->points.at(point_idx);
      const auto & pose = tier4_autoware_utils::getPose(path_point);
      const auto & velocity = tier4_autoware_utils::getLongitudinalVelocity(path_point);

      // path
      if (property_path_view_.getBool()) {
        Ogre::ColourValue color;
        if (property_path_color_view_.getBool()) {
          color = rviz_common::properties::qtToOgre(property_path_color_.getColor());
        } else {
          // color change depending on velocity
          std::unique_ptr<Ogre::ColourValue> dynamic_color_ptr =
            setColorDependsOnVelocity(property_vel_max_.getFloat(), velocity);
          color = *dynamic_color_ptr;
        }
        color.a = property_path_alpha_.getFloat();
        Eigen::Vector3f vec_in;
        Eigen::Vector3f vec_out;
        Eigen::Quaternionf quat_yaw_reverse(0, 0, 0, 1);
        {
          vec_in << 0, right, 0;
          Eigen::Quaternionf quat(
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
          if (!isDrivingForward(msg_ptr->points, point_idx)) {
            quat *= quat_yaw_reverse;
          }
          vec_out = quat * vec_in;
          path_manual_object_->position(
            static_cast<float>(pose.position.x) + vec_out.x(),
            static_cast<float>(pose.position.y) + vec_out.y(),
            static_cast<float>(pose.position.z) + vec_out.z());
          path_manual_object_->colour(color);
        }
        {
          vec_in << 0, left, 0;
          Eigen::Quaternionf quat(
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
          if (!isDrivingForward(msg_ptr->points, point_idx)) {
            quat *= quat_yaw_reverse;
          }
          vec_out = quat * vec_in;
          path_manual_object_->position(
            static_cast<float>(pose.position.x) + vec_out.x(),
            static_cast<float>(pose.position.y) + vec_out.y(),
            static_cast<float>(pose.position.z) + vec_out.z());
          path_manual_object_->colour(color);
        }
      }

      // velocity
      if (property_velocity_view_.getBool()) {
        Ogre::ColourValue color;
        if (property_velocity_color_view_.getBool()) {
          color = rviz_common::properties::qtToOgre(property_velocity_color_.getColor());
        } else {
          /* color change depending on velocity */
          std::unique_ptr<Ogre::ColourValue> dynamic_color_ptr =
            setColorDependsOnVelocity(property_vel_max_.getFloat(), velocity);
          color = *dynamic_color_ptr;
        }
        color.a = property_velocity_alpha_.getFloat();

        velocity_manual_object_->position(
          pose.position.x, pose.position.y,
          static_cast<float>(pose.position.z) + velocity * property_velocity_scale_.getFloat());
        velocity_manual_object_->colour(color);
      }

      // velocity text
      if (property_velocity_text_view_.getBool()) {
        Ogre::Vector3 position;
        position.x = pose.position.x;
        position.y = pose.position.y;
        position.z = pose.position.z;
        Ogre::SceneNode * node = velocity_text_nodes_.at(point_idx);
        node->setPosition(position);

        rviz_rendering::MovableText * text = velocity_texts_.at(point_idx);
        const double vel = velocity;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << vel;
        text->setCaption(ss.str());
        text->setCharacterHeight(property_velocity_text_scale_.getFloat());
        text->setVisible(true);
      } else {
        rviz_rendering::MovableText * text = velocity_texts_.at(point_idx);
        text->setVisible(false);
      }

      // slope text
      if (property_slope_text_view_.getBool() && 1 < msg_ptr->points.size()) {
        const size_t prev_idx =
          (point_idx != msg_ptr->points.size() - 1) ? point_idx : point_idx - 1;
        const size_t next_idx =
          (point_idx != msg_ptr->points.size() - 1) ? point_idx + 1 : point_idx;

        const auto & prev_path_pos =
          tier4_autoware_utils::getPose(msg_ptr->points.at(prev_idx)).position;
        const auto & next_path_pos =
          tier4_autoware_utils::getPose(msg_ptr->points.at(next_idx)).position;

        Ogre::Vector3 position;
        position.x = pose.position.x;
        position.y = pose.position.y;
        position.z = pose.position.z;
        Ogre::SceneNode * node = slope_text_nodes_.at(point_idx);
        node->setPosition(position);

        rviz_rendering::MovableText * text = slope_texts_.at(point_idx);
        const double slope = tier4_autoware_utils::calcElevationAngle(prev_path_pos, next_path_pos);

        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << slope;
        text->setCaption(ss.str());
        text->setCharacterHeight(property_slope_text_scale_.getFloat());
        text->setVisible(true);
      } else {
        rviz_rendering::MovableText * text = slope_texts_.at(point_idx);
        text->setVisible(false);
      }
    }

    path_manual_object_->end();
    velocity_manual_object_->end();
  }

  void visualizePathFootprint(const typename T::ConstSharedPtr msg_ptr)
  {
    if (msg_ptr->points.empty()) {
      return;
    }

    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
      "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material->setDepthWriteEnabled(false);

    footprint_manual_object_->estimateVertexCount(msg_ptr->points.size() * 4 * 2);
    footprint_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
    point_manual_object_->estimateVertexCount(msg_ptr->points.size() * 3 * 8);
    point_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);

    preVisualizePathFootprintDetail(msg_ptr);

    const float offset_from_baselink = property_offset_.getFloat();
    const auto info = vehicle_footprint_info_;
    const float top = info->length - info->rear_overhang - offset_from_baselink;
    const float bottom = -info->rear_overhang + offset_from_baselink;
    const float left = -info->width / 2.0;
    const float right = info->width / 2.0;

    for (size_t p_idx = 0; p_idx < msg_ptr->points.size(); p_idx++) {
      const auto & point = msg_ptr->points.at(p_idx);
      const auto & pose = tier4_autoware_utils::getPose(point);
      // footprint
      if (property_footprint_view_.getBool()) {
        Ogre::ColourValue color;
        color = rviz_common::properties::qtToOgre(property_footprint_color_.getColor());
        color.a = property_footprint_alpha_.getFloat();

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
            base_x + radius * std::cos(next_angle), base_y + radius * std::sin(next_angle), base_z);
          point_manual_object_->colour(color);

          point_manual_object_->position(base_x, base_y, base_z);
          point_manual_object_->colour(color);
        }
      }

      visualizePathFootprintDetail(msg_ptr, p_idx);
    }

    footprint_manual_object_->end();
    point_manual_object_->end();
  }

  void updateVehicleInfo()
  {
    if (vehicle_info_) {
      vehicle_footprint_info_ = std::make_shared<VehicleFootprintInfo>(
        vehicle_info_->vehicle_length_m, vehicle_info_->vehicle_width_m,
        vehicle_info_->rear_overhang_m);

      property_vehicle_length_.setValue(vehicle_info_->vehicle_length_m);
      property_vehicle_width_.setValue(vehicle_info_->vehicle_width_m);
      property_rear_overhang_.setValue(vehicle_info_->rear_overhang_m);
    } else {
      const float length{property_vehicle_length_.getFloat()};
      const float width{property_vehicle_width_.getFloat()};
      const float rear_overhang{property_rear_overhang_.getFloat()};

      vehicle_footprint_info_ =
        std::make_shared<VehicleFootprintInfo>(length, width, rear_overhang);
    }
  }

  Ogre::ManualObject * path_manual_object_{nullptr};
  Ogre::ManualObject * velocity_manual_object_{nullptr};
  Ogre::ManualObject * footprint_manual_object_{nullptr};
  Ogre::ManualObject * point_manual_object_{nullptr};
  std::vector<rviz_rendering::MovableText *> velocity_texts_;
  std::vector<Ogre::SceneNode *> velocity_text_nodes_;
  std::vector<rviz_rendering::MovableText *> slope_texts_;
  std::vector<Ogre::SceneNode *> slope_text_nodes_;

  rviz_common::properties::BoolProperty property_path_view_;
  rviz_common::properties::BoolProperty property_path_width_view_;
  rviz_common::properties::FloatProperty property_path_width_;
  rviz_common::properties::FloatProperty property_path_alpha_;
  rviz_common::properties::BoolProperty property_path_color_view_;
  rviz_common::properties::ColorProperty property_path_color_;
  rviz_common::properties::FloatProperty property_vel_max_;
  rviz_common::properties::BoolProperty property_velocity_view_;
  rviz_common::properties::FloatProperty property_velocity_alpha_;
  rviz_common::properties::FloatProperty property_velocity_scale_;
  rviz_common::properties::BoolProperty property_velocity_color_view_;
  rviz_common::properties::ColorProperty property_velocity_color_;
  rviz_common::properties::BoolProperty property_velocity_text_view_;
  rviz_common::properties::FloatProperty property_velocity_text_scale_;

  rviz_common::properties::BoolProperty property_footprint_view_;
  rviz_common::properties::FloatProperty property_footprint_alpha_;
  rviz_common::properties::ColorProperty property_footprint_color_;
  rviz_common::properties::FloatProperty property_vehicle_length_;
  rviz_common::properties::FloatProperty property_vehicle_width_;
  rviz_common::properties::FloatProperty property_rear_overhang_;
  rviz_common::properties::FloatProperty property_offset_;

  rviz_common::properties::BoolProperty property_point_view_;
  rviz_common::properties::FloatProperty property_point_alpha_;
  rviz_common::properties::ColorProperty property_point_color_;
  rviz_common::properties::FloatProperty property_point_radius_;
  rviz_common::properties::FloatProperty property_point_offset_;

  rviz_common::properties::BoolProperty property_slope_text_view_;
  rviz_common::properties::FloatProperty property_slope_text_scale_;

  std::shared_ptr<VehicleInfo> vehicle_info_;

private:
  typename T::ConstSharedPtr last_msg_ptr_;

  struct VehicleFootprintInfo
  {
    VehicleFootprintInfo(const float l, const float w, const float r)
    : length(l), width(w), rear_overhang(r)
    {
    }
    float length, width, rear_overhang;
  };

  std::shared_ptr<VehicleFootprintInfo> vehicle_footprint_info_;
};
}  // namespace rviz_plugins

#endif  // TIER4_PLANNING_RVIZ_PLUGIN__PATH__DISPLAY_BASE_HPP_
