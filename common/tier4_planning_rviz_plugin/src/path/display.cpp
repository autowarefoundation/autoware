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

#include <path/display.hpp>
#include <utils.hpp>

#include <memory>
#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace rviz_plugins
{
std::unique_ptr<Ogre::ColourValue> AutowarePathDisplay::gradation(
  const QColor & color_min, const QColor & color_max, const double ratio)
{
  std::unique_ptr<Ogre::ColourValue> color_ptr(new Ogre::ColourValue);
  color_ptr->g =
    static_cast<float>(color_max.greenF() * ratio + color_min.greenF() * (1.0 - ratio));
  color_ptr->r = static_cast<float>(color_max.redF() * ratio + color_min.redF() * (1.0 - ratio));
  color_ptr->b = static_cast<float>(color_max.blueF() * ratio + color_min.blueF() * (1.0 - ratio));

  return color_ptr;
}

std::unique_ptr<Ogre::ColourValue> AutowarePathDisplay::setColorDependsOnVelocity(
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

AutowarePathDisplay::AutowarePathDisplay()
{
  property_path_view_ = new rviz_common::properties::BoolProperty(
    "View Path", true, "", this, SLOT(updateVisualization()));
  property_path_width_ = new rviz_common::properties::FloatProperty(
    "Width", 2.0, "", property_path_view_, SLOT(updateVisualization()), this);
  property_path_width_->setMin(0.0);
  property_path_alpha_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "", property_path_view_, SLOT(updateVisualization()), this);
  property_path_alpha_->setMin(0.0);
  property_path_alpha_->setMax(1.0);
  property_path_color_view_ = new rviz_common::properties::BoolProperty(
    "Constant Color", false, "", property_path_view_, SLOT(updateVisualization()), this);
  property_path_color_ = new rviz_common::properties::ColorProperty(
    "Color", Qt::black, "", property_path_view_, SLOT(updateVisualization()), this);

  property_velocity_view_ = new rviz_common::properties::BoolProperty(
    "View Velocity", true, "", this, SLOT(updateVisualization()), this);
  property_velocity_alpha_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0, "", property_velocity_view_, SLOT(updateVisualization()), this);
  property_velocity_alpha_->setMin(0.0);
  property_velocity_alpha_->setMax(1.0);
  property_velocity_scale_ = new rviz_common::properties::FloatProperty(
    "Scale", 0.3, "", property_velocity_view_, SLOT(updateVisualization()), this);
  property_velocity_scale_->setMin(0.1);
  property_velocity_scale_->setMax(10.0);
  property_velocity_color_view_ = new rviz_common::properties::BoolProperty(
    "Constant Color", false, "", property_velocity_view_, SLOT(updateVisualization()), this);
  property_velocity_color_ = new rviz_common::properties::ColorProperty(
    "Color", Qt::black, "", property_velocity_view_, SLOT(updateVisualization()), this);

  property_vel_max_ = new rviz_common::properties::FloatProperty(
    "Color Border Vel Max", 3.0, "[m/s]", this, SLOT(updateVisualization()), this);
  property_vel_max_->setMin(0.0);
}

AutowarePathDisplay::~AutowarePathDisplay()
{
  if (initialized()) {
    scene_manager_->destroyManualObject(path_manual_object_);
    scene_manager_->destroyManualObject(velocity_manual_object_);
  }
}

void AutowarePathDisplay::onInitialize()
{
  MFDClass::onInitialize();

  path_manual_object_ = scene_manager_->createManualObject();
  velocity_manual_object_ = scene_manager_->createManualObject();
  path_manual_object_->setDynamic(true);
  velocity_manual_object_->setDynamic(true);
  scene_node_->attachObject(path_manual_object_);
  scene_node_->attachObject(velocity_manual_object_);
}

void AutowarePathDisplay::reset()
{
  MFDClass::reset();
  path_manual_object_->clear();
  velocity_manual_object_->clear();
}

bool AutowarePathDisplay::validateFloats(
  const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr & msg_ptr)
{
  for (auto && path_point : msg_ptr->points) {
    if (
      !rviz_common::validateFloats(path_point.pose) &&
      !rviz_common::validateFloats(path_point.longitudinal_velocity_mps)) {
      return false;
    }
  }
  return true;
}

void AutowarePathDisplay::processMessage(
  const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr msg_ptr)
{
  if (!validateFloats(msg_ptr)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg_ptr->header, position, orientation)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("AutowarePathDisplay"), "Error transforming from frame '%s' to frame '%s'",
      msg_ptr->header.frame_id.c_str(), qPrintable(fixed_frame_));
  }

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  path_manual_object_->clear();
  velocity_manual_object_->clear();

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().getByName(
    "BaseWhiteNoLighting", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setDepthWriteEnabled(false);

  if (!msg_ptr->points.empty()) {
    path_manual_object_->estimateVertexCount(msg_ptr->points.size() * 2);
    velocity_manual_object_->estimateVertexCount(msg_ptr->points.size());
    path_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    // path_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    velocity_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);

    for (size_t point_idx = 0; point_idx < msg_ptr->points.size(); point_idx++) {
      const auto & path_point = msg_ptr->points.at(point_idx);
      /*
       * Path
       */
      if (property_path_view_->getBool()) {
        Ogre::ColourValue color;
        if (property_path_color_view_->getBool()) {
          color = rviz_common::properties::qtToOgre(property_path_color_->getColor());
        } else {
          /* color change depending on velocity */
          std::unique_ptr<Ogre::ColourValue> dynamic_color_ptr = setColorDependsOnVelocity(
            property_vel_max_->getFloat(), path_point.longitudinal_velocity_mps);
          color = *dynamic_color_ptr;
        }
        color.a = property_path_alpha_->getFloat();
        Eigen::Vector3f vec_in;
        Eigen::Vector3f vec_out;
        Eigen::Quaternionf quat_yaw_reverse(0, 0, 0, 1);
        {
          vec_in << 0, (property_path_width_->getFloat() / 2.0), 0;
          Eigen::Quaternionf quat(
            path_point.pose.orientation.w, path_point.pose.orientation.x,
            path_point.pose.orientation.y, path_point.pose.orientation.z);
          if (!isDrivingForward(msg_ptr->points, point_idx)) {
            quat *= quat_yaw_reverse;
          }
          vec_out = quat * vec_in;
          path_manual_object_->position(
            static_cast<float>(path_point.pose.position.x) + vec_out.x(),
            static_cast<float>(path_point.pose.position.y) + vec_out.y(),
            static_cast<float>(path_point.pose.position.z) + vec_out.z());
          path_manual_object_->colour(color);
        }
        {
          vec_in << 0, -(property_path_width_->getFloat() / 2.0), 0;
          Eigen::Quaternionf quat(
            path_point.pose.orientation.w, path_point.pose.orientation.x,
            path_point.pose.orientation.y, path_point.pose.orientation.z);
          if (!isDrivingForward(msg_ptr->points, point_idx)) {
            quat *= quat_yaw_reverse;
          }
          vec_out = quat * vec_in;
          path_manual_object_->position(
            static_cast<float>(path_point.pose.position.x) + vec_out.x(),
            static_cast<float>(path_point.pose.position.y) + vec_out.y(),
            static_cast<float>(path_point.pose.position.z) + vec_out.z());
          path_manual_object_->colour(color);
        }
      }
      /*
       * Velocity
       */
      if (property_velocity_view_->getBool()) {
        Ogre::ColourValue color;
        if (property_velocity_color_view_->getBool()) {
          color = rviz_common::properties::qtToOgre(property_velocity_color_->getColor());
        } else {
          /* color change depending on velocity */
          std::unique_ptr<Ogre::ColourValue> dynamic_color_ptr = setColorDependsOnVelocity(
            property_vel_max_->getFloat(), path_point.longitudinal_velocity_mps);
          color = *dynamic_color_ptr;
        }
        color.a = property_velocity_alpha_->getFloat();

        velocity_manual_object_->position(
          path_point.pose.position.x, path_point.pose.position.y,
          static_cast<float>(path_point.pose.position.z) +
            path_point.longitudinal_velocity_mps * property_velocity_scale_->getFloat());
        velocity_manual_object_->colour(color);
      }
    }

    path_manual_object_->end();
    velocity_manual_object_->end();
  }
  last_msg_ptr_ = msg_ptr;
}

void AutowarePathDisplay::updateVisualization()
{
  if (last_msg_ptr_ != nullptr) {
    processMessage(last_msg_ptr_);
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowarePathDisplay, rviz_common::Display)
