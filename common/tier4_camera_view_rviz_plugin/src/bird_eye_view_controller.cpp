// Copyright 2023 Autoware Foundation
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
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "bird_eye_view_controller.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_rendering/orthographic.hpp"

#include <OgreCamera.h>
#include <OgreQuaternion.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreViewport.h>

namespace tier4_camera_view_rviz_plugin
{
static const Ogre::Quaternion ROBOT_TO_CAMERA_ROTATION =
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y) *
  Ogre::Quaternion(Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

static const float PITCH_LIMIT_LOW = -Ogre::Math::HALF_PI + 0.001;
static const float PITCH_LIMIT_HIGH = Ogre::Math::HALF_PI - 0.001;

BirdEyeViewController::BirdEyeViewController() : dragging_(false)
{
  scale_property_ = new rviz_common::properties::FloatProperty(
    "Scale", 10, "How much to scale up the size of things in the scene.", this);
  angle_property_ = new rviz_common::properties::FloatProperty(
    "Angle", 0, "Angle around the Z axis to rotate.", this);
  x_property_ =
    new rviz_common::properties::FloatProperty("X", 0, "X component of camera position.", this);
  y_property_ =
    new rviz_common::properties::FloatProperty("Y", 0, "Y component of camera position.", this);
}

BirdEyeViewController::~BirdEyeViewController()
{
}

void BirdEyeViewController::onInitialize()
{
  FramePositionTrackingViewController::onInitialize();

  camera_->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  auto camera_parent = getCameraParent(camera_);
  camera_parent->setFixedYawAxis(false);
  invert_z_->hide();
}

void BirdEyeViewController::reset()
{
  scale_property_->setFloat(10);
  angle_property_->setFloat(0);
  x_property_->setFloat(0);
  y_property_->setFloat(0);
}

void BirdEyeViewController::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.shift()) {
    setStatus("<b>Left-Click:</b> Move X/Y.");
  } else {
    setStatus(
      "<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.  "
      "<b>Shift</b>: More options.");
  }

  bool moved = false;

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  if (event.type == QEvent::MouseButtonPress) {
    dragging_ = true;
  } else if (event.type == QEvent::MouseButtonRelease) {
    dragging_ = false;
  } else if (dragging_ && event.type == QEvent::MouseMove) {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  if (event.left() && !event.shift()) {
    setCursor(Rotate2D);
    angle_property_->add(diff_x * 0.005);
    orientCamera();
  } else if (event.middle() || (event.shift() && event.left())) {
    setCursor(MoveXY);
    float scale = scale_property_->getFloat();
    move_camera(-diff_x / scale, diff_y / scale);
  } else if (event.right()) {
    setCursor(Zoom);
    scale_property_->multiply(1.0 - diff_y * 0.01);
  } else {
    setCursor(event.shift() ? MoveXY : Rotate2D);
  }

  if (event.wheel_delta != 0) {
    int diff = event.wheel_delta;
    scale_property_->multiply(1.0 - (-diff) * 0.001);

    moved = true;
  }

  if (moved) {
    context_->queueRender();
    emitConfigChanged();
  }
}

void BirdEyeViewController::orientCamera()
{
  camera_->setOrientation(
    Ogre::Quaternion(Ogre::Radian(angle_property_->getFloat()), Ogre::Vector3::UNIT_Z));
}

void BirdEyeViewController::mimic(rviz_common::ViewController * source_view)
{
  FramePositionTrackingViewController::mimic(source_view);

  if (BirdEyeViewController * source_ortho = qobject_cast<BirdEyeViewController *>(source_view)) {
    scale_property_->setFloat(source_ortho->scale_property_->getFloat());
    angle_property_->setFloat(source_ortho->angle_property_->getFloat());
    x_property_->setFloat(source_ortho->x_property_->getFloat());
    y_property_->setFloat(source_ortho->y_property_->getFloat());
  } else {
    auto source_camera_parent = getCameraParent(source_view->getCamera());
    setPosition(source_camera_parent->getPosition());
  }
}
void BirdEyeViewController::update(float dt, float ros_dt)
{
  FramePositionTrackingViewController::update(dt, ros_dt);
  updateCamera();
}

void BirdEyeViewController::lookAt(const Ogre::Vector3 & point)
{
  setPosition(point - target_scene_node_->getPosition());
}

void BirdEyeViewController::onTargetFrameChanged(
  const Ogre::Vector3 & old_reference_position,
  const Ogre::Quaternion & /*old_reference_orientation*/)
{
  move_camera(
    old_reference_position.x - reference_position_.x,
    old_reference_position.y - reference_position_.y);
}

void BirdEyeViewController::updateCamera()
{
  orientCamera();

  float width = camera_->getViewport()->getActualWidth();
  float height = camera_->getViewport()->getActualHeight();

  float scale = scale_property_->getFloat();
  Ogre::Matrix4 proj = rviz_rendering::buildScaledOrthoMatrix(
    -width / scale / 2, width / scale / 2, -height / scale / 2, height / scale / 2,
    camera_->getNearClipDistance(), camera_->getFarClipDistance());
  camera_->setCustomProjectionMatrix(true, proj);

  // For Z, we use half of the far-clip distance set in
  // selection_context.cpp, so that the shader program which computes
  // depth can see equal distances above and below the Z=0 plane.
  auto camera_parent = getCameraParent(camera_);
  camera_parent->setPosition(x_property_->getFloat(), y_property_->getFloat(), 500);
}

Ogre::SceneNode * BirdEyeViewController::getCameraParent(Ogre::Camera * camera)
{
  auto camera_parent = camera->getParentSceneNode();

  if (!camera_parent) {
    throw std::runtime_error("camera's parent scene node pointer unexpectedly nullptr");
  }
  return camera_parent;
}

void BirdEyeViewController::setPosition(const Ogre::Vector3 & pos_rel_target)
{
  x_property_->setFloat(pos_rel_target.x);
  y_property_->setFloat(pos_rel_target.y);
}

void BirdEyeViewController::move_camera(float dx, float dy)
{
  float angle = angle_property_->getFloat();
  x_property_->add(dx * cos(angle) - dy * sin(angle));
  y_property_->add(dx * sin(angle) + dy * cos(angle));
}

}  // namespace tier4_camera_view_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_camera_view_rviz_plugin::BirdEyeViewController, rviz_common::ViewController)
