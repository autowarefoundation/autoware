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

#include "third_person_view_controller.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/shape.hpp"

#include <OgreCamera.h>
#include <OgreMath.h>
#include <OgrePlane.h>
#include <OgreQuaternion.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreViewport.h>
#include <stdint.h>

#include <utility>

namespace tier4_camera_view_rviz_plugin
{
static const float PITCH_START = Ogre::Math::HALF_PI / 3;
static const float YAW_START = Ogre::Math::PI;
static const float DISTANCE_START = 30;
static const float FOCAL_SHAPE_SIZE_START = 0.05;
static const bool FOCAL_SHAPE_FIXED_SIZE = true;
static const char * TARGET_FRAME_START = "base_link";

// move camera up so the focal point appears in the lower image half
static const float CAMERA_OFFSET = 0.2;

void ThirdPersonViewController::onInitialize()
{
  OrbitViewController::onInitialize();
  focal_shape_->setColor(0.0f, 1.0f, 1.0f, 0.5f);
}

void ThirdPersonViewController::reset()
{
  yaw_property_->setFloat(YAW_START);
  pitch_property_->setFloat(PITCH_START);
  distance_property_->setFloat(DISTANCE_START);
  focal_shape_size_property_->setFloat(FOCAL_SHAPE_SIZE_START);
  focal_shape_fixed_size_property_->setBool(false);
  updateFocalShapeSize();
  focal_point_property_->setVector(Ogre::Vector3::ZERO);
}

std::pair<bool, Ogre::Vector3> ThirdPersonViewController::intersectGroundPlane(Ogre::Ray mouse_ray)
{
  // convert rays into reference frame
  mouse_ray.setOrigin(target_scene_node_->convertWorldToLocalPosition(mouse_ray.getOrigin()));
  mouse_ray.setDirection(
    target_scene_node_->convertWorldToLocalOrientation(Ogre::Quaternion::IDENTITY) *
    mouse_ray.getDirection());

  Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0);

  std::pair<bool, Ogre::Real> intersection = mouse_ray.intersects(ground_plane);
  return std::make_pair(intersection.first, mouse_ray.getPoint(intersection.second));
}

void ThirdPersonViewController::handleMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.shift()) {
    setStatus("<b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.");
  } else {
    setStatus(
      "<b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z. "
      " <b>Shift</b>: More options.");
  }

  int32_t diff_x = 0;
  int32_t diff_y = 0;

  bool moved = false;
  if (event.type == QEvent::MouseButtonPress) {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
  } else if (event.type == QEvent::MouseButtonRelease) {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
  } else if (event.type == QEvent::MouseMove) {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  if (event.left() && !event.shift()) {
    setCursor(Rotate3D);
    yaw(diff_x * 0.005);
    pitch(-diff_y * 0.005);
  } else if (event.middle() || (event.left() && event.shift())) {
    setCursor(MoveXY);
    // handle mouse movement
    int width = camera_->getViewport()->getActualWidth();
    int height = camera_->getViewport()->getActualHeight();

    Ogre::Ray mouse_ray = camera_->getCameraToViewportRay(
      event.x / static_cast<float>(width), event.y / static_cast<float>(height));

    Ogre::Ray last_mouse_ray = camera_->getCameraToViewportRay(
      event.last_x / static_cast<float>(width), event.last_y / static_cast<float>(height));

    auto last_intersect_pair = intersectGroundPlane(last_mouse_ray);
    auto intersect_pair = intersectGroundPlane(mouse_ray);

    if (last_intersect_pair.first && intersect_pair.first) {
      Ogre::Vector3 motion = intersect_pair.second - last_intersect_pair.second;

      // When dragging near the horizon, the motion can get out of
      // control.  This throttles it to an arbitrary limit per mouse
      // event.
      float motion_distance_limit = 1; /*meter*/
      if (motion.length() > motion_distance_limit) {
        motion.normalise();
        motion *= motion_distance_limit;
      }

      focal_point_property_->add(motion);
      emitConfigChanged();
    }
  } else if (event.right()) {
    setCursor(Zoom);
    zoom(-diff_y * 0.1 * (distance_property_->getFloat() / 10.0f));
  } else {
    setCursor(event.shift() ? MoveXY : Rotate3D);
  }

  if (event.wheel_delta != 0) {
    int diff = event.wheel_delta;
    zoom(diff * 0.001 * distance_property_->getFloat());
    moved = true;
  }

  if (moved) {
    context_->queueRender();
  }
}

void ThirdPersonViewController::mimic(rviz_common::ViewController * source_view)
{
  FramePositionTrackingViewController::mimic(source_view);

  target_frame_property_->setValue(TARGET_FRAME_START);
  getNewTransform();

  Ogre::Camera * source_camera = source_view->getCamera();

  Ogre::Ray camera_dir_ray(source_camera->getRealPosition(), source_camera->getRealDirection());
  Ogre::Ray camera_down_ray(source_camera->getRealPosition(), -1.0f * source_camera->getRealUp());

  auto camera_intersection = intersectGroundPlane(camera_dir_ray);
  auto camera_down_intersection = intersectGroundPlane(camera_down_ray);

  if (camera_intersection.first && camera_down_intersection.first) {
    // Set a focal point by intersecting with the ground plane from above. This will be possible
    // if some part of the ground plane is visible in the view and the camera is above the z-plane.
    float l_b = source_camera->getRealPosition().distance(camera_intersection.second);
    float l_a = source_camera->getRealPosition().distance(camera_down_intersection.second);

    distance_property_->setFloat((l_b * l_a) / (CAMERA_OFFSET * l_b + l_a));
    Ogre::Vector3 position_offset =
      source_camera->getRealUp() * distance_property_->getFloat() * CAMERA_OFFSET;

    camera_dir_ray.setOrigin(source_camera->getRealPosition() - position_offset);
    auto new_focal_point = intersectGroundPlane(camera_dir_ray);
    focal_point_property_->setVector(new_focal_point.second - reference_position_);

    calculatePitchYawFromPosition(
      source_camera->getParentSceneNode()->getPosition() - position_offset);
  }
}

void ThirdPersonViewController::updateCamera()
{
  OrbitViewController::updateCamera();
  camera_->getParentSceneNode()->setPosition(
    camera_->getParentSceneNode()->getPosition() +
    camera_->getParentSceneNode()->getLocalAxes() * Ogre::Vector3::UNIT_Y *
      distance_property_->getFloat() * CAMERA_OFFSET);
}

void ThirdPersonViewController::updateTargetSceneNode()
{
  if (FramePositionTrackingViewController::getNewTransform()) {
    target_scene_node_->setPosition(reference_position_);
    Ogre::Radian ref_yaw = reference_orientation_.getRoll(
      false);  // OGRE camera frame looks along -Z, so they call rotation around Z "roll".
    Ogre::Quaternion ref_yaw_quat(Ogre::Math::Cos(ref_yaw / 2), 0, 0, Ogre::Math::Sin(ref_yaw / 2));
    target_scene_node_->setOrientation(ref_yaw_quat);

    context_->queueRender();
  }
}

void ThirdPersonViewController::lookAt(const Ogre::Vector3 & point)
{
  Ogre::Vector3 camera_position = camera_->getParentSceneNode()->getPosition();
  Ogre::Vector3 new_focal_point =
    target_scene_node_->getOrientation().Inverse() * (point - target_scene_node_->getPosition());
  new_focal_point.z = 0;
  distance_property_->setFloat(new_focal_point.distance(camera_position));
  focal_point_property_->setVector(new_focal_point);

  calculatePitchYawFromPosition(camera_position);
}

}  // namespace tier4_camera_view_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  tier4_camera_view_rviz_plugin::ThirdPersonViewController, rviz_common::ViewController)
