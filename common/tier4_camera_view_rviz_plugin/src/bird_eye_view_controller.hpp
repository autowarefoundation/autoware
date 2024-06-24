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
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef BIRD_EYE_VIEW_CONTROLLER_HPP_
#define BIRD_EYE_VIEW_CONTROLLER_HPP_

#include "rviz_common/frame_position_tracking_view_controller.hpp"

#include <OgreQuaternion.h>
#include <OgreVector.h>

namespace rviz_common
{
namespace properties
{
class FloatProperty;
class Shape;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace tier4_camera_view_rviz_plugin
{
/** @brief A first-person camera, controlled by yaw, pitch, and position. */
class BirdEyeViewController : public rviz_common::FramePositionTrackingViewController
{
  Q_OBJECT

public:
  BirdEyeViewController();

  ~BirdEyeViewController() override;

  void onInitialize() override;

  void handleMouseEvent(rviz_common::ViewportMouseEvent & event) override;

  void lookAt(const Ogre::Vector3 & point) override;

  void reset() override;

  /** @brief Configure the settings of this view controller to give,
   * as much as possible, a similar view as that given by the
   * @param source_view.
   *
   * @param source_view must return a valid @c Ogre::Camera* from getCamera(). */
  void mimic(rviz_common::ViewController * source_view) override;

  void update(float dt, float ros_dt) override;

protected:
  void onTargetFrameChanged(
    const Ogre::Vector3 & old_reference_position,
    const Ogre::Quaternion & old_reference_orientation) override;

  /** Set the camera orientation based on angle_. */
  void orientCamera();

  void setPosition(const Ogre::Vector3 & pos_rel_target);
  void move_camera(float dx, float dy);
  void updateCamera();
  Ogre::SceneNode * getCameraParent(Ogre::Camera * camera);

  rviz_common::properties::FloatProperty * scale_property_;
  rviz_common::properties::FloatProperty * angle_property_;
  rviz_common::properties::FloatProperty * x_property_;
  rviz_common::properties::FloatProperty * y_property_;
  bool dragging_;
};

}  // namespace tier4_camera_view_rviz_plugin

#endif  // BIRD_EYE_VIEW_CONTROLLER_HPP_
