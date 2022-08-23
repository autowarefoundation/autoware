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
/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef POLAR_GRID_DISPLAY_HPP_
#define POLAR_GRID_DISPLAY_HPP_

#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/tf_frame_property.hpp>
#include <rviz_common/properties/vector_property.hpp>

#include <OgreBillboardSet.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace rviz_plugins
{
/**
 * \class PolarGridDisplay
 * \brief Displays a grid in either the XY, YZ, or XZ plane.
 *
 * For more information see Grid
 */
class PolarGridDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  PolarGridDisplay();
  ~PolarGridDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float dt, float ros_dt) override;
  void reset() override;

private Q_SLOTS:
  void updatePlane();

private:
  Ogre::ManualObject * rings_manual_object_{nullptr};
  Ogre::ManualObject * wave_manual_object_{nullptr};
  float wave_range_{0.0};

  rviz_common::properties::TfFrameProperty * frame_property_;
  rviz_common::properties::FloatProperty * d_range_property_;
  rviz_common::properties::FloatProperty * max_range_property_;
  rviz_common::properties::FloatProperty * max_alpha_property_;
  rviz_common::properties::FloatProperty * min_alpha_property_;
  rviz_common::properties::ColorProperty * color_property_;
  rviz_common::properties::FloatProperty * wave_velocity_property_;
  rviz_common::properties::ColorProperty * wave_color_property_;
  rviz_common::properties::FloatProperty * max_wave_alpha_property_;
  rviz_common::properties::FloatProperty * min_wave_alpha_property_;
};

}  // namespace rviz_plugins

#endif  // POLAR_GRID_DISPLAY_HPP_
