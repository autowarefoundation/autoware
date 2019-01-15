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

#ifndef RVIZ_POSE_ARRAY_DISPLAY_H_
#define RVIZ_POSE_ARRAY_DISPLAY_H_

#include <geometry_msgs/PoseArray.h>

#include "rviz/message_filter_display.h"

namespace Ogre
{
class ManualObject;
};

namespace rviz
{
class ColorProperty;
class FloatProperty;
class EnumProperty;
class Axes;
};

namespace jsk_rviz_plugins
{
/** @brief Displays a geometry_msgs/PoseArray message as a bunch of line-drawn arrows. */
  class PoseArrayDisplay: public rviz::MessageFilterDisplay<geometry_msgs::PoseArray>
{
Q_OBJECT
public:
  enum Shape
  {
    Arrow,
    Axes,
  };
  PoseArrayDisplay();
  virtual ~PoseArrayDisplay();

  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateShapeChoice();
  void updateShapeVisibility();
  void updateAxisGeometry();
  void allocateCoords(int num);

private:
  virtual void processMessage( const geometry_msgs::PoseArray::ConstPtr& msg );

  Ogre::ManualObject* manual_object_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* length_property_;
  rviz::FloatProperty* axes_length_property_;
  rviz::FloatProperty* axes_radius_property_;
  rviz::EnumProperty* shape_property_;
  std::vector<rviz::Axes*> coords_objects_;
  std::vector<Ogre::SceneNode*> coords_nodes_;

  bool pose_valid_;
};

} // namespace rviz

#endif /* RVIZ_POSE_ARRAY_DISPLAY_H_ */
