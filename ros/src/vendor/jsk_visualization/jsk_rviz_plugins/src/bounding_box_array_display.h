// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef JSK_RVIZ_PLUGINS_BOUDNING_BOX_ARRAY_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_BOUDNING_BOX_ARRAY_DISPLAY_H_

#ifndef Q_MOC_RUN
#include "bounding_box_display_common.h"
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/arrow.h>
#include <OGRE/OgreSceneNode.h>
#endif

namespace jsk_rviz_plugins
{

  class BoundingBoxArrayDisplay: public BoundingBoxDisplayCommon<jsk_recognition_msgs::BoundingBoxArray>
  {
    Q_OBJECT
  public:
    BoundingBoxArrayDisplay();
    virtual ~BoundingBoxArrayDisplay();
  protected:
    void onInitialize();
    virtual void reset();

    bool only_edge_;
    bool show_coords_;
    // Properties
    rviz::EnumProperty* coloring_property_;
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::BoolProperty* only_edge_property_;
    rviz::FloatProperty* line_width_property_;
    rviz::BoolProperty* show_coords_property_;

    jsk_recognition_msgs::BoundingBoxArray::ConstPtr latest_msg_;
  protected Q_SLOTS:
    void updateColor();
    void updateAlpha();
    void updateOnlyEdge();
    void updateColoring();
    void updateLineWidth();
    void updateShowCoords();
  private:
    void processMessage(
      const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg);
  };

}  // namespace jsk_rviz_plugins

#endif  // JSK_RVIZ_PLUGINS_BOUDNING_BOX_ARRAY_DISPLAY_H_
