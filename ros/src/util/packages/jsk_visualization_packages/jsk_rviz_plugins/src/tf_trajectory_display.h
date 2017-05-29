// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#ifndef JSK_RVIZ_PLUGINS_TF_TRAJECTORY_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_TF_TRAJECTORY_DISPLAY_H_

#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <geometry_msgs/PointStamped.h>

namespace jsk_rviz_plugins
{

  class TFTrajectoryDisplay: public rviz::Display
  {
    Q_OBJECT
  public:
    TFTrajectoryDisplay();
    virtual ~TFTrajectoryDisplay();
  protected:
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);

    rviz::TfFrameProperty* frame_property_;
    rviz::FloatProperty* duration_property_;
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* line_width_property_;
    rviz::BillboardLine* line_;
    std::vector<geometry_msgs::PointStamped> trajectory_;
    std::string frame_;
    std::string fixed_frame_;
    float duration_;
    QColor color_;
    float line_width_;
  protected Q_SLOTS:
    void updateFrame();
    void updateDuration();
    void updateColor();
    void updateLineWidth();
  };
}

#endif
