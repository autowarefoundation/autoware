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

#ifndef JSK_RVIZ_PLUGIN_DIAGNOSTICS_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_DIAGNOSTICS_DISPLAY_H_

#include <rviz/display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/movable_text.h>

namespace jsk_rviz_plugins
{
  class DiagnosticsDisplay : public rviz::Display
  {
    Q_OBJECT
  public:
    DiagnosticsDisplay();
    virtual ~DiagnosticsDisplay();
  protected:
    virtual void onEnable();
    virtual void onDisable();
    virtual void onInitialize();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void updateLine();
    virtual void update(float wall_dt, float ros_dt);
    virtual void processMessage
    (const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
    
    rviz::RosTopicProperty* ros_topic_property_;
    rviz::EditableEnumProperty* diagnostics_namespace_property_;
    rviz::TfFrameProperty* frame_id_property_;
    rviz::FloatProperty* radius_property_;
    rviz::FloatProperty* line_width_property_;
    rviz::FloatProperty* font_size_property_;
    rviz::EnumProperty* axis_property_;
    ros::Subscriber sub_;
    
    double radius_;
    double line_width_;
    std::string frame_id_;
    std::string diagnostics_namespace_;
    rviz::MovableText* msg_;
    rviz::BillboardLine* line_;
    Ogre::SceneNode* orbit_node_;
    std::set<std::string> namespaces_;
    int axis_;
    double orbit_theta_;
    double font_size_;
    bool line_update_required_;
  protected Q_SLOTS:
    virtual void updateRosTopic();
    virtual void updateDiagnosticsNamespace();
    virtual void updateRadius();
    virtual void updateLineWidth();
    virtual void updateAxis();
    virtual void updateFontSize();
    virtual void fillNamespaceList();
  private:
    
  };
  
}


#endif
