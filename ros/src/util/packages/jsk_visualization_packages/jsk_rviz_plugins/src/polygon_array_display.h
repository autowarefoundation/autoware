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

#ifndef POLYGON_ARRAY_DISPLAY_H
#define POLYGON_ARRAY_DISPLAY_H

#include <jsk_recognition_msgs/PolygonArray.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/shape.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/arrow.h>

namespace jsk_rviz_plugins
{
  
  class PolygonArrayDisplay:
    public rviz::MessageFilterDisplay<jsk_recognition_msgs::PolygonArray>
  {
    Q_OBJECT
  public:
    typedef boost::shared_ptr<rviz::Arrow> ArrowPtr;
    PolygonArrayDisplay();
    virtual ~PolygonArrayDisplay();
  protected:
    virtual void onInitialize();
    virtual void reset();
    virtual void updateSceneNodes(
      const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
    virtual void allocateMaterials(int num);
    virtual void updateLines(int num);
    virtual Ogre::ColourValue getColor(size_t index);
    virtual void processLine(
      const size_t i, const geometry_msgs::PolygonStamped& polygon);
    virtual void processPolygon(
      const size_t i, const geometry_msgs::PolygonStamped& polygon);
    virtual void processNormal(
      const size_t i, const geometry_msgs::PolygonStamped& polygon);
    virtual void processPolygonMaterial(const size_t i);
    virtual void processMessage(
      const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::BoolProperty* only_border_property_;
    // rviz::BoolProperty* auto_coloring_property_;
    rviz::EnumProperty* coloring_property_;
    rviz::BoolProperty* show_normal_property_;
    rviz::FloatProperty* normal_length_property_;
    bool only_border_;
    std::string coloring_method_;
    bool show_normal_;
    double normal_length_;
    jsk_recognition_msgs::PolygonArray::ConstPtr latest_msg_;
    std::vector<Ogre::ManualObject*> manual_objects_;
    std::vector<Ogre::SceneNode*> scene_nodes_;
    std::vector<Ogre::SceneNode*> arrow_nodes_;
    std::vector<ArrowPtr> arrow_objects_;
    std::vector<Ogre::MaterialPtr> materials_;
    std::vector<rviz::BillboardLine*> lines_;
  private Q_SLOTS:
    void updateColoring();
    void updateOnlyBorder();
    void updateShowNormal();
    void updateNormalLength();
  private:
    
  };
}

#endif

