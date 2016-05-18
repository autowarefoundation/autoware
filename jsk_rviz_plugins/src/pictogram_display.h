// -*- mode: c++ -*-
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


#ifndef JSK_RVIZ_PLUGIN_PICTOGRAM_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_PICTOGRAM_DISPLAY_H_

#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/enum_property.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include "facing_visualizer.h"

namespace jsk_rviz_plugins
{
  void setupFont();
  int addFont(unsigned char* data, unsigned int data_len);
  bool isFontAwesome(std::string);
  bool isEntypo(std::string);
  bool isCharacterSupported(std::string character);
  QFont getFont(std::string character);
  QString lookupPictogramText(std::string character);
  ////////////////////////////////////////////////////////
  // PictogramObject
  ////////////////////////////////////////////////////////
  class PictogramObject: public FacingTexturedObject
  {
  public:
    typedef boost::shared_ptr<PictogramObject> Ptr;
    PictogramObject(Ogre::SceneManager* manager,
                    Ogre::SceneNode* parent,
                    double size);
    virtual void update(float wall_dt, float ros_dt);
    virtual void setEnable(bool enable);
    virtual void setText(std::string text);
    virtual void setAlpha(double alpha);
    virtual void setColor(QColor color);
    virtual void setSize(double size);
    virtual void setSpeed(double speed);
    virtual void setPose(const geometry_msgs::Pose& pose,
                         const std::string& frame_id);
    virtual void start();
    virtual void setContext(rviz::DisplayContext* context);
    virtual void setAction(uint8_t action);
    virtual void setMode(uint8_t mode);
    virtual void setTTL(double ttl);
  protected:
    virtual void updatePose(float dt);
    virtual void updateColor();
    virtual void updateText();
    
    bool need_to_update_;
    uint8_t action_;
    geometry_msgs::Pose pose_;
    std::string frame_id_;
    rviz::DisplayContext* context_;
    ros::WallTime time_;
    double ttl_;
    double speed_;
    uint8_t mode_;
  private:
    
  };

  
  ////////////////////////////////////////////////////////
  // Display to visualize pictogram on rviz
  ////////////////////////////////////////////////////////
  class PictogramDisplay:
    public rviz::MessageFilterDisplay<jsk_rviz_plugins::Pictogram>
  {
    Q_OBJECT
  public:
    PictogramDisplay();
    virtual ~PictogramDisplay();
  protected:
    
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInitialize();
    virtual void reset();
    virtual void onEnable();
    void processMessage(const jsk_rviz_plugins::Pictogram::ConstPtr& msg);
    void update(float wall_dt, float ros_dt);

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    boost::mutex mutex_;
    PictogramObject::Ptr pictogram_;
  private Q_SLOTS:
    
  private:
  };
}

#endif
