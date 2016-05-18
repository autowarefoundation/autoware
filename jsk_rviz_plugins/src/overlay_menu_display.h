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
#ifndef JSK_RVIZ_PLUGIN_OVERLAY_MENU_DISPLAY_H_
#define JSK_RVIZ_PLUGIN_OVERLAY_MENU_DISPLAY_H_

#include <rviz/display.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreColourValue.h>
#include <OGRE/OgreMaterial.h>

#include <QPainter>

#include <rviz/properties/ros_topic_property.h>

#include <jsk_rviz_plugins/OverlayMenu.h>

#include "overlay_utils.h"

namespace jsk_rviz_plugins
{
  class OverlayMenuDisplay : public rviz::Display
  {
    Q_OBJECT
  public:
    OverlayMenuDisplay();
    virtual ~OverlayMenuDisplay();

    enum AnimationState
    {
      CLOSED,
      OPENED,
      OPENING,
      CLOSING,
    };
    
  protected:
    OverlayObject::Ptr overlay_;
    ros::Subscriber sub_;
    rviz::RosTopicProperty* update_topic_property_;
    AnimationState animation_state_;
    bool require_update_texture_;
    jsk_rviz_plugins::OverlayMenu::ConstPtr current_menu_;
    jsk_rviz_plugins::OverlayMenu::ConstPtr next_menu_;
    double animation_t_;
    virtual void prepareOverlay();
    virtual void openingAnimation();
    virtual std::string getMenuString(
      const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg,
      size_t index);
    virtual QFont font();
    virtual QFontMetrics fontMetrics();
    virtual int drawAreaWidth(
      const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg);
    virtual int drawAreaHeight(
      const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg);
    virtual bool isNeedToResize();
    virtual bool isNeedToRedraw();
    virtual void redraw();
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    virtual void update(float wall_dt, float ros_dt);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void processMessage
    (const jsk_rviz_plugins::OverlayMenu::ConstPtr& msg);
  protected Q_SLOTS:
    void updateTopic();
    
  };

}

#endif

