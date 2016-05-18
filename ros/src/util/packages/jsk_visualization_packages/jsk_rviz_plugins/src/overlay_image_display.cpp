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

#include "overlay_image_display.h"

#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <OGRE/OgreTechnique.h>

#include <rviz/uniform_string_stream.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_rviz_plugins
{

  OverlayImageDisplay::OverlayImageDisplay()
    : Display(), width_(128), height_(128), left_(128), top_(128), alpha_(0.8),
      is_msg_available_(false), require_update_(false)
  {
    // setup properties
    update_topic_property_ = new rviz::RosTopicProperty(
      "Topic", "",
      ros::message_traits::datatype<sensor_msgs::Image>(),
      "sensor_msgs::Image topic to subscribe to.",
      this, SLOT( updateTopic() ));
    width_property_ = new rviz::IntProperty("width", 128,
                                            "width of the image window",
                                            this, SLOT(updateWidth()));
    height_property_ = new rviz::IntProperty("height", 128,
                                             "height of the image window",
                                             this, SLOT(updateHeight()));
    left_property_ = new rviz::IntProperty("left", 128,
                                           "left of the image window",
                                           this, SLOT(updateLeft()));
    top_property_ = new rviz::IntProperty("top", 128,
                                          "top of the image window",
                                          this, SLOT(updateTop()));
    alpha_property_ = new rviz::FloatProperty("alpha", 0.8,
                                              "alpha belnding value",
                                              this, SLOT(updateAlpha()));
  }

  OverlayImageDisplay::~OverlayImageDisplay()
  {
    delete update_topic_property_;
    delete width_property_;
    delete height_property_;
    delete left_property_;
    delete top_property_;
    delete alpha_property_;
  }
  
  void OverlayImageDisplay::onInitialize()
  {
    ros::NodeHandle nh;
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
    
    updateWidth();
    updateHeight();
    updateTop();
    updateLeft();
    updateAlpha();
    updateTopic();
  }
  
  void OverlayImageDisplay::onEnable()
  {
    if (overlay_) {
      overlay_->show();
    }
    subscribe();
  }
  void OverlayImageDisplay::onDisable()
  {
    if (overlay_) {
      overlay_->hide();
    }
    unsubscribe();
  }

  void OverlayImageDisplay::unsubscribe()
  {
    sub_.shutdown();
    // clear clear clear...
  }

  void OverlayImageDisplay::subscribe()
  {
    std::string topic_name = update_topic_property_->getTopicStd();
    
    if (topic_name.length() > 0 && topic_name != "/") {
      sub_ = it_->subscribe(topic_name, 1, &OverlayImageDisplay::processMessage, this);
    }
  }

  void OverlayImageDisplay::processMessage(
    const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock(mutex_);
    msg_ = msg;
    is_msg_available_ = true;
    require_update_ = true;
    if ((width_property_->getInt() < 0) || (height_property_->getInt() < 0)) {
      // automatically setup display size
      updateWidth();
      updateHeight();
    }
  }
  

  void OverlayImageDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock(mutex_);

    if (!isEnabled()) {
      return;
    }
    
    if (require_update_) {
      if (!overlay_) {
        static int count = 0;
        rviz::UniformStringStream ss;
        ss << "OverlayImageDisplayObject" << count++;
        overlay_.reset(new OverlayObject(ss.str()));
        overlay_->show();
      }
      overlay_->updateTextureSize(msg_->width, msg_->height);
      redraw();
      require_update_ = false;
    }
    if (overlay_) {
      overlay_->setDimensions(width_, height_);
      overlay_->setPosition(left_, top_);
    }
  }

  void OverlayImageDisplay::redraw()
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if (msg_->width == 0 || msg_->height == 0) {
        // image width/height and texture width/height should be same
        // but they are not when input image width/height is 0
        return;
      }
      else if (msg_->encoding == sensor_msgs::image_encodings::RGBA8 ||
          msg_->encoding == sensor_msgs::image_encodings::BGRA8) {
        cv_ptr = cv_bridge::toCvCopy(msg_, sensor_msgs::image_encodings::RGBA8);
        cv::Mat mat = cv_ptr->image;
        ScopedPixelBuffer buffer = overlay_->getBuffer();
        QImage Hud = buffer.getQImage(*overlay_);
        for (int i = 0; i < overlay_->getTextureWidth(); i++) {
          for (int j = 0; j < overlay_->getTextureHeight(); j++) {
            QColor color(mat.data[j * mat.step + i * mat.elemSize() + 0],
                         mat.data[j * mat.step + i * mat.elemSize() + 1],
                         mat.data[j * mat.step + i * mat.elemSize() + 2],
                         mat.data[j * mat.step + i * mat.elemSize() + 3]);
            Hud.setPixel(i, j, color.rgba());
          }
        }
      }
      else {
        cv_ptr = cv_bridge::toCvCopy(msg_, sensor_msgs::image_encodings::RGB8);
        cv::Mat mat = cv_ptr->image;
        ScopedPixelBuffer buffer = overlay_->getBuffer();
        QImage Hud = buffer.getQImage(*overlay_);
        for (int i = 0; i < overlay_->getTextureWidth(); i++) {
          for (int j = 0; j < overlay_->getTextureHeight(); j++) {
            QColor color(mat.data[j * mat.step + i * mat.elemSize() + 0],
                         mat.data[j * mat.step + i * mat.elemSize() + 1],
                         mat.data[j * mat.step + i * mat.elemSize() + 2],
                         alpha_ * 255.0);
            Hud.setPixel(i, j, color.rgba());
          }
        }
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void OverlayImageDisplay::updateTopic()
  {
    unsubscribe();
    subscribe();
  }
  
  void OverlayImageDisplay::updateWidth()
  {
    boost::mutex::scoped_lock lock(mutex_);
    int input_value = width_property_->getInt();
    if (input_value >= 0) {
      width_ = input_value;
    } else {
      if (is_msg_available_) {  // will automatically set width
        if (height_property_->getInt() == -1) {
          // same size as input image
          width_ = msg_->width;
          height_ = msg_->height;
        } else {
          // same scale as height
          float scale = (float)height_ / msg_->height;
          width_ = (int)(scale * msg_->width);
        }
      } else {
        width_ = 128;
      }
    }
  }
  
  void OverlayImageDisplay::updateHeight()
  {
    boost::mutex::scoped_lock lock(mutex_);
    int input_value = height_property_->getInt();
    if (input_value >= 0) {
      height_ = input_value;
    } else {
      if (is_msg_available_) {  // will automatically set height
        if (width_property_->getInt() == -1) {
          // same size as input image
          width_ = msg_->width;
          height_ = msg_->height;
        } else {
          // same scale as width
          float scale = (float)width_ / msg_->width;
          height_ = (int)(scale * msg_->height);
        }
      } else {
        height_ = 128;
      }
    }
  }

  void OverlayImageDisplay::updateTop()
  {
    boost::mutex::scoped_lock lock(mutex_);
    top_ = top_property_->getInt();
  }

  void OverlayImageDisplay::updateLeft()
  {
    boost::mutex::scoped_lock lock(mutex_);
    left_ = left_property_->getInt();
  }
  
  void OverlayImageDisplay::updateAlpha()
  {
    boost::mutex::scoped_lock lock(mutex_);
    alpha_ = alpha_property_->getFloat();
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::OverlayImageDisplay, rviz::Display )
