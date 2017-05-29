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

#include "video_capture_display.h"
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/display_group.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <QImage>
#include <boost/filesystem.hpp>

namespace jsk_rviz_plugins
{
  VideoCaptureDisplay::VideoCaptureDisplay():
    Display(), capturing_(false), first_time_(true)
  {
    start_capture_property_ = new rviz::BoolProperty(
      "start capture", false, "start capture",
      this, SLOT(updateStartCapture()));
    file_name_property_ = new rviz::StringProperty(
      "filename", "output.avi",
      "filename", this, SLOT(updateFileName()));
    fps_property_ = new rviz::FloatProperty(
      "fps", 30.0,
      "fps", this, SLOT(updateFps()));
    fps_property_->setMin(0.1);
  }

  VideoCaptureDisplay::~VideoCaptureDisplay()
  {
    delete start_capture_property_;
    delete file_name_property_;
  }

  void VideoCaptureDisplay::onInitialize()
  {
    updateFileName();
    updateFps();
    //updateStartCapture();
    start_capture_property_->setBool(false); // always false when starting up
    context_->queueRender();
  }

  void VideoCaptureDisplay::onEnable()
  {
    start_capture_property_->setBool(false); // always false when starting up
    context_->queueRender();
  }
  
  void VideoCaptureDisplay::updateFileName()
  {
    if (capturing_) {
      ROS_WARN("cannot change name wile recording");
      file_name_property_->setStdString(file_name_);
    }
    else {
      file_name_ = file_name_property_->getStdString();
      int exists_check = access(file_name_.c_str(), F_OK);
      if (exists_check == 0) {
        int access_result = access(file_name_.c_str(), W_OK);
        ROS_INFO("access_result to %s: %d", file_name_.c_str(), access_result);
        if (access_result != 0) {
          setStatus(rviz::StatusProperty::Error, "File", "NOT Writable");
        }
        else {
          setStatus(rviz::StatusProperty::Ok, "File", "Writable");
        }
      }
      else {                    // do not exists, check directory permission
        ROS_INFO("%s do not exists", file_name_.c_str());
        boost::filesystem::path pathname(file_name_);
        std::string dirname  = pathname.parent_path().string();
        if (dirname.length() == 0) { // Special case for without path
          dirname = ".";
        }
        ROS_INFO("dirname: %s", dirname.c_str());
        int directory_access_result = access(dirname.c_str(), W_OK);
        if (directory_access_result != 0) {
          setStatus(rviz::StatusProperty::Error, "File", "NOT Writable (direcotry)");
        }
        else {
          setStatus(rviz::StatusProperty::Ok, "File", "Writable");
        }
      }
    }
  }

  void VideoCaptureDisplay::updateStartCapture()
  {
    ROS_INFO("updateStartCapture");
    if (first_time_) {
      ROS_WARN("ignore first time capture enabling");
    }
    else {
      // start capture!
      if (start_capture_property_->getBool()) {
        capturing_ = true;
        startCapture();
      }
      else {
        capturing_ = false;
        stopCapture();
      }
    }
  }

  void VideoCaptureDisplay::updateFps()
  {
    fps_ = fps_property_->getFloat();
  }
  
  void VideoCaptureDisplay::startCapture()
  {
    ROS_INFO("start capturing");
    frame_counter_ = 0;
    rviz::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
    int width = panel->width();
    int height = panel->height();
    writer_.open(file_name_, CV_FOURCC_DEFAULT, fps_, cv::Size(width, height));
  }
  
  void VideoCaptureDisplay::stopCapture()
  {
    ROS_INFO("stop capturing");
    writer_.release();
    frame_counter_ = 0;
  }

  void VideoCaptureDisplay::update(float wall_dt, float ros_dt)
  {
    if (first_time_) {
      ROS_WARN("force to disable capturing");
      start_capture_property_->setBool(false); // always false when starting up
      first_time_ = false;
      return;
    }
    if (capturing_) {
      rviz::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
      QPixmap screenshot
        = QPixmap::grabWindow(context_->getViewManager()->getRenderPanel()->winId());
      QImage src = screenshot.toImage().convertToFormat(QImage::Format_RGB888);
      cv::Mat image(src.height(), src.width(), CV_8UC3,
                    (uchar*)src.bits(), src.bytesPerLine());
      writer_ << image;
      ++frame_counter_;
      if (frame_counter_ % 100 == 0) {
        ROS_INFO("taking %d frames as video", frame_counter_);
      }
    }
    // convert QPixmap into cv::Mat
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::VideoCaptureDisplay, rviz::Display)

