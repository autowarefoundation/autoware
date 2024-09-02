// Copyright 2022 Tier IV, Inc.
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

// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Software License Agreement (BSD License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.S SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef TOOLS__JSK_OVERLAY_UTILS_HPP_
#define TOOLS__JSK_OVERLAY_UTILS_HPP_

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <memory>
#include <string>
// see OGRE/OgrePrerequisites.h
// #define OGRE_VERSION
// ((OGRE_VERSION_MAJOR << 16) | (OGRE_VERSION_MINOR << 8) | OGRE_VERSION_PATCH)
#if OGRE_VERSION < ((1 << 16) | (9 << 8) | 0)
#include <OGRE/OgreOverlayContainer.h>
#include <OGRE/OgreOverlayElement.h>
#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgrePanelOverlayElement.h>
#else
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>
#endif

#include <QColor>
#include <QCursor>
#include <QImage>
#include <QVariant>
#include <rclcpp/rclcpp.hpp>
#include <rviz_rendering/render_system.hpp>

namespace jsk_rviz_plugins
{
class OverlayObject;

class ScopedPixelBuffer
{
public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  virtual ~ScopedPixelBuffer();
  virtual QImage getQImage(unsigned int width, unsigned int height);
  virtual QImage getQImage(OverlayObject & overlay);
  virtual QImage getQImage(unsigned int width, unsigned int height, QColor & bg_color);
  virtual QImage getQImage(OverlayObject & overlay, QColor & bg_color);

protected:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;

private:
};

// this is a class for put overlay object on rviz 3D panel.
// This class suppose to be instantiated in onInitialize method
// of rviz::Display class.
class OverlayObject
{
public:
  typedef std::shared_ptr<OverlayObject> Ptr;

  OverlayObject(Ogre::SceneManager * manager, rclcpp::Logger logger, const std::string & name);
  virtual ~OverlayObject();

  virtual std::string getName();
  virtual void hide();
  virtual void show();
  virtual bool isTextureReady();
  virtual void updateTextureSize(unsigned int width, unsigned int height);
  virtual ScopedPixelBuffer getBuffer();
  virtual void setPosition(double left, double top);
  virtual void setDimensions(double width, double height);
  virtual bool isVisible();
  virtual unsigned int getTextureWidth();
  virtual unsigned int getTextureHeight();

protected:
  const std::string name_;
  rclcpp::Logger logger_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;

private:
};

// Ogre::Overlay* createOverlay(std::string name);
// Ogre::PanelOverlayElement* createOverlayPanel(Ogre::Overlay* overlay);
// Ogre::MaterialPtr createOverlayMaterial(Ogre::Overlay* overlay);
}  // namespace jsk_rviz_plugins

#endif  // TOOLS__JSK_OVERLAY_UTILS_HPP_
