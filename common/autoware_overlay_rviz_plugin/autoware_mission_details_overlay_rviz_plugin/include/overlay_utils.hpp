// Copyright 2024 The Autoware Contributors
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

// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Team Spatzenhirn
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

#ifndef OVERLAY_UTILS_HPP_
#define OVERLAY_UTILS_HPP_

#include <QColor>
#include <QImage>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include <memory>
#include <string>

namespace autoware::mission_details_overlay_rviz_plugin
{
class OverlayObject;

class ScopedPixelBuffer
{
public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  virtual ~ScopedPixelBuffer();
  virtual Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
  virtual QImage getQImage(unsigned int width, unsigned int height);
  virtual QImage getQImage(OverlayObject & overlay);
  virtual QImage getQImage(unsigned int width, unsigned int height, QColor & bg_color);
  virtual QImage getQImage(OverlayObject & overlay, QColor & bg_color);

protected:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
};

enum class VerticalAlignment : uint8_t { CENTER, TOP, BOTTOM };

enum class HorizontalAlignment : uint8_t { LEFT, RIGHT, CENTER };

/**
 * Helper class for realizing an overlay object on top of the rviz 3D panel.
 *
 * This class is supposed to be instantiated in the onInitialize method of the
 * rviz_common::Display class.
 */
class OverlayObject
{
public:
  using SharedPtr = std::shared_ptr<OverlayObject>;

  explicit OverlayObject(const std::string & name);
  virtual ~OverlayObject();

  virtual std::string getName() const;
  virtual void hide();
  virtual void show();
  virtual bool isTextureReady() const;
  virtual void updateTextureSize(unsigned int width, unsigned int height);
  virtual ScopedPixelBuffer getBuffer();
  virtual void setPosition(
    double hor_dist, double ver_dist, HorizontalAlignment hor_alignment = HorizontalAlignment::LEFT,
    VerticalAlignment ver_alignment = VerticalAlignment::TOP);
  virtual void setDimensions(double width, double height);
  virtual bool isVisible() const;
  virtual unsigned int getTextureWidth() const;
  virtual unsigned int getTextureHeight() const;

protected:
  const std::string name_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
};
}  // namespace autoware::mission_details_overlay_rviz_plugin

#endif  // OVERLAY_UTILS_HPP_
