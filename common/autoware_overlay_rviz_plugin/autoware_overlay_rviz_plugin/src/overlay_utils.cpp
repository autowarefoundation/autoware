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

#include "overlay_utils.hpp"

#include <rviz_common/logging.hpp>

namespace autoware_overlay_rviz_plugin
{
ScopedPixelBuffer::ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer)
: pixel_buffer_(pixel_buffer)
{
  pixel_buffer_->lock(Ogre::HardwareBuffer::HBL_NORMAL);
}

ScopedPixelBuffer::~ScopedPixelBuffer()
{
  pixel_buffer_->unlock();
}

Ogre::HardwarePixelBufferSharedPtr ScopedPixelBuffer::getPixelBuffer()
{
  return pixel_buffer_;
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height)
{
  const Ogre::PixelBox & pixelBox = pixel_buffer_->getCurrentLock();
  Ogre::uint8 * pDest = static_cast<Ogre::uint8 *>(pixelBox.data);
  memset(pDest, 0, width * height);
  return QImage(pDest, width, height, QImage::Format_ARGB32);
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height, QColor & bg_color)
{
  QImage Hud = getQImage(width, height);
  for (unsigned int i = 0; i < width; i++) {
    for (unsigned int j = 0; j < height; j++) {
      Hud.setPixel(i, j, bg_color.rgba());
    }
  }
  return Hud;
}

QImage ScopedPixelBuffer::getQImage(OverlayObject & overlay)
{
  return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight());
}

QImage ScopedPixelBuffer::getQImage(OverlayObject & overlay, QColor & bg_color)
{
  return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight(), bg_color);
}

OverlayObject::OverlayObject(const std::string & name) : name_(name)
{
  std::string material_name = name_ + "Material";
  Ogre::OverlayManager * mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
  overlay_ = mOverlayMgr->create(name_);
  panel_ = static_cast<Ogre::PanelOverlayElement *>(
    mOverlayMgr->createOverlayElement("Panel", name_ + "Panel"));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);

  panel_material_ = Ogre::MaterialManager::getSingleton().create(
    material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_->setMaterialName(panel_material_->getName());
  overlay_->add2D(panel_);
}

OverlayObject::~OverlayObject()
{
  Ogre::OverlayManager * mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
  if (mOverlayMgr) {
    mOverlayMgr->destroyOverlayElement(panel_);
    mOverlayMgr->destroy(overlay_);
  }
  if (panel_material_) {
    panel_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
  }
}

std::string OverlayObject::getName() const
{
  return name_;
}

void OverlayObject::hide()
{
  if (overlay_->isVisible()) {
    overlay_->hide();
  }
}

void OverlayObject::show()
{
  if (!overlay_->isVisible()) {
    overlay_->show();
  }
}

bool OverlayObject::isTextureReady() const
{
  return texture_ != nullptr;
}

void OverlayObject::updateTextureSize(unsigned int width, unsigned int height)
{
  const std::string texture_name = name_ + "Texture";
  if (width == 0) {
    RVIZ_COMMON_LOG_WARNING_STREAM("[OverlayObject] width=0 is specified as texture size");
    width = 1;
  }

  if (height == 0) {
    RVIZ_COMMON_LOG_WARNING_STREAM("[OverlayObject] height=0 is specified as texture size");
    height = 1;
  }

  if (!isTextureReady() || ((width != texture_->getWidth()) || (height != texture_->getHeight()))) {
    if (isTextureReady()) {
      Ogre::TextureManager::getSingleton().remove(texture_name);
      panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }

    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name,  // name
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,  // type
      width, height,      // width & height of the render window
      0,                  // number of mipmaps
      Ogre::PF_A8R8G8B8,  // pixel format chosen to match a format Qt can use
      Ogre::TU_DEFAULT    // usage
    );
    panel_material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);

    panel_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }
}

ScopedPixelBuffer OverlayObject::getBuffer()
{
  if (isTextureReady()) {
    return ScopedPixelBuffer(texture_->getBuffer());
  } else {
    return ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr());
  }
}

void OverlayObject::setPosition(
  double hor_dist, double ver_dist, HorizontalAlignment hor_alignment,
  VerticalAlignment ver_alignment)
{
  // ogre position is always based on the top left corner of the panel, while our position input
  // depends on the chosen alignment, i.e. if the horizontal alignment is right, increasing the
  // horizontal dist moves the panel to the left (further away from the right border)
  double left = 0;
  double top = 0;

  switch (hor_alignment) {
    case HorizontalAlignment::LEFT:
      panel_->setHorizontalAlignment(Ogre::GuiHorizontalAlignment::GHA_LEFT);
      left = hor_dist;
      break;
    case HorizontalAlignment::CENTER:
      panel_->setHorizontalAlignment(Ogre::GuiHorizontalAlignment::GHA_CENTER);
      left = hor_dist - panel_->getWidth() / 2;
      break;
    case HorizontalAlignment::RIGHT:
      panel_->setHorizontalAlignment(Ogre::GuiHorizontalAlignment::GHA_RIGHT);
      left = -hor_dist - panel_->getWidth();
      break;
  }

  switch (ver_alignment) {
    case VerticalAlignment::BOTTOM:
      panel_->setVerticalAlignment(Ogre::GuiVerticalAlignment::GVA_BOTTOM);
      top = -ver_dist - panel_->getHeight();
      break;
    case VerticalAlignment::CENTER:
      panel_->setVerticalAlignment(Ogre::GuiVerticalAlignment::GVA_CENTER);
      top = ver_dist - panel_->getHeight() / 2;
      break;
    case VerticalAlignment::TOP:
      panel_->setVerticalAlignment(Ogre::GuiVerticalAlignment::GVA_TOP);
      top = ver_dist;
      break;
  }

  panel_->setPosition(left, top);
}

void OverlayObject::setDimensions(double width, double height)
{
  panel_->setDimensions(width, height);
}

bool OverlayObject::isVisible() const
{
  return overlay_->isVisible();
}

unsigned int OverlayObject::getTextureWidth() const
{
  if (isTextureReady()) {
    return texture_->getWidth();
  } else {
    return 0;
  }
}

unsigned int OverlayObject::getTextureHeight() const
{
  if (isTextureReady()) {
    return texture_->getHeight();
  } else {
    return 0;
  }
}
}  // namespace autoware_overlay_rviz_plugin
