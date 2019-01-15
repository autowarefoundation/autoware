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

#include "overlay_utils.h"
#include <ros/ros.h>

namespace jsk_rviz_plugins
{
  ScopedPixelBuffer::ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer):
    pixel_buffer_(pixel_buffer)
  {
    pixel_buffer_->lock( Ogre::HardwareBuffer::HBL_NORMAL );
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
    const Ogre::PixelBox& pixelBox = pixel_buffer_->getCurrentLock();
    Ogre::uint8* pDest = static_cast<Ogre::uint8*> (pixelBox.data);
    memset(pDest, 0, width * height);
    return QImage(pDest, width, height, QImage::Format_ARGB32 );
  }

  QImage ScopedPixelBuffer::getQImage(
    unsigned int width, unsigned int height, QColor& bg_color)
  {
    QImage Hud = getQImage(width, height);
    for (unsigned int i = 0; i < width; i++) {
      for (unsigned int j = 0; j < height; j++) {
        Hud.setPixel(i, j, bg_color.rgba());
      }
    }
    return Hud;
  }

  QImage ScopedPixelBuffer::getQImage(OverlayObject& overlay)
  {
    return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight());
  }
  
  QImage ScopedPixelBuffer::getQImage(OverlayObject& overlay,
                                      QColor& bg_color)
  {
    return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight(),
                     bg_color);
  }
  
  OverlayObject::OverlayObject(const std::string& name)
    : name_(name)
  {
    std::string material_name = name_ + "Material";
    Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    overlay_ = mOverlayMgr->create(name_);
    panel_ = static_cast<Ogre::PanelOverlayElement*> (
      mOverlayMgr->createOverlayElement("Panel", name_ + "Panel"));
    panel_->setMetricsMode(Ogre::GMM_PIXELS);
    
    panel_material_
      = Ogre::MaterialManager::getSingleton().create(
        material_name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    panel_->setMaterialName(panel_material_->getName());
    overlay_->add2D(panel_);
  }

  OverlayObject::~OverlayObject()
  {
    hide();
    panel_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
    // Ogre::OverlayManager* mOverlayMgr = Ogre::OverlayManager::getSingletonPtr();
    // mOverlayMgr->destroyOverlayElement(panel_);
    //delete panel_;
    //delete overlay_;
  }

  std::string OverlayObject::getName()
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

  bool OverlayObject::isTextureReady()
  {
    return !texture_.isNull();
  }

  bool OverlayObject::updateTextureSize(unsigned int width, unsigned int height)
  {
    const std::string texture_name = name_ + "Texture";
    if (width == 0) {
      ROS_WARN("[OverlayObject] width=0 is specified as texture size");
      width = 1;
    }
    if (height == 0) {
      ROS_WARN("[OverlayObject] height=0 is specified as texture size");
      height = 1;
    }
    if (!isTextureReady() ||
        ((width != texture_->getWidth()) ||
         (height != texture_->getHeight()))) {
      if (isTextureReady()) {
        Ogre::TextureManager::getSingleton().remove(texture_name);
        panel_material_->getTechnique(0)->getPass(0)
          ->removeAllTextureUnitStates();
      }
      texture_ = Ogre::TextureManager::getSingleton().createManual(
        texture_name,        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,   // type
        width, height,   // width & height of the render window 
        0,                   // number of mipmaps
        Ogre::PF_A8R8G8B8,   // pixel format chosen to match a format Qt can use
        Ogre::TU_DEFAULT     // usage
        );
      panel_material_->getTechnique(0)->getPass(0)
        ->createTextureUnitState(texture_name);
        
      panel_material_->getTechnique(0)->getPass(0)
        ->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    }
  }

  ScopedPixelBuffer OverlayObject::getBuffer()
  {
    if (isTextureReady()) {
      return ScopedPixelBuffer(texture_->getBuffer());
    }
    else {
      return ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr());
    }
  }

  void OverlayObject::setPosition(double left, double top)
  {
    panel_->setPosition(left, top);
  }

  void OverlayObject::setDimensions(double width, double height)
  {
    panel_->setDimensions(width, height);
  }

  bool OverlayObject::isVisible()
  {
    return overlay_->isVisible();
  }

  unsigned int OverlayObject::getTextureWidth()
  {
    if (isTextureReady()) {
      return texture_->getWidth();
    }
    else {
      return 0;
    }
  }

  unsigned int OverlayObject::getTextureHeight()
  {
    if (isTextureReady()) {
      return texture_->getHeight();
    }
    else {
      return 0;
    }
  }

  
}
