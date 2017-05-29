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

#include <rviz/uniform_string_stream.h>
#include "camera_info_display.h"
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreBlendMode.h>
#include <QImage>
#include <OGRE/OgreHardwarePixelBuffer.h>

namespace jsk_rviz_plugins
{
  TrianglePolygon::TrianglePolygon(
    Ogre::SceneManager* manager,
    Ogre::SceneNode* node,
    const cv::Point3d& O,
    const cv::Point3d& A,
    const cv::Point3d& B,
    const std::string& name,
    const Ogre::ColourValue& color,
    bool use_color,
    bool upper_triangle)
  {
    // uniq string is requred for name
    
    manual_ = manager->createManualObject();
    manual_->clear();
    manual_->begin(name,
                   Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    manual_->position(O.x, O.y, O.z);
    if (upper_triangle) {
      manual_->textureCoord(0, 0);
    }
    else {
      manual_->textureCoord(1, 0);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->position(A.x, A.y, A.z);
    if (upper_triangle) {
      manual_->textureCoord(1, 0);
    }
    else {
      manual_->textureCoord(1, 1);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->position(B.x, B.y, B.z);
    if (upper_triangle) {
      manual_->textureCoord(0, 1);
    }
    else {
      manual_->textureCoord(0, 1);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->end();
    node->attachObject(manual_);
  }

  TrianglePolygon::~TrianglePolygon()
  {
    manual_->detachFromParent();
    //manager_->destroyManualObject(manual_); // this crashes rviz
  }
  
  CameraInfoDisplay::CameraInfoDisplay(): image_updated_(true)
  {
    ////////////////////////////////////////////////////////
    // initialize properties
    ////////////////////////////////////////////////////////
    far_clip_distance_property_ = new rviz::FloatProperty(
      "far clip",
      1.0,
      "far clip distance from the origin of camera info",
      this, SLOT(updateFarClipDistance()));
    show_edges_property_ = new rviz::BoolProperty(
      "show edges",
      true,
      "show edges of the region of the camera info",
      this, SLOT(updateShowEdges()));
    show_polygons_property_ = new rviz::BoolProperty(
      "show polygons",
      true,
      "show polygons of the region of the camera info",
      this, SLOT(updateShowPolygons()));
    not_show_side_polygons_property_ = new rviz::BoolProperty(
      "not show side polygons",
      true,
      "do not show polygons of the region of the camera info",
      this, SLOT(updateNotShowSidePolygons()));
    use_image_property_ = new rviz::BoolProperty(
      "use image",
      false,
      "use image as texture",
      this, SLOT(updateUseImage()));
    image_topic_property_ = new rviz::RosTopicProperty(
      "Image Topic", "",
      ros::message_traits::datatype<sensor_msgs::Image>(),
      "sensor_msgs::Image topic to subscribe to.",
      this, SLOT( updateImageTopic() ));
    color_property_ = new rviz::ColorProperty(
      "color",
      QColor(85, 255, 255),
      "color of CameraInfo",
      this, SLOT(updateColor()));
    edge_color_property_ = new rviz::ColorProperty(
      "edge color",
      QColor(125, 125, 125),
      "edge color of CameraInfo",
      this, SLOT(updateEdgeColor()));
    alpha_property_ = new rviz::FloatProperty(
      "alpha",
      0.5,
      "alpha blending value",
      this, SLOT(updateAlpha()));
  }
  
  CameraInfoDisplay::~CameraInfoDisplay()
  {
    if (edges_) {
      edges_->clear();
    }
    polygons_.clear();
    delete far_clip_distance_property_;
    delete color_property_;
    delete alpha_property_;
    delete show_polygons_property_;
    delete edge_color_property_;
  }

  void CameraInfoDisplay::reset()
  {
    MFDClass::reset();
    if (edges_) {
      edges_->clear();
    }
    polygons_.clear();
    camera_info_ = sensor_msgs::CameraInfo::ConstPtr(); // reset to NULL
  }

  void CameraInfoDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    updateColor();
    updateAlpha();
    updateFarClipDistance();
    updateShowPolygons();
    updateNotShowSidePolygons();
    updateShowEdges();
    updateImageTopic();
    updateUseImage();
    updateEdgeColor();
  }
  
  void CameraInfoDisplay::processMessage(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    if (!isSameCameraInfo(msg)) {
      createCameraInfoShapes(msg);
    }
    // move scene_node according to tf
     Ogre::Vector3 position;
     Ogre::Quaternion quaternion;
     if(!context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                   msg->header.stamp,
                                                   position,
                                                   quaternion)) {
       ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                  qPrintable( getName() ), msg->header.frame_id.c_str(),
                  qPrintable( fixed_frame_ ));
     }
     scene_node_->setPosition(position);
     scene_node_->setOrientation(quaternion);
     camera_info_ = msg;        // store for caching
  }

  void CameraInfoDisplay::update(float wall_dt, float ros_dt)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (image_updated_) {
      ROS_DEBUG("image updated");
      if (!bottom_texture_.isNull()) {
        drawImageTexture();
        image_updated_ = false;
      }
    }
  }

  bool CameraInfoDisplay::isSameCameraInfo(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    if (camera_info_) {
      bool meta_same_p = 
        msg->header.frame_id == camera_info_->header.frame_id &&
        msg->height == camera_info_->height &&
        msg->width == camera_info_->width &&
        msg->distortion_model == camera_info_->distortion_model;
      if (meta_same_p) {
        for (size_t i = 0; i < msg->P.size(); i++) {
          if (msg->P[i] != camera_info_->P[i]) {
            return false;
          }
        }
        return true;
      }
      else {
        return false;
      }
    }
    else {
      return false;
    }
  }
  
  void CameraInfoDisplay::addPointToEdge(
    const cv::Point3d& point)
  {
    Ogre::Vector3 p;
    p[0] = point.x;
    p[1] = point.y;
    p[2] = point.z;
    edges_->addPoint(p);
  }

  void CameraInfoDisplay::addPolygon(
    const cv::Point3d& O, const cv::Point3d& A, const cv::Point3d& B, std::string name, bool use_color, bool upper_triangle)
  {
    Ogre::ColourValue color = rviz::qtToOgre(color_);
    color.a = alpha_;
    TrianglePolygon::Ptr triangle (new TrianglePolygon(
                                     scene_manager_,
                                     scene_node_,
                                     O, A, B, name,
                                     color,
                                     use_color,
                                     upper_triangle));
    polygons_.push_back(triangle);
  }

  void CameraInfoDisplay::createTextureForBottom(int width, int height)
  {
    ROS_INFO("%dx%d", width, height);
    if (bottom_texture_.isNull() 
        || bottom_texture_->getWidth() != width
        || bottom_texture_->getHeight() != height) {
      static uint32_t count = 0;
      rviz::UniformStringStream ss;
      ss << "CameraInfoDisplayPolygonBottom" << count++;
      material_bottom_
        = Ogre::MaterialManager::getSingleton().create(
          ss.str(),
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      bottom_texture_ = Ogre::TextureManager::getSingleton().createManual(
        material_bottom_->getName() + "Texture",        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);
      material_bottom_->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
      Ogre::ColourValue color = rviz::qtToOgre(color_);
      color.a = alpha_;
      material_bottom_->getTechnique(0)->getPass(0)->setAmbient(color);
      material_bottom_->setReceiveShadows(false);
      material_bottom_->getTechnique(0)->setLightingEnabled(true);
      material_bottom_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
      material_bottom_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
      material_bottom_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
      material_bottom_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
    
      material_bottom_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
      material_bottom_->getTechnique(0)->getPass(0)->createTextureUnitState(bottom_texture_->getName());
      material_bottom_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    }
  }

  void CameraInfoDisplay::prepareMaterial()
  {
    if (texture_.isNull()) {
      // material
      static uint32_t count = 0;
      rviz::UniformStringStream ss;
      ss << "CameraInfoDisplayPolygon" << count++;
      material_
        = Ogre::MaterialManager::getSingleton().create(
          ss.str(),
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      texture_ = Ogre::TextureManager::getSingleton().createManual(
        material_->getName() + "Texture",        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, 1, 1, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);
      material_->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
      Ogre::ColourValue color = rviz::qtToOgre(color_);
      color.a = alpha_;
      material_->getTechnique(0)->getPass(0)->setAmbient(color);
      material_->setReceiveShadows(false);
      material_->getTechnique(0)->setLightingEnabled(true);
      material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
      material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
      material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
      material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
      
      material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
      material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_->getName());
      material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      createTextureForBottom(640, 480);
    }
  }

  void CameraInfoDisplay::subscribeImage(std::string topic)
  {
    
    image_sub_.shutdown();
    if (topic.empty()) {
      ROS_WARN("topic name is empty");
    }
    ros::NodeHandle nh;
    image_sub_ = nh.subscribe(topic, 1, 
                              &CameraInfoDisplay::imageCallback, this);
  }
  
  void CameraInfoDisplay::drawImageTexture()
  {
    bottom_texture_->getBuffer()->lock( Ogre::HardwareBuffer::HBL_NORMAL );
    const Ogre::PixelBox& pixelBox 
      = bottom_texture_->getBuffer()->getCurrentLock();
    Ogre::uint8* pDest = static_cast<Ogre::uint8*> (pixelBox.data);
    memset(pDest, 0, bottom_texture_->getWidth() * bottom_texture_->getHeight());
    QImage Hud(pDest, bottom_texture_->getWidth(), bottom_texture_->getHeight(), QImage::Format_ARGB32 );
    for (size_t j = 0; j < bottom_texture_->getHeight(); j++) {
      for (size_t i = 0; i < bottom_texture_->getWidth(); i++) {
        if (use_image_ && !image_.empty() &&
            bottom_texture_->getHeight() == image_.rows &&
            bottom_texture_->getWidth() == image_.cols) {
          ROS_DEBUG("bottom_texture_->getHeight(): %lu", bottom_texture_->getHeight());
          ROS_DEBUG("bottom_texture_->getWidth(): %lu", bottom_texture_->getWidth());
          ROS_DEBUG("image_.rows: %d", image_.rows);
          ROS_DEBUG("image_.cols: %d", image_.cols);
          QColor color(image_.data[j * image_.step + i * image_.elemSize() + 0],
                       image_.data[j * image_.step + i * image_.elemSize() + 1],
                       image_.data[j * image_.step + i * image_.elemSize() + 2],
                       alpha_ * 255.0);
          Hud.setPixel(i, j, color.rgba());
        }
        else {
          Hud.setPixel(i, j, color_.rgba());
        }
      }
    }
    bottom_texture_->getBuffer()->unlock();
  }

  // convert sensor_msgs::Image into cv::Mat
  void CameraInfoDisplay::imageCallback(
      const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      image_ = cv_ptr->image;
      // check the size of bottom texture
      if (bottom_texture_.isNull()
          || bottom_texture_->getWidth() != image_.cols
          || bottom_texture_->getHeight() != image_.rows) {
        createTextureForBottom(image_.cols, image_.rows);
        if (camera_info_) {
          createCameraInfoShapes(camera_info_);
        }
      }
      image_updated_ = true;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void CameraInfoDisplay::createCameraInfoShapes(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    polygons_.clear();
    if (edges_) {
      edges_->clear();
    }
    image_geometry::PinholeCameraModel model;
    bool model_success_p = model.fromCameraInfo(msg);
    if (!model_success_p) {
      ROS_ERROR("failed to create camera model");
      return;
    }
    
    ////////////////////////////////////////////////////////
    // initialize BillboardLine
    ////////////////////////////////////////////////////////
    if (!edges_) {
      edges_.reset(new rviz::BillboardLine(context_->getSceneManager(),
                                           scene_node_));
      edges_->setLineWidth(0.01);
    }
    
    cv::Point2d a(0, 0), b(msg->width, 0),
      c(msg->width, msg->height), d(0, msg->height);
    // all the z = 1.0
    cv::Point3d A = model.projectPixelTo3dRay(a);
    cv::Point3d B = model.projectPixelTo3dRay(b);
    cv::Point3d C = model.projectPixelTo3dRay(c);
    cv::Point3d D = model.projectPixelTo3dRay(d);

    cv::Point3d scaled_A = A * far_clip_distance_;
    cv::Point3d scaled_B = B * far_clip_distance_;
    cv::Point3d scaled_C = C * far_clip_distance_;
    cv::Point3d scaled_D = D * far_clip_distance_;

    cv::Point3d O(0, 0, 0);

    ////////////////////////////////////////////////////////
    // build polygons
    ////////////////////////////////////////////////////////
    if (show_polygons_) {
      ////////////////////////////////////////////////////////
      // setup color for polygons
      ////////////////////////////////////////////////////////
      Ogre::ColourValue color = rviz::qtToOgre(color_);
      color.a = alpha_;
      prepareMaterial();
      if (!not_show_side_polygons_) {
        material_->getTechnique(0)->getPass(0)->setAmbient(color);
        {      
          texture_->getBuffer()->lock( Ogre::HardwareBuffer::HBL_NORMAL );
          const Ogre::PixelBox& pixelBox 
            = texture_->getBuffer()->getCurrentLock();
          Ogre::uint8* pDest = static_cast<Ogre::uint8*> (pixelBox.data);
          memset(pDest, 0, 1);
          QImage Hud(pDest, 1, 1, QImage::Format_ARGB32 );
          Hud.setPixel(0, 0, color_.rgba());
          texture_->getBuffer()->unlock();
        }
        addPolygon(O, scaled_B, scaled_A, material_->getName(), true, true);
        addPolygon(O, scaled_C, scaled_B, material_->getName(), true, true);
        addPolygon(O, scaled_D, scaled_C, material_->getName(), true, true);
        addPolygon(O, scaled_A, scaled_D, material_->getName(), true, true);
      }
      // bottom
      drawImageTexture();

      addPolygon(scaled_A, scaled_B, scaled_D, material_bottom_->getName(), false, true);
      addPolygon(scaled_B, scaled_C, scaled_D, material_bottom_->getName(), false, false);
    }
    ////////////////////////////////////////////////////////
    // build edges
    ////////////////////////////////////////////////////////
    if (show_edges_) {
      edges_->clear();
      edges_->setMaxPointsPerLine(2);
      edges_->setNumLines(8);
      edges_->setColor(edge_color_.red() / 255.0,
                       edge_color_.green() / 255.0,
                       edge_color_.blue() / 255.0,
                       alpha_);
      addPointToEdge(O); addPointToEdge(scaled_A); edges_->newLine();
      addPointToEdge(O); addPointToEdge(scaled_B); edges_->newLine();
      addPointToEdge(O); addPointToEdge(scaled_C); edges_->newLine();
      addPointToEdge(O); addPointToEdge(scaled_D); edges_->newLine();
      addPointToEdge(scaled_A); addPointToEdge(scaled_B); edges_->newLine();
      addPointToEdge(scaled_B); addPointToEdge(scaled_C); edges_->newLine();
      addPointToEdge(scaled_C); addPointToEdge(scaled_D); edges_->newLine();
      addPointToEdge(scaled_D); addPointToEdge(scaled_A);
    }
  }

  ////////////////////////////////////////////////////////
  // Properties updating functions
  ////////////////////////////////////////////////////////
  void CameraInfoDisplay::updateColor()
  {
    color_ = color_property_->getColor();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateEdgeColor()
  {
    edge_color_ = edge_color_property_->getColor();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateFarClipDistance()
  {
    far_clip_distance_ = far_clip_distance_property_->getFloat();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateShowPolygons()
  {
    show_polygons_ = show_polygons_property_->getBool();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateShowEdges()
  {
    show_edges_ = show_edges_property_->getBool();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateImageTopic()
  {
    if (use_image_) {
      std::string topic = image_topic_property_->getStdString();
      subscribeImage(topic);
    }
  }

  void CameraInfoDisplay::updateUseImage()
  {
    use_image_ = use_image_property_->getBool();
    updateImageTopic();
  }
  void CameraInfoDisplay::updateNotShowSidePolygons()
  {
    not_show_side_polygons_ = not_show_side_polygons_property_->getBool();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::CameraInfoDisplay, rviz::Display )
