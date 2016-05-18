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


#ifndef JSK_RVIZ_PLUGINS_CAMERA_INFO_DISPLAY_H_
#define JSK_RVIZ_PLUGINS_CAMERA_INFO_DISPLAY_H_

#include <rviz/display.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <OGRE/OgreSceneNode.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_rviz_plugins
{
  class TrianglePolygon
  {
  public:
    typedef boost::shared_ptr<TrianglePolygon> Ptr;
    TrianglePolygon(Ogre::SceneManager* manager,
                    Ogre::SceneNode* node,
                    const cv::Point3d& O,
                    const cv::Point3d& A,
                    const cv::Point3d& B,
                    const std::string& name,
                    const Ogre::ColourValue& color,
                    bool use_color,
                    bool upper_triangle);
    virtual ~TrianglePolygon();
  protected:
    Ogre::ManualObject* manual_;
    Ogre::SceneManager* manager_;
  private:
    
  };
  
  class CameraInfoDisplay:
    public rviz::MessageFilterDisplay<sensor_msgs::CameraInfo>
  {
    Q_OBJECT
  public:
    typedef boost::shared_ptr<rviz::Shape> ShapePtr;
    typedef boost::shared_ptr<rviz::BillboardLine> BillboardLinePtr;
    CameraInfoDisplay();
    virtual ~CameraInfoDisplay();
    
  protected:
    ////////////////////////////////////////////////////////
    // methods required by super virtual class
    ////////////////////////////////////////////////////////
    virtual void onInitialize();
    virtual void reset();
    virtual void processMessage(const sensor_msgs::CameraInfo::ConstPtr& msg);
    ////////////////////////////////////////////////////////
    // methods
    ///////////////////////////////////////////////////////
    virtual void update(float wall_dt, float ros_dt);
    virtual bool isSameCameraInfo(
      const sensor_msgs::CameraInfo::ConstPtr& camera_info);
    virtual void createCameraInfoShapes(
      const sensor_msgs::CameraInfo::ConstPtr& camera_info);
    virtual void addPointToEdge(
      const cv::Point3d& point);
    virtual void addPolygon(
      const cv::Point3d& O, const cv::Point3d& A, const cv::Point3d& B, std::string name,
      bool use_color, bool upper_triangle);
    virtual void prepareMaterial();
    virtual void createTextureForBottom(int width, int height);
    virtual void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    virtual void drawImageTexture();
    virtual void subscribeImage(std::string topic);
    /////////////////////////////////////////////////////////
    // variables
    //////////////////////////////////////////////////////// 
    std::vector<TrianglePolygon::Ptr> polygons_;
    BillboardLinePtr edges_;
    sensor_msgs::CameraInfo::ConstPtr camera_info_;
    Ogre::MaterialPtr material_;
    Ogre::TexturePtr texture_;
    Ogre::MaterialPtr material_bottom_;
    Ogre::TexturePtr bottom_texture_;
    ros::Subscriber image_sub_;
    boost::mutex mutex_;
    ////////////////////////////////////////////////////////
    // variables updated by rviz properties
    ////////////////////////////////////////////////////////
    double alpha_;
    double far_clip_distance_;
    QColor color_;
    QColor edge_color_;
    bool show_polygons_;
    bool show_edges_;
    bool use_image_;
    bool image_updated_;
    bool not_show_side_polygons_;
    cv::Mat image_;
    ////////////////////////////////////////////////////////
    // properties
    ////////////////////////////////////////////////////////
    rviz::FloatProperty* far_clip_distance_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::ColorProperty* color_property_;
    rviz::ColorProperty* edge_color_property_;
    rviz::BoolProperty* show_polygons_property_;
    rviz::BoolProperty* not_show_side_polygons_property_;
    rviz::BoolProperty* use_image_property_;
    rviz::RosTopicProperty* image_topic_property_;
    rviz::BoolProperty* show_edges_property_;
    
  protected Q_SLOTS:
    void updateFarClipDistance();
    void updateAlpha();
    void updateColor();
    void updateShowEdges();
    void updateShowPolygons();
    void updateNotShowSidePolygons();
    void updateImageTopic();
    void updateUseImage();
    void updateEdgeColor();
  };
  
}

#endif
