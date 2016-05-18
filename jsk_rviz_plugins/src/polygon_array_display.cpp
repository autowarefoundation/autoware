// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#define BOOST_PARAMETER_MAX_ARITY 7

#include "polygon_array_display.h"
#include "rviz/properties/parse_color.h"
#include <rviz/validate_floats.h>
#include <jsk_topic_tools/color_utils.h>
#include <jsk_recognition_utils/geo/polygon.h>

namespace jsk_rviz_plugins
{
  PolygonArrayDisplay::PolygonArrayDisplay()
  {
    coloring_property_ = new rviz::EnumProperty(
      "coloring", "Auto",
      "coloring method",
      this, SLOT(updateColoring()));
    coloring_property_->addOption("Auto", 0);
    coloring_property_->addOption("Flat color", 1);
    coloring_property_->addOption("Liekelihood", 2);
    coloring_property_->addOption("Label", 3);
    color_property_ = new rviz::ColorProperty(
      "Color", QColor(25, 255, 0),
      "Color to draw the polygons.",
      this, SLOT(queueRender()));
    alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0,
      "Amount of transparency to apply to the polygon.",
      this, SLOT(queueRender()));
    only_border_property_ = new rviz::BoolProperty(
      "only border", true,
      "only shows the borders of polygons",
      this, SLOT(updateOnlyBorder()));
    show_normal_property_ = new rviz::BoolProperty(
      "show normal", true,
      "show normal direction",
      this, SLOT(updateShowNormal()));
    normal_length_property_ = new rviz::FloatProperty(
      "normal length", 0.1,
      "normal length",
      this, SLOT(updateNormalLength()));
    normal_length_property_->setMin(0);
    //only_border_ = true;
    alpha_property_->setMin(0);
    alpha_property_->setMax(1);
  }
  
  PolygonArrayDisplay::~PolygonArrayDisplay()
  {
    delete alpha_property_;
    delete color_property_;
    delete only_border_property_;
    delete coloring_property_;
    delete show_normal_property_;
    delete normal_length_property_;
    for (size_t i = 0; i < lines_.size(); i++) {
      delete lines_[i];
    }
    
    for (size_t i = 0; i < materials_.size(); i++) {
      materials_[i]->unload();
      Ogre::MaterialManager::getSingleton().remove(materials_[i]->getName());
    }
    
    for (size_t i = 0; i < manual_objects_.size(); i++) {
      scene_manager_->destroyManualObject(manual_objects_[i]);
      scene_manager_->destroySceneNode(scene_nodes_[i]);
    }
  }
  
  void PolygonArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    updateOnlyBorder();
    updateColoring();
    updateShowNormal();
    updateNormalLength();
  }

  void PolygonArrayDisplay::allocateMaterials(int num)
  {
    if (only_border_) {
      return;
    }
    static uint32_t count = 0;
    
    if (num > materials_.size()) {
      for (size_t i = materials_.size(); num > i; i++) {
        std::stringstream ss;
        ss << "PolygonArrayMaterial" << count++;
        Ogre::MaterialPtr material
          = Ogre::MaterialManager::getSingleton().create(ss.str(), "rviz");
        material->setReceiveShadows(false);
        material->getTechnique(0)->setLightingEnabled(true);
        material->getTechnique(0)->setAmbient(0.5, 0.5, 0.5);
        materials_.push_back(material);
      }
    }
  }
  
  bool validateFloats(const jsk_recognition_msgs::PolygonArray& msg)
  {
    for (size_t i = 0; i < msg.polygons.size(); i++) {
      if (!rviz::validateFloats(msg.polygons[i].polygon.points))
        return false;
    }
    return true;
  }
  
  void PolygonArrayDisplay::reset()
  {
    MFDClass::reset();
    for (size_t i = 0; i < manual_objects_.size(); i++) {
      manual_objects_[i]->clear();
    }
  }

  void PolygonArrayDisplay::updateSceneNodes(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    int scale_factor = 2;
    if (only_border_) {
      scale_factor = 1;
    }
    if (msg->polygons.size() * scale_factor > manual_objects_.size()) {
      for (size_t i = manual_objects_.size();
           i < msg->polygons.size() * scale_factor;
           i++) {
        Ogre::SceneNode* scene_node
          = scene_node_->createChildSceneNode();
        Ogre::ManualObject* manual_object
          = scene_manager_->createManualObject();
        manual_object->setDynamic(true);
        scene_node->attachObject(manual_object);
        manual_objects_.push_back(manual_object);
        scene_nodes_.push_back(scene_node);
      }
    }
    else if (msg->polygons.size() * scale_factor < manual_objects_.size()) {
      for (size_t i = msg->polygons.size() * scale_factor;
           i < manual_objects_.size(); i++) {
        manual_objects_[i]->setVisible(false);
      }
    }
    // arrow nodes
    if (msg->polygons.size() > arrow_objects_.size()) {
      for (size_t i = arrow_objects_.size(); i < msg->polygons.size(); i++) {
        Ogre::SceneNode* scene_node = scene_node_->createChildSceneNode();
        ArrowPtr arrow (new rviz::Arrow(scene_manager_, scene_node));
        scene_node->setVisible(false);
        arrow_objects_.push_back(arrow);
        arrow_nodes_.push_back(scene_node);
      }
    }
    else if (msg->polygons.size() < manual_objects_.size()) {
      for (size_t i = msg->polygons.size(); i < arrow_nodes_.size(); i++) {
        //arrow_objects_[i]->setVisible(false);
        arrow_nodes_[i]->setVisible(false);
      }
    }
  }

  void PolygonArrayDisplay::updateLines(int num)
  {
    if (num > lines_.size()) {
      for (size_t i = lines_.size(); i < num; i++) {
        rviz::BillboardLine* line
          = new rviz::BillboardLine(context_->getSceneManager(),
                                    scene_nodes_[i]);
        line->setLineWidth(0.01);
        line->setNumLines(1);
        lines_.push_back(line);
      }
    }
    for (size_t i = 0; i < lines_.size(); i++) {
      lines_[i]->clear();
    }
  }
  
  Ogre::ColourValue PolygonArrayDisplay::getColor(size_t index)
  {
    Ogre::ColourValue color;
    if (coloring_method_ == "auto") {
      std_msgs::ColorRGBA ros_color = jsk_topic_tools::colorCategory20(index);
      color.r = ros_color.r;
      color.g = ros_color.g;
      color.b = ros_color.b;
      color.a = ros_color.a;
    }
    else if (coloring_method_ == "flat") {
      color = rviz::qtToOgre(color_property_->getColor());
    }
    else if (coloring_method_ == "likelihood") {
      if (latest_msg_->likelihood.size() == 0 ||
          latest_msg_->likelihood.size() < index) {
        setStatus(rviz::StatusProperty::Error,
                  "Topic",
                  "Message does not have lieklihood fields");
      }
      else {
        std_msgs::ColorRGBA ros_color
          = jsk_topic_tools::heatColor(latest_msg_->likelihood[index]);
        color.r = ros_color.r;
        color.g = ros_color.g;
        color.b = ros_color.b;
        color.a = ros_color.a;
      }
    }
    else if (coloring_method_ == "label") {
      if (latest_msg_->labels.size() == 0 ||
          latest_msg_->labels.size() < index) {
        setStatus(rviz::StatusProperty::Error,
                  "Topic",
                  "Message does not have lebels fields");
      }
      else {
        std_msgs::ColorRGBA ros_color
          = jsk_topic_tools::colorCategory20(latest_msg_->labels[index]);
        color.r = ros_color.r;
        color.g = ros_color.g;
        color.b = ros_color.b;
        color.a = ros_color.a;
      }
    }
    color.a = alpha_property_->getFloat();
    return color;
  }

  void PolygonArrayDisplay::processLine(
    const size_t i,
    const geometry_msgs::PolygonStamped& polygon)
  {
    Ogre::SceneNode* scene_node = scene_nodes_[i];
    //Ogre::ManualObject* manual_object = manual_objects_[i];
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(
         polygon.header, position, orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                 polygon.header.frame_id.c_str(), qPrintable(fixed_frame_));
    }
    scene_node->setPosition(position);
    scene_node->setOrientation(orientation);
    rviz::BillboardLine* line = lines_[i];
    line->clear();
    line->setMaxPointsPerLine(polygon.polygon.points.size() + 1);
        
    Ogre::ColourValue color = getColor(i);
    line->setColor(color.r, color.g, color.b, color.a);

    for (size_t i = 0; i < polygon.polygon.points.size(); ++i) {
      Ogre::Vector3 step_position;
      step_position.x = polygon.polygon.points[i].x;
      step_position.y = polygon.polygon.points[i].y;
      step_position.z = polygon.polygon.points[i].z;
      line->addPoint(step_position);
    }
    Ogre::Vector3 step_position;
    step_position.x = polygon.polygon.points[0].x;
    step_position.y = polygon.polygon.points[0].y;
    step_position.z = polygon.polygon.points[0].z;
    line->addPoint(step_position);
  }

  void PolygonArrayDisplay::processPolygonMaterial(const size_t i)
  {
    Ogre::ColourValue color = getColor(i);
    materials_[i]->getTechnique(0)->setAmbient(color * 0.5);
    materials_[i]->getTechnique(0)->setDiffuse(color);
    if (color.a < 0.9998) {
      materials_[i]->getTechnique(0)->setSceneBlending(
        Ogre::SBT_TRANSPARENT_ALPHA);
      materials_[i]->getTechnique(0)->setDepthWriteEnabled(false);
    }
    else {
      materials_[i]->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
      materials_[i]->getTechnique(0)->setDepthWriteEnabled(true);
    }
      
    materials_[i]->getTechnique(0)->setAmbient(color * 0.5);
    materials_[i]->getTechnique(0)->setDiffuse(color);
  }

  void PolygonArrayDisplay::processPolygon(
    const size_t i, const geometry_msgs::PolygonStamped& polygon)
  {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(
         polygon.header, position, orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                 polygon.header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }
    
    {
      Ogre::SceneNode* scene_node = scene_nodes_[i * 2];
      Ogre::ManualObject* manual_object = manual_objects_[i * 2];
      scene_node->setPosition(position);
      scene_node->setOrientation(orientation);
      manual_object->clear();
      manual_object->setVisible(true);
      
      jsk_recognition_utils::Polygon geo_polygon
        = jsk_recognition_utils::Polygon::fromROSMsg(polygon.polygon);
      std::vector<jsk_recognition_utils::Polygon::Ptr>
        triangles = geo_polygon.decomposeToTriangles();
        
      uint32_t num_points = 0;
      for (size_t j = 0; j < triangles.size(); j++) {
        num_points += triangles[j]->getNumVertices();
      }
      if(num_points > 0) {
        manual_object->estimateVertexCount(num_points * 2);
        manual_object->begin(
          materials_[i]->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
        for (size_t ii = 0; ii < triangles.size(); ii++) {
          jsk_recognition_utils::Polygon::Ptr triangle = triangles[ii];
          size_t num_vertices = triangle->getNumVertices();
          for (size_t j = 0; j < num_vertices; j++) {
            Eigen::Vector3f v = triangle->getVertex(j);
            manual_object->position(v[0], v[1], v[2]);
          }
          for (int j = num_vertices - 1; j >= 0; j--) {
            Eigen::Vector3f v = triangle->getVertex(j);
            manual_object->position(v[0], v[1], v[2]);
          }
        }
        manual_object->end();
      }
    }
  }
  
  void PolygonArrayDisplay::processNormal(
    const size_t i, const geometry_msgs::PolygonStamped& polygon)
  {
    Ogre::SceneNode* scene_node = arrow_nodes_[i];
    scene_node->setVisible(true);
    ArrowPtr arrow = arrow_objects_[i];
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(
         polygon.header, position, orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                 polygon.header.frame_id.c_str(), qPrintable(fixed_frame_));
      return;
    }
    scene_node->setPosition(position);
    scene_node->setOrientation(orientation); // scene node is at frame pose
    jsk_recognition_utils::Polygon geo_polygon
      = jsk_recognition_utils::Polygon::fromROSMsg(polygon.polygon);
    jsk_recognition_utils::Vertices vertices
      = geo_polygon.getVertices();
    Eigen::Vector3f centroid(0, 0, 0); // should be replaced by centroid method
    if (vertices.size() == 0) {
      ROS_ERROR("the size of vertices is 0");
    }
    else {
      for (size_t j = 0; j < vertices.size(); j++) {
        centroid = vertices[j] + centroid;
      }
      centroid = centroid / vertices.size();
    }
    Ogre::Vector3 pos(centroid[0], centroid[1], centroid[2]);
    Eigen::Vector3f normal = geo_polygon.getNormal();
    Ogre::Vector3 direction(normal[0], normal[1], normal[2]);
    if (isnan(direction[0]) || isnan(direction[1]) || isnan(direction[2])) {
      ROS_ERROR("failed to compute normal direction");
      Ogre::Vector3 zeroscale(0, 0, 0);
      arrow->setScale(zeroscale);
      return;
    }
    Ogre::Vector3 scale(normal_length_, normal_length_, normal_length_);
    arrow->setPosition(pos);
    arrow->setDirection(direction);
    
    arrow->setScale(scale);
    arrow->setColor(getColor(i));
  }
  
  void PolygonArrayDisplay::processMessage(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    if (!validateFloats(*msg)) {
      setStatus(rviz::StatusProperty::Error,
                "Topic",
                "Message contained invalid floating point values"
                "(nans or infs)");
      return;
    }
    setStatus(rviz::StatusProperty::Ok,
              "Topic",
              "ok");
    latest_msg_ = msg;
    // create nodes and manual objects
    updateSceneNodes(msg);
    allocateMaterials(msg->polygons.size());
    updateLines(msg->polygons.size());
    if (only_border_) {
      // use line_
      for (size_t i = 0; i < manual_objects_.size(); i++) {
        manual_objects_[i]->setVisible(false);
      }
      for (size_t i = 0; i < msg->polygons.size(); i++) {
        geometry_msgs::PolygonStamped polygon = msg->polygons[i];
        if (polygon.polygon.points.size() >= 3) {
          processLine(i, polygon);
        }
      }
    }
    else {
      for (size_t i = 0; i < msg->polygons.size(); i++) {
        processPolygonMaterial(i);
      }
      
      for (size_t i = 0; i < msg->polygons.size(); i++) {
        geometry_msgs::PolygonStamped polygon = msg->polygons[i];
        processPolygon(i, polygon);
      }
    }

    if (show_normal_) {
      for (size_t i = 0; i < msg->polygons.size(); i++) {
        geometry_msgs::PolygonStamped polygon = msg->polygons[i];
        processNormal(i, polygon);
      }
    }
  }

  void PolygonArrayDisplay::updateColoring()
  {
    if (coloring_property_->getOptionInt() == 0) {
      coloring_method_ = "auto";
      color_property_->hide();
    }
    else if (coloring_property_->getOptionInt() == 1) {
      coloring_method_ = "flat";
      color_property_->show();
    }
    else if (coloring_property_->getOptionInt() == 2) {
      coloring_method_ = "likelihood";
      color_property_->hide();
    }
    else if (coloring_property_->getOptionInt() == 3) {
      coloring_method_ = "label";
      color_property_->hide();
    }
  }

  void PolygonArrayDisplay::updateOnlyBorder()
  {
    only_border_ = only_border_property_->getBool();
  }

  void PolygonArrayDisplay::updateShowNormal()
  {
    show_normal_ = show_normal_property_->getBool();
    if (show_normal_) {
      normal_length_property_->show();
    }
    else {
      normal_length_property_->hide();
      for (size_t i = 0; i < arrow_nodes_.size(); i++) {
        arrow_nodes_[i]->setVisible(false);
      }
    }
  }

  void PolygonArrayDisplay::updateNormalLength()
  {
    normal_length_ = normal_length_property_->getFloat();
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::PolygonArrayDisplay, rviz::Display)
