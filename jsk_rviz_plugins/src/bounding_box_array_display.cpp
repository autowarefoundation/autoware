/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include "bounding_box_array_display.h"
#include <jsk_topic_tools/color_utils.h>
namespace jsk_rviz_plugins
{  
  BoundingBoxArrayDisplay::BoundingBoxArrayDisplay()
  {
    coloring_property_ = new rviz::EnumProperty(
      "coloring", "Auto",
      "coloring method",
      this, SLOT(updateColoring()));
    coloring_property_->addOption("Auto", 0);
    coloring_property_->addOption("Flat color", 1);
    coloring_property_->addOption("Label", 2);
    coloring_property_->addOption("Value", 3);
    
    color_property_ = new rviz::ColorProperty(
      "color", QColor(25, 255, 0),
      "color to draw the bounding boxes",
      this, SLOT(updateColor()));
    alpha_property_ = new rviz::FloatProperty(
      "alpha", 0.8,
      "alpha value to draw the bounding boxes",
      this, SLOT(updateAlpha()));
    only_edge_property_ = new rviz::BoolProperty(
      "only edge", false,
      "show only the edges of the boxes",
      this, SLOT(updateOnlyEdge()));
    line_width_property_ = new rviz::FloatProperty(
      "line width", 0.005,
      "line width of the edges",
      this, SLOT(updateLineWidth()));
    show_coords_property_ = new rviz::BoolProperty(
      "show coords", true,
      "show coordinate of bounding box",
      this, SLOT(updateShowCoords()));
  }

  BoundingBoxArrayDisplay::~BoundingBoxArrayDisplay()
  {
    delete color_property_;
    delete alpha_property_;
    delete only_edge_property_;
    delete coloring_property_;
    delete show_coords_property_;
  }

  QColor BoundingBoxArrayDisplay::getColor(
    size_t index,
    const jsk_recognition_msgs::BoundingBox& box,
    double min_value,
    double max_value)
  {
    if (coloring_method_ == "auto") {
      std_msgs::ColorRGBA ros_color = jsk_topic_tools::colorCategory20(index);
      return QColor(ros_color.r * 255.0,
                    ros_color.g * 255.0,
                    ros_color.b * 255.0,
                    ros_color.a * 255.0);
    }
    else if (coloring_method_ == "flat") {
      return color_;
    }
    else if (coloring_method_ == "label") {
      std_msgs::ColorRGBA ros_color = jsk_topic_tools::colorCategory20(box.label);
      return QColor(ros_color.r * 255.0,
                    ros_color.g * 255.0,
                    ros_color.b * 255.0,
                    ros_color.a * 255.0);
    }
    else if (coloring_method_ == "value") {
      if (min_value != max_value) {
        std_msgs::ColorRGBA ros_color = jsk_topic_tools::heatColor((box.value - min_value) / (max_value - min_value));
        return QColor(ros_color.r * 255.0,
                      ros_color.g * 255.0,
                      ros_color.b * 255.0,
                      ros_color.a * 255.0);
      }
      else {
        return QColor(255.0, 255.0, 255.0, 255.0);
      }
    }
  }
  
  void BoundingBoxArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    updateColor();
    updateAlpha();
    updateOnlyEdge();
    updateColoring();
    updateLineWidth();
    updateShowCoords();
  }

  void BoundingBoxArrayDisplay::updateLineWidth()
  {
    line_width_ = line_width_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }
  
  void BoundingBoxArrayDisplay::updateColor()
  {
    color_ = color_property_->getColor();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxArrayDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }

  void BoundingBoxArrayDisplay::updateOnlyEdge()
  {
    only_edge_ = only_edge_property_->getBool();
    if (only_edge_) {
      line_width_property_->show();
    }
    else {
      line_width_property_->hide();;
    }
    // Imediately apply attribute
    if (latest_msg_) {
      if (only_edge_) {
        showEdges(latest_msg_);
      }
      else {
        showBoxes(latest_msg_);
      }
    }
  }

  void BoundingBoxArrayDisplay::updateColoring()
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
      coloring_method_ = "label";
      color_property_->hide();
    }
    else if (coloring_property_->getOptionInt() == 3) {
      coloring_method_ = "value";
      color_property_->hide();
    }
    
    if (latest_msg_) {
      processMessage(latest_msg_);
    }
  }
  
  void BoundingBoxArrayDisplay::updateShowCoords()
  {
    show_coords_ = show_coords_property_->getBool();
    // Immediately apply show_coords attribute
    if (!show_coords_) {
      hideCoords();
    }
    else if (show_coords_ && latest_msg_) {
      showCoords(latest_msg_);
    }
  }

  void BoundingBoxArrayDisplay::reset()
  {
    MFDClass::reset();
    shapes_.clear();
    edges_.clear();
    coords_nodes_.clear();
    coords_objects_.clear();
    latest_msg_.reset();
  }

  void BoundingBoxArrayDisplay::allocateShapes(int num)
  {
    if (num > shapes_.size()) {
      for (size_t i = shapes_.size(); i < num; i++) {
        ShapePtr shape (new rviz::Shape(
                          rviz::Shape::Cube, context_->getSceneManager(),
                          scene_node_));
        shapes_.push_back(shape);
      }
    }
    else if (num < shapes_.size())
    {
      shapes_.resize(num);
    }
  }

  void BoundingBoxArrayDisplay::allocateBillboardLines(int num)
  {
    if (num > edges_.size()) {
      for (size_t i = edges_.size(); i < num; i++) {
        BillboardLinePtr line(new rviz::BillboardLine(
                                context_->getSceneManager(), scene_node_));
        edges_.push_back(line);
      }
    }
    else if (num < edges_.size())
      {
        edges_.resize(num);       // ok??
      }
  }

  void BoundingBoxArrayDisplay::allocateCoords(int num)
  {
    if (num > coords_objects_.size()) {
      for (size_t i = coords_objects_.size(); i < num; i++) {
        Ogre::SceneNode* scene_node = scene_node_->createChildSceneNode();
        std::vector<ArrowPtr> coord;
        for (int i = 0; i < 3; i++) {
          ArrowPtr arrow (new rviz::Arrow(scene_manager_, scene_node));
          coord.push_back(arrow);
        }
        coords_nodes_.push_back(scene_node);
        coords_objects_.push_back(coord);
      }
    }
    else if (num < coords_objects_.size()) {
      coords_objects_.resize(num);
    }
  }

  bool BoundingBoxArrayDisplay::isValid(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg)
  {
    // Check size
    for (size_t i = 0; i < msg->boxes.size(); i++) {
      jsk_recognition_msgs::BoundingBox box = msg->boxes[i];
      if (box.dimensions.x < 1.0e-9 || 
          box.dimensions.y < 1.0e-9 ||
          box.dimensions.z < 1.0e-9 ||
          std::isnan(box.dimensions.x) ||
          std::isnan(box.dimensions.y) ||
          std::isnan(box.dimensions.z)) {
        ROS_FATAL("Size of bounding box is [%f, %f, %f]",
                  box.dimensions.x,
                  box.dimensions.y,
                  box.dimensions.z);
        return false;
      }
    }
    return true;
  }

  void BoundingBoxArrayDisplay::showBoxes(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg)
  {
    edges_.clear();
    allocateShapes(msg->boxes.size());
    float min_value = DBL_MAX;
    float max_value = -DBL_MAX;
    for (size_t i = 0; i < msg->boxes.size(); i++) {
      min_value = std::min(min_value, msg->boxes[i].value);
      max_value = std::max(max_value, msg->boxes[i].value);
    }
    for (size_t i = 0; i < msg->boxes.size(); i++) {
      jsk_recognition_msgs::BoundingBox box = msg->boxes[i];
      ShapePtr shape = shapes_[i];
      Ogre::Vector3 position;
      Ogre::Quaternion quaternion;
      if(!context_->getFrameManager()->transform(box.header, box.pose,
                                                 position,
                                                 quaternion)) {
        ROS_ERROR( "Error transforming pose"
                   "'%s' from frame '%s' to frame '%s'",
                   qPrintable( getName() ), box.header.frame_id.c_str(),
                   qPrintable( fixed_frame_ ));
        return;                 // return?
      }
      shape->setPosition(position);
      shape->setOrientation(quaternion);
      Ogre::Vector3 dimensions;
      dimensions[0] = box.dimensions.x;
      dimensions[1] = box.dimensions.y;
      dimensions[2] = box.dimensions.z;
      shape->setScale(dimensions);
      QColor color = getColor(i, box, min_value, max_value);
      shape->setColor(color.red() / 255.0,
                      color.green() / 255.0,
                      color.blue() / 255.0,
                      alpha_);
    }
  }

  void BoundingBoxArrayDisplay::showEdges(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg)
  {
    shapes_.clear();
    allocateBillboardLines(msg->boxes.size());
    float min_value = DBL_MAX;
    float max_value = -DBL_MAX;
    for (size_t i = 0; i < msg->boxes.size(); i++) {
      min_value = std::min(min_value, msg->boxes[i].value);
      max_value = std::max(max_value, msg->boxes[i].value);
    }

    for (size_t i = 0; i < msg->boxes.size(); i++) {
      jsk_recognition_msgs::BoundingBox box = msg->boxes[i];
      geometry_msgs::Vector3 dimensions = box.dimensions;
      
      BillboardLinePtr edge = edges_[i];
      edge->clear();
      Ogre::Vector3 position;
      Ogre::Quaternion quaternion;
      if(!context_->getFrameManager()->transform(box.header, box.pose,
                                                 position,
                                                 quaternion)) {
        ROS_ERROR( "Error transforming pose"
                   "'%s' from frame '%s' to frame '%s'",
                   qPrintable( getName() ), box.header.frame_id.c_str(),
                   qPrintable( fixed_frame_ ));
        return;                 // return?
      }
      edge->setPosition(position);
      edge->setOrientation(quaternion);

      edge->setMaxPointsPerLine(2);
      edge->setNumLines(12);
      edge->setLineWidth(line_width_);
      QColor color = getColor(i, box, min_value, max_value);
      edge->setColor(color.red() / 255.0,
                     color.green() / 255.0,
                     color.blue() / 255.0,
                     alpha_);


      
      Ogre::Vector3 A, B, C, D, E, F, G, H;
      A[0] = dimensions.x / 2.0;
      A[1] = dimensions.y / 2.0;
      A[2] = dimensions.z / 2.0;
      B[0] = - dimensions.x / 2.0;
      B[1] = dimensions.y / 2.0;
      B[2] = dimensions.z / 2.0;
      C[0] = - dimensions.x / 2.0;
      C[1] = - dimensions.y / 2.0;
      C[2] = dimensions.z / 2.0;
      D[0] = dimensions.x / 2.0;
      D[1] = - dimensions.y / 2.0;
      D[2] = dimensions.z / 2.0;

      E[0] = dimensions.x / 2.0;
      E[1] = dimensions.y / 2.0;
      E[2] = - dimensions.z / 2.0;
      F[0] = - dimensions.x / 2.0;
      F[1] = dimensions.y / 2.0;
      F[2] = - dimensions.z / 2.0;
      G[0] = - dimensions.x / 2.0;
      G[1] = - dimensions.y / 2.0;
      G[2] = - dimensions.z / 2.0;
      H[0] = dimensions.x / 2.0;
      H[1] = - dimensions.y / 2.0;
      H[2] = - dimensions.z / 2.0;
      
      edge->addPoint(A); edge->addPoint(B); edge->newLine();
      edge->addPoint(B); edge->addPoint(C); edge->newLine();
      edge->addPoint(C); edge->addPoint(D); edge->newLine();
      edge->addPoint(D); edge->addPoint(A); edge->newLine();
      edge->addPoint(E); edge->addPoint(F); edge->newLine();
      edge->addPoint(F); edge->addPoint(G); edge->newLine();
      edge->addPoint(G); edge->addPoint(H); edge->newLine();
      edge->addPoint(H); edge->addPoint(E); edge->newLine();
      edge->addPoint(A); edge->addPoint(E); edge->newLine();
      edge->addPoint(B); edge->addPoint(F); edge->newLine();
      edge->addPoint(C); edge->addPoint(G); edge->newLine();
      edge->addPoint(D); edge->addPoint(H);
    }
  }

  void BoundingBoxArrayDisplay::showCoords(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg)
  {
    allocateCoords(msg->boxes.size());
    for (size_t i = 0; i < msg->boxes.size(); i++) {
      jsk_recognition_msgs::BoundingBox box = msg->boxes[i];
      std::vector<ArrowPtr> coord = coords_objects_[i];

      Ogre::SceneNode* scene_node = coords_nodes_[i];
      scene_node->setVisible(true);
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      if(!context_->getFrameManager()->getTransform(
           box.header, position, orientation)) {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                  box.header.frame_id.c_str(), qPrintable(fixed_frame_));
        return;
      }
      scene_node->setPosition(position);
      scene_node->setOrientation(orientation); // scene node is at frame pose

      float color[3][3] = {{1, 0, 0},
                           {0, 1, 0},
                           {0, 0, 1}};
      for (int j = 0; j < 3; j++) {
        // check radius diraction
        Ogre::Vector3 scale;
        if (color[j][0] == 1) {
          scale = Ogre::Vector3(
            box.dimensions.x,
            std::min(box.dimensions.y, box.dimensions.z),
            std::min(box.dimensions.y, box.dimensions.z));
        }
        if (color[j][1] == 1) {
          scale = Ogre::Vector3(
            box.dimensions.y,
            std::min(box.dimensions.x, box.dimensions.z),
            std::min(box.dimensions.x, box.dimensions.z));
        }
        if (color[j][2] == 1) {
          scale = Ogre::Vector3(
            box.dimensions.z,
            std::min(box.dimensions.x, box.dimensions.y),
            std::min(box.dimensions.x, box.dimensions.y));
        }
          
        Ogre::Vector3 direction(color[j][0], color[j][1], color[j][2]);
        Ogre::Vector3 pos(box.pose.position.x,
                          box.pose.position.y,
                          box.pose.position.z);
        Ogre::Quaternion qua(box.pose.orientation.w,
                             box.pose.orientation.x,
                             box.pose.orientation.y,
                             box.pose.orientation.z);
        direction = qua * direction;
        Ogre::ColourValue rgba;
        rgba.a = 1;
        rgba.r = color[j][0];
        rgba.g = color[j][1];
        rgba.b = color[j][2];

        ArrowPtr arrow = coords_objects_[i][j];
        arrow->setPosition(pos);
        arrow->setDirection(direction);
        arrow->setScale(scale);
        arrow->setColor(rgba);
      }
    }
  }

  void BoundingBoxArrayDisplay::hideCoords()
  {
    for (size_t i = 0; i < coords_nodes_.size(); i++) {
      coords_nodes_[i]->setVisible(false);
    }
  }
  
  void BoundingBoxArrayDisplay::processMessage(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg)
  {
    if (!isValid(msg)) {
      return;
    }
    // Store latest message
    latest_msg_ = msg;
    
    if (!only_edge_) {
      showBoxes(msg);
    }
    else {
      showEdges(msg);
    }

    if (show_coords_) {
      showCoords(msg);
    }
    else {
      hideCoords();
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
  jsk_rviz_plugins::BoundingBoxArrayDisplay, rviz::Display)
