/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab.
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

#ifndef JSK_RVIZ_PLUGINS_BOUDNING_BOX_DISPLAY_COMMON_H_
#define JSK_RVIZ_PLUGINS_BOUDNING_BOX_DISPLAY_COMMON_H_

#ifndef Q_MOC_RUN
#include "bounding_box_display_common.h"
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_topic_tools/color_utils.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/arrow.h>
#include <OGRE/OgreSceneNode.h>
#endif

namespace jsk_rviz_plugins
{

  template <class MessageType>
  class BoundingBoxDisplayCommon: public rviz::MessageFilterDisplay<MessageType>
  {
public:
    BoundingBoxDisplayCommon() {};
    ~BoundingBoxDisplayCommon() {};
    typedef std::shared_ptr<rviz::Shape> ShapePtr;
    typedef std::shared_ptr<rviz::BillboardLine> BillboardLinePtr;
    typedef std::shared_ptr<rviz::Arrow> ArrowPtr;

protected:
    QColor color_;
    std::string coloring_method_;
    double alpha_;
    double line_width_;

    std::vector<std::vector<ArrowPtr> > coords_objects_;
    std::vector<Ogre::SceneNode*> coords_nodes_;
    std::vector<BillboardLinePtr> edges_;
    std::vector<ShapePtr> shapes_;

    QColor getColor(
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

    bool isValidBoundingBox(
      const jsk_recognition_msgs::BoundingBox box_msg)
    {
      // Check size
      if (box_msg.dimensions.x < 1.0e-9 ||
          box_msg.dimensions.y < 1.0e-9 ||
          box_msg.dimensions.z < 1.0e-9 ||
          std::isnan(box_msg.dimensions.x) ||
          std::isnan(box_msg.dimensions.y) ||
          std::isnan(box_msg.dimensions.z)) {
        return false;
      }
      return true;
    }

    void allocateShapes(int num)
    {
      if (num > shapes_.size()) {
        for (size_t i = shapes_.size(); i < num; i++) {
          ShapePtr shape (new rviz::Shape(
                            rviz::Shape::Cube, this->context_->getSceneManager(),
                            this->scene_node_));
          shapes_.push_back(shape);
        }
      }
      else if (num < shapes_.size())
      {
        shapes_.resize(num);
      }
    }

    void allocateBillboardLines(int num)
    {
      if (num > edges_.size()) {
        for (size_t i = edges_.size(); i < num; i++) {
          BillboardLinePtr line(new rviz::BillboardLine(
                                  this->context_->getSceneManager(), this->scene_node_));
          edges_.push_back(line);
        }
      }
      else if (num < edges_.size())
        {
          edges_.resize(num);       // ok??
        }
    }

    void allocateCoords(int num)
    {
      if (num > coords_objects_.size()) {
        for (size_t i = coords_objects_.size(); i < num; i++) {
          Ogre::SceneNode* scene_node = this->scene_node_->createChildSceneNode();
          std::vector<ArrowPtr> coord;
          for (int i = 0; i < 3; i++) {
            ArrowPtr arrow (new rviz::Arrow(this->scene_manager_, scene_node));
            coord.push_back(arrow);
          }
          coords_nodes_.push_back(scene_node);
          coords_objects_.push_back(coord);
        }
      }
      else if (num < coords_objects_.size()) {
        for (size_t i = num; i < coords_objects_.size(); i++) {
          // coords_nodes_[i];
          // coords_objects_[i][0]->setVisible(false);
          // coords_objects_[i][1]->setVisible(false);
          // coords_objects_[i][2]->setVisible(false);
          coords_nodes_[i]->setVisible(false);
        }
        coords_objects_.resize(num);
        coords_nodes_.resize(num);
      }
    }

    void showBoxes(
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
        if (!isValidBoundingBox(box)) {
          ROS_WARN_THROTTLE(10, "Invalid size of bounding box is included and skipped: [%f, %f, %f]",
            box.dimensions.x, box.dimensions.y, box.dimensions.z);
          continue;
        }

        ShapePtr shape = shapes_[i];
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
        if(!this->context_->getFrameManager()->transform(
            box.header, box.pose, position, orientation)) {
          std::ostringstream oss;
          oss << "Error transforming pose";
          oss << " from frame '" << box.header.frame_id << "'";
          oss << " to frame '" << qPrintable(this->fixed_frame_) << "'";
          ROS_ERROR_STREAM(oss.str());
          this->setStatus(rviz::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
          return;
        }

        // Ogre::Vector3 p(box.pose.position.x,
        //                 box.pose.position.y,
        //                 box.pose.position.z);
        // Ogre::Quaternion q(box.pose.orientation.w,
        //                    box.pose.orientation.x,
        //                    box.pose.orientation.y,
        //                    box.pose.orientation.z);
        shape->setPosition(position);
        shape->setOrientation(orientation);
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

    void showEdges(
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
        if (!isValidBoundingBox(box)) {
          ROS_WARN_THROTTLE(10, "Invalid size of bounding box is included and skipped: [%f, %f, %f]",
            box.dimensions.x, box.dimensions.y, box.dimensions.z);
          continue;
        }

        geometry_msgs::Vector3 dimensions = box.dimensions;

        BillboardLinePtr edge = edges_[i];
        edge->clear();
        Ogre::Vector3 position;
        Ogre::Quaternion quaternion;
        if(!this->context_->getFrameManager()->transform(box.header, box.pose,
                                                  position,
                                                  quaternion)) {
          std::ostringstream oss;
          oss << "Error transforming pose";
          oss << " from frame '" << box.header.frame_id << "'";
          oss << " to frame '" << qPrintable(this->fixed_frame_) << "'";
          ROS_ERROR_STREAM(oss.str());
          this->setStatus(rviz::StatusProperty::Error, "Transform", QString::fromStdString(oss.str()));
          return;
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

    void showCoords(
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
        if(!this->context_->getFrameManager()->getTransform(
            box.header, position, orientation)) {
          ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
                    box.header.frame_id.c_str(), qPrintable(this->fixed_frame_));
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

    void hideCoords()
    {
      for (size_t i = 0; i < coords_nodes_.size(); i++) {
        coords_nodes_[i]->setVisible(false);
      }
    }

  };  // class BoundingBoxDisplayCommon

}  // namespace jsk_rviz_plugins

#endif  // JSK_RVIZ_PLUGINS_BOUDNING_BOX_DISPLAY_COMMON_H_
