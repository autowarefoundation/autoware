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

#include "bounding_box_display_common.h"
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
      "show coords", false,
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

  void BoundingBoxArrayDisplay::processMessage(
    const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg)
  {
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

}  // namespace jsk_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_rviz_plugins::BoundingBoxArrayDisplay, rviz::Display)
