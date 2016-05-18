// -*- mode: c++; -*-
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
#define BOOST_PARAMETER_MAX_ARITY 7
#include "simple_occupancy_grid_array_display.h"
#include <Eigen/Geometry>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_topic_tools/color_utils.h>
#include "rviz_util.h"
#include <jsk_recognition_utils/geo/plane.h>

namespace jsk_rviz_plugins
{
  SimpleOccupancyGridArrayDisplay::SimpleOccupancyGridArrayDisplay()
  {
    auto_color_property_ = new rviz::BoolProperty(
      "Auto Color", true,
      "Auto coloring",
      this, SLOT(updateAutoColor()));
    
    alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0,
      "Amount of transparency to apply to the polygon.",
      this, SLOT(updateAlpha()));

    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);

    
  }

  SimpleOccupancyGridArrayDisplay::~SimpleOccupancyGridArrayDisplay()
  {
    delete alpha_property_;
    allocateCloudsAndNodes(0);
  }

  void SimpleOccupancyGridArrayDisplay::onInitialize()
  {
    MFDClass::onInitialize();
    updateAlpha();
    updateAutoColor();
  }
  
  void SimpleOccupancyGridArrayDisplay::allocateCloudsAndNodes(const size_t num)
  {
    if (num > clouds_.size()) { // need to allocate new node and clouds
      for (size_t i = clouds_.size(); i < num; i++) {
        Ogre::SceneNode* node = scene_node_->createChildSceneNode();
        rviz::PointCloud* cloud = new rviz::PointCloud();
        cloud->setRenderMode(rviz::PointCloud::RM_TILES);
        cloud->setCommonDirection( Ogre::Vector3::UNIT_Z );
        cloud->setCommonUpVector( Ogre::Vector3::UNIT_Y );
        node->attachObject(cloud);
        clouds_.push_back(cloud);
        nodes_.push_back(node);
      }
    }
    else if (num < clouds_.size()) // need to destroy
    {
      for (size_t i = num; i < clouds_.size(); i++) {
        nodes_[i]->detachObject(clouds_[i]);
        delete clouds_[i];
        scene_manager_->destroySceneNode(nodes_[i]);
      }
      clouds_.resize(num);
      nodes_.resize(num);
    }
  }

  void SimpleOccupancyGridArrayDisplay::reset()
  {
    MFDClass::reset();
    allocateCloudsAndNodes(0);
  }

  void SimpleOccupancyGridArrayDisplay::processMessage(
    const jsk_recognition_msgs::SimpleOccupancyGridArray::ConstPtr& msg)
  {
    Ogre::ColourValue white(1, 1, 1, 1);
    allocateCloudsAndNodes(msg->grids.size()); // not enough
    for (size_t i = 0; i < msg->grids.size(); i++) {
      Ogre::SceneNode* node = nodes_[i];
      rviz::PointCloud* cloud = clouds_[i];
      const jsk_recognition_msgs::SimpleOccupancyGrid grid = msg->grids[i];
      Ogre::Vector3 position;
      Ogre::Quaternion quaternion;
      
      // coefficients
      geometry_msgs::Pose plane_pose;
      jsk_recognition_utils::Plane::Ptr plane(new jsk_recognition_utils::Plane(grid.coefficients));
      Eigen::Affine3f plane_pose_eigen = plane->coordinates();
      tf::poseEigenToMsg(plane_pose_eigen, plane_pose);
      if(!context_->getFrameManager()->transform(grid.header, plane_pose,
                                                 position,
                                                 quaternion)) {
        ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
                   qPrintable( getName() ), grid.header.frame_id.c_str(),
                   qPrintable( fixed_frame_ ));
        return;                 // return?
      }
      node->setPosition(position);
      node->setOrientation(quaternion);
      cloud->setDimensions(grid.resolution, grid.resolution, 0.0);
      std::vector<rviz::PointCloud::Point> points;
      for (size_t ci = 0; ci < grid.cells.size(); ci++) {
        const geometry_msgs::Point p = grid.cells[ci];
        rviz::PointCloud::Point point;
        if (auto_color_) {
          point.color = rviz::colorMsgToOgre(jsk_topic_tools::colorCategory20(i));
        }
        else {
          point.color = white;
        }
        point.position.x = p.x;
        point.position.y = p.y;
        point.position.z = p.z;
        points.push_back(point);
      }
      cloud->clear();
      cloud->setAlpha(alpha_);
      if (!points.empty()) {
        cloud->addPoints(&points.front(), points.size());
      }
    }
    context_->queueRender();
  }
  
  void SimpleOccupancyGridArrayDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
  }

  void SimpleOccupancyGridArrayDisplay::updateAutoColor()
  {
    auto_color_ = auto_color_property_->getBool();
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::SimpleOccupancyGridArrayDisplay, rviz::Display )

