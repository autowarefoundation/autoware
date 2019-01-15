/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/validate_floats.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/axes.h"

#include "pose_array_display.h"

namespace jsk_rviz_plugins
{

PoseArrayDisplay::PoseArrayDisplay()
  : manual_object_( NULL )
{
  color_property_ = new rviz::ColorProperty( "Color", QColor( 255, 25, 0 ), "Color to draw the arrows.", this );
  length_property_ = new rviz::FloatProperty( "Arrow Length", 0.3, "Length of the arrows.", this );
  axes_length_property_ = new rviz::FloatProperty( "Axes Length", 1, "Length of each axis, in meters.",
                                             this, SLOT( updateAxisGeometry() ));
  axes_radius_property_ = new rviz::FloatProperty( "Axes Radius", 0.1, "Radius of each axis, in meters.",
                                             this, SLOT( updateAxisGeometry() ));
  shape_property_ = new rviz::EnumProperty( "Shape", "Arrow", "Shape to display the pose as.",
                                      this, SLOT( updateShapeChoice() ));
  shape_property_->addOption( "Arrow", Arrow );
  shape_property_->addOption( "Axes", Axes );
}

PoseArrayDisplay::~PoseArrayDisplay()
{
  if ( initialized() )
  {
    scene_manager_->destroyManualObject( manual_object_ );
  }
}

void PoseArrayDisplay::onInitialize()
{
  MFDClass::onInitialize();
  manual_object_ = scene_manager_->createManualObject();
  manual_object_->setDynamic( true );
  scene_node_->attachObject( manual_object_ );

  updateShapeChoice();
  updateShapeVisibility();
  updateAxisGeometry();
}

void PoseArrayDisplay::updateShapeChoice()
{
    bool use_arrow = ( shape_property_->getOptionInt() == Arrow );

  color_property_->setHidden( !use_arrow );
  length_property_->setHidden( !use_arrow );

  axes_length_property_->setHidden( use_arrow );
  axes_radius_property_->setHidden( use_arrow );

  updateShapeVisibility();

  context_->queueRender();
}

void PoseArrayDisplay::updateShapeVisibility()
{

  if( !pose_valid_ )
  {
    manual_object_->setVisible(false);
    for (int i = 0; i < coords_nodes_.size() ; i++)
      coords_nodes_[i]->setVisible(false);
  }
  else
  {
    bool use_arrow = (shape_property_->getOptionInt() == Arrow);
    for (int i = 0; i < coords_nodes_.size() ; i++)
      coords_nodes_[i]->setVisible(!use_arrow);

    manual_object_->setVisible(use_arrow);
  }
}

void PoseArrayDisplay::updateAxisGeometry()
{
  for(size_t i = 0; i < coords_objects_.size() ; i++)
    coords_objects_[i]->set( axes_length_property_->getFloat(),
                             axes_radius_property_->getFloat() );
  context_->queueRender();
}

void PoseArrayDisplay::allocateCoords(int num)
{
  if (num > coords_objects_.size()) {
    for (size_t i = coords_objects_.size(); i < num; i++) {
      Ogre::SceneNode* scene_node = scene_node_->createChildSceneNode();
      rviz::Axes* axes = new rviz::Axes( scene_manager_, scene_node,
                                        axes_length_property_->getFloat(),
                                        axes_radius_property_->getFloat());
      coords_nodes_.push_back(scene_node);
      coords_objects_.push_back(axes);
    }
  }
  else if (num < coords_objects_.size()) {
    for (int i = coords_objects_.size() - 1; num <= i; i--) {
      delete coords_objects_[i];
      scene_manager_->destroySceneNode(coords_nodes_[i]);
    }
    coords_objects_.resize(num);
    coords_nodes_.resize(num);
  }
}


bool validateFloats( const geometry_msgs::PoseArray& msg )
{
  return rviz::validateFloats( msg.poses );
}

void PoseArrayDisplay::processMessage( const geometry_msgs::PoseArray::ConstPtr& msg )
{
  if( !validateFloats( *msg ))
  {
    setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  manual_object_->clear();

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->getTransform( msg->header, position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  pose_valid_ = true;
  updateShapeVisibility();

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  manual_object_->clear();

  if(shape_property_->getOptionInt() == Arrow ) {
    for (int i = 0; i < coords_nodes_.size() ; i++)
      coords_nodes_[i]->setVisible(false);
    Ogre::ColourValue color = color_property_->getOgreColor();
    float length = length_property_->getFloat();
    size_t num_poses = msg->poses.size();
    manual_object_->estimateVertexCount( num_poses * 6 );
    manual_object_->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
    for( size_t i=0; i < num_poses; ++i )
      {
        Ogre::Vector3 pos( msg->poses[i].position.x,
                           msg->poses[i].position.y,
                           msg->poses[i].position.z );
        Ogre::Quaternion orient( msg->poses[i].orientation.w,
                                 msg->poses[i].orientation.x,
                                 msg->poses[i].orientation.y,
                                 msg->poses[i].orientation.z );
        // orient here is not normalized, so the scale of the quaternion
        // will affect the scale of the arrow.

        Ogre::Vector3 vertices[6];
        vertices[0] = pos; // back of arrow
        vertices[1] = pos + orient * Ogre::Vector3( length, 0, 0 ); // tip of arrow
        vertices[2] = vertices[ 1 ];
        vertices[3] = pos + orient * Ogre::Vector3( 0.75*length, 0.2*length, 0 );
        vertices[4] = vertices[ 1 ];
        vertices[5] = pos + orient * Ogre::Vector3( 0.75*length, -0.2*length, 0 );

        for( int i = 0; i < 6; ++i )
          {
            manual_object_->position( vertices[i] );
            manual_object_->colour( color );
          }
      }
    manual_object_->end();
  }
  else{
    allocateCoords(msg->poses.size());
    for (int i = 0; i < msg->poses.size() ; i++){
      geometry_msgs::Pose pose = msg->poses[i];
      Ogre::SceneNode* scene_node = coords_nodes_[i];
      scene_node->setVisible(true);

      Ogre::Vector3 position( msg->poses[i].position.x,
                              msg->poses[i].position.y,
                              msg->poses[i].position.z );
      Ogre::Quaternion orientation( msg->poses[i].orientation.w,
                                    msg->poses[i].orientation.x,
                                    msg->poses[i].orientation.y,
                                    msg->poses[i].orientation.z );
      scene_node->setPosition(position);
      scene_node->setOrientation(orientation); // scene node is at frame pose
    }
  }

  context_->queueRender();
}

void PoseArrayDisplay::reset()
{
  MFDClass::reset();
  if( manual_object_ )
  {
    manual_object_->clear();
  }
  if ( coords_objects_.size() > 0 ) {
    allocateCoords(0);
  }
  
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::PoseArrayDisplay, rviz::Display )
