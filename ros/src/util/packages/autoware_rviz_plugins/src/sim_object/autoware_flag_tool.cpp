/*
 * Copyright (c) 2017, TierIV, Inc.
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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

#include <ros/console.h>

#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include <rviz/frame_manager.h>

#include "autoware_flag_tool.h"
#include <geometry_msgs/PointStamped.h>

namespace autoware_sim_object
{

AutowareSimObjTool::AutowareSimObjTool()
  : moving_flag_node_( NULL )
  , current_flag_property_( NULL )
{
  shortcut_key_ = 'l';
  pub_ = nh_.advertise<geometry_msgs::PointStamped>("/clicked_point", 1);
}

AutowareSimObjTool::~AutowareSimObjTool()
{
  for( unsigned i = 0; i < flag_nodes_.size(); i++ )
  {
    scene_manager_->destroySceneNode( flag_nodes_[ i ]);
  }
}

void AutowareSimObjTool::onInitialize()
{
  flag_resource_ = "package://rviz_plugin_tutorials/media/flag.dae";

  if( rviz::loadMeshFromResource( flag_resource_ ).isNull() )
  {
    ROS_ERROR( "AutowareSimObjTool: failed to load model resource '%s'.", flag_resource_.c_str() );
    return;
  }

  moving_flag_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  moving_flag_node_->attachObject( entity );
  moving_flag_node_->setVisible( false );
}

void AutowareSimObjTool::activate()
{
  if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( true );

    current_flag_property_ = new rviz::VectorProperty( "Flag " + QString::number( flag_nodes_.size() ));
    current_flag_property_->setReadOnly( true );
    getPropertyContainer()->addChild( current_flag_property_ );
  }
}

void AutowareSimObjTool::deactivate()
{
  if( moving_flag_node_ )
  {
    moving_flag_node_->setVisible( false );
    delete current_flag_property_;
    current_flag_property_ = NULL;
  }
}

int AutowareSimObjTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  if( !moving_flag_node_ )
  {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
  
  if( rviz::getPointOnPlaneFromWindowXY( event.viewport,
                                         ground_plane,
                                         event.x, event.y, intersection ))
  {
    moving_flag_node_->setVisible( true );
    moving_flag_node_->setPosition( intersection );
    current_flag_property_->setVector( intersection );

    if( event.leftDown() )
    {
      makeFlag( intersection );
      current_flag_property_ = NULL; // Drop the reference so that deactivate() won't remove it.

	geometry_msgs::PointStamped ps;
	ps.header.stamp = ros::Time::now();
	ps.header.frame_id = context_->getFrameManager()->getFixedFrame();
	ps.point.x = intersection.x;//event.x;
	ps.point.y = intersection.y;//event.y;
	ps.point.z = intersection.z;//0;
	pub_.publish(ps);     

       return Render | Finished;
    }
  }
  else
  {
    moving_flag_node_->setVisible( false ); // If the mouse is not pointing at the ground plane, don't show the flag.
  }
  return Render;
}

void AutowareSimObjTool::makeFlag( const Ogre::Vector3& position )
{
  Ogre::SceneNode* node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity* entity = scene_manager_->createEntity( flag_resource_ );
  node->attachObject( entity );
  node->setVisible( false );
  node->setPosition( position );
  flag_nodes_.push_back( node );
}
void AutowareSimObjTool::save( rviz::Config config ) const
{
  config.mapSetValue( "Class", getClassId() );
  rviz::Config flags_config = config.mapMakeChild( "Flags" );

  rviz::Property* container = getPropertyContainer();
  int num_children = container->numChildren();
  for( int i = 0; i < num_children; i++ )
  {
    rviz::Property* position_prop = container->childAt( i );
    rviz::Config flag_config = flags_config.listAppendNew();
    flag_config.mapSetValue( "Name", position_prop->getName() );
    position_prop->save( flag_config );
  }
}

void AutowareSimObjTool::load( const rviz::Config& config )
{
  rviz::Config flags_config = config.mapGetChild( "Flags" );
  int num_flags = flags_config.listLength();
  for( int i = 0; i < num_flags; i++ )
  {
    rviz::Config flag_config = flags_config.listChildAt( i );
    QString name = "Flag " + QString::number( i + 1 );
    flag_config.mapGetString( "Name", &name );
    rviz::VectorProperty* prop = new rviz::VectorProperty( name );
    prop->load( flag_config );
    prop->setReadOnly( true );
    getPropertyContainer()->addChild( prop );
    makeFlag( prop->getVector() );
  }
}
} 

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(autoware_sim_object::AutowareSimObjTool,rviz::Tool )
// END_TUTORIAL
