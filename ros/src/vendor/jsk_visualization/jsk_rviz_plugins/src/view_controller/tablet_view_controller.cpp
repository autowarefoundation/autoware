/*
 * Copyright (c) 2012, JSK Lab, Inc.
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
 *     * Neither the name of the JSK Lab, Inc. nor the names of its
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
 *
 * Author: Adam Leeper
 * Author: Ryohei Ueda
 */

#include "tablet_view_controller.h"

#include "rviz/load_resource.h"
#include "rviz/uniform_string_stream.h"
#include "rviz/display_context.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/frame_manager.h"
#include "rviz/geometry.h"
#include "rviz/ogre_helpers/shape.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/editable_enum_property.h"
#include "rviz/properties/ros_topic_property.h"

#include "view_controller_msgs/CameraPlacement.h"
#include "geometry_msgs/PointStamped.h"

#include <OGRE/OgreViewport.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>

#include <rviz/render_panel.h>
#include <rviz/view_manager.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <OGRE/OgreRenderWindow.h>

namespace jsk_rviz_plugins
{
using namespace view_controller_msgs;
using namespace rviz;

// Strings for selecting control mode styles
static const std::string MODE_ORBIT = "Orbit";
static const std::string MODE_FPS = "FPS";

// Limits to prevent orbit controller singularity, but not currently used.
//static const Ogre::Radian PITCH_LIMIT_LOW  = Ogre::Radian(-Ogre::Math::HALF_PI + 0.02);
//static const Ogre::Radian PITCH_LIMIT_HIGH = Ogre::Radian( Ogre::Math::HALF_PI - 0.02);
static const Ogre::Radian PITCH_LIMIT_LOW  = Ogre::Radian( 0.02 );
static const Ogre::Radian PITCH_LIMIT_HIGH = Ogre::Radian( Ogre::Math::PI - 0.02);


// Some convenience functions for Ogre / geometry_msgs conversions
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::Point &m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline Ogre::Vector3 vectorFromMsg(const geometry_msgs::Vector3 &m) { return Ogre::Vector3(m.x, m.y, m.z); }
static inline geometry_msgs::Point pointOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Point m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}
static inline void pointOgreToMsg(const Ogre::Vector3 &o, geometry_msgs::Point &m)  { m.x = o.x; m.y = o.y; m.z = o.z; }

static inline geometry_msgs::Vector3 vectorOgreToMsg(const Ogre::Vector3 &o)
{
  geometry_msgs::Vector3 m;
  m.x = o.x; m.y = o.y; m.z = o.z;
  return m;
}
static inline void vectorOgreToMsg(const Ogre::Vector3 &o, geometry_msgs::Vector3 &m) { m.x = o.x; m.y = o.y; m.z = o.z; }

// -----------------------------------------------------------------------------


TabletViewController::TabletViewController()
  : nh_(""), animate_(false), dragging_( false )
{
  interaction_disabled_cursor_ = makeIconCursor( "package://rviz/icons/forbidden.svg" );

  mouse_enabled_property_ = new BoolProperty("Mouse Enabled", true,
                                   "Enables mouse control of the camera.",
                                   this);
  interaction_mode_property_ = new EditableEnumProperty("Control Mode", QString::fromStdString(MODE_ORBIT),
                                   "Select the style of mouse interaction.",
                                   this);
  interaction_mode_property_->addOptionStd(MODE_ORBIT);
  interaction_mode_property_->addOptionStd(MODE_FPS);
  interaction_mode_property_->setStdString(MODE_ORBIT);

  fixed_up_property_ = new BoolProperty( "Maintain Vertical Axis", true,
                                         "If enabled, the camera is not allowed to roll side-to-side.",
                                          this);
  attached_frame_property_ = new TfFrameProperty("Target Frame",
                                                 TfFrameProperty::FIXED_FRAME_STRING,
                                                 "TF frame the camera is attached to.",
                                                 this, NULL, true );
  eye_point_property_    = new VectorProperty( "Eye", Ogre::Vector3( 5, 5, 10 ),
                                              "Position of the camera.", this );
  focus_point_property_ = new VectorProperty( "Focus", Ogre::Vector3::ZERO,
                                              "Position of the focus/orbit point.", this );
  up_vector_property_ = new VectorProperty( "Up", Ogre::Vector3::UNIT_Z,
                                            "The vector which maps to \"up\" in the camera image plane.", this );
  distance_property_    = new FloatProperty( "Distance", getDistanceFromCameraToFocalPoint(),
                                             "The distance between the camera position and the focus point.",
                                             this );
  distance_property_->setMin( 0.01 );
  default_transition_time_property_ = new FloatProperty( "Transition Time", 0.5,
                                                         "The default time to use for camera transitions.",
                                                         this );
  camera_placement_topic_property_ = new RosTopicProperty("Placement Topic", "/rviz/camera_placement",
                                                          QString::fromStdString(ros::message_traits::datatype<view_controller_msgs::CameraPlacement>() ),
                                                          "Topic for CameraPlacement messages", this, SLOT(updateTopics()));
  
  camera_placement_publish_topic_property_ = new RosTopicProperty("Placement Publish Topic", "/rviz/current_camera_placement",
                                                          QString::fromStdString(ros::message_traits::datatype<view_controller_msgs::CameraPlacement>() ),
                                                          "Publishing Topic for CameraPlacement messages", this, SLOT(updatePublishTopics()));

  mouse_point_publish_topic_property_ = new RosTopicProperty("Placement Mouse Point", "/rviz/current_mouse_point",
                                                             QString::fromStdString(ros::message_traits::datatype<geometry_msgs::PointStamped>() ),
                                                             "Publishing position of mouse", this, SLOT(updateMousePointPublishTopics()));

//  camera_placement_trajectory_topic_property_ = new RosTopicProperty("Trajectory Topic", "/rviz/camera_placement_trajectory",
//                                                          QString::fromStdString(ros::message_traits::datatype<view_controller_msgs::CameraPlacementTrajectory>() ),
//                                                          "Topic for CameraPlacementTrajectory messages", this, SLOT(updateTopics()));
}

TabletViewController::~TabletViewController()
{
    delete focal_shape_;
    context_->getSceneManager()->destroySceneNode( attached_scene_node_ );
}

void TabletViewController::updatePublishTopics()
{
  placement_publisher_ = nh_.advertise<view_controller_msgs::CameraPlacement>
    (camera_placement_publish_topic_property_->getStdString(), 1);
}

void TabletViewController::updateMousePointPublishTopics()
{
  mouse_point_publisher_ = nh_.advertise<geometry_msgs::PointStamped>
    (mouse_point_publish_topic_property_->getStdString(), 1);
}

void TabletViewController::publishMouseEvent(rviz::ViewportMouseEvent& event)
{
  geometry_msgs::PointStamped msg;
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.header.stamp = ros::Time::now();
  rviz::ViewManager* manager = context_->getViewManager();
  rviz::RenderPanel* panel = manager->getRenderPanel();
  Ogre::RenderWindow* window = panel->getRenderWindow();
  msg.point.x = (double)event.x / window->getWidth();
  msg.point.y = (double)event.y / window->getHeight();
  msg.point.z = 0;
  mouse_point_publisher_.publish(msg);
}
  
void TabletViewController::publishCurrentPlacement()
{
  view_controller_msgs::CameraPlacement msg;
  ros::Time now = ros::Time::now();
  msg.target_frame = attached_frame_property_->getFrameStd();
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  // eye
  msg.eye.header.stamp = now;
  msg.eye.header.frame_id = fixed_frame;
  Ogre::Vector3 eye = eye_point_property_->getVector();
  msg.eye.point.x = eye[0];
  msg.eye.point.y = eye[1];
  msg.eye.point.z = eye[2];
  // focus
  msg.focus.header.stamp = now;
  msg.focus.header.frame_id = fixed_frame;
  Ogre::Vector3 focus = focus_point_property_->getVector();
  msg.focus.point.x = focus[0];
  msg.focus.point.y = focus[1];
  msg.focus.point.z = focus[2];
  // up
  msg.up.header.stamp = now;
  msg.up.header.frame_id = fixed_frame;
  Ogre::Vector3 up = up_vector_property_->getVector();
  msg.up.vector.x = up[0];
  msg.up.vector.y = up[1];
  msg.up.vector.z = up[2];

  placement_publisher_.publish(msg);
}
  
void TabletViewController::updateTopics()
{
//  trajectory_subscriber_ = nh_.subscribe<view_controller_msgs::CameraPlacementTrajectory>
//                              (camera_placement_trajectory_topic_property_->getStdString(), 1,
//                              boost::bind(&TabletViewController::cameraPlacementTrajectoryCallback, this, _1));
  placement_subscriber_  = nh_.subscribe<view_controller_msgs::CameraPlacement>
                              (camera_placement_topic_property_->getStdString(), 1,
                              boost::bind(&TabletViewController::cameraPlacementCallback, this, _1));
}

void TabletViewController::onInitialize()
{
    attached_frame_property_->setFrameManager( context_->getFrameManager() );
    attached_scene_node_ = context_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
    camera_->detachFromParent();
    attached_scene_node_->attachObject( camera_ );

    camera_->setProjectionType( Ogre::PT_PERSPECTIVE );

    focal_shape_ = new Shape(Shape::Sphere, context_->getSceneManager(), attached_scene_node_);
    focal_shape_->setScale(Ogre::Vector3(0.05f, 0.05f, 0.01f));
    focal_shape_->setColor(1.0f, 1.0f, 0.0f, 0.5f);
    focal_shape_->getRootNode()->setVisible(false);

}

void TabletViewController::onActivate()
{
  updateAttachedSceneNode();

  // Before activation, changes to target frame property should have
  // no side-effects.  After activation, changing target frame
  // property has the side effect (typically) of changing an offset
  // property so that the view does not jump.  Therefore we make the
  // signal/slot connection from the property here in onActivate()
  // instead of in the constructor.
  connect( attached_frame_property_, SIGNAL( changed() ), this, SLOT( updateAttachedFrame() ));
  connect( fixed_up_property_,       SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ));
  connectPositionProperties();

  // Only do this once activated!
  updateTopics();
  updatePublishTopics();
  updateMousePointPublishTopics();
}

void TabletViewController::connectPositionProperties()
{
  connect( distance_property_,    SIGNAL( changed() ), this, SLOT( onDistancePropertyChanged() ), Qt::UniqueConnection);
  connect( eye_point_property_,   SIGNAL( changed() ), this, SLOT( onEyePropertyChanged() ),      Qt::UniqueConnection);
  connect( focus_point_property_, SIGNAL( changed() ), this, SLOT( onFocusPropertyChanged() ),    Qt::UniqueConnection);
  connect( up_vector_property_,   SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ),       Qt::UniqueConnection);
}

void TabletViewController::disconnectPositionProperties()
{
  disconnect( distance_property_,    SIGNAL( changed() ), this, SLOT( onDistancePropertyChanged() ));
  disconnect( eye_point_property_,   SIGNAL( changed() ), this, SLOT( onEyePropertyChanged() ));
  disconnect( focus_point_property_, SIGNAL( changed() ), this, SLOT( onFocusPropertyChanged() ));
  disconnect( up_vector_property_,   SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ));
}

void TabletViewController::onEyePropertyChanged()
{
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

void TabletViewController::onFocusPropertyChanged()
{
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

void TabletViewController::onDistancePropertyChanged()
{
  disconnectPositionProperties();
  Ogre::Vector3 new_eye_position = focus_point_property_->getVector() + distance_property_->getFloat()* camera_->getOrientation().zAxis();
  eye_point_property_->setVector(new_eye_position);
  connectPositionProperties();
}

void TabletViewController::onUpPropertyChanged()
{
  disconnect( up_vector_property_,   SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ));
  if(fixed_up_property_->getBool()){
    up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
    camera_->setFixedYawAxis(true, reference_orientation_ * Ogre::Vector3::UNIT_Z);
  }
  else {
    // force orientation to match up vector; first call doesn't actually change the quaternion
    camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
    camera_->setDirection( reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
    // restore normal behavior
    camera_->setFixedYawAxis(false);
  }
  connect( up_vector_property_,   SIGNAL( changed() ), this, SLOT( onUpPropertyChanged() ),       Qt::UniqueConnection);
}

void TabletViewController::updateAttachedFrame()
{
  Ogre::Vector3 old_position = attached_scene_node_->getPosition();
  Ogre::Quaternion old_orientation = attached_scene_node_->getOrientation();

  updateAttachedSceneNode();

  onAttachedFrameChanged( old_position, old_orientation );
}

void TabletViewController::updateAttachedSceneNode()
{
  Ogre::Vector3 new_reference_position;
  Ogre::Quaternion new_reference_orientation;

  bool queue = false;
  if( context_->getFrameManager()->getTransform( attached_frame_property_->getFrameStd(), ros::Time(),
                                                 new_reference_position, new_reference_orientation ))
  {
    attached_scene_node_->setPosition( new_reference_position );
    attached_scene_node_->setOrientation( new_reference_orientation );
    reference_position_ = new_reference_position;
    reference_orientation_ = new_reference_orientation;
    queue = true;
  }
  if(queue) context_->queueRender();
}

void TabletViewController::onAttachedFrameChanged(const Ogre::Vector3& old_reference_position, const Ogre::Quaternion& old_reference_orientation)
{
  Ogre::Vector3 fixed_frame_focus_position = old_reference_orientation*focus_point_property_->getVector() + old_reference_position;
  Ogre::Vector3 fixed_frame_eye_position = old_reference_orientation*eye_point_property_->getVector() + old_reference_position;
  Ogre::Vector3 new_focus_position = fixedFrameToAttachedLocal(fixed_frame_focus_position);
  Ogre::Vector3 new_eye_position = fixedFrameToAttachedLocal(fixed_frame_eye_position);
  Ogre::Vector3 new_up_vector = reference_orientation_.Inverse()*old_reference_orientation*up_vector_property_->getVector();

  //Ogre::Quaternion new_camera_orientation = reference_orientation_.Inverse()*old_reference_orientation*getOrientation();

  focus_point_property_->setVector(new_focus_position);
  eye_point_property_->setVector(new_eye_position);
  up_vector_property_->setVector(fixed_up_property_->getBool() ? Ogre::Vector3::UNIT_Z : new_up_vector);
  distance_property_->setFloat( getDistanceFromCameraToFocalPoint());

  // force orientation to match up vector; first call doesn't actually change the quaternion
  camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
  camera_->setDirection( reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
}

float TabletViewController::getDistanceFromCameraToFocalPoint()
{
    return (eye_point_property_->getVector() - focus_point_property_->getVector()).length();
}

void TabletViewController::reset()
{
    eye_point_property_->setVector(Ogre::Vector3(5, 5, 10));
    focus_point_property_->setVector(Ogre::Vector3::ZERO);
    up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
    distance_property_->setFloat( getDistanceFromCameraToFocalPoint());
    mouse_enabled_property_->setBool(true);
    interaction_mode_property_->setStdString(MODE_ORBIT);


  // Hersh says: why is the following junk necessary?  I don't know.
  // However, without this you need to call reset() twice after
  // switching from TopDownOrtho to FPS.  After the first call the
  // camera is in the right position but pointing the wrong way.
  updateCamera();
  camera_->lookAt( 0, 0, 0 );
  setPropertiesFromCamera( camera_ );
}

void TabletViewController::handleMouseEvent(ViewportMouseEvent& event)
{
  if( !mouse_enabled_property_->getBool() )
  {
    setCursor( interaction_disabled_cursor_ );
    setStatus( "<b>Mouse interaction is disabled. You can enable it by checking the \"Mouse Enabled\" check-box in the Views panel." );
    return;
  }
  else if ( event.shift() )
  {
    setStatus( "TODO: Fix me! <b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z." );
  }
  else if ( event.control() )
  {
    setStatus( "TODO: Fix me! <b>Left-Click:</b> Move X/Y.  <b>Right-Click:</b>: Move Z." );
  }
  else
  {
    setStatus( "TODO: Fix me! <b>Left-Click:</b> Rotate.  <b>Middle-Click:</b> Move X/Y.  <b>Right-Click:</b>: Zoom.  <b>Shift</b>: More options." );
  }
  if (event.type == QEvent::MouseButtonPress
      || event.type == QEvent::MouseButtonRelease
      || dragging_ && event.type == QEvent::MouseMove) {
    publishMouseEvent(event);
  }
  float distance = distance_property_->getFloat();
  int32_t diff_x = 0;
  int32_t diff_y = 0;
  bool moved = false;

  if( event.type == QEvent::MouseButtonPress )
  {
    focal_shape_->getRootNode()->setVisible(true);
    moved = true;
    dragging_ = true;
    cancelTransition();  // Stop any automated movement
  }
  else if( event.type == QEvent::MouseButtonRelease )
  {
    focal_shape_->getRootNode()->setVisible(false);
    moved = true;
    dragging_ = false;
  }
  else if( dragging_ && event.type == QEvent::MouseMove )
  {
    diff_x = event.x - event.last_x;
    diff_y = event.y - event.last_y;
    moved = true;
  }

  // regular left-button drag
  if( event.left() && !event.shift() )
  {
    setCursor( Rotate3D );
    yaw_pitch_roll( -diff_x*0.005, -diff_y*0.005, 0 );
  }
  // middle or shift-left drag
  else if( event.middle() || ( event.shift() && event.left() ))
  {
    setCursor( MoveXY );
    if(interaction_mode_property_->getStdString() == MODE_ORBIT)  // Orbit style
    {
        float fovY = camera_->getFOVy().valueRadians();
        float fovX = 2.0f * atan( tan( fovY / 2.0f ) * camera_->getAspectRatio() );

        int width = camera_->getViewport()->getActualWidth();
        int height = camera_->getViewport()->getActualHeight();

        move_focus_and_eye( -((float)diff_x / (float)width) * distance * tan( fovX / 2.0f ) * 2.0f,
              ((float)diff_y / (float)height) * distance * tan( fovY / 2.0f ) * 2.0f,
              0.0f );
    }
    else if(interaction_mode_property_->getStdString() == MODE_FPS)  // Orbit style
    {
      move_focus_and_eye( diff_x*0.01, -diff_y*0.01, 0.0f );
    }
  }
  else if( event.right() )
  {
    if( event.shift() ||  (interaction_mode_property_->getStdString() == MODE_FPS) )
    {
      setCursor( MoveZ );
      move_focus_and_eye(0.0f, 0.0f, diff_y * 0.01 * distance);
    }
    else
    {
      setCursor( Zoom );
      move_eye( 0, 0, diff_y * 0.01 * distance );
    }
  }
  else
  {
    setCursor( event.shift() ? MoveXY : Rotate3D );
  }

  if ( event.wheel_delta != 0 )
  {
    int diff = event.wheel_delta;

    if( event.shift() )
    {
      move_focus_and_eye(0, 0, -diff * 0.001 * distance );
    }
    else if(event.control())
    {
      yaw_pitch_roll(0, 0, diff*0.001 );
    }
    else
    {
      move_eye( 0, 0, -diff * 0.001 * distance );
    }
    moved = true;
  }

  if(event.type == QEvent::MouseButtonPress && event.left() && event.control() && event.shift())
  {
    bool was_orbit = (interaction_mode_property_->getStdString() == MODE_ORBIT);
    interaction_mode_property_->setStdString(was_orbit ? MODE_FPS : MODE_ORBIT );
  }

  if (moved)
  {
    publishCurrentPlacement();
    context_->queueRender();
  }
}

//void TabletViewController::setUpVectorPropertyModeDependent( const Ogre::Vector3 &vector )
//{
//  if(fixed_up_property_->getBool())
//  {
//    //up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
//  }
//  else {
//    up_vector_property_->setVector(vector);
//  }
//}


void TabletViewController::setPropertiesFromCamera( Ogre::Camera* source_camera )
{
  disconnectPositionProperties();
  Ogre::Vector3 direction = source_camera->getOrientation() * Ogre::Vector3::NEGATIVE_UNIT_Z;
  eye_point_property_->setVector( source_camera->getPosition() );
  focus_point_property_->setVector( source_camera->getPosition() + direction*distance_property_->getFloat());
  if(fixed_up_property_->getBool())
    up_vector_property_->setVector(Ogre::Vector3::UNIT_Z);
  else
    up_vector_property_->setVector(source_camera->getOrientation().yAxis());

  //setUpVectorPropertyModeDependent(source_camera->getOrientation().yAxis());
  connectPositionProperties();
}

void TabletViewController::mimic( ViewController* source_view )
{
    QVariant target_frame = source_view->subProp( "Target Frame" )->getValue();
    if( target_frame.isValid() )
    {
      attached_frame_property_->setValue( target_frame );
    }

    Ogre::Camera* source_camera = source_view->getCamera();
    Ogre::Vector3 position = source_camera->getPosition();
    Ogre::Quaternion orientation = source_camera->getOrientation();

    if( source_view->getClassId() == "rviz/Orbit" )
    {
        distance_property_->setFloat( source_view->subProp( "Distance" )->getValue().toFloat() );
    }
    else
    {
        distance_property_->setFloat( position.length() );
    }
    interaction_mode_property_->setStdString( MODE_ORBIT );

    Ogre::Vector3 direction = orientation * (Ogre::Vector3::NEGATIVE_UNIT_Z * distance_property_->getFloat() );
    focus_point_property_->setVector( position + direction );
    eye_point_property_->setVector(position);
    updateCamera();
}

void TabletViewController::transitionFrom( ViewController* previous_view )
{
  TabletViewController* fvc = dynamic_cast<TabletViewController*>(previous_view);
  if(fvc)
  {
    Ogre::Vector3 new_eye =   eye_point_property_->getVector();
    Ogre::Vector3 new_focus = focus_point_property_->getVector();
    Ogre::Vector3 new_up =    up_vector_property_->getVector();

    eye_point_property_->setVector(fvc->eye_point_property_->getVector());
    focus_point_property_->setVector(fvc->focus_point_property_->getVector());
    up_vector_property_->setVector(fvc->up_vector_property_->getVector());

    beginNewTransition(new_eye, new_focus, new_up, ros::Duration(default_transition_time_property_->getFloat()));
  }
}

void TabletViewController::beginNewTransition(const Ogre::Vector3 &eye, const Ogre::Vector3 &focus, const Ogre::Vector3 &up,
                                            const ros::Duration &transition_time)
{
  if(ros::Duration(transition_time).isZero())
  {
    eye_point_property_->setVector(eye);
    focus_point_property_->setVector(focus);
    up_vector_property_->setVector(up);
    distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
    return;
  }

  start_position_ = eye_point_property_->getVector();
  goal_position_ = eye;

  start_focus_ = focus_point_property_->getVector();
  goal_focus_ = focus;

//  start_up_ = fixed_up_property_->getBool() ? (Ogre::Vector3::UNIT_Z) : getOrientation().yAxis();
//  goal_up_ =  fixed_up_property_->getBool() ? (Ogre::Vector3::UNIT_Z) : up;
  start_up_ = up_vector_property_->getVector();
  goal_up_ =  up;

  current_transition_duration_ = ros::Duration(transition_time);
  transition_start_time_ = ros::Time::now();

  animate_ = true;
}

void TabletViewController::cancelTransition()
{
  animate_ = false;
}

void TabletViewController::cameraPlacementCallback(const CameraPlacementConstPtr &cp_ptr)
{
  CameraPlacement cp = *cp_ptr;

  // Handle control parameters
  mouse_enabled_property_->setBool( !cp.interaction_disabled );
  fixed_up_property_->setBool( !cp.allow_free_yaw_axis );
  if(cp.mouse_interaction_mode != cp.NO_CHANGE)
  {
    std::string name = "";
    if(cp.mouse_interaction_mode == cp.ORBIT) name = MODE_ORBIT;
    else if(cp.mouse_interaction_mode == cp.FPS) name = MODE_FPS;
    interaction_mode_property_->setStdString(name);
  }

  if(cp.target_frame != "")
  {
    attached_frame_property_->setStdString(cp.target_frame);
    updateAttachedFrame();
  }

  if(cp.time_from_start.toSec() >= 0)
  {
    ROS_DEBUG_STREAM("Received a camera placement request! \n" << cp);
    transformCameraPlacementToAttachedFrame(cp);
    ROS_DEBUG_STREAM("After transform, we have \n" << cp);

    Ogre::Vector3 eye = vectorFromMsg(cp.eye.point);
    Ogre::Vector3 focus = vectorFromMsg(cp.focus.point);
    Ogre::Vector3 up = vectorFromMsg(cp.up.vector);

    beginNewTransition(eye, focus, up, cp.time_from_start);
  }
}

//void TabletViewController::cameraPlacementTrajectoryCallback(const CameraPlacementTrajectoryConstPtr &cptptr)
//{
//  CameraPlacementTrajectory cpt = *cptptr;
//  ROS_DEBUG_STREAM("Received a camera placement trajectory request! \n" << cpt);
  
//  // Handle control parameters
//  mouse_enabled_property_->setBool( cpt.interaction_enabled );
//  fixed_up_property_->setBool( !cpt.allow_free_yaw_axis );
//  if(cpt.mouse_interaction_mode != cpt.NO_CHANGE)
//  {
//    std::string name = "";
//    if(cpt.mouse_interaction_mode == cpt.ORBIT) name = MODE_ORBIT;
//    else if(cpt.mouse_interaction_mode == cpt.FPS) name = MODE_FPS;
//    interaction_mode_property_->setStdString(name);
//  }

//  // TODO should transform the interpolated positions (later), or transform info will only reflect the TF tree state at the beginning...
//  for(size_t i = 0; i<cpt.placements.size(); i++)
//  {
//    transformCameraPlacementToAttachedFrame(cpt.placements[i]);
//  }

//  // For now, just transition to the first placement until we put in the capacity for a trajectory
//  CameraPlacement cp = cpt.placements[0];
//  if(cp.target_frame != "")
//  {
//    attached_frame_property_->setStdString(cp.target_frame);
//    updateAttachedFrame();
//  }
//  Ogre::Vector3 eye = vectorFromMsg(cp.eye.point);
//  Ogre::Vector3 focus = vectorFromMsg(cp.focus.point);
//  Ogre::Vector3 up = vectorFromMsg(cp.up.vector);

//  beginNewTransition(eye, focus, up, cp.time_from_start);
//}

void TabletViewController::transformCameraPlacementToAttachedFrame(CameraPlacement &cp)
{
  Ogre::Vector3 position_fixed_eye, position_fixed_focus, position_fixed_up; // position_fixed_attached;
  Ogre::Quaternion rotation_fixed_eye, rotation_fixed_focus, rotation_fixed_up; // rotation_fixed_attached;

  context_->getFrameManager()->getTransform(cp.eye.header.frame_id, ros::Time(0), position_fixed_eye, rotation_fixed_eye);
  context_->getFrameManager()->getTransform(cp.focus.header.frame_id,  ros::Time(0), position_fixed_focus, rotation_fixed_focus);
  context_->getFrameManager()->getTransform(cp.up.header.frame_id,  ros::Time(0), position_fixed_up, rotation_fixed_up);
  //context_->getFrameManager()->getTransform(attached_frame_property_->getStdString(),  ros::Time(0), position_fixed_attached, rotation_fixed_attached);

  Ogre::Vector3 eye = vectorFromMsg(cp.eye.point); 
  Ogre::Vector3 focus = vectorFromMsg(cp.focus.point); 
  Ogre::Vector3 up = vectorFromMsg(cp.up.vector); 

  eye = fixedFrameToAttachedLocal(position_fixed_eye + rotation_fixed_eye*eye);
  focus = fixedFrameToAttachedLocal(position_fixed_focus + rotation_fixed_focus*focus);
  up = reference_orientation_.Inverse()*rotation_fixed_up*up;
  //up = rotation_fixed_up*up;

  cp.eye.point = pointOgreToMsg(eye);
  cp.focus.point = pointOgreToMsg(focus);
  cp.up.vector = vectorOgreToMsg(up);
  cp.eye.header.frame_id = attached_frame_property_->getStdString();
  cp.focus.header.frame_id = attached_frame_property_->getStdString();
  cp.up.header.frame_id = attached_frame_property_->getStdString();
}


// We must assume that this point is in the Rviz Fixed frame since it came from Rviz...
void TabletViewController::lookAt( const Ogre::Vector3& point )
{
  if( !mouse_enabled_property_->getBool() ) return;

  Ogre::Vector3 new_point = fixedFrameToAttachedLocal(point);

  beginNewTransition(eye_point_property_->getVector(), new_point,
                     up_vector_property_->getVector(),
                     ros::Duration(default_transition_time_property_->getFloat()));

  //  // Just for easily testing the other movement styles:
  //  orbitCameraTo(point);
  //  moveCameraWithFocusTo(point);
}

void TabletViewController::orbitCameraTo( const Ogre::Vector3& point)
{
  beginNewTransition(point, focus_point_property_->getVector(),
                     up_vector_property_->getVector(),
                     ros::Duration(default_transition_time_property_->getFloat()));
}

void TabletViewController::moveEyeWithFocusTo( const Ogre::Vector3& point)
{
  beginNewTransition(point, focus_point_property_->getVector() + (point - eye_point_property_->getVector()),
                     up_vector_property_->getVector(),
                     ros::Duration(default_transition_time_property_->getFloat()));
}


void TabletViewController::update(float dt, float ros_dt)
{
  updateAttachedSceneNode();

  if(animate_)
  {
    ros::Duration time_from_start = ros::Time::now() - transition_start_time_;
    float fraction = time_from_start.toSec()/current_transition_duration_.toSec();
    // make sure we get all the way there before turning off
    if(fraction > 1.0f)
    {
      fraction = 1.0f;
      animate_ = false;
    }

    // TODO remap progress to progress_out, which can give us a new interpolation profile.
    float progress = 0.5*(1-cos(fraction*M_PI));

    Ogre::Vector3 new_position = start_position_ + progress*(goal_position_ - start_position_);
    Ogre::Vector3 new_focus  = start_focus_ + progress*(goal_focus_ - start_focus_);
    Ogre::Vector3 new_up  = start_up_ + progress*(goal_up_ - start_up_);

    disconnectPositionProperties();
    eye_point_property_->setVector( new_position );
    focus_point_property_->setVector( new_focus );
    up_vector_property_->setVector(new_up);
    distance_property_->setFloat( getDistanceFromCameraToFocalPoint());
    connectPositionProperties();

    // This needs to happen so that the camera orientation will update properly when fixed_up_property == false
    camera_->setFixedYawAxis(true, reference_orientation_ * up_vector_property_->getVector());
    camera_->setDirection(reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
  }
  updateCamera();
}

void TabletViewController::updateCamera()
{
  camera_->setPosition( eye_point_property_->getVector() );
  camera_->setFixedYawAxis(fixed_up_property_->getBool(), reference_orientation_ * up_vector_property_->getVector());
  camera_->setDirection( reference_orientation_ * (focus_point_property_->getVector() - eye_point_property_->getVector()));
  //camera_->setDirection( (focus_point_property_->getVector() - eye_point_property_->getVector()));
  focal_shape_->setPosition( focus_point_property_->getVector() );
}

void TabletViewController::yaw_pitch_roll( float yaw, float pitch, float roll )
{
  Ogre::Quaternion old_camera_orientation = camera_->getOrientation();
  Ogre::Radian old_pitch = old_camera_orientation.getPitch(false);// - Ogre::Radian(Ogre::Math::HALF_PI);

  if(fixed_up_property_->getBool()) yaw = cos(old_pitch.valueRadians() - Ogre::Math::HALF_PI)*yaw; // helps to reduce crazy spinning!

  Ogre::Quaternion yaw_quat, pitch_quat, roll_quat;
  yaw_quat.FromAngleAxis( Ogre::Radian( yaw ), Ogre::Vector3::UNIT_Y );
  pitch_quat.FromAngleAxis( Ogre::Radian( pitch ), Ogre::Vector3::UNIT_X );
  roll_quat.FromAngleAxis( Ogre::Radian( roll ), Ogre::Vector3::UNIT_Z );
  Ogre::Quaternion orientation_change = yaw_quat * pitch_quat * roll_quat;
  Ogre::Quaternion new_camera_orientation = old_camera_orientation * orientation_change;
  Ogre::Radian new_pitch = new_camera_orientation.getPitch(false);// - Ogre::Radian(Ogre::Math::HALF_PI);

  if( fixed_up_property_->getBool() &&
      ((new_pitch > PITCH_LIMIT_HIGH && new_pitch > old_pitch) || (new_pitch < PITCH_LIMIT_LOW && new_pitch < old_pitch)) )
  {
    orientation_change = yaw_quat * roll_quat;
    new_camera_orientation = old_camera_orientation * orientation_change;
  }

//  Ogre::Radian new_roll = new_camera_orientation.getRoll(false);
//  Ogre::Radian new_yaw = new_camera_orientation.getYaw(false);
  //ROS_INFO("old_pitch: %.3f, new_pitch: %.3f", old_pitch.valueRadians(), new_pitch.valueRadians());

  camera_->setOrientation( new_camera_orientation );
  if( interaction_mode_property_->getStdString() == MODE_ORBIT )
  {
    // In orbit mode the focal point stays fixed, so we need to compute the new camera position.
    Ogre::Vector3 new_eye_position = focus_point_property_->getVector() + distance_property_->getFloat()* new_camera_orientation.zAxis();
    eye_point_property_->setVector(new_eye_position);
    camera_->setPosition(new_eye_position);
    setPropertiesFromCamera(camera_);
  }
  else
  {
    // In FPS mode the camera stays fixed, so we can just apply the rotations and then rely on the property update to set the new focal point.
    setPropertiesFromCamera(camera_);
  }
}

Ogre::Quaternion TabletViewController::getOrientation()  // Do we need this?
{
  return camera_->getOrientation();
}

void TabletViewController::move_focus_and_eye( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  eye_point_property_->add( getOrientation() * translate );
  focus_point_property_->add( getOrientation() * translate );
}

void TabletViewController::move_eye( float x, float y, float z )
{
  Ogre::Vector3 translate( x, y, z );
  // Only update the camera position if it won't "pass through" the origin
  Ogre::Vector3 new_position = eye_point_property_->getVector() + getOrientation() * translate;
  if( (new_position - focus_point_property_->getVector()).length() > distance_property_->getMin() )
    eye_point_property_->setVector(new_position);
  distance_property_->setFloat(getDistanceFromCameraToFocalPoint());
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::TabletViewController, rviz::ViewController )
