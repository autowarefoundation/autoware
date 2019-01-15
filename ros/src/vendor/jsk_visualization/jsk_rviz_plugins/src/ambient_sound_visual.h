#ifndef __AMBIENT_SOUND_VISUAL__
#define __AMBIENT_SOUND_VISUAL__

#include <jsk_hark_msgs/HarkPower.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

namespace Ogre
{
class SceneManager;
class SceneNode;
//class Vector3;
//class Quaternion;
}

namespace rviz
{
class BillboardLine;
//class Axes;
}

namespace jsk_rviz_plugins
{

// BEGIN_TUTORIAL
// Declare the visual class for this display.
//
// Each instance of AmbientSoundVisual represents the visualization of a single
// sensor_msgs::Imu message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class AmbientSoundVisual
{
public:
  // Constructor.  Creates the visual stuff and puts it into the
  // scene, but in an unconfigured state.
  AmbientSoundVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~AmbientSoundVisual();

  // Configure the visual to show the data in the message.
  void setMessage( const jsk_hark_msgs::HarkPower::ConstPtr& msg );

  // Set the pose of the coordinate frame the message refers to.
  // These could be done inside setMessage(), but that would require
  // calls to FrameManager and error handling inside setMessage(),
  // which doesn't seem as clean.  This way AmbientSoundVisual is only
  // responsible for visualization.
  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Imu message.
  void setColor( float r, float g, float b, float a );

  void setWidth( float w );
  void setScale( float s );
  void setBias( float b );
  void setGrad( float g );

private:
  // The object implementing the actual arrow shape
  rviz::BillboardLine* ambient_sound_power_line_;
  //rviz::Axes* axes_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the Imu message header.
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;
  
  Ogre::Vector3 position_;
  Ogre::Quaternion orientation_;

  //std::map<std::string, Ogre::Vector3> position_;
  //std::map<std::string, Ogre::Quaternion> orientation_;
  float width_,scale_,bias_,grad_;
};
// END_TUTORIAL

} // end namespace jsk_rviz_plugins

#endif // __AMBIENT_SOUND_VISUAL__
