#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/billboard_line.h>
#include <rviz/ogre_helpers/axes.h>

#include "ambient_sound_visual.h"

namespace jsk_rviz_plugins
{

    // BEGIN_TUTORIAL
    AmbientSoundVisual::AmbientSoundVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ): /*{{{*/
        width_ (0.1),
        scale_ (1.0),
        bias_ (28),
        grad_ (0.4)
    {
        scene_manager_ = scene_manager;

        // Ogre::SceneNode s form a tree, with each node storing the
        // transform (position and orientation) of itself relative to its
        // parent.  Ogre does the math of combining those transforms when it
        // is time to render.
        //
        // Here we create a node to store the pose of the Imu's header frame
        // relative to the RViz fixed frame.
        frame_node_ = parent_node->createChildSceneNode();

        // We create the arrow object within the frame node so that we can
        // set its position and direction relative to its header frame.
        ambient_sound_power_line_ = new rviz::BillboardLine( scene_manager_, frame_node_ );
        //axes_ = new rviz::Axes( scene_manager_ , frame_node_);
    }/*}}}*/

    AmbientSoundVisual::~AmbientSoundVisual()/*{{{*/
    {
        // Delete the arrow to make it disappear.
        delete ambient_sound_power_line_;
        //delete axes_;

        // Destroy the frame node since we don't need it anymore.
        scene_manager_->destroySceneNode( frame_node_ );
    }/*}}}*/

    void AmbientSoundVisual::setMessage( const jsk_hark_msgs::HarkPower::ConstPtr& msg )/*{{{*/
    {
        int directions = msg->directions;
        std::vector<float> powers = msg->powers;

        //float powers[] = {/*{{{*/
            //25,25,28,30,34,32,29,25,25,25,
            //25,25,25,25,25,25,25,25,25,25,
            //25,25,25,25,25,25,25,25,25,25,
            //25,25,25,25,25,25,25,25,25,25,
            //25,25,25,25,25,25,25,25,25,25,
            //25,25,25,25,25,25,25,25,25,25,
            //25,25,25,25,25,25,25,25,25,25,
            //25,25,
        //};/*}}}*/

        if ( powers[0] == 0.0 ){ return;}
        //float min_elem = *std::min_element(powers.begin(),powers.end());
        //axes_->setOrientation(orientation_);
        //axes_->setPosition(position_);
        ambient_sound_power_line_->clear();
        ambient_sound_power_line_->setLineWidth(width_);
        for (int i = 0; i <= directions ; i++) {
            float biased_power = (powers[(i%directions)] - bias_) * grad_;
            if (biased_power <= 0.0) { biased_power = 0.001; }
            Ogre::Vector3 point = Ogre::Vector3((biased_power*scale_)*cos(i*(2*M_PI/directions)- M_PI), (biased_power*scale_)*sin(i*(2*M_PI/directions) - M_PI), 0);
            ambient_sound_power_line_->addPoint(orientation_ * point + position_);
            //std::cout << biased_power << " ";
        }
        //std::cout << std::endl;

        //Ogre::ColourValue color;
        //ambient_sound_power_line_->setColor(color.r, color.g, color.b, color.a);
    }/*}}}*/

    // Position and orientation are passed through to the SceneNode.
    void AmbientSoundVisual::setFramePosition( const Ogre::Vector3& position )/*{{{*/
    {
        //ROS_INFO_STREAM("pos: " << position);
        //frame_node_->setPosition( position ); //<- unnecessary
        position_ = position;
    }

    void AmbientSoundVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
    {
        //ROS_INFO_STREAM("orientation: " << orientation);
        //frame_node_->setOrientation( orientation ); //<- unnecessary
        orientation_ = orientation;
    }
    /*}}}*/

    // Color is passed through to the Arrow object.
    /*{{{*/
    void AmbientSoundVisual::setColor( float r, float g, float b, float a )
    {
        ambient_sound_power_line_->setColor( r, g, b, a );
    }

    // added by sifi
    void AmbientSoundVisual::setWidth( float w )
    {
        width_ = w;
    }
    void AmbientSoundVisual::setScale( float s )
    {
        scale_ = s;
    }
    void AmbientSoundVisual::setBias( float b )
    {
        bias_ = b;
    }
    void AmbientSoundVisual::setGrad( float g )
    {
        grad_ = g;
    }
    /*}}}*/

    // END_TUTORIAL

} // end namespace jsk_rviz_plugins

