#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>

#include <boost/foreach.hpp>
#include "ambient_sound_visual.h"
#include "ambient_sound_display_groovy.h"

namespace jsk_rviz_plugins
{

    AmbientSoundDisplay::AmbientSoundDisplay()/*{{{*/
    {
          color_property_ = new rviz::ColorProperty("Color",QColor( 204, 51, 204),
                  "Color to draw the acceleration arrows." ,
                  this, SLOT(updateColorAndAlpha()));
          alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                  "0 is fully transparent, 1.0 is fully opaque.",
                  this, SLOT( updateColorAndAlpha() ));
          history_length_property_ = new rviz::IntProperty("History Length", 1,
                  "Number of prior measurements to display." ,
                  this, SLOT(updateHistoryLength()));
          width_property_ = new rviz::FloatProperty("Width", 0.1,
                  "Width of line",
                  this, SLOT(updateAppearance()));
          scale_property_ = new rviz::FloatProperty("Scale", 1.0,
                  "Scale of line",
                  this, SLOT(updateAppearance()));
          bias_property_ = new rviz::FloatProperty("Bias", 10,
                  "Bias",
                  this, SLOT(updateAppearance()));
          grad_property_ = new rviz::FloatProperty("Gradient", 0.1,
                  "Gradient",
                  this, SLOT(updateAppearance()));
          history_length_property_->setMin( 1 );
          history_length_property_->setMax( 1 );
    }/*}}}*/

    // After the parent rviz::Display::initialize() does its own setup, it
    // calls the subclass's onInitialize() function.  This is where we
    // instantiate all the workings of the class.
    void AmbientSoundDisplay::onInitialize()/*{{{*/
    {
        MFDClass::onInitialize();
        // Make an Ogre::SceneNode to contain all our visuals.
        //scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

        // Set the default history length and resize the ``visuals_`` array.
        //setHistoryLength( 1 );
        updateHistoryLength();

        // A tf::MessageFilter listens to ROS messages and calls our
        // callback with them when they can be matched up with valid tf
        // transform data.
//        tf_filter_ =
//            new tf::MessageFilter<jsk_hark_msgs::HarkPower>( *context_->getTFClient(),
//                    "", 100, update_nh_ );
//        tf_filter_->connectInput( sub_ );
//        tf_filter_->registerCallback( boost::bind( &AmbientSoundDisplay::incomingMessage,
//                    this, _1 ));

        // FrameManager has some built-in functions to set the status of a
        // Display based on callbacks from a tf::MessageFilter.  These work
        // fine for this simple display.
//        vis_manager_->getFrameManager()
//            ->registerFilterForTransformStatusCheck( tf_filter_, this );
    }/*}}}*/

    AmbientSoundDisplay::~AmbientSoundDisplay()/*{{{*/
    {
//        for( size_t i = 0; i < visuals_.size(); i++ )
//        {
//            delete visuals_[ i ];
//        }
//
//        delete tf_filter_;
    }/*}}}*/

    // Clear the visuals by deleting their objects.
    void AmbientSoundDisplay::reset()
    {
        MFDClass::reset();
        visuals_.clear();
    }

/*{{{*/
//    void AmbientSoundDisplay::clear()
//    {
//        for( size_t i = 0; i < visuals_.size(); i++ )
//        {
//            delete visuals_[ i ];
//            visuals_[ i ] = NULL;
//        }
//        tf_filter_->clear();
//        messages_received_ = 0;
//        setStatus( rviz::status_levels::Warn, "Topic", "No messages received" );
//    }/*}}}*/

    // Set the current color and alpha values for each visual.
    void AmbientSoundDisplay::updateColorAndAlpha()/*{{{*/
    {
        float alpha = alpha_property_->getFloat();
        Ogre::ColourValue color = color_property_->getOgreColor();

        for( size_t i = 0; i < visuals_.size(); i++ )
        {
            if( visuals_[ i ] )
            {
                visuals_[ i ]->setColor( color.r, color.g, color.b, alpha );
            }
        }
    }/*}}}*/

    // Set the appearance of the line
    void AmbientSoundDisplay::updateAppearance(){/*{{{*/
        float width = width_property_->getFloat();
        float scale = scale_property_->getFloat();
        float bias = bias_property_->getFloat();
        float grad = grad_property_->getFloat();

        for( size_t i = 0; i < visuals_.size(); i++ )
        {
            if( visuals_[ i ] )
            {
                visuals_[ i ]->setWidth( width );
                visuals_[ i ]->setScale( scale );
                visuals_[ i ]->setBias ( bias );
                visuals_[ i ]->setGrad ( grad );
            }
        }
    }/*}}}*/

    // Set the number of past visuals to show.
    void AmbientSoundDisplay::updateHistoryLength(){/*{{{*/
        visuals_.rset_capacity(history_length_property_->getInt());
    }/*}}}*/

    bool AmbientSoundDisplay::validateFloats( const jsk_hark_msgs::HarkPower& msg )
    {
        std::vector<float>::const_iterator it = msg.powers.begin();
        for (; it < msg.powers.end(); ++it) {
            if(!rviz::validateFloats(*it)){
                return false;
            };
        }        
        return true;
    }

    //void AmbientSoundDisplay::setHistoryLength( int length )[>{{{<]
    //{
        //// Don't let people enter invalid values.
        //if( length < 1 )
        //{
            //length = 1;
        //}
        //// If the length is not changing, we don't need to do anything.
        //if( history_length_ == length )
        //{
            //return;
        //}

        //// Set the actual variable.
        //history_length_ = length;
        //propertyChanged( history_length_property_ );

        //// Create a new array of visual pointers, all NULL.
        //std::vector<AmbientSoundVisual*> new_visuals( history_length_, (AmbientSoundVisual*)0 );

        //// Copy the contents from the old array to the new.
        //// (Number to copy is the minimum of the 2 vector lengths).
        //size_t copy_len =
            //(new_visuals.size() > visuals_.size()) ?
            //visuals_.size() : new_visuals.size();
        //for( size_t i = 0; i < copy_len; i++ )
        //{
            //int new_index = (messages_received_ - i) % new_visuals.size();
            //int old_index = (messages_received_ - i) % visuals_.size();
            //new_visuals[ new_index ] = visuals_[ old_index ];
            //visuals_[ old_index ] = NULL;
        //}

        //// Delete any remaining old visuals
        //for( size_t i = 0; i < visuals_.size(); i++ ) {
            //delete visuals_[ i ];
        //}

        //// We don't need to create any new visuals here, they are created as
        //// needed when messages are received.

        //// Put the new vector into the member variable version and let the
        //// old one go out of scope.
        //visuals_.swap( new_visuals );
    //}[>}}}<]

    // When the "Fixed Frame" changes, we need to update our
    // tf::MessageFilter and erase existing visuals.
    //void AmbientSoundDisplay::fixedFrameChanged()[>{{{<]
    //{
        //tf_filter_->setTargetFrame( fixed_frame_ );
        //clear();
    //}[>}}}<]

    // This is our callback to handle an incoming message.
    void AmbientSoundDisplay::processMessage( const jsk_hark_msgs::HarkPower::ConstPtr& msg )/*{{{*/
    {
        if( !validateFloats( *msg ))
        {
            setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
            return;
        }

        // Here we call the rviz::FrameManager to get the transform from the
        // fixed frame to the frame in the header of this Imu message.  If
        // it fails, we can't do anything else so we return.
        Ogre::Quaternion orientation;
        Ogre::Vector3 position;
        if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                    msg->header.stamp,
                    position, orientation ))
        {
            ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
            return;
        }

        // We are keeping a circular buffer of visual pointers.  This gets
        // the next one, or creates and stores it if it was missing.
        boost::shared_ptr<AmbientSoundVisual> visual;
        //AmbientSoundVisual* visual_ptr;
        if( visuals_.full())
        {
            visual =  visuals_.front();
            //visual.reset(visuals_.front());
            //visual_ptr = visuals_.front();
            //visual.reset(visual_ptr);
        } else {
            visual.reset(new AmbientSoundVisual(context_->getSceneManager(), scene_node_));
        }

        // Now set or update the contents of the chosen visual.
        visual->setMessage( msg );
        visual->setFramePosition( position );
        visual->setFrameOrientation( orientation );

        float alpha = alpha_property_->getFloat();
        Ogre::ColourValue color = color_property_->getOgreColor();
        float width = width_property_->getFloat();
        float scale = scale_property_->getFloat();
        float bias = bias_property_->getFloat();
        float grad = grad_property_->getFloat();

        visual->setColor( color.r, color.g, color.b, alpha );
        visual->setWidth( width );
        visual->setScale( scale );
        visual->setBias ( bias );
        visual->setGrad ( grad );

        visuals_.push_back(visual);
    }/*}}}*/

} // end namespace jsk_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::AmbientSoundDisplay, rviz::Display )

