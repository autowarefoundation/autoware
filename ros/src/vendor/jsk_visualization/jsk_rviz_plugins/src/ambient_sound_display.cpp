#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>

#include "ambient_sound_visual.h"

#include "ambient_sound_display.h"

namespace jsk_rviz_plugins
{

    // BEGIN_TUTORIAL
    // The constructor must have no arguments, so we can't give the
    // constructor the parameters it needs to fully initialize.
    AmbientSoundDisplay::AmbientSoundDisplay()/*{{{*/
        : Display()
          , scene_node_( NULL )
          , messages_received_( 0 )
          , color_( .8, .2, .8 )       // Default color is bright purple.
          , alpha_( 1.0 )              // Default alpha is completely opaque.
          , width_(0.1)
          , scale_(0.1)
    {
    }/*}}}*/

    // After the parent rviz::Display::initialize() does its own setup, it
    // calls the subclass's onInitialize() function.  This is where we
    // instantiate all the workings of the class.
    void AmbientSoundDisplay::onInitialize()/*{{{*/
    {
        // Make an Ogre::SceneNode to contain all our visuals.
        scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

        // Set the default history length and resize the ``visuals_`` array.
        setHistoryLength( 1 );

        // A tf::MessageFilter listens to ROS messages and calls our
        // callback with them when they can be matched up with valid tf
        // transform data.
        tf_filter_ =
            new tf::MessageFilter<jsk_hark_msgs::HarkPower>( *vis_manager_->getTFClient(),
                    "", 100, update_nh_ );
        tf_filter_->connectInput( sub_ );
        tf_filter_->registerCallback( boost::bind( &AmbientSoundDisplay::incomingMessage,
                    this, _1 ));

        // FrameManager has some built-in functions to set the status of a
        // Display based on callbacks from a tf::MessageFilter.  These work
        // fine for this simple display.
        vis_manager_->getFrameManager()
            ->registerFilterForTransformStatusCheck( tf_filter_, this );
    }/*}}}*/

    AmbientSoundDisplay::~AmbientSoundDisplay()/*{{{*/
    {
        unsubscribe();
        clear();
        for( size_t i = 0; i < visuals_.size(); i++ )
        {
            delete visuals_[ i ];
        }

        delete tf_filter_;
    }/*}}}*/

    // Clear the visuals by deleting their objects.
    void AmbientSoundDisplay::clear()/*{{{*/
    {
        for( size_t i = 0; i < visuals_.size(); i++ )
        {
            delete visuals_[ i ];
            visuals_[ i ] = NULL;
        }
        tf_filter_->clear();
        messages_received_ = 0;
        setStatus( rviz::status_levels::Warn, "Topic", "No messages received" );
    }/*}}}*/

    void AmbientSoundDisplay::setTopic( const std::string& topic )/*{{{*/
    {
        unsubscribe();
        clear();
        topic_ = topic;
        subscribe();

        // Broadcast the fact that the variable has changed.
        propertyChanged( topic_property_ );

        // Make sure rviz renders the next time it gets a chance.
        causeRender();
    }/*}}}*/

    void AmbientSoundDisplay::setColor( const rviz::Color& color )/*{{{*/
    {
        color_ = color;

        propertyChanged( color_property_ );
        updateColorAndAlpha();
        causeRender();
    }/*}}}*/

    void AmbientSoundDisplay::setAlpha( float alpha )/*{{{*/
    {
        alpha_ = alpha;

        propertyChanged( alpha_property_ );
        updateColorAndAlpha();
        causeRender();
    }/*}}}*/

    void AmbientSoundDisplay::setWidth( float width )/*{{{*/
    {
        width_ = width;
        
        propertyChanged( width_property_ );
        updateColorAndAlpha();
        causeRender();
    }/*}}}*/

    void AmbientSoundDisplay::setScale( float scale )/*{{{*/
    {
        scale_ = scale;
        
        propertyChanged( scale_property_ );
        updateColorAndAlpha();
        causeRender();
    }
    /*}}}*/
    
    void AmbientSoundDisplay::setBias( float bias )/*{{{*/
    {
        bias_ = bias;
        
        propertyChanged( bias_property_ );
        updateColorAndAlpha();
        causeRender();
    }/*}}}*/

    void AmbientSoundDisplay::setGrad( float grad )/*{{{*/
    {
        grad_ = grad;
        
        propertyChanged( grad_property_ );
        updateColorAndAlpha();
        causeRender();
    }/*}}}*/

    // Set the current color and alpha values for each visual.
    void AmbientSoundDisplay::updateColorAndAlpha()/*{{{*/
    {
        for( size_t i = 0; i < visuals_.size(); i++ )
        {
            if( visuals_[ i ] )
            {
                visuals_[ i ]->setColor( color_.r_, color_.g_, color_.b_, alpha_ );
            }
        }
    }/*}}}*/

    // Set the number of past visuals to show.
    void AmbientSoundDisplay::setHistoryLength( int length )/*{{{*/
    {
        // Don't let people enter invalid values.
        if( length < 1 )
        {
            length = 1;
        }
        // If the length is not changing, we don't need to do anything.
        if( history_length_ == length )
        {
            return;
        }

        // Set the actual variable.
        history_length_ = length;
        propertyChanged( history_length_property_ );

        // Create a new array of visual pointers, all NULL.
        std::vector<AmbientSoundVisual*> new_visuals( history_length_, (AmbientSoundVisual*)0 );

        // Copy the contents from the old array to the new.
        // (Number to copy is the minimum of the 2 vector lengths).
        size_t copy_len =
            (new_visuals.size() > visuals_.size()) ?
            visuals_.size() : new_visuals.size();
        for( size_t i = 0; i < copy_len; i++ )
        {
            int new_index = (messages_received_ - i) % new_visuals.size();
            int old_index = (messages_received_ - i) % visuals_.size();
            new_visuals[ new_index ] = visuals_[ old_index ];
            visuals_[ old_index ] = NULL;
        }

        // Delete any remaining old visuals
        for( size_t i = 0; i < visuals_.size(); i++ ) {
            delete visuals_[ i ];
        }

        // We don't need to create any new visuals here, they are created as
        // needed when messages are received.

        // Put the new vector into the member variable version and let the
        // old one go out of scope.
        visuals_.swap( new_visuals );
    }/*}}}*/

    void AmbientSoundDisplay::subscribe()/*{{{*/
    {
        // If we are not actually enabled, don't do it.
        if ( !isEnabled() )
        {
            return;
        }

        // Try to subscribe to the current topic name (in ``topic_``).  Make
        // sure to catch exceptions and set the status to a descriptive
        // error message.
        try
        {
            sub_.subscribe( update_nh_, topic_, 10 );
            setStatus( rviz::status_levels::Ok, "Topic", "OK" );
        }
        catch( ros::Exception& e )
        {
            setStatus( rviz::status_levels::Error, "Topic",
                    std::string( "Error subscribing: " ) + e.what() );
        }
    }/*}}}*/

    void AmbientSoundDisplay::unsubscribe()/*{{{*/
    {
        sub_.unsubscribe();
    }/*}}}*/

    void AmbientSoundDisplay::onEnable()/*{{{*/
    {
        subscribe();
    }/*}}}*/

    void AmbientSoundDisplay::onDisable()/*{{{*/
    {
        unsubscribe();
        clear();
    }/*}}}*/

    // When the "Fixed Frame" changes, we need to update our
    // tf::MessageFilter and erase existing visuals.
    void AmbientSoundDisplay::fixedFrameChanged()/*{{{*/
    {
        tf_filter_->setTargetFrame( fixed_frame_ );
        clear();
    }/*}}}*/

    // This is our callback to handle an incoming message.
    void AmbientSoundDisplay::incomingMessage( const jsk_hark_msgs::HarkPower::ConstPtr& msg )/*{{{*/
    {
        ++messages_received_;

        // Each display can have multiple status lines.  This one is called
        // "Topic" and says how many messages have been received in this case.
        std::stringstream ss;
        ss << messages_received_ << " messages received";
        setStatus( rviz::status_levels::Ok, "Topic", ss.str() );

        // Here we call the rviz::FrameManager to get the transform from the
        // fixed frame to the frame in the header of this Imu message.  If
        // it fails, we can't do anything else so we return.
        Ogre::Quaternion orientation;
        Ogre::Vector3 position;
        if( !vis_manager_->getFrameManager()->getTransform( msg->header.frame_id,
                    msg->header.stamp,
                    position, orientation ))
        {
            ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), fixed_frame_.c_str() );
            return;
        }

        // We are keeping a circular buffer of visual pointers.  This gets
        // the next one, or creates and stores it if it was missing.
        AmbientSoundVisual* visual = visuals_[ messages_received_ % history_length_ ];
        if( visual == NULL )
        {
            visual = new AmbientSoundVisual( vis_manager_->getSceneManager(), scene_node_ );
            visuals_[ messages_received_ % history_length_ ] = visual;
        }

        // Now set or update the contents of the chosen visual.
        visual->setFramePosition( position );
        visual->setFrameOrientation( orientation );
        visual->setColor( color_.r_, color_.g_, color_.b_, alpha_ );
        visual->setWidth( width_ );
        visual->setScale( scale_ );
        visual->setBias ( bias_ );
        visual->setGrad ( grad_ );
        visual->setMessage( msg );
    }/*}}}*/

    // Override rviz::Display's reset() function to add a call to clear().
    void AmbientSoundDisplay::reset()/*{{{*/
    {
        Display::reset();
        clear();
    }/*}}}*/

    // Override createProperties() to build and configure a Property
    // object for each user-editable property.  ``property_manager_``,
    // ``property_prefix_``, and ``parent_category_`` are all initialized before
    // this is called.
    void AmbientSoundDisplay::createProperties()/*{{{*/
    {
        topic_property_ =
            property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic",
                    property_prefix_,
                    boost::bind( &AmbientSoundDisplay::getTopic, this ),
                    boost::bind( &AmbientSoundDisplay::setTopic, this, _1 ),
                    parent_category_,
                    this );
        setPropertyHelpText( topic_property_, "jsk_hark_msgs::HarkPower topic to subscribe to." );
        rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
        topic_prop->setMessageType( ros::message_traits::datatype<jsk_hark_msgs::HarkPower>() );

        color_property_ =
            property_manager_->createProperty<rviz::ColorProperty>( "Color",
                    property_prefix_,
                    boost::bind( &AmbientSoundDisplay::getColor, this ),
                    boost::bind( &AmbientSoundDisplay::setColor, this, _1 ),
                    parent_category_,
                    this );
        setPropertyHelpText( color_property_, "Color to draw the acceleration arrows." );

        alpha_property_ =
            property_manager_->createProperty<rviz::FloatProperty>( "Alpha",
                    property_prefix_,
                    boost::bind( &AmbientSoundDisplay::getAlpha, this ),
                    boost::bind( &AmbientSoundDisplay::setAlpha, this, _1 ),
                    parent_category_,
                    this );
        setPropertyHelpText( alpha_property_, "0 is fully transparent, 1.0 is fully opaque." );

        history_length_property_ =
            property_manager_->createProperty<rviz::IntProperty>( "History Length",
                    property_prefix_,
                    boost::bind( &AmbientSoundDisplay::getHistoryLength, this ),
                    boost::bind( &AmbientSoundDisplay::setHistoryLength, this, _1 ),
                    parent_category_,
                    this );
        setPropertyHelpText( history_length_property_, "Number of prior measurements to display." );
        
        width_property_ = 
            property_manager_->createProperty<rviz::FloatProperty>( "Width",
                    property_prefix_,
                    boost::bind( &AmbientSoundDisplay::getWidth, this ),
                    boost::bind( &AmbientSoundDisplay::setWidth, this, _1 ),
                    parent_category_,
                    this );
        setPropertyHelpText( width_property_, "Width of line" );

        scale_property_ = 
            property_manager_->createProperty<rviz::FloatProperty>( "Scale",
                    property_prefix_,
                    boost::bind( &AmbientSoundDisplay::getScale, this ),
                    boost::bind( &AmbientSoundDisplay::setScale, this, _1 ),
                    parent_category_,
                    this );
        setPropertyHelpText( scale_property_, "Scale of line" );
        
        grad_property_ = 
            property_manager_->createProperty<rviz::FloatProperty>( "Grad",
                    property_prefix_,
                    boost::bind( &AmbientSoundDisplay::getGrad, this ),
                    boost::bind( &AmbientSoundDisplay::setGrad, this, _1 ),
                    parent_category_,
                    this );
        setPropertyHelpText( grad_property_, "Graduation" );

        bias_property_ = 
            property_manager_->createProperty<rviz::FloatProperty>( "Bias",
                    property_prefix_,
                    boost::bind( &AmbientSoundDisplay::getBias, this ),
                    boost::bind( &AmbientSoundDisplay::setBias, this, _1 ),
                    parent_category_,
                    this );
        setPropertyHelpText( bias_property_, "Bias to subtract" );
    }/*}}}*/

} // end namespace jsk_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( jsk_rviz_plugins::AmbientSoundDisplay, rviz::Display )

