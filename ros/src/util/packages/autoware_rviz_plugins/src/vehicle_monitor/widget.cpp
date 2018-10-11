#include "widget.h"

//#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
//#include <QTimer>
//#include <geometry_msgs/Twist.h>



#include <ros/ros.h>

namespace autoware_rviz_plugins
{

class AutoLabel : public QLabel
{

};


VehicleVelocityWidget::VehicleVelocityWidget( QWidget* parent ) : QWidget( parent )
{

	cmd_title    = new QLabel( "cmd" );
	status_title = new QLabel( "status" );
	cmd_kph      = new QLabel( "XX km/h" );
	status_kph   = new QLabel( "XX km/h" );
	cmd_mps      = new QLabel( "XX m/s" );
	status_mps   = new QLabel( "XX m/s" );

	cmd_title->setStyleSheet("background:#4080C0; color:#FFFFFF; padding: 20px 5px 10px 15px;");
	cmd_kph->setStyleSheet("background:#4080C0; color:#FFFFFF;");
	cmd_mps->setStyleSheet("background:#4080C0; color:#FFFFFF;");

	status_title->setStyleSheet("background:#4080C0; color:#FFCC00;");
	status_kph->setStyleSheet("background:#4080C0; color:#FFCC00;");
	status_mps->setStyleSheet("background:#4080C0; color:#FFCC00;");


	QHBoxLayout* cmd_layout = new QHBoxLayout;
	cmd_layout->addWidget( cmd_title );
	cmd_layout->addWidget( cmd_kph );
	cmd_layout->addWidget( cmd_mps );

	QHBoxLayout* status_layout = new QHBoxLayout;
	status_layout->addWidget( status_title );
	status_layout->addWidget( status_kph );
	status_layout->addWidget( status_mps );

	QVBoxLayout* widget_layout = new QVBoxLayout;
	widget_layout->addLayout( cmd_layout  );
	widget_layout->addLayout( status_layout );
	setLayout( widget_layout );

  // Then create the control widget.
  //velocity_graph_ = new DriveWidget;
  //steering_graph_ = new DriveWidget;
  //pedal_shift_graph_ = new DriveWidget;

  //pedal_shift_graph_->setStyleSheet("border-style : solid;");
  //pedal_shift_graph_->setStyleSheet("border-width : 1px;");

  // Lay out the topic field above the control widget.


  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this VehicleVelocityWidget
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  //QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  //connect( velocity_graph_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  //connect( output_topic_editor_, SIGNAL( editingFinished() ), this, SLOT( updateTopic() ));
  //connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Start the timer.
  //output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  //velocity_graph_->setEnabled( false );
}

void VehicleVelocityWidget::setVelocity(double kph)
{
	ROS_INFO("Widget: %p %f", cmd_mps, kph);
	cmd_kph->setText(QString::fromStdString(std::to_string(kph      )));
	cmd_mps->setText(QString::fromStdString(std::to_string(kph / 3.6)));
}


/*
// setVel() is connected to the DriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.
void VehicleVelocityWidget::setVel( float lin, float ang )
{
  linear_velocity_ = lin;
  angular_velocity_ = ang;
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void VehicleVelocityWidget::updateTopic()
{
  //setTopic( output_topic_editor_->text() );
}

// Set the topic name we are publishing to.
void VehicleVelocityWidget::setTopic( const QString& new_topic )
{
  // Only take action if the name has changed.
  if( new_topic != output_topic_ )
  {
    output_topic_ = new_topic;
    // If the topic is the empty string, don't publish anything.
    if( output_topic_ == "" )
    {
      velocity_publisher_.shutdown();
    }
    else
    {
      // The old ``velocity_publisher_`` is destroyed by this assignment,
      // and thus the old topic advertisement is removed.  The call to
      // nh_advertise() says we want to publish data on the new topic
      // name.
      velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

  // Gray out the control widget when the output topic is empty.
  velocity_graph_->setEnabled( output_topic_ != "" );
}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void VehicleVelocityWidget::sendVel()
{
  if( ros::ok() && velocity_publisher_ )
  {
    geometry_msgs::Twist msg;
    msg.linear.x = linear_velocity_;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angular_velocity_;
    velocity_publisher_.publish( msg );
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void VehicleVelocityWidget::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void VehicleVelocityWidget::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    //output_topic_editor_->setText( topic );
    updateTopic();
  }
}
*/

}

