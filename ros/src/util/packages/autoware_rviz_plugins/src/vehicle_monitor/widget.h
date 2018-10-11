#ifndef VEHICLE_MONITOR_WIDGET_H_INCLUDED
#define VEHICLE_MONITOR_WIDGET_H_INCLUDED

#include <QWidget>
#include <QLabel>

namespace autoware_rviz_plugins
{

	class VehicleVelocityWidget : public QWidget
	{
		Q_OBJECT

		public:

			VehicleVelocityWidget( QWidget* parent = 0 );

			/*
			virtual void load( const rviz::Config& config );
			virtual void save( rviz::Config config ) const;
			*/


		public Q_SLOTS:

			void setVelocity( double kph );

		private:

			QLabel* cmd_title;
			QLabel* cmd_kph;
			QLabel* cmd_mps;
			QLabel* status_title;
			QLabel* status_kph;
			QLabel* status_mps;

			/*
			DriveWidget* velocity_graph_;
			DriveWidget* steering_graph_;
			DriveWidget* pedal_shift_graph_;

			QString output_topic_;
			ros::Publisher velocity_publisher_;
			ros::NodeHandle nh_;
			float linear_velocity_;
			float angular_velocity_;
			*/
	};

}

#endif // VEHICLE_MONITOR_WIDGET_H_INCLUDED
