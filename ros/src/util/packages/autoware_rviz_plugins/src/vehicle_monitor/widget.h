#ifndef VEHICLE_MONITOR_VELOCITY_GRAPH_H_INCLUDED
#define VEHICLE_MONITOR_VELOCITY_GRAPH_H_INCLUDED

#include <QWidget>

namespace autoware_rviz_plugins {

class VehicleMonitorWidget : public QWidget
{
	Q_OBJECT

	public:

		VehicleMonitorWidget( QWidget* parent = 0 );



	public Q_SLOTS:

		void setValue( double value );

	protected:

		void paintEvent( QPaintEvent*event ) override;

		void drawControlMode    ( QPainter& painter );
		void drawVelocityGraph  ( QPainter& painter );
		void drawVelocityCommand( QPainter& painter );
		void drawSteeringGraph  ( QPainter& painter );
		void drawSteeringCommand( QPainter& painter );
		void drawPedal          ( QPainter& painter );
		void drawShift          ( QPainter& painter );
};

}

#endif // VEHICLE_MONITOR_VELOCITY_TEXT_H_INCLUDED
