#ifndef VEHICLE_MONITOR_WIDGET_H_INCLUDED
#define VEHICLE_MONITOR_WIDGET_H_INCLUDED

#include <QWidget>

namespace autoware_rviz_plugins {

class VehicleMonitorWidget : public QWidget
{
	Q_OBJECT

	public:

		VehicleMonitorWidget( QWidget* parent = 0 );

	public Q_SLOTS:

		void setSpeedCmd( double mps );
		void setAngleCmd( double rad );
		void setSpeedStatus( double mps );
		void setAngleStatus( double rad );

	protected:

		void paintEvent( QPaintEvent*event ) override;

		void drawControlMode  ( QPainter& painter );
		void drawSpeedGraph   ( QPainter& painter );
		void drawAngleGraph   ( QPainter& painter );
		void drawDualUnitsText( QPainter& painter, const QColor& color, const QString& title, const QString& unit1, const QString& unit2 );
		void drawPedal        ( QPainter& painter );
		void drawShift        ( QPainter& painter );

	private:

		QString speed_cmd_kph;
		QString speed_cmd_mps;
		QString angle_cmd_deg;
		QString angle_cmd_rad;

		double  speed_status_val;
		QString speed_status_kph;
		QString speed_status_mps;

		double  angle_status_val;
		QString angle_status_deg;
		QString angle_status_rad;
};

}

#endif // VEHICLE_MONITOR_WIDGET_H_INCLUDED
