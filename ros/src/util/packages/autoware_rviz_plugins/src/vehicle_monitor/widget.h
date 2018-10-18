#ifndef VEHICLE_MONITOR_WIDGET_H_INCLUDED
#define VEHICLE_MONITOR_WIDGET_H_INCLUDED

#include <QWidget>

namespace autoware_rviz_plugins {

class VehicleMonitorWidget : public QWidget
{
	Q_OBJECT

	public:

		VehicleMonitorWidget( QWidget* parent = 0 );

        enum CtrlMode
        {
            REMOTE, AUTO, UNKNOWN
        };

        struct Gear
        {
            std::string name;
            int value;
        };
        using GearList = std::vector<Gear>;

	public Q_SLOTS:

        void setCtrlMode( CtrlMode ctrl_mode );
		void setSpeedCmd( double kph );
        void setAngleCmd( double deg );
        void setSpeedStatus( double kph );
        void setAngleStatus( double deg );
        void setShiftStatus( double value );
        void setBrakeStatus( double value );
        void setAccelStatus( double value );

        void configureSpeedLimit( int limit );
        void configureBrakeLimit( int limit );
        void configureAccelLimit( int limit );
        void configureTransmission( const GearList& gear_list );

	protected:

        void paintEvent( QPaintEvent* event ) override;

		void drawControlMode  ( QPainter& painter );
		void drawSpeedGraph   ( QPainter& painter );
		void drawAngleGraph   ( QPainter& painter );
		void drawDualUnitsText( QPainter& painter, const QColor& color, const QString& title, const QString& unit1, const QString& unit2 );
		void drawPedal        ( QPainter& painter );
		void drawShift        ( QPainter& painter );

        QString toString(double value, int precision);

	private:

        CtrlMode twist_gate_ctrl_mode;
        double cmd_speed_kph;
        double cmd_angle_deg;
        double status_speed_kph;
        double status_angle_deg;
        int status_shift;
        int status_accel;
        int status_brake;
        //int status_speed_mode;
        //int status_angle_mode;

        int config_speed_limit;
        int config_brake_limit;
        int config_accel_limit;
        GearList config_gear_list;
};

}

#endif // VEHICLE_MONITOR_WIDGET_H_INCLUDED
