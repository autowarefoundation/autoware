#ifndef VEHICLE_MONITOR_VELOCITY_TEXT_H_INCLUDED
#define VEHICLE_MONITOR_VELOCITY_TEXT_H_INCLUDED

#include <QWidget>
#include <QLabel>

namespace autoware_rviz_plugins {
namespace vehicle_monitor {

class VelocityText : public QWidget
{
	Q_OBJECT

	public:

		VelocityText( QWidget* parent = 0 );

	public Q_SLOTS:

		void setLabel( QString title );
		void setVelocityKph( double kph );
		void setVelocityMps( double mps );

	private:

		QLabel* label_str;
		QLabel* label_kph;
		QLabel* label_mps;
};

}}

#endif // VEHICLE_MONITOR_VELOCITY_TEXT_H_INCLUDED
