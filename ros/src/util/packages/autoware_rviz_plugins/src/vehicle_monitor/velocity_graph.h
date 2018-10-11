#ifndef VEHICLE_MONITOR_VELOCITY_GRAPH_H_INCLUDED
#define VEHICLE_MONITOR_VELOCITY_GRAPH_H_INCLUDED

#include <QWidget>

namespace autoware_rviz_plugins {
namespace vehicle_monitor {

class VelocityGraph : public QWidget
{
	Q_OBJECT

	public:

		VelocityGraph( QWidget* parent = 0 );



	public Q_SLOTS:

		void setValue( double value );

	protected:

		void paintEvent(QPaintEvent *event) override;
};

}}

#endif // VEHICLE_MONITOR_VELOCITY_TEXT_H_INCLUDED
