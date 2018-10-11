#include "velocity_graph.h"

#include <QPainter>

namespace autoware_rviz_plugins {
namespace vehicle_monitor {

VelocityGraph::VelocityGraph( QWidget* parent ) : QWidget( parent )
{
}

void VelocityGraph::setValue( double value )
{

}

void VelocityGraph::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setWindow(0, 0, 200, 100);
    //painter.setViewport( QRect(0, 0, width(), height()) );
    //QPoint center = this->geometry().center();

    QFont font = painter.font() ;
    font.setPointSize(8);
    painter.setFont(font);

    painter.setPen( QPen(Qt::black, 10, Qt::SolidLine) );
    painter.drawArc(20, 20, 160, 160, 0*16, 180*16);
    painter.drawText(80, 60, 40, 20, Qt::AlignHCenter, QString("TEST"));
}

}}

