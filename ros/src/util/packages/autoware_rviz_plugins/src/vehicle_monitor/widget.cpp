#include "widget.h"

#include <QPainter>

namespace autoware_rviz_plugins {

VehicleMonitorWidget::VehicleMonitorWidget( QWidget* parent ) : QWidget( parent )
{
}

void VehicleMonitorWidget::setValue( double value )
{

}

void VehicleMonitorWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    //painter.setViewport( QRect(0, 0, width(), height()) );
    //QPoint center = this->geometry().center();

    /*
     	label_str->setStyleSheet("background:#4080C0; color:#FFCC00F; padding: 20px 5px 10px 15px;");
-       label_kph->setStyleSheet("background:#4080C0; color:#FFFFFF;");
-       label_mps->setStyleSheet("background:#4080C0; color:#FFFFFF;");
     */
	/*
		QFont font = painter.font() ;
		font.setPointSize(8);
    	painter.setFont(font);
    */

    int text_height = width() / 8;
    int curr_height = 0;
	painter.setBrush(QColor(0x00, 0x00, 0x00));
    painter.drawRect(0, 0, width(), height());


    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawControlMode( painter );

    painter.setViewport(0, curr_height, width(), width()*2/3);
    curr_height += width()*2/3;
    drawVelocityGraph( painter );

    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawVelocityCommand( painter );
    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawVelocityCommand( painter );

    painter.setViewport(0, curr_height, width(), width());
    curr_height += width();
    drawSteeringGraph( painter );

    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawSteeringCommand( painter );
    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawSteeringCommand( painter );

    painter.setViewport(0, curr_height, width(),  width()/2);
    curr_height += width()/2;
    drawPedal( painter );

    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawSteeringCommand( painter );
}


void VehicleMonitorWidget::drawControlMode( QPainter& painter )
{
	constexpr QRect rect_remote = QRect(  2, 0, 97, 23);
	constexpr QRect rect_auto   = QRect(101, 0, 97, 23);

	painter.setWindow(0, 0, 200, 25);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor(0x40, 0x80, 0xC0));
    painter.drawRect( rect_remote );
    painter.drawRect( rect_auto   );

	painter.setPen(QColor(0xFF, 0xFF, 0xFF));
	painter.setBrush(Qt::NoBrush);
    painter.drawText( rect_remote, Qt::AlignHCenter | Qt::AlignVCenter, "REMOTE");
    painter.drawText( rect_auto,   Qt::AlignHCenter | Qt::AlignVCenter, "AUTO"  );
}

void VehicleMonitorWidget::drawVelocityGraph( QPainter& painter )
{
    painter.setWindow(-120, -130, 240, 160);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor(0x40, 0x80, 0xC0));
    painter.drawPie(-70, -70, 140, 140, 0*16, 180*16);
	painter.setBrush(QColor(0x00, 0x00, 0x00));
    painter.drawPie(-50, -50, 100, 100, 0*16, 180*16);

	painter.setPen(QColor(0xFF, 0xFF, 0xFF));
	painter.setBrush(Qt::NoBrush);
    painter.drawText( -100 - 20,    0 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, "0");
    painter.drawText(  -70 - 20,  -70 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, "30");
    painter.drawText(    0 - 20, -100 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, "60");
    painter.drawText(   70 - 20,  -70 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, "90");
    painter.drawText(  100 - 20,    0 - 20, 40, 40 , Qt::AlignHCenter | Qt::AlignVCenter, "120");
}

void VehicleMonitorWidget::drawVelocityCommand( QPainter& painter )
{
	constexpr QRect rect_str = QRect(  2, 0, 48, 23);
	constexpr QRect rect_kph = QRect( 52, 0, 72, 23);
	constexpr QRect rect_mps = QRect(126, 0, 72, 23);

	painter.setWindow(0, 0, 200, 25);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor(0x40, 0x80, 0xC0));
    painter.drawRect( rect_str );
    painter.drawRect( rect_kph );
    painter.drawRect( rect_mps );

	painter.setPen(QColor(0xFF, 0xFF, 0xFF));
	painter.setBrush(Qt::NoBrush);
    painter.drawText( rect_str, Qt::AlignHCenter | Qt::AlignVCenter, "cmd");
    painter.drawText( rect_kph, Qt::AlignHCenter | Qt::AlignVCenter, "XX km/h");
    painter.drawText( rect_mps, Qt::AlignHCenter | Qt::AlignVCenter, "XX.X m/s");
}


void VehicleMonitorWidget::drawSteeringGraph( QPainter& painter )
{
    painter.setWindow(-120, -120, 240, 240);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor(0x40, 0x80, 0xC0));
    painter.drawPie(-70, -70, 140, 140, 0*16, 360*16);
	painter.setBrush(QColor(0x00, 0x00, 0x00));
    painter.drawPie(-50, -50, 100, 100, 0*16, 360*16);
}

void VehicleMonitorWidget::drawSteeringCommand( QPainter& painter )
{
	constexpr QRect rect_str = QRect(  2, 0, 48, 23);
	constexpr QRect rect_deg = QRect( 52, 0, 72, 23);
	constexpr QRect rect_rad = QRect(126, 0, 72, 23);

	painter.setWindow(0, 0, 200, 25);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor(0x40, 0x80, 0xC0));
    painter.drawRect( rect_str );
    painter.drawRect( rect_deg );
    painter.drawRect( rect_rad );

	painter.setPen(QColor(0xFF, 0xFF, 0xFF));
	painter.setBrush(Qt::NoBrush);
    painter.drawText( rect_str, Qt::AlignHCenter | Qt::AlignVCenter, "cmd");
    painter.drawText( rect_deg, Qt::AlignHCenter | Qt::AlignVCenter, "XXX deg");
    painter.drawText( rect_rad, Qt::AlignHCenter | Qt::AlignVCenter, "X.XX rad");
}

void VehicleMonitorWidget::drawPedal( QPainter& painter )
{
	painter.setWindow(0, 0, 240, 120);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor(0x40, 0x80, 0xC0));
    painter.drawRect(  40, 20, 60, 70 );
    painter.drawRect( 140, 20, 60, 70 );

	painter.setPen(QColor(0xFF, 0xFF, 0xFF));
	painter.setBrush(Qt::NoBrush);
    painter.drawText(  40, 90, 60, 20, Qt::AlignHCenter | Qt::AlignVCenter, "BRAKE");
    painter.drawText( 140, 90, 60, 20, Qt::AlignHCenter | Qt::AlignVCenter, "ACCEL");
}

void VehicleMonitorWidget::drawShift( QPainter& painter )
{
	constexpr QRect rect_p = QRect(  2, 0, 48, 23);
	constexpr QRect rect_r = QRect( 52, 0, 48, 23);
	constexpr QRect rect_d = QRect(102, 0, 48, 23);
	constexpr QRect rect_n = QRect(152, 0, 48, 23);

	painter.setWindow(0, 0, 202, 25);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor(0x40, 0x80, 0xC0));
    painter.drawRect( rect_p );
    painter.drawRect( rect_r );
    painter.drawRect( rect_d );
    painter.drawRect( rect_n );

	painter.setPen(QColor(0xFF, 0xFF, 0xFF));
	painter.setBrush(Qt::NoBrush);
    painter.drawText( rect_p, Qt::AlignHCenter | Qt::AlignVCenter, "P");
    painter.drawText( rect_r, Qt::AlignHCenter | Qt::AlignVCenter, "R");
    painter.drawText( rect_d, Qt::AlignHCenter | Qt::AlignVCenter, "D");
    painter.drawText( rect_n, Qt::AlignHCenter | Qt::AlignVCenter, "N");
}

}
