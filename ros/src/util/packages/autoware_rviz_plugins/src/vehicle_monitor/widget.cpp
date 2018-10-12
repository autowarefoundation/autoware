#include "widget.h"

#include <QPainter>

#include <sstream>
#include <iomanip>

namespace autoware_rviz_plugins {

VehicleMonitorWidget::VehicleMonitorWidget( QWidget* parent ) : QWidget( parent )
{
	speed_status_val = 0;
	angle_status_val = 0;
}

void VehicleMonitorWidget::setSpeedCmd( double mps )
{
	// mps
	{
		std::ostringstream sout;
		sout << std::fixed << std::setprecision(1)  << mps;
		speed_cmd_mps = QString::fromStdString( sout.str() + " m/s" );
	}

	// kpm
	{
		std::ostringstream sout;
		sout << std::fixed << std::setprecision(0) << (mps * 3.6);
		speed_cmd_kph = QString::fromStdString( sout.str() + " km/h" );
	}
}


void VehicleMonitorWidget::setAngleCmd( double rad )
{
	// radian
	{
		std::ostringstream sout;
		sout << std::fixed << std::setprecision(2)  << rad;
		angle_cmd_rad = QString::fromStdString( sout.str() + " rad" );
	}

	// degree
	{
		std::ostringstream sout;
		sout << std::fixed << std::setprecision(0) << (rad * 180 / M_PI);
		angle_cmd_deg = QString::fromStdString( sout.str() + " deg" );
	}
}

void VehicleMonitorWidget::setSpeedStatus( double mps )
{
	speed_status_val = mps * 3.6;

	// mps
	{
		std::ostringstream sout;
		sout << std::fixed << std::setprecision(1)  << mps;
		speed_status_mps = QString::fromStdString( sout.str() + " m/s" );
	}

	// kpm
	{
		std::ostringstream sout;
		sout << std::fixed << std::setprecision(0) << (mps * 3.6);
		speed_status_kph = QString::fromStdString( sout.str() + " km/h" );
	}
}


void VehicleMonitorWidget::setAngleStatus( double rad )
{
	angle_status_val = rad * 180 / M_PI;

	// radian
	{
		std::ostringstream sout;
		sout << std::fixed << std::setprecision(2)  << rad;
		angle_status_rad = QString::fromStdString( sout.str() + " rad" );
	}

	// degree
	{
		std::ostringstream sout;
		sout << std::fixed << std::setprecision(0) << (rad * 180 / M_PI);
		angle_status_deg = QString::fromStdString( sout.str() + " deg" );
	}
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
    drawSpeedGraph( painter );

    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawDualUnitsText( painter, QColor(0xFF, 0xFF, 0xFF), "cmd", speed_cmd_kph, speed_cmd_mps );

    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawDualUnitsText( painter, QColor(0xFF, 0xCC, 0x00), "status", speed_status_kph, speed_status_mps );

    painter.setViewport(0, curr_height, width(), width());
    curr_height += width();
    drawAngleGraph( painter );

    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawDualUnitsText( painter, QColor(0xFF, 0xFF, 0xFF), "cmd", angle_cmd_deg, angle_cmd_rad );

    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawDualUnitsText( painter, QColor(0xFF, 0xCC, 0x00), "status", angle_status_deg, angle_status_rad );

    painter.setViewport(0, curr_height, width(),  width()/2);
    curr_height += width()/2;
    drawPedal( painter );

    painter.setViewport(0, curr_height, width(), text_height);
    curr_height += text_height;
    drawShift( painter );

    int rest_height = height() - curr_height;
    painter.setViewport(0, curr_height, width(), rest_height);
	painter.setWindow(0, 0, 200, 200 * rest_height / width());
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

void VehicleMonitorWidget::drawSpeedGraph( QPainter& painter )
{
	int status_arc = 180 * (speed_status_val / 120.0);

    painter.setWindow(-120, -130, 240, 160);

	painter.setPen(Qt::NoPen);
	painter.setBrush(QColor(0x40, 0x80, 0xC0));
    painter.drawPie(-70, -70, 140, 140, 0*16, 180*16);
	painter.setBrush(QColor(0xFF, 0xCC, 0x00));
    painter.drawPie(-70, -70, 140, 140, (180-status_arc)*16, status_arc*16);
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

void VehicleMonitorWidget::drawDualUnitsText( QPainter& painter, const QColor& color, const QString& title, const QString& unit1, const QString& unit2 )
{
	constexpr QRect rect_title = QRect(  2, 0, 48, 23);
	constexpr QRect rect_unit1 = QRect( 52, 0, 72, 23);
	constexpr QRect rect_unit2 = QRect(126, 0, 72, 23);

	painter.setWindow(0, 0, 200, 25);

	painter.setPen( Qt::NoPen );
	painter.setBrush( QColor(0x40, 0x80, 0xC0) );
    painter.drawRect( rect_title );
    painter.drawRect( rect_unit1 );
    painter.drawRect( rect_unit2 );

	painter.setPen( color );
	painter.setBrush( Qt::NoBrush );
    painter.drawText( rect_title, Qt::AlignHCenter | Qt::AlignVCenter, title );
    painter.drawText( rect_unit1, Qt::AlignHCenter | Qt::AlignVCenter, unit1 );
    painter.drawText( rect_unit2, Qt::AlignHCenter | Qt::AlignVCenter, unit2 );
}


void VehicleMonitorWidget::drawAngleGraph( QPainter& painter )
{
	int status_arc = angle_status_val;
	QColor color1 = QColor(0x40, 0x80, 0xC0);
	QColor color2 = QColor(0xFF, 0xCC, 0x00);

	if(status_arc > +360)
	{
		status_arc -= 360;
		color1 = QColor(0xFF, 0xCC, 0x00);
		color2 = QColor(0xFF, 0x00, 0x00);
	}
	if(status_arc < -360)
	{
		status_arc += 360;
		color1 = QColor(0xFF, 0xCC, 0x00);
		color2 = QColor(0xFF, 0x00, 0x00);
	}

    painter.setWindow(-120, -120, 240, 240);

	painter.setPen(Qt::NoPen);
	painter.setBrush(color1);
    painter.drawPie(-70, -70, 140, 140, 0*16, 360*16);
	painter.setBrush(color2);
    painter.drawPie(-70, -70, 140, 140, 90*16, status_arc*16);
	painter.setBrush(QColor(0x00, 0x00, 0x00));
    painter.drawPie(-50, -50, 100, 100, 0*16, 360*16);
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


    /*

    QPointF position(width_*x,height_*y);
    if(last_status_data_.get().gearshift == gear_status_.get_drive_value())
        painter->drawText(position,QString("SHIFT:D"));
    else if(last_status_data_.get().gearshift == gear_status_.get_rear_value())
        painter->drawText(position,QString("SHIFT:R"));
    else if(last_status_data_.get().gearshift == gear_status_.get_brake_value())
        painter->drawText(position,QString("SHIFT:B"));
    else if(last_status_data_.get().gearshift == gear_status_.get_neutral_value())
        painter->drawText(position,QString("SHIFT:N"));
    else if(last_status_data_.get().gearshift == gear_status_.get_parking_value())
        painter->drawText(position,QString("SHIFT:P"));
    else
        painter->drawText(position,QString("---"));
    return;
    */
}

}
