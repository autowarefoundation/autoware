#include "velocity_text.h"

#include <QHBoxLayout>

namespace autoware_rviz_plugins {
namespace vehicle_monitor {

VelocityText::VelocityText( QWidget* parent ) : QWidget( parent )
{
	// These are free by VelocityText, do not delete.
	label_str = new QLabel( "cmd" );
	label_kph = new QLabel( "status" );
	label_mps = new QLabel( "XX km/h" );

	// Set layout, label parent is set here..
	QHBoxLayout* layout = new QHBoxLayout;
	layout->addWidget( label_str );
	layout->addWidget( label_kph );
	layout->addWidget( label_mps );
	setLayout( layout );

	// Set style
	label_str->setStyleSheet("background:#4080C0; color:#FFCC00F; padding: 20px 5px 10px 15px;");
	label_kph->setStyleSheet("background:#4080C0; color:#FFFFFF;");
	label_mps->setStyleSheet("background:#4080C0; color:#FFFFFF;");
}

void VelocityText::setLabel( QString title )
{
	label_str->setText( title );
}

void VelocityText::setVelocityKph( double kph )
{
	label_kph->setText( QString::fromStdString(std::to_string(kph      )) );
	label_mps->setText( QString::fromStdString(std::to_string(kph / 3.6)) );
}

void VelocityText::setVelocityMps( double mps )
{
	label_kph->setText( QString::fromStdString(std::to_string(mps * 3.6)) );
	label_mps->setText( QString::fromStdString(std::to_string(mps      )) );
}

}}

