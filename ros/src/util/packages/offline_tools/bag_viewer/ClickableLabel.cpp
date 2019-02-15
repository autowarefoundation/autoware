/*
 * ClickableLabel.cpp
 *
 *  Created on: Dec 25, 2018
 *      Author: sujiwo
 */

#include <ClickableLabel.h>

ClickableLabel::ClickableLabel(QWidget* parent, Qt::WindowFlags f) :
QLabel(parent, f)
{}


ClickableLabel::ClickableLabel(const QString &text, QWidget *parent, Qt::WindowFlags f) :
	QLabel(text, parent, f)
{}


ClickableLabel::~ClickableLabel() {}


void ClickableLabel::mousePressEvent(QMouseEvent* event)
{
	emit clicked();
}
