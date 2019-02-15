/*
 * ClickableLabel.h
 *
 *  Created on: Dec 25, 2018
 *      Author: sujiwo
 */

#ifndef _CLICKABLELABEL_H_
#define _CLICKABLELABEL_H_


#include <QLabel>
#include <QWidget>
#include <Qt>


class ClickableLabel : public QLabel
{
Q_OBJECT

public:
	explicit ClickableLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
	explicit ClickableLabel(const QString &text, QWidget *parent = nullptr, Qt::WindowFlags f = Qt::WindowFlags());
	~ClickableLabel();

signals:
	void clicked();

protected:
	void mousePressEvent(QMouseEvent* event);

};

#endif /* _CLICKABLELABEL_H_ */
