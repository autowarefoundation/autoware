#include "custom_graphics_view.h"

#include <iostream>
#include <QTimeLine>
#include <QGraphicsScene>
#include <QPainter>


CustomGraphicsView::CustomGraphicsView(QWidget *parent) :
  QGraphicsView(parent) {
  setStyleSheet("background-color:black");
}


CustomGraphicsView::~CustomGraphicsView() {
}


// Custom wheel event slot for smooth zoom
// Ref: https://wiki.qt.io/SmoothZoomInQGraphicsView
void CustomGraphicsView::wheelEvent(QWheelEvent *event) {
  // Set zooming center to mouse pointer
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

  int degree = event->delta() / 8;
  int step = degree / 15;
  scheduled_scalings_ += step;

  // if user moved the wheel in another direction, we reset previously scheduled scalings
  if (scheduled_scalings_ * step < 0) {
    scheduled_scalings_ = step;
  }

  QTimeLine *animation = new QTimeLine(350, this);
  animation->setUpdateInterval(20);

  connect(animation,
          SIGNAL(valueChanged(qreal)),
          SLOT(ScalingTime(qreal)));
  connect(animation,
          SIGNAL(finished()),
          SLOT(FinishAnimation()));
  animation->start();
}


void CustomGraphicsView::ScalingTime(qreal) {
  qreal factor = 1.0 + qreal(scheduled_scalings_) / 300;
  scale(factor, factor);
}


void CustomGraphicsView::FinishAnimation() {
  if (scheduled_scalings_ > 0) {
    scheduled_scalings_--;
  } else {
    sender()->~QObject();
  }
}


void CustomGraphicsView::ResetDisplay() {
  resetCachedContent();
  scene_.clear();
}


void CustomGraphicsView::SetPixmap(const QImage &image) {
  // Reset the current display
  ResetDisplay();

  // Set Specified image
  original_image_ = image;
  scene_.addPixmap(QPixmap::fromImage(image));
  setScene(&scene_);
  setAlignment(Qt::AlignCenter);
  show();
}


void CustomGraphicsView::SetText(const QString &text) {
  // Reset the current display
  ResetDisplay();

  // Set Specified text
  scene_.addText(text);
  setScene(&scene_);
  setAlignment(Qt::AlignCenter);
  show();
}
