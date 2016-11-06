#include "custom_graphics_view.h"

#include <iostream>
#include <QTimeLine>


CustomGraphicsView::CustomGraphicsView(QWidget *parent) :
  QGraphicsView(parent),
  scheduled_scalings_(0),
  rectangle_item_(nullptr),
  k_initial_position_(QPoint(-1, -1)),
  start_position_(k_initial_position_),
  end_position_(k_initial_position_),
  dragging_(false) {
  // Set background color as Black
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
  qreal factor = 1.0 + qreal(scheduled_scalings_) / 300.0;
  scale(factor, factor);
}


void CustomGraphicsView::FinishAnimation() {
  if (scheduled_scalings_ > 0) {
    scheduled_scalings_--;
  } else {
    scheduled_scalings_++;
    sender()->~QObject();
  }
}


void CustomGraphicsView::ResetDisplay() {
  ResetSelectedArea();

  // Reset all contents in the view
  resetCachedContent();
  scene_.clear();
}


void CustomGraphicsView::SetPixmap(const QImage &image) {
  // Reset the current display
  ResetDisplay();

  // Set Specified image
  original_image_ = image.copy();
  scene_.addPixmap(QPixmap::fromImage(original_image_));
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


void CustomGraphicsView::mousePressEvent(QMouseEvent *mouse_event) {
  if (mouse_event->button() == Qt::LeftButton) {
    // Get start position of specified region on the image coordinate
    start_position_ = ConvertPointOnImage(mouse_event->pos());
    dragging_ = true;
  }
}


void CustomGraphicsView::mouseMoveEvent(QMouseEvent *mouse_event) {
  // Check whether dragging
  if (dragging_) {
    // Delete current rectangle
    if (rectangle_item_ != nullptr) {
      scene_.removeItem(rectangle_item_);
      delete rectangle_item_;
      rectangle_item_ = nullptr;
    }

    // Draw new rectangle
    QRectF current_selection(start_position_, mapToScene(mouse_event->pos()));
    rectangle_item_ = new QGraphicsRectItem(current_selection);
    rectangle_item_->setPen(QPen(Qt::NoPen)); // Show no border line
    rectangle_item_->setBrush(QColor(0, 0, 255, 125)); // Show in transparent blue
    scene_.addItem(rectangle_item_);
  }
}


void CustomGraphicsView::mouseReleaseEvent(QMouseEvent *mouse_event) {
  if (mouse_event->button() == Qt::LeftButton) {
    // Get end position of specified region on the image coordinate
    end_position_ = ConvertPointOnImage(mouse_event->pos());
    dragging_ = false;
  }
}


QPoint CustomGraphicsView::ConvertPointOnImage(QPoint point) {
  // Convert global point coordinate value into scene (image) coordinate value
  QPointF scene_point = mapToScene(point);

  QPoint converted_point(scene_point.x(), scene_point.y());
  QPoint limit(original_image_.size().width() - 1, original_image_.size().height() - 1);

  // Check range of x
  if (converted_point.x() < 0) {
    converted_point.setX(0);
  } else if (limit.x() < converted_point.x()) {
    converted_point.setX(limit.x());
  }

  // Check range of y
  if (converted_point.y() < 0) {
    converted_point.setY(0);
  } else if (limit.y() < converted_point.y()) {
    converted_point.setY(limit.y());
  }

  return converted_point;
}


bool CustomGraphicsView::GetSelectedArea(QPoint *left_upper, QPoint *right_bottom) {
  // If area specification has not been done, return false
  if (start_position_ == k_initial_position_ ||
      end_position_ == k_initial_position_) {
    return false;
  }

  // Assign surely "Left-Upper" and "Right-Bottom" coordinate value of selected area
  if (start_position_.x() <= end_position_.x()) {
    left_upper->setX(start_position_.x());
    right_bottom->setX(end_position_.x());
  } else {
    left_upper->setX(end_position_.x());
    right_bottom->setX(start_position_.x());
  }

  if (start_position_.y() <= end_position_.y()) {
    left_upper->setY(start_position_.y());
    right_bottom->setY(end_position_.y());
  } else {
    left_upper->setY(end_position_.y());
    right_bottom->setY(start_position_.y());
  }

  return true;
}


void CustomGraphicsView::ResetSelectedArea() {
  // Remove displayed rectangle
  if (rectangle_item_ != nullptr) {
    scene_.removeItem(rectangle_item_);
    delete rectangle_item_;
    rectangle_item_ = nullptr;
  }

  // Reset selected coordinate
  start_position_ = k_initial_position_;
  end_position_ = k_initial_position_;
}
