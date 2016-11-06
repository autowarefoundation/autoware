#ifndef CUSTOMQLABEL_H
#define CUSTOMQLABEL_H

#include <QGraphicsView>
#include <QWheelEvent>
#include <QPixmap>
#include <QImage>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QGraphicsRectItem>


class CustomGraphicsView : public QGraphicsView
{
  Q_OBJECT

public:
  CustomGraphicsView(QWidget *parent = 0);
  ~CustomGraphicsView();

  // The function to set pixel map
  void SetPixmap(const QImage& image);

  // The function to set text
  void SetText(const QString& text);

  // The function to return selected area
  bool GetSelectedArea(QPoint* left_upper, QPoint* right_bottom);

  // The function to reset selected area
  void ResetSelectedArea();

private slots:
  void ScalingTime(qreal /* x */);
  void FinishAnimation();

protected:
  // Custom wheel event slot so that displayed image can be zoomed by mouse operation
  // Ref: https://wiki.qt.io/SmoothZoomInQGraphicsView
  virtual void wheelEvent(QWheelEvent *event);

  // Custom mouse event slot in order to get specified position
  virtual void mousePressEvent(QMouseEvent* mouse_event);
  virtual void mouseMoveEvent(QMouseEvent* mouse_event);
  virtual void mouseReleaseEvent(QMouseEvent* mouse_event);

  // How many times scaling is done
  int scheduled_scalings_;

private:
  // The utility function to reset display
  void ResetDisplay();

  // The utility function to convert point coordinate into valid image coordinate
  QPoint ConvertPointOnImage(QPoint point);

  // Stuffs to show image on display
  QGraphicsScene scene_;
  QGraphicsRectItem* rectangle_item_;
  QImage original_image_;

  // Stuffs to specify area on an image
  const QPoint k_initial_position_;
  QPoint start_position_;
  QPoint end_position_;
  bool dragging_;
};

#endif // CUSTOMQLABEL_H
