#ifndef CUSTOMQLABEL_H
#define CUSTOMQLABEL_H

#include <QGraphicsView>
#include <QWheelEvent>
#include <QPixmap>
#include <QImage>
#include <QGraphicsScene>


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

private slots:
  void ScalingTime(qreal /* x */);
  void FinishAnimation();

protected:
  // Custom wheel event slot so that displayed image can be zoomed by mouse operation
  // Ref: https://wiki.qt.io/SmoothZoomInQGraphicsView
  virtual void wheelEvent(QWheelEvent *event);

  int scheduled_scalings_;

private:
  // The utility function to reset display
  void ResetDisplay();

  // Stuffs to show image on display
  QGraphicsScene scene_;
  QImage original_image_;
};

#endif // CUSTOMQLABEL_H
