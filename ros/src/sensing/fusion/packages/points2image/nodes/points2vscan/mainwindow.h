#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <string>
#include <QMainWindow>
#include <QPainter>
#include <QImage>
#include <QTime>

#include <rosinterface/rosinterface.h>
#include <fastvirtualscan/fastvirtualscan.h>
#include <sensor_msgs/LaserScan.h>

//#define DEBUG_GUI

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = 0);
  ~MainWindow();

private:
  Ui::MainWindow* ui;

protected:
  ROSSub<sensor_msgs::PointCloud2ConstPtr>* velodyne;
  ROSPub<sensor_msgs::PointCloud2>* vsros;
  ROSPub<sensor_msgs::LaserScan>* scanros;
  FastVirtualScan virtualscan;
  QVector<double> beams;
  QVector<double> heights;
  QImage image;
public slots:
  void generateVirtualScanSlot();
  void showMatrixSlot(int beamid);
  void recalculateSlot();

protected:
  QPointF convert2RealPoint(QPoint point);
  QPoint convert2ImagePoint(QPointF point);
  void drawGrid();
  void drawPoints();
  void drawBeam(int beamid);
};

#endif  // MAINWINDOW_H
