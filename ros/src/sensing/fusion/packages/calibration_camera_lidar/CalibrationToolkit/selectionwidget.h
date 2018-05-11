#ifndef SELECTIONWIDGET_H
#define SELECTIONWIDGET_H

#include<qlabel.h>
#include<QVector>
#include<qrgb.h>
#include<QMouseEvent>
#include<QLabel>
#include<QImage>
#include<QPainter>
#include<QDebug>

#include<opencv2/opencv.hpp>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include<Eigen/Dense>
#pragma GCC diagnostic pop
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/LaserScan.h>

#include<glviewer/glviewer.h>

class PlaneExtractor : public GLViewer
{
    Q_OBJECT
public:
    PlaneExtractor(sensor_msgs::PointCloud2ConstPtr velodynePoints, int id, double neighborRadius=0.2, double distanceThreshold=0.05, QWidget * parent=0);
    PlaneExtractor(pcl::PointCloud<pcl::PointXYZI>::Ptr velodynePoints, int id, double neighborRadius=0.2, double distanceThreshold=0.05, QWidget * parent=0);
protected:
    int pointsid;
    double neighborradius;
    double distance;
    sensor_msgs::PointCloud2ConstPtr pointsptr;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    GLuint pointsdisplaylist;
    GLuint selecteddisplaylist;
    GLuint mousedisplaylist;
    bool extracted;
protected slots:
    void mousePositionSlot(QMouseEvent * event, CAMERAPARAMETERS * parameters);
signals:
    void extractionResultSignal(pcl::PointCloud<pcl::PointXYZI>::Ptr extraction, cv::Mat normal, int id);
protected:
    void drawMouse(Eigen::Vector3d point, Eigen::Vector3d norm, Eigen::Vector3d direction,Eigen::Vector3d nearestpoint);
    void extractPlane(Eigen::Vector3d seed, Eigen::Matrix3d eigenvectors);
};

class PointsExtractor : public QLabel
{
    Q_OBJECT
public:
    PointsExtractor(int imageSize, double maxRange, double gridSize);
    PointsExtractor(sensor_msgs::LaserScanConstPtr lidarPoints, int id, int imageSize, double maxRange, double gridSize);
    PointsExtractor(QVector<QPointF> lidarPoints, int id, int imageSize, double maxRange, double gridSize);
protected:
    int pointsid;
    bool justshow;
    bool startextraction;
    sensor_msgs::LaserScanConstPtr pointsptr;
    QVector<QPointF> points;
    bool extracted;
    QPoint startcorner;
    QPoint endcorner;
    QImage image;
public:
    int imagesize;
    double maxrange;
    double gridsize;
public:
    void updateLaserScan(sensor_msgs::LaserScanConstPtr lidarPoints);
    void update();
protected:
    void mousePressEvent(QMouseEvent * ev);
    void mouseMoveEvent(QMouseEvent * ev);
    void mouseReleaseEvent(QMouseEvent *ev);
    void wheelEvent(QWheelEvent * ev);
signals:
    void extractionResultSignal(QVector<QPointF> extraction, int id);
protected:
    QPoint convert2ImagePoint(QPointF point);
    QPointF convert2RealPoint(QPoint point);
    void drawPoints();    
    void drawRectangle();
    QVector<QPointF> extractPoints();
};

#endif // SELECTIONWIDGET_H
