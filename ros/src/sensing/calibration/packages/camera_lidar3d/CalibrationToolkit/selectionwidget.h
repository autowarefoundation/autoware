#ifndef SELECTIONWIDGET_H
#define SELECTIONWIDGET_H

#include<qlabel.h>
#include<QVector>
#include<qrgb.h>
#include<QMouseEvent>

#include<opencv2/opencv.hpp>

#include<Eigen/Dense>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>

#include<glviewer.h>

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


#endif // SELECTIONWIDGET_H
