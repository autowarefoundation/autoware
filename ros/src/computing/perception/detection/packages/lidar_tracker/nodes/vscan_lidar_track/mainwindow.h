/*#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include<QMainWindow>
#include<rosinterface.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<visualization_msgs/MarkerArray.h>

#include<QGraphicsView>
#include<QGraphicsScene>
#include<QGraphicsLineItem>
#include<QGraphicsEllipseItem>
#include<QGraphicsPathItem>
#include<QPainterPath>
#include<QLayout>
#include<QMouseEvent>
#include<QKeyEvent>
#include<QWheelEvent>
#include<QPointF>
#include<QListWidget>
#include<QMap>
#include<QTime>
#include<QList>
#include<QPair>
#include<QSplitter>
#include<Eigen/Dense>

#include"rbsspf_tracker.cuh"

// using namespace RobotSDK;

namespace Ui {
class MainWindow;
}

class InitTrackerView : public QGraphicsView
{
    Q_OBJECT
public:
    InitTrackerView(QWidget * parent=NULL);
public:
    void showLaserScan(LaserScan & scan);
    void getInitState(QVector<Tracker> & initState);
protected:
    void mousePressEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
protected:
    bool pressflag=0;
    bool ctrlflag=0;
    QPointF point1,point2;
    QGraphicsEllipseItem * point1item;
    QGraphicsLineItem * lineitem;
public:
    double sx=1,sy=1;
    QGraphicsScene * scene=NULL;
    LaserScan scan;
    QVector<QGraphicsLineItem *> state;
    int idcount;
};

class UpdateTrackerView : public QGraphicsView
{
    Q_OBJECT
public:
    UpdateTrackerView(QWidget * parent=NULL);
public slots:
    void slotUpdateTrackerFinish(LaserScan scan, QVector<Tracker> trackers);
protected:
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
protected:
    bool ctrlflag=0;
    QMap<int, QGraphicsPathItem *> pathmap;
public:
    double sx=1,sy=1;
    QGraphicsScene * scene=NULL;
};

struct EGOMOTION
{
    double x,y,theta;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
//    void getInitStateFromTopic(QVector<Tracker> & initState);
private:
    Ui::MainWindow *ui;
public:
    ROSSub<sensor_msgs::LaserScanConstPtr> * scansub;
//    ROSSub<cv_tracker::obj_label::ConstPtr> * detectionsub;
    ROSTFSub * tfsub;
//    ROSTFSub * tfMap2Lidarsub;
    QList< QPair<double,LaserScan> > scanlist;
//    QList< QPair<double,VehicleState> > detectionlist;
    QList< QPair<double,EGOMOTION> > tflist;
public:
    QVector<Tracker> trackers;
    LaserScan curscan;
public:
    InitTrackerView * initview;
    UpdateTrackerView * updateview;
    bool initflag=0;
public slots:
    void slotReceive();
    //void slotReceiveDetection();
    void slotReceiveTF();
//    void slotReceiveTFMap2Lidar();
    void slotShowScan();
//private:
//    tf::StampedTransform transformMap2Lidar;
};

#endif // MAINWINDOW_H
*/


#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include<QMainWindow>
#include<rosinterface.h>
#include<sensor_msgs/LaserScan.h>
#include<sensor_msgs/PointCloud2.h>
#include<visualization_msgs/MarkerArray.h>
#include "cv_tracker_msgs/obj_label.h"

#include<QGraphicsView>
#include<QGraphicsScene>
#include<QGraphicsLineItem>
#include<QGraphicsEllipseItem>
#include<QGraphicsPathItem>
#include<QPainterPath>
#include<QLayout>
#include<QMouseEvent>
#include<QKeyEvent>
#include<QWheelEvent>
#include<QPointF>
#include<QListWidget>
#include<QMap>
#include<QTime>
#include<QList>
#include<QPair>
#include<QSplitter>
#include<Eigen/Dense>

#include<rbsspfvehicletracker.h>


#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

namespace Ui {
class MainWindow;
}

// using namespace RobotSDK;

class InitTrackerView : public QGraphicsView
{
    Q_OBJECT
public:
    InitTrackerView(QWidget * parent=NULL);
public:
    void showLaserScan(LaserScan & scan);
    void getInitState(QVector<VehicleState> & initState);
protected:
    void mousePressEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
protected:
    bool clicked_flag_=0;
    bool ctrl_key_pressed_=0;
    QPointF qt_click_point_1,qt_click_point_2;
    QGraphicsEllipseItem * qt_clicked_point_1;
    QGraphicsLineItem * qt_line_pose_;
public:
    double sx=1,sy=1;
    QGraphicsScene * qt_init_view_graphics_scene_=NULL;
    LaserScan scan;

    QVector<QGraphicsLineItem *> qt_line_states_;
};

class UpdateTrackerView : public QGraphicsView
{
    Q_OBJECT
public:
    UpdateTrackerView(QWidget * parent=NULL);
public slots:
    void slotUpdateTrackerFinish(LaserScan scan, QMap<int, TrackerResultContainer> trackerresultmap);
protected:
    void wheelEvent(QWheelEvent *event);
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);
protected:
    bool ctrl_key_pressed_=0;
    QMap<int, QGraphicsPathItem *> pathmap;
public:
    double sx=1,sy=1;
    ROSPub<jsk_recognition_msgs::BoundingBoxArray> *boxes_publisher_;//added to enable publishing of tracking results
    std_msgs::Header boxes_header_;
    QGraphicsScene * qt_view_tracker_scene_=NULL;
};

struct EGOMOTION
{
    double x,y,theta;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void getInitStateFromTopic(QVector<VehicleState> & initState);
private:
    Ui::MainWindow *ui;
public:
    ROSSub<sensor_msgs::LaserScanConstPtr> * laserscan_subscriber_;
    ROSSub<cv_tracker_msgs::obj_label::ConstPtr> * detection_subscriber_;
    ROSSub<jsk_recognition_msgs::BoundingBoxArray::ConstPtr> * boxes_subscriber_;

    ROSPub<jsk_recognition_msgs::BoundingBoxArray> *boxes_publisher_;//added to enable publishing of tracking results
    std_msgs::Header boxes_header_; //used to publish tracking results with correct frame and timestamp
    bool using_local_coords_;//defines whether to use the global map TF or local coords for tracking

    ROSTFSub * tf_subscriber_;
    ROSTFSub * tf_map_to_lidar_subscriber_;
    QList< QPair<QTime,LaserScan> > scanlist;
    QList< QPair<QTime,VehicleState> > detectionlist;
    QList< QPair<QTime,EGOMOTION> > tflist;
public:
    RBSSPFVehicleTracker * vehicletracker;
    LaserScan curscan;
public:
    InitTrackerView * initview;
    UpdateTrackerView * updateview;
    bool initflag=0;
public slots:
    void slotReceive();
    void slotReceiveDetection();
    void slotReceiveBoxes();
    void slotReceiveTF();
    void slotReceiveTFMap2Lidar();
    void slotShowScan();
private:
    tf::StampedTransform transformMap2Lidar;
};

#endif // MAINWINDOW_H

