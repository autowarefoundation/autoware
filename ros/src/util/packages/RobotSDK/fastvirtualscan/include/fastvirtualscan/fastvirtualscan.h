#ifndef FASTVIRTUALSCAN_H
#define FASTVIRTUALSCAN_H

#include<QVector>
#include<QtAlgorithms>
#include<QtGlobal>
#include<sensor_msgs/PointCloud2.h>

struct SimpleVirtualScan
{
    int rotid;
    double rotlength;
    double rotheight;
    double length;
    double height;
};

class FastVirtualScan
{
public:
    sensor_msgs::PointCloud2ConstPtr velodyne;
public:
    int beamnum;
    double step;
    double minfloor;
    double maxceiling;
    double rotation;
    double minrange;
    QVector<QVector<SimpleVirtualScan> > svs;
    QVector<QVector<SimpleVirtualScan> > svsback;
    QVector<double> minheights;
    QVector<double> maxheights;
public:
    FastVirtualScan();
    virtual ~FastVirtualScan();
public:
    void calculateVirtualScans(int beamNum, double heightStep, double minFloor, double maxCeiling, double obstacleMinHeight=1, double maxBackDistance=1, double beamRotation=0, double minRange=0);
    void getVirtualScan(double thetaminheight, double thetamaxheight, double maxFloor, double minCeiling, double passHeight, QVector<double> & virtualScan);
};

#endif // FASTVIRTUALSCAN_H
