#ifndef RBSSPFVEHICLETRACKER_H
#define RBSSPFVEHICLETRACKER_H

#include"rbsspfvehicletracker.cuh"

#include<QObject>
#include<QThread>
#include<QMap>
#include<QVector>
#include<QList>

class RBSSPFVehicleTrackerInstance : public QObject
{
    Q_OBJECT
public:
    RBSSPFVehicleTrackerInstance(int vehicleID, QThread * thread);
    ~RBSSPFVehicleTrackerInstance();
protected:
    int id;
    enum TrackerState
    {
        InitGeometry,
        InitMotion,
        UpdateTracker
    } trackerstate;
    TrackerDataContainer trackerdatacontainer;
    TrackerResultContainer trackerresultcontainer;
signals:
    void signalCheckInitStateFinish();
    void signalUpdateTrackerFinish(int vehicleID, TrackerResultContainer * trackerResult);
public slots:
    void slotCheckInitState(int initNum, VehicleState * initState, bool * initFlag);
    void slotUpdateTracker(QMap<int, VehicleState> * initStateMap);
};

class RBSSPFVehicleTracker : public QObject
{
    Q_OBJECT
public:
    RBSSPFVehicleTracker();
    ~RBSSPFVehicleTracker();
protected:
    enum TrackerState
    {
        NoLaserData,
        OneLaserData,
        ReadyForTracking,
        Processing
    } trackerstate;
    int initnum;
    QVector<VehicleState> initstate;
    QVector<bool> initflag;
    int trackercount;
    QMap<int, VehicleState> initstatemap;
protected:
    int idcount=0;
    LaserScan curscan;
    QMap<int, TrackerResultContainer> trackerresultmap;
    QMap<int, QThread *> trackerthreadmap;
    QMap<int, int> discontinuemap;
protected:
    QList<LaserScan> scanbuffer;
    QList< QVector<VehicleState> > initstatebuffer;
public:
    void addTrackerData(LaserScan & scan, QVector<VehicleState> & initState);
signals:
    void signalCheckInitState(int initNum, VehicleState * initState, bool * initFlag);
    void signalUpdateTracker(QMap<int, VehicleState> * initStateMap);
    void signalUpdateTrackerFinish(LaserScan scan, QMap<int, TrackerResultContainer> trackerresultmap);
public slots:
    void slotCheckInitStateFinish();
    void slotUpdateTrackerFinish(int vehicleID, TrackerResultContainer * trackerResult);
};

#endif // RBSSPFVEHICLETRACKER_H
