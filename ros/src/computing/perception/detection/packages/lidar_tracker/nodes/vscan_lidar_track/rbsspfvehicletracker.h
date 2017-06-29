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
    } tracker_instance_state_;
    TrackerDataContainer tracker_data_container_;
    TrackerResultContainer tracker_result_container_;
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
    } tracker_state_;
    int detection_num;
    QVector<VehicleState> detections_;
    QVector<bool> detections_matched_;
    int tracker_count_;
    QMap<int, VehicleState> detections_unmatched_;
protected:
    int tracker_id_=0;
    LaserScan current_laserscan_;
    QMap<int, TrackerResultContainer> tracker_result_container_map_;
    QMap<int, QThread *> trackers_thread_container_map_;
    QMap<int, int> tracker_lifespan_map_;
protected:
    QList<LaserScan> laserscan_processing_queue_;
    QList< QVector<VehicleState> > detections_processing_queue_;
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
