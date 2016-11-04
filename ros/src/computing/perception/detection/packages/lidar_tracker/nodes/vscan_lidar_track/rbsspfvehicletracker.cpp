#include "rbsspfvehicletracker.h"
#include "moc_rbsspfvehicletracker.cpp"


RBSSPFVehicleTrackerInstance::RBSSPFVehicleTrackerInstance(int vehicleID, QThread *thread)
    : QObject(NULL)
{
    id=vehicleID;
    trackerstate=InitGeometry;
    cuda_OpenTracker(trackerdatacontainer);
    this->moveToThread(thread);
    connect(thread,SIGNAL(finished()),this,SLOT(deleteLater()));
    thread->start();
}

RBSSPFVehicleTrackerInstance::~RBSSPFVehicleTrackerInstance()
{
    cuda_CloseTracker(trackerdatacontainer);
}

void RBSSPFVehicleTrackerInstance::slotCheckInitState(int initNum, VehicleState * initState, bool * initFlag)
{
    for(int i=0;i<initNum;i++)
    {
        double tmpdx=initState[i].x-trackerresultcontainer.estimate.x;
        double tmpdy=initState[i].y-trackerresultcontainer.estimate.y;
        double dx=tmpdx*cos(-trackerresultcontainer.estimate.theta)-tmpdy*sin(-trackerresultcontainer.estimate.theta);
        double dy=tmpdx*sin(-trackerresultcontainer.estimate.theta)+tmpdy*cos(-trackerresultcontainer.estimate.theta);
        if((dx<=trackerresultcontainer.estimate.wl+MARGIN2)
                &&(dx>=-trackerresultcontainer.estimate.wr-MARGIN2)
                &&(dy<=trackerresultcontainer.estimate.lf+MARGIN2)
                &&(dy>=-trackerresultcontainer.estimate.lb-MARGIN2))
        {
            initFlag[i]=0;
        }
    }
    emit signalCheckInitStateFinish();
    return;
}

void RBSSPFVehicleTrackerInstance::slotUpdateTracker(QMap<int, VehicleState> * initStateMap)
{
    switch(trackerstate)
    {
    case InitGeometry:
        trackerstate=InitMotion;
        trackerresultcontainer.estimate=(*initStateMap)[id];
        cuda_InitGeometry(trackerdatacontainer,trackerresultcontainer);
        break;
    case InitMotion:
        trackerstate=UpdateTracker;
        cuda_InitMotion(trackerdatacontainer,trackerresultcontainer);
        break;
    case UpdateTracker:
//        cuda_InitMotion(trackerdatacontainer,trackerresultcontainer);
        cuda_UpdateTracker(trackerdatacontainer,trackerresultcontainer);
        break;
    default:
        return;
    }
    emit signalUpdateTrackerFinish(id,&trackerresultcontainer);
    return;
}

RBSSPFVehicleTracker::RBSSPFVehicleTracker()
    : QObject(NULL)
{
    //qRegisterMetaType<TrackerResultContainer>("TrackerResultContainer");
    trackerstate=NoLaserData;
    cuda_InitLaserScan();
}

RBSSPFVehicleTracker::~RBSSPFVehicleTracker()
{
    QList<QThread *> threadlist=trackerthreadmap.values();
    int i,n=threadlist.size();
    for(i=0;i<n;i++)
    {
        threadlist[i]->exit();
        threadlist[i]->wait();
    }
    cuda_FreeLaserScan();
}

void RBSSPFVehicleTracker::addTrackerData(LaserScan & scan, QVector<VehicleState> & initState)
{
    switch(trackerstate)
    {
    case NoLaserData:
        trackerstate=OneLaserData;
        cuda_SetLaserScan(scan);
        break;
    case OneLaserData:
        trackerstate=ReadyForTracking;
        cuda_SetLaserScan(scan);
        break;
    case ReadyForTracking:
        trackerstate=Processing;
        cuda_SetLaserScan(scan);
        curscan=scan;
        initnum=initState.size();
        if(initnum>0)
        {
            initstate=initState;
            initflag.fill(1,initnum);
            trackercount=trackerthreadmap.size();
            if(trackercount>0)
            {
                emit signalCheckInitState(initnum,initstate.data(),initflag.data());
            }
            else
            {
                slotCheckInitStateFinish();
            }
        }
        else if(trackerthreadmap.size()>0)
        {            
            initstatemap.clear();
            trackercount=trackerthreadmap.size();
            emit signalUpdateTracker(&initstatemap);
        }
        else
        {
            trackerstate=ReadyForTracking;
            emit signalUpdateTrackerFinish(curscan,trackerresultmap);
        }
        break;
    case Processing:
        scanbuffer.push_back(scan);
        initstatebuffer.push_back(initState);
        break;
    }
}

void RBSSPFVehicleTracker::slotCheckInitStateFinish()
{
    trackercount--;

    if(trackercount<=0)
    {
        initstatemap.clear();
        int i,n=initflag.size();
        for(i=0;i<n;i++)
        {
            if(initflag[i])
            {
                initstatemap.insert(idcount,initstate[i]);

                QThread * thread=new QThread;
                RBSSPFVehicleTrackerInstance * trackerinstance=new RBSSPFVehicleTrackerInstance(idcount,thread);
                trackerresultmap.insert(idcount,TrackerResultContainer());
                trackerthreadmap.insert(idcount,thread);
                discontinuemap.insert(idcount,0);

                connect(this,SIGNAL(signalCheckInitState(int,VehicleState*,bool*)),trackerinstance,SLOT(slotCheckInitState(int,VehicleState*,bool*)));
                connect(this,SIGNAL(signalUpdateTracker(QMap<int,VehicleState>*)),trackerinstance,SLOT(slotUpdateTracker(QMap<int,VehicleState>*)));

                connect(trackerinstance,SIGNAL(signalCheckInitStateFinish()),this,SLOT(slotCheckInitStateFinish()));
                connect(trackerinstance,SIGNAL(signalUpdateTrackerFinish(int,TrackerResultContainer *)),this,SLOT(slotUpdateTrackerFinish(int,TrackerResultContainer *)));

                idcount++;
            }
        }

        trackercount=trackerthreadmap.size();
        emit signalUpdateTracker(&initstatemap);
    }
}

void RBSSPFVehicleTracker::slotUpdateTrackerFinish(int vehicleID, TrackerResultContainer * trackerResult)
{
    trackercount--;

    trackerresultmap[vehicleID]=*trackerResult;
    if(trackerResult->estimate.dx>1||trackerResult->estimate.dy>1||trackerResult->estimate.dtheta>DEG2RAD(20)||trackerResult->estimate.count<1)
    {
        discontinuemap[vehicleID]++;
    }
    else
    {
        discontinuemap[vehicleID]/=2;
    }
    if(discontinuemap[vehicleID]>10)
    {
        trackerthreadmap[vehicleID]->exit();
        trackerresultmap.remove(vehicleID);
        trackerthreadmap.remove(vehicleID);
        discontinuemap.remove(vehicleID);
    }

    if(trackercount<=0)
    {
        emit signalUpdateTrackerFinish(curscan,trackerresultmap);
        if(scanbuffer.size()>0)
        {
            LaserScan scan=scanbuffer.front();
            curscan=scan;
            scanbuffer.pop_front();
            cuda_SetLaserScan(scan);

            initstate=initstatebuffer.front();
            initstatebuffer.pop_front();

            initnum=initstate.size();
            if(initnum>0)
            {
                initflag.fill(1,initnum);
                trackercount=trackerthreadmap.size();
                emit signalCheckInitState(initnum,initstate.data(),initflag.data());
            }
            else
            {
                initstatemap.clear();
                trackercount=trackerthreadmap.size();
                emit signalUpdateTracker(&initstatemap);
            }
        }
        else
        {
            trackerstate=ReadyForTracking;
        }
    }
}
