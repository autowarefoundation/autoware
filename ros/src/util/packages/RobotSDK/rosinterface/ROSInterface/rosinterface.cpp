/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rosinterface/rosinterface.h"

//using namespace RobotSDK;

bool ROSInterfaceBase::initflag=0;

ROSInterfaceBase::ROSInterfaceBase(QObject *parent)
    : QObject(parent)
{
    if(!initflag)
    {
        QStringList arguments=QApplication::instance()->arguments();
        int argc=1;
        QString NodeName;
        if(arguments.size()>1)
        {
            NodeName=arguments[1];
        }
        else
        {
            NodeName=QFileInfo(arguments[0]).baseName();
        }
        char *argv=NodeName.toUtf8().data();
        ros::init(argc,&argv,NodeName.toStdString());
        initflag=1;
    }

    nh=new ros::NodeHandle;
}

ROSInterfaceBase::~ROSInterfaceBase()
{
    if(nh!=NULL)
    {
        if(nh->ok())
        {
            nh->shutdown();
        }
        delete nh;
        nh=NULL;
    }
}

ROSSubBase::ROSSubBase(int QueryInterval, QObject *parent)
    : ROSInterfaceBase(parent)
{
    nh->setCallbackQueue(&queue);
    timer=new QTimer(this);
    timer->setInterval(QueryInterval);
    connect(timer,SIGNAL(timeout()),this,SLOT(receiveMessageSlot()));
    connect(this,SIGNAL(startReceiveSignal()),timer,SLOT(start()));
    connect(this,SIGNAL(stopReceiveSignal()),timer,SLOT(stop()));
    connect(this,SIGNAL(resetQueryIntervalSignal(int)),timer,SLOT(start(int)));
    receiveflag=0;
    emit startReceiveSignal();
}

ROSSubBase::~ROSSubBase()
{
    receiveflag=0;
    emit stopReceiveSignal();
    disconnect(timer,SIGNAL(timeout()),this,SLOT(receiveMessageSlot()));
    disconnect(this,SIGNAL(startReceiveSignal()),timer,SLOT(start()));
    disconnect(this,SIGNAL(stopReceiveSignal()),timer,SLOT(stop()));
    delete timer;
}

void ROSSubBase::startReceiveSlot()
{
    lock.lockForWrite();
    receiveflag=1;
    clearMessage();
    lock.unlock();
}

void ROSSubBase::stopReceiveSlot()
{
    lock.lockForWrite();
    receiveflag=0;
    lock.unlock();
}

void ROSSubBase::receiveMessageSlot()
{
    if(ros::ok()&&nh->ok())
    {
        receiveMessage(queue.callOne(ros::WallDuration(0)));
    }
}

void ROSSubBase::receiveMessage(ros::CallbackQueue::CallOneResult result)
{
    if(receiveflag)
    {
        switch(result)
        {
        case ros::CallbackQueue::Called:
            emit receiveMessageSignal();
            break;
        case ros::CallbackQueue::TryAgain:
        case ros::CallbackQueue::Disabled:
        case ros::CallbackQueue::Empty:
        default:
            break;
        }
    }
    return;
}

ROSTFPub::ROSTFPub(QString childFrameID, QString frameID, QObject *parent)
    : ROSInterfaceBase(parent)
{
    childframeid=childFrameID;
    frameid=frameID;
}

bool ROSTFPub::sendTF(tf::Transform &transform)
{
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),frameid.toStdString(),childframeid.toStdString()));
    return 1;
}

QString ROSTFPub::getChildFrameID()
{
    return childframeid;
}

void ROSTFPub::resetChildFrameID(QString childFrameID)
{
    childframeid=childFrameID;
}

QString ROSTFPub::getFrameID()
{
    return frameid;
}

void ROSTFPub::resetFrameID(QString frameID)
{
    frameid=frameID;
}

ROSTFSub::ROSTFSub(QString destinationFrame, QString originalFrame, int QueryInterval, QObject *parent)
    : ROSInterfaceBase(parent)
{
    destinationframe=destinationFrame;
    originalframe=originalFrame;
    timer=new QTimer(this);
    timer->setInterval(QueryInterval);
    connect(timer,SIGNAL(timeout()),this,SLOT(receiveTFSlot()));
    connect(this,SIGNAL(startReceiveSignal()),timer,SLOT(start()));
    connect(this,SIGNAL(stopReceiveSignal()),timer,SLOT(stop()));
    connect(this,SIGNAL(resetQueryIntervalSignal(int)),timer,SLOT(start(int)));
    receiveflag=0;
    lastflag=0;
    emit startReceiveSignal();
}

ROSTFSub::~ROSTFSub()
{
    receiveflag=0;
    lastflag=0;
    emit stopReceiveSignal();
    disconnect(timer,SIGNAL(timeout()),this,SLOT(receiveTFSlot()));
    disconnect(this,SIGNAL(startReceiveSignal()),timer,SLOT(start()));
    disconnect(this,SIGNAL(stopReceiveSignal()),timer,SLOT(stop()));
    delete timer;
}

void ROSTFSub::startReceiveSlot()
{
    lock.lockForWrite();
    receiveflag=1;
    lastflag=0;
    clearTFs();
    lock.unlock();
}

void ROSTFSub::stopReceiveSlot()
{
    lock.lockForWrite();
    receiveflag=0;
    lastflag=0;
    lock.unlock();
}

void ROSTFSub::receiveTFSlot()
{
    if(ros::ok()&&receiveflag)
    {
        lock.lockForWrite();
        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform(destinationframe.toStdString(),originalframe.toStdString(),ros::Time(0),transform);
        }
        catch(tf::TransformException & ex)
        {
            //qDebug()<<QString(ex.what());
            lock.unlock();
            return;
        }
        if(!lastflag||lasttf.stamp_.sec!=transform.stamp_.sec||lasttf.stamp_.nsec!=transform.stamp_.nsec)
        {
            tfs.push_back(transform);
            lasttf=transform;
            lastflag=1;
            lock.unlock();
            emit receiveTFSignal();
        }
        else
        {
            lock.unlock();
        }
    }
}

void ROSTFSub::clearTFs()
{
    tfs.clear();
}

bool ROSTFSub::getTF(tf::StampedTransform & transform)
{
    lock.lockForWrite();
    bool flag=0;
    if(receiveflag&&!tfs.isEmpty())
    {
        transform=tfs.front();
        tfs.pop_front();
        flag=1;
    }
    lock.unlock();
    return flag;
}

QString ROSTFSub::getDestinationFrame()
{
    return destinationframe;
}

void ROSTFSub::resetDestinationFrame(QString destinationFrame)
{
    lock.lockForWrite();
    destinationframe=destinationFrame;
    lock.unlock();
}

QString ROSTFSub::getOriginalFrame()
{
    return originalframe;
}

void ROSTFSub::resetOriginalFrame(QString orignalFrame)
{
    lock.lockForWrite();
    originalframe=orignalFrame;
    lock.unlock();
}

void ROSTFSub::resetQueryInterval(int QueryInterval)
{
    emit resetQueryIntervalSignal(QueryInterval);
}
