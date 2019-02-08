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

#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include<QObject>
#include<qglobal.h>
#include<qstring.h>
#include<qstringlist.h>
#include<qqueue.h>
//#include<QApplication>
#include<qapplication.h>
#include<qtimer.h>
#include<ros/callback_queue.h>
#include<qreadwritelock.h>
#include<qfileinfo.h>
#include<qthread.h>
#include<qdebug.h>
//#include<QRegExp>
#include<qregexp.h>

#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<tf/transform_listener.h>

#include<stdlib.h>

// namespace RobotSDK
// {

class ROSInterfaceBase : public QObject
{
    Q_OBJECT
public:
    explicit ROSInterfaceBase(QObject *parent);
    ~ROSInterfaceBase();
protected:
    ros::NodeHandle * nh;
public:
    static bool initflag;
};

template<class MSGTYPE>
class ROSPub : public ROSInterfaceBase
{
public:
    ROSPub(QString Topic, u_int32_t QueueSize, QObject * parent=0);
    ~ROSPub();
protected:
    ros::Publisher pub;
public:
    bool sendMessage(MSGTYPE & msg);
    QString getTopic();
    void resetTopic(QString Topic, u_int32_t QueueSize);
};

template<class MSGTYPE>
ROSPub<MSGTYPE>::ROSPub(QString Topic, u_int32_t QueueSize, QObject *parent)
    : ROSInterfaceBase(parent)
{
    pub=nh->advertise<MSGTYPE>(Topic.toStdString(),QueueSize);
}

template<class MSGTYPE>
ROSPub<MSGTYPE>::~ROSPub()
{
    pub.shutdown();
}

template<class MSGTYPE>
bool ROSPub<MSGTYPE>::sendMessage(MSGTYPE & msg)
{
    if(ros::ok()&&nh->ok())
    {
        pub.publish(msg);
        return 1;
    }
    else
    {
        return 0;
    }
}

template<class MSGTYPE>
QString ROSPub<MSGTYPE>::getTopic()
{
    return QString::fromStdString(pub.getTopic());
}

template<class MSGTYPE>
void ROSPub<MSGTYPE>::resetTopic(QString Topic, u_int32_t QueueSize)
{
    pub.shutdown();
    pub=nh->advertise<MSGTYPE>(Topic.toStdString(),QueueSize);
}

class ROSSubBase : public ROSInterfaceBase
{
    Q_OBJECT
public:
    ROSSubBase(int QueryInterval, QObject * parent);
    ~ROSSubBase();
protected:
    ros::CallbackQueue queue;
    QTimer * timer;
    bool receiveflag;
    QReadWriteLock lock;
signals:
    void receiveMessageSignal();
    void startReceiveSignal();
    void stopReceiveSignal();
    void resetQueryIntervalSignal(int QueryInterval);
public slots:
    void startReceiveSlot();
    void stopReceiveSlot();
protected slots:
    void receiveMessageSlot();
protected:
    void receiveMessage(ros::CallbackQueue::CallOneResult result);
    virtual void clearMessage()=0;
};

template<class MSGTYPE>
class ROSSub : public ROSSubBase
{
public:
    ROSSub(QString Topic, u_int32_t QueueSize, int QueryInterval, QObject * parent=0);
    ~ROSSub();
public:
    void receiveMessageCallback(const MSGTYPE & msg);
protected:
    ros::Subscriber sub;
    QQueue<MSGTYPE> msgs;
protected:
    void clearMessage();
public:
    MSGTYPE getMessage();
    QString getTopic();
    void resetTopic(QString Topic, u_int32_t QueueSize);
    void resetQueryInterval(int QueryInterval);
};

template<class MSGTYPE>
ROSSub<MSGTYPE>::ROSSub(QString Topic, u_int32_t QueueSize, int QueryInterval, QObject *parent)
    : ROSSubBase(QueryInterval,parent)
{
    sub=nh->subscribe(Topic.toStdString(),QueueSize,&ROSSub<MSGTYPE>::receiveMessageCallback,this);
}

template<class MSGTYPE>
ROSSub<MSGTYPE>::~ROSSub()
{
    sub.shutdown();
}

template<class MSGTYPE>
void ROSSub<MSGTYPE>::receiveMessageCallback(const MSGTYPE & msg)
{
    lock.lockForWrite();
    if(receiveflag)
    {        
        msgs.push_back(msg);
    }
    lock.unlock();
}

template<class MSGTYPE>
void ROSSub<MSGTYPE>::clearMessage()
{
    msgs.clear();
}

template<class MSGTYPE>
MSGTYPE ROSSub<MSGTYPE>::getMessage()
{
    MSGTYPE msg;
    lock.lockForWrite();
    if(receiveflag&&!msgs.isEmpty())
    {
        msg=msgs.front();
        msgs.pop_front();
    }
    lock.unlock();
    return msg;
}

template<class MSGTYPE>
QString ROSSub<MSGTYPE>::getTopic()
{
    return QString::fromStdString(sub.getTopic());
}

template<class MSGTYPE>
void ROSSub<MSGTYPE>::resetTopic(QString Topic, u_int32_t QueueSize)
{
    lock.lockForWrite();
    sub.shutdown();
    sub=nh->subscribe(Topic.toStdString(),QueueSize,&ROSSub<MSGTYPE>::receiveMessageCallback,this);
    lock.unlock();
}

template<class MSGTYPE>
void ROSSub<MSGTYPE>::resetQueryInterval(int QueryInterval)
{
    emit resetQueryIntervalSignal(QueryInterval);
}

class ROSTFPub : public ROSInterfaceBase
{
    Q_OBJECT
public:
    ROSTFPub(QString childFrameID, QString frameID="world", QObject * parent=0);
protected:
    QString childframeid;
    QString frameid;
    tf::TransformBroadcaster br;
public:
    bool sendTF(tf::Transform & transform);
    QString getChildFrameID();
    void resetChildFrameID(QString childFrameID);
    QString getFrameID();
    void resetFrameID(QString frameID="world");
};

class ROSTFSub : public ROSInterfaceBase
{
    Q_OBJECT
public:
    ROSTFSub(QString destinationFrame, QString originalFrame="world", int QueryInterval=10, QObject * parent=0);
    ~ROSTFSub();
protected:
    QString destinationframe;
    QString originalframe;
    QTimer * timer;
    bool receiveflag;
    QReadWriteLock lock;
    tf::TransformListener listener;
    QQueue<tf::StampedTransform> tfs;
    tf::StampedTransform lasttf;
    bool lastflag;
signals:
    void receiveTFSignal();
    void startReceiveSignal();
    void stopReceiveSignal();
    void resetQueryIntervalSignal(int QueryInterval);
public slots:
    void startReceiveSlot();
    void stopReceiveSlot();
protected slots:
    void receiveTFSlot();
protected:
    void clearTFs();
public:
    bool getTF(tf::StampedTransform & transform);
    QString getDestinationFrame();
    void resetDestinationFrame(QString destinationFrame);
    QString getOriginalFrame();
    void resetOriginalFrame(QString orignalFrame="world");
    void resetQueryInterval(int QueryInterval);
};

//}

#endif // ROSINTERFACE_H
