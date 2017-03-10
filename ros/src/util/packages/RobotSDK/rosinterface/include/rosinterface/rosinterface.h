/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
