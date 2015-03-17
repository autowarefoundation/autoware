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
#include<ros/ros.h>
#include<qstring.h>
#include<qstringlist.h>
#include<qqueue.h>
#include<qapplication.h>
#include<qtimer.h>
#include<ros/callback_queue.h>
#include<qreadwritelock.h>
#include<qfileinfo.h>
#include<qthread.h>
#include<qdebug.h>

#ifndef INITROSMASTERURI
#define INITROSMASTERURI "http://localhost:11311"
#endif

class ROSInterfaceBase : public QObject
{
    Q_OBJECT
public:
    explicit ROSInterfaceBase(QString NodeName, QString ROSMasterURI, QObject *parent);
    ~ROSInterfaceBase();
protected:
    ros::NodeHandle * nh;
    QThread thread;
};

template<class MSGTYPE>
class ROSPub : public ROSInterfaceBase
{
public:
    ROSPub(QString Topic, u_int32_t QueueSize, QString NodeName=QString(), QString ROSMasterURi=INITROSMASTERURI, QObject * parent=0);
    ~ROSPub();
protected:
    ros::Publisher pub;
public:
    bool sendMessage(MSGTYPE & msg);
    QString getTopic();
    void resetTopic(QString Topic, u_int32_t QueueSize);
};

template<class MSGTYPE>
ROSPub<MSGTYPE>::ROSPub(QString Topic, u_int32_t QueueSize, QString NodeName, QString ROSMasterURi, QObject *parent)
    : ROSInterfaceBase(NodeName,ROSMasterURi,parent)
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
    ROSSubBase(int Interval, QString NodeName, QString ROSMasterURi, QObject * parent);
    ~ROSSubBase();
protected:
    ros::CallbackQueue queue;
    QTimer timer;
    bool receiveflag;
    QReadWriteLock lock;
signals:
    void receiveMessageSignal();
    void startReceiveSignal();
    void stopReceiveSignal();
public slots:
    void startReceiveSlot();
    void stopReceiveSlot();
protected:
    void receiveMessage(ros::CallbackQueue::CallOneResult result);
    virtual void clearMessage()=0;
protected slots:
    void receiveMessageSlot();
};

template<class MSGTYPE>
class ROSSub : public ROSSubBase
{
public:
    ROSSub(QString Topic, u_int32_t QueueSize, int Interval, QString NodeName=QString(), QString ROSMasterURi=INITROSMASTERURI, QObject * parent=0);
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
};

template<class MSGTYPE>
ROSSub<MSGTYPE>::ROSSub(QString Topic, u_int32_t QueueSize, int Interval, QString NodeName, QString ROSMasterURi, QObject *parent)
    : ROSSubBase(Interval,NodeName,ROSMasterURi,parent)
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
    lock.lockForRead();
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

#endif // ROSINTERFACE_H
