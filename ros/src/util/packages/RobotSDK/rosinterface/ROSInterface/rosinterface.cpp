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

#include "rosinterface.h"
#include "rosinterface_moc.cpp"

ROSInterfaceBase::ROSInterfaceBase(QString NodeName, QString ROSMasterURI, QObject *parent)
    : QObject(parent)
{
    qputenv("ROS_MASTER_URI",ROSMasterURI.toUtf8());
    QStringList arguments=QApplication::instance()->arguments();
    int argc=1;
    if(NodeName.isEmpty())
    {
        QFileInfo fileinfo(arguments[0]);
        if(fileinfo.exists())
        {
            NodeName=fileinfo.baseName();
        }
    }
    char *argv=arguments[0].toUtf8().data();

    ros::init(argc,&argv,NodeName.toStdString());
    nh=new ros::NodeHandle;
    this->moveToThread(&thread);
    thread.start();
}

ROSInterfaceBase::~ROSInterfaceBase()
{
    thread.exit();
    thread.wait();
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

ROSSubBase::ROSSubBase(int Interval, QString NodeName, QString ROSMasterURi, QObject *parent)
    : ROSInterfaceBase(NodeName,ROSMasterURi,parent)
{
    nh->setCallbackQueue(&queue);
    timer.setInterval(Interval);
    connect(&timer,SIGNAL(timeout()),this,SLOT(receiveMessageSlot()));
    connect(this,SIGNAL(startReceiveSignal()),&timer,SLOT(start()));
    connect(this,SIGNAL(stopReceiveSignal()),&timer,SLOT(stop()));
    receiveflag=0;
    emit startReceiveSignal();
}

ROSSubBase::~ROSSubBase()
{
    receiveflag=0;
    emit stopReceiveSignal();
    disconnect(&timer,SIGNAL(timeout()),this,SLOT(receiveMessageSlot()));
    disconnect(this,SIGNAL(startReceiveSignal()),&timer,SLOT(start()));
    disconnect(this,SIGNAL(stopReceiveSignal()),&timer,SLOT(stop()));
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

void ROSSubBase::receiveMessageSlot()
{
    if(ros::ok()&&nh->ok())
    {
        receiveMessage(queue.callOne(ros::WallDuration(0)));
    }
}

