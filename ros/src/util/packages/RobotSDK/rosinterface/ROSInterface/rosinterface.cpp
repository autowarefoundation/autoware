#include "rosinterface.h"

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

