#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "moc_mainwindow.cpp"

InitTrackerView::InitTrackerView(QWidget * parent)
    : QGraphicsView(parent)
{
    scene=new QGraphicsScene;
    scene->setSceneRect(-5000,-5000,10000,10000);
    this->setScene(scene);
    sx=sy=15;
    this->scale(sx,sy);
}

void InitTrackerView::showLaserScan(LaserScan & scan)
{
    scene->clear();
    state.clear();
    double density=2*PI/scan.beamnum;
    for(int i=0;i<scan.beamnum;i++)
    {
        double theta=i*density-PI;
        double x=scan.length[i]*cos(theta);
        double y=scan.length[i]*sin(theta);
        scene->addEllipse(-y-0.05,-x-0.05,0.1,0.1,QPen(Qt::blue,0.1))->setZValue(1);
    }
    for(int i=10;i<=100;i+=10)
    {
        scene->addEllipse(-i/2,-i/2,i,i,QPen(Qt::gray,0.2,Qt::DotLine))->setZValue(0);
    }
    scene->addLine(0,0,0,-5,QPen(Qt::red,0.2,Qt::DotLine))->setZValue(0);
    scene->addLine(0,0,-5,0,QPen(Qt::green,0.2,Qt::DotLine))->setZValue(0);
}

void InitTrackerView::getInitState(QVector<VehicleState> &initState)
{
    int statenum=state.size();
    initState.resize(statenum);
    for(int i=0;i<statenum;i++)
    {
        initState[i].x=-state[i]->line().p1().y();
        initState[i].y=-state[i]->line().p1().x();
        initState[i].theta=atan2(state[i]->line().p1().x()-state[i]->line().p2().x(),state[i]->line().p1().y()-state[i]->line().p2().y());
        initState[i].wl=1.5;initState[i].wr=1.5;initState[i].lf=2.5;initState[i].lb=2.5;
        initState[i].a=0;initState[i].v=10;initState[i].k=0;initState[i].omega=0;
    }
}

void InitTrackerView::mousePressEvent(QMouseEvent * event)
{
    switch(event->button())
    {
    case Qt::LeftButton:
        if(pressflag)
        {
            point2=this->mapToScene(event->pos());
            lineitem=scene->addLine(point1.x(),point1.y(),point2.x(),point2.y(),QPen(Qt::red,0.2));
            lineitem->setZValue(2);
            state.push_back(lineitem);
            scene->removeItem(point1item);
            delete point1item;
            pressflag=0;
        }
        else
        {
            point1=this->mapToScene(event->pos());
            point1item=scene->addEllipse(point1.x()-0.05,point1.y()-0.05,0.1,0.1,QPen(Qt::red,0.2));
            point1item->setZValue(2);
            pressflag=1;
        }
        break;
    case Qt::RightButton:
        if(pressflag)
        {
            scene->removeItem(point1item);
            delete point1item;
            pressflag=0;
        }
        else
        {
            QGraphicsLineItem * item=(QGraphicsLineItem *)(this->itemAt(event->pos()));
            if(state.contains(item))
            {
                scene->removeItem(item);
                delete item;
            }
        }
        break;
    default:
        QGraphicsView::mousePressEvent(event);
        break;
    }
}

void InitTrackerView::wheelEvent(QWheelEvent *event)
{
    if(ctrlflag)
    {
        if(event->delta()>0)
        {
            sx*=1.1;sy*=1.1;
            this->scale(1.1,1.1);
        }
        else
        {
            sx*=0.9;sy*=0.9;
            this->scale(0.9,0.9);
        }
    }
    else
    {
        QGraphicsView::wheelEvent(event);
    }
}

void InitTrackerView::keyPressEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_Control:
        ctrlflag=1;
        break;
    default:
        QGraphicsView::keyPressEvent(event);
        break;
    }
}

void InitTrackerView::keyReleaseEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_Control:
        ctrlflag=0;
        break;
    default:
        QGraphicsView::keyReleaseEvent(event);
        break;
    }
}

UpdateTrackerView::UpdateTrackerView(QWidget *parent)
    : QGraphicsView(parent)
{
    scene=new QGraphicsScene;
    scene->setSceneRect(-5000,-5000,10000,10000);
    this->setScene(scene);
    sx=sy=15;
    this->scale(sx,sy);
}

//function to handle final tracking result stored in trackerresultmap [vehicleID, tracking result]
void UpdateTrackerView::slotUpdateTrackerFinish(LaserScan scan, QMap<int, TrackerResultContainer> trackerresultmap)
{
    scene->clear();

    QTransform transform;
    transform.rotateRadians(-scan.theta);
    transform.translate(-scan.y,-scan.x);
    transform.scale(sx,sy);
    setTransform(transform);

    centerOn(0,0);

    double density=2*PI/scan.beamnum;
    for(int i=0;i<scan.beamnum;i++)
    {
        double theta=i*density-PI;
        double x=scan.length[i]*cos(theta);
        double y=scan.length[i]*sin(theta);
        scene->addEllipse(-y-0.05,-x-0.05,0.1,0.1,QPen(Qt::blue,0.1))->setZValue(1);
    }
    for(int i=10;i<=100;i+=10)
    {
        scene->addEllipse(-i/2,-i/2,i,i,QPen(Qt::gray,0.2,Qt::DotLine))->setZValue(0);
    }
    scene->addLine(0,0,0,-5,QPen(Qt::red,0.2,Qt::DotLine))->setZValue(0);
    scene->addLine(0,0,-5,0,QPen(Qt::green,0.2,Qt::DotLine))->setZValue(0);

    QList<int> curidlist=pathmap.keys();
    QList<int> trackidlist=trackerresultmap.keys();
    //for loop to get all tracking results
    for(int i=0;i<trackidlist.size();i++)
    {
        //get one tracking result stored in trackerresult
        TrackerResultContainer trackerresult=trackerresultmap[trackidlist[i]];
        //for loop to draw rectangle of vehicle. the corner is represented as (cx[i],cy[i]) (0<=i<=3)
        for(int j=0;j<4;j++)
        {
            scene->addLine(-trackerresult.estimate.cy[j],-trackerresult.estimate.cx[j],-trackerresult.estimate.cy[(j+1)%4],-trackerresult.estimate.cx[(j+1)%4],QPen(Qt::red,0.1,Qt::DotLine));
        }
        scene->addLine(-trackerresult.estimate.cy[0],-trackerresult.estimate.cx[0],-trackerresult.estimate.cy[2],-trackerresult.estimate.cx[2],QPen(Qt::red,0.1,Qt::DotLine));
        scene->addLine(-trackerresult.estimate.cy[1],-trackerresult.estimate.cx[1],-trackerresult.estimate.cy[3],-trackerresult.estimate.cx[3],QPen(Qt::red,0.1,Qt::DotLine));

        for(int j=0;j<2;j++)
        {
            for(int k=0;k<trackerresult.edgepointnum[j];k++)
            {
                int id=trackerresult.edgepointid[j][k];
                double theta=id*density-PI;
                double x=scan.length[id]*cos(theta);
                double y=scan.length[id]*sin(theta);
                scene->addEllipse(-y-0.05,-x-0.05,0.1,0.1,QPen(Qt::green,0.1))->setZValue(2);
            }
        }
    }

    for(int i=0;i<curidlist.size();i++)
    {
        bool flag=trackidlist.contains(curidlist[i]);
        if(!flag)
        {            
            pathmap.remove(curidlist[i]);
        }
    }
}

void UpdateTrackerView::wheelEvent(QWheelEvent *event)
{
    if(ctrlflag)
    {
        if(event->delta()>0)
        {
            sx*=1.1;sy*=1.1;
            this->scale(1.1,1.1);
        }
        else
        {
            sx*=0.9;sy*=0.9;
            this->scale(0.9,0.9);
        }
    }
    else
    {
        QGraphicsView::wheelEvent(event);
    }
}

void UpdateTrackerView::keyPressEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_Control:
        ctrlflag=1;
        break;
    default:
        QGraphicsView::keyPressEvent(event);
        break;
    }
}

void UpdateTrackerView::keyReleaseEvent(QKeyEvent *event)
{
    switch(event->key())
    {
    case Qt::Key_Control:
        ctrlflag=0;
        break;
    default:
        QGraphicsView::keyReleaseEvent(event);
        break;
    }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    scansub=new ROSSub<sensor_msgs::LaserScanConstPtr>("/scan",1000,10,this);
    //detectionsub=new ROSSub<cv_tracker::obj_label::ConstPtr>("obj_label",1000,10,this);
    boxessub=new ROSSub<jsk_recognition_msgs::BoundingBoxArray::ConstPtr>("bounding_boxes",1000,10,this);
    tfsub=new ROSTFSub("/world","/velodyne",10,this);
    tfMap2Lidarsub=new ROSTFSub("/velodyne","/map",10,this); // obj_pose is published into "map" frame


    //subscribe vehicle detection results (array of [x,y,theta])

    connect(scansub,SIGNAL(receiveMessageSignal()),this,SLOT(slotReceive()));
    //connect(detectionsub,SIGNAL(receiveMessageSignal()), this, SLOT(slotReceiveDetection()));
    connect(boxessub,SIGNAL(receiveMessageSignal()), this, SLOT(slotReceiveBoxes()));
    connect(tfsub,SIGNAL(receiveTFSignal()),this,SLOT(slotReceiveTF()));
    connect(tfMap2Lidarsub,SIGNAL(receiveTFSignal()),this,SLOT(slotReceiveTFMap2Lidar()));

    QSplitter * splitter=new QSplitter(Qt::Horizontal);
    ui->layout->addWidget(splitter);

    initview=new InitTrackerView;
    splitter->addWidget(initview);

    updateview=new UpdateTrackerView;
    splitter->addWidget(updateview);

    vehicletracker=new RBSSPFVehicleTracker;
    connect(vehicletracker,SIGNAL(signalUpdateTrackerFinish(LaserScan,QMap<int,TrackerResultContainer>)),updateview,SLOT(slotUpdateTrackerFinish(LaserScan,QMap<int,TrackerResultContainer>)));

    scansub->startReceiveSlot();
    //detectionsub->startReceiveSlot();
    boxessub->startReceiveSlot();
    tfsub->startReceiveSlot();
    tfMap2Lidarsub->startReceiveSlot();
}

MainWindow::~MainWindow()
{
    scansub->stopReceiveSlot();
    //detectionsub->stopReceiveSlot();
    boxessub->stopReceiveSlot();
    tfsub->stopReceiveSlot();
    tfMap2Lidarsub->stopReceiveSlot();
    delete ui;
}

void MainWindow::slotReceive()
{
    sensor_msgs::LaserScanConstPtr msg=scansub->getMessage();
    int msec=(msg->header.stamp.sec)%(24*60*60)*1000+(msg->header.stamp.nsec)/1000000;
    QTime timestamp=QTime::fromMSecsSinceStartOfDay(msec);
    LaserScan scan;
    scan.timestamp=timestamp.msecsSinceStartOfDay();
    scan.beamnum=msg->ranges.size();
    for(int i=0;i<scan.beamnum;i++)
    {
        scan.length[i]=msg->ranges[i];
    }
    scanlist.push_back(QPair<QTime, LaserScan >(timestamp,scan));
    if(ui->trigger->isChecked())
    {
        slotShowScan();
    }
}

void MainWindow::slotReceiveDetection()
{
    cv_tracker::obj_label::ConstPtr msg=detectionsub->getMessage();

    for (const auto& point : msg->reprojected_pos) {
        int msec=(msg->header.stamp.sec)%(24*60*60)*1000+(msg->header.stamp.nsec)/1000000;
        QTime timestamp=QTime::fromMSecsSinceStartOfDay(msec);
        VehicleState state;
        //fill state from msg;
        // convert object position from map coordinate to sensor coordinate
        tf::Vector3 pt(point.x, point.y, point.z);
        tf::Vector3 converted = transformMap2Lidar * pt;
        state.x = converted.x();
        state.y = converted.y();

        detectionlist.push_back(QPair<QTime, VehicleState >(timestamp,state));
        if(ui->trigger->isChecked())
            {
                slotShowScan();
            }
    }
}

void MainWindow::slotReceiveBoxes()
{
	jsk_recognition_msgs::BoundingBoxArray::ConstPtr msg=boxessub->getMessage();
    for (const auto& box : msg->boxes) {
        int msec=(msg->header.stamp.sec)%(24*60*60)*1000+(msg->header.stamp.nsec)/1000000;
        QTime timestamp=QTime::fromMSecsSinceStartOfDay(msec);
        VehicleState vstate;
        //fill state from msg;
        // convert object position from map coordinate to sensor coordinate
        tf::Vector3 pt(box.pose.position.x, box.pose.position.y, box.pose.position.z);
        //tf::Vector3 converted = transformMap2Lidar * pt;
        vstate.x = pt.x();
        vstate.y = pt.y();
        tf::Quaternion quat;
        tf::quaternionMsgToTF(box.pose.orientation, quat);
        vstate.theta = tf::getYaw(quat);

        detectionlist.push_back(QPair<QTime, VehicleState >(timestamp,vstate));
        if(ui->trigger->isChecked())
            {
                slotShowScan();
            }
    }
}

void MainWindow::slotReceiveTF()
{
    tf::StampedTransform tf;
    tfsub->getTF(tf);
    int msec=(tf.stamp_.sec)%(24*60*60)*1000+(tf.stamp_.nsec)/1000000;
    QTime timestamp=QTime::fromMSecsSinceStartOfDay(msec);

    tf::Vector3 pos=tf.getOrigin();
    tf::Matrix3x3 rot=tf.getBasis();
    Eigen::Vector3d head;
    head(0)=1;head(1)=0;head(2)=0;
    Eigen::Matrix3d rotmat;
    for(int i=0;i<3;i++)
    {
        rotmat(i,0)=(double)(rot.getRow(i).x());
        rotmat(i,1)=(double)(rot.getRow(i).y());
        rotmat(i,2)=(double)(rot.getRow(i).z());
    }
    head=rotmat*head;
    EGOMOTION ego={pos.x(),pos.y(),atan2(head(1),head(0))};

    tflist.push_back(QPair<QTime,EGOMOTION>(timestamp,ego));

    if(ui->trigger->isChecked())
    {
        slotShowScan();
    }
}


void MainWindow::slotReceiveTFMap2Lidar()
{
    tfMap2Lidarsub->getTF(transformMap2Lidar);
}


void MainWindow::getInitStateFromTopic(QVector<VehicleState> &initState)
{
    for (auto& detected : detectionlist)
    {
        initState.push_back(detected.second);
    }
    detectionlist.clear();
}

void MainWindow::slotShowScan()
{
    //synchronization between vscan and tf
    bool flag=1;
    while(flag&&!scanlist.isEmpty()&&!tflist.isEmpty())
    {
        QTime scantime=scanlist[0].first;
        QTime tftime=tflist[0].first;
        if(scantime==tftime)
        {
            flag=0;
        }
        else if(scantime>tftime)
        {
            tflist.pop_front();
        }
        else
        {
            scanlist.pop_front();
        }
    }

    int scannum=scanlist.size();
    int tfnum=tflist.size();
    //it's better to synchronize detection with them


    if(scannum>=1&&tfnum>=1)
    {
        if(initflag)
        {
            //=====================================
            //fill initstate from subscribed detection topic
            //=====================================
            QVector<VehicleState> initstate;
            //initview->getInitState(initstate);
            getInitStateFromTopic(initstate);
            vehicletracker->addTrackerData(curscan,initstate);
        }
        curscan=scanlist[0].second;
        curscan.x=tflist[0].second.x;
        curscan.y=tflist[0].second.y;
        curscan.theta=tflist[0].second.theta;
        initview->showLaserScan(curscan);
        initflag=1;

        scanlist.pop_front();
        //detectionlist.pop_front();
        tflist.pop_front();

        if(ui->local->isChecked())
        {
            QTransform transform;
            transform.scale(initview->sx,initview->sy);
            initview->setTransform(transform);
        }
        else
        {
            QTransform transform;
            transform.rotateRadians(-curscan.theta);
            transform.translate(-curscan.y,-curscan.x);
            transform.scale(initview->sx,initview->sy);
            initview->setTransform(transform);
        }
        initview->centerOn(0,0);
    }
}
