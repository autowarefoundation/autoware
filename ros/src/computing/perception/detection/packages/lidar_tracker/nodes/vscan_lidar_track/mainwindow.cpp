/*#include "mainwindow.h"
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
    idcount=0;
}

void InitTrackerView::showLaserScan(LaserScan & scan)
{
    scene->clear();
    state.clear();
    printf("Cleared\n");
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

void InitTrackerView::getInitState(QVector<Tracker> & initState)
{
    int statenum=state.size();
    initState.resize(statenum);
    printf("%d available\n", statenum);
    for(int i=0;i<statenum;i++)
    {
        initState[i].mean.x=-state[i]->line().p1().y();
        initState[i].mean.y=-state[i]->line().p1().x();
        initState[i].mean.theta=atan2(state[i]->line().p1().x()-state[i]->line().p2().x(),
        								state[i]->line().p1().y()-state[i]->line().p2().y());
        initState[i].mean.wl=1.5;
        initState[i].mean.wr=1.5;
        initState[i].mean.lf=2.5;
        initState[i].mean.lb=2.5;
        initState[i].mean.a=0;initState[i].mean.v=10;initState[i].mean.k=0;initState[i].mean.omega=0;
        initState[i].pfcount=0;
        initState[i].id=idcount++;
        initState[i].status=StatusInitGeometry;
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
            printf("Pushed\n");
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
void UpdateTrackerView::slotUpdateTrackerFinish(LaserScan scan, QVector<Tracker> trackers)
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

    for(int i=0;i<trackers.size();i++)
    {
        Tracker tracker=trackers[i];
        for(int j=0;j<4;j++)
        {
            scene->addLine(-tracker.cy[j], 		-tracker.cx[j],
            				-tracker.cy[(j+1)%4],-tracker.cx[(j+1)%4],
            				QPen(tracker.status!=StatusUpdateTracker_PF?Qt::red:Qt::cyan,0.1,Qt::DotLine));
            printf("Tracker Corner %d, %f, %f\n", j, tracker.cx[j], tracker.cy[j]);
        }
        scene->addLine(-tracker.cy[0],-tracker.cx[0],-tracker.cy[2],-tracker.cx[2],QPen(tracker.status!=StatusUpdateTracker_PF?Qt::red:Qt::cyan,0.1,Qt::DotLine));
        scene->addLine(-tracker.cy[1],-tracker.cx[1],-tracker.cy[3],-tracker.cx[3],QPen(tracker.status!=StatusUpdateTracker_PF?Qt::red:Qt::cyan,0.1,Qt::DotLine));
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
    tfsub=new ROSTFSub("/world","/velodyne",10,this);
//    tfMap2Lidarsub=new ROSTFSub("/velodyne","/map",10,this); // obj_pose is published into "map" frame


    //subscribe vehicle detection results (array of [x,y,theta])

    connect(scansub,SIGNAL(receiveMessageSignal()),this,SLOT(slotReceive()));
    //connect(detectionsub,SIGNAL(receiveMessageSignal()), this, SLOT(slotReceiveDetection()));
    connect(tfsub,SIGNAL(receiveTFSignal()),this,SLOT(slotReceiveTF()));
//    connect(tfMap2Lidarsub,SIGNAL(receiveTFSignal()),this,SLOT(slotReceiveTFMap2Lidar()));

    QSplitter * splitter=new QSplitter(Qt::Horizontal);
    ui->layout->addWidget(splitter);

    initview=new InitTrackerView;
    splitter->addWidget(initview);

    updateview=new UpdateTrackerView;
    splitter->addWidget(updateview);

    scansub->startReceiveSlot();
//    detectionsub->startReceiveSlot();
    tfsub->startReceiveSlot();
//    tfMap2Lidarsub->startReceiveSlot();

    cudaOpenTracker();
}

MainWindow::~MainWindow()
{
    scansub->stopReceiveSlot();
//    detectionsub->stopReceiveSlot();
    tfsub->stopReceiveSlot();
//    tfMap2Lidarsub->stopReceiveSlot();
    delete ui;

    cudaCloseTracker();
}

void MainWindow::slotReceive()
{
    sensor_msgs::LaserScanConstPtr msg=scansub->getMessage();
    double timestamp=msg->header.stamp.toSec();
    LaserScan scan;
    scan.timestamp=timestamp;
    scan.beamnum=msg->ranges.size();
    for(int i=0;i<scan.beamnum;i++)
    {
        scan.length[i]=msg->ranges[i];
    }
    scanlist.push_back(QPair<double, LaserScan >(timestamp,scan));
    if(ui->trigger->isChecked())
    {
        slotShowScan();
    }
}

//void MainWindow::slotReceiveDetection()
//{
//    cv_tracker::obj_label::ConstPtr msg=detectionsub->getMessage();

//    for (const auto& point : msg->reprojected_pos) {
//        double timestamp=msg->header.stamp.toSec();
//        VehicleState state;
//        //fill state from msg;
//        // convert object position from map coordinate to sensor coordinate
//        tf::Vector3 pt(point.x, point.y, point.z);
//        tf::Vector3 converted = transformMap2Lidar * pt;
//        state.x = converted.x();
//        state.y = converted.y();

//        detectionlist.push_back(QPair<double, VehicleState >(timestamp,state));
//        if(ui->trigger->isChecked())
//        {
//            slotShowScan();
//        }
//    }
//}

void MainWindow::slotReceiveTF()
{
    tf::StampedTransform tf;
    tfsub->getTF(tf);
    double timestamp=tf.stamp_.toSec();

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

    tflist.push_back(QPair<double,EGOMOTION>(timestamp,ego));

    if(ui->trigger->isChecked())
    {
        slotShowScan();
    }
}


//void MainWindow::slotReceiveTFMap2Lidar()
//{
//    tfMap2Lidarsub->getTF(transformMap2Lidar);
//}


//void MainWindow::getInitStateFromTopic(QVector<VehicleState> &initState)
//{
//    for (auto& detected : detectionlist)
//    {
//        initState.push_back(detected.second);
//    }
//    detectionlist.clear();
//}

void MainWindow::slotShowScan()
{
    //synchronization between vscan and tf
    bool flag=1;
    while(flag&&!scanlist.isEmpty()&&!tflist.isEmpty())
    {
        double scantime=scanlist[0].first;
        double tftime=tflist[0].first;
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
            QVector<Tracker> initstate;
            initview->getInitState(initstate);
            int initstatenum=initstate.size();
            //getInitStateFromTopic(initstate);
            int trackersnum=trackers.size();
            if(initstatenum>0||trackersnum>0)
            {
                std::vector<Tracker> tmptrackers;
                for(int i=0,j=0;i<trackersnum||j<initstatenum;)
                {
                    if(i>=trackersnum)
                    {
                        tmptrackers.push_back(initstate[j]);
                        j++;
                    }
                    else if(j>=initstatenum)
                    {
                        tmptrackers.push_back(trackers[i]);
                        i++;
                    }
                    else if(trackers[i].id<initstate[j].id)
                    {
                        tmptrackers.push_back(trackers[i]);
                        i++;
                    }
                    else if(trackers[i].id>initstate[j].id)
                    {
                        tmptrackers.push_back(initstate[j]);
                        j++;
                    }
                    else
                    {
                        tmptrackers.push_back(trackers[i]);
                        i++;
                        j++;
                    }
                }
                cudaUpdateTracker(tmptrackers);
                trackers.clear();
                for(int i=0;i<tmptrackers.size();i++)
                {
                    if(tmptrackers[i].pfcount<20)
                    {
                        trackers.push_back(tmptrackers[i]);
                    }
                }
            }
            updateview->slotUpdateTrackerFinish(curscan,trackers);
        }
        curscan=scanlist[0].second;
        curscan.x=tflist[0].second.x;
        curscan.y=tflist[0].second.y;
        curscan.theta=tflist[0].second.theta;
        cudaSetLaserScan(curscan);

        initflag=1;

        initview->showLaserScan(curscan);

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
*/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "moc_mainwindow.cpp"

InitTrackerView::InitTrackerView(QWidget * parent)
    : QGraphicsView(parent)
{
	qt_init_view_graphics_scene_ = new QGraphicsScene;
	qt_init_view_graphics_scene_->setSceneRect(-5000, -5000, 10000, 10000);
	this->setScene(qt_init_view_graphics_scene_);
	sx = sy = 15;
	this->scale(sx, sy);
}

void InitTrackerView::showLaserScan(LaserScan & scan)
{
	qt_init_view_graphics_scene_->clear();
	qt_line_states_.clear();
	double density = 2 * PI / scan.beamnum;
	for (int i = 0; i < scan.beamnum; i++)
	{
		double theta = i * density - PI;
		double x = scan.length[i] * cos(theta);
		double y = scan.length[i] * sin(theta);
		qt_init_view_graphics_scene_->addEllipse(-y - 0.05, -x - 0.05, 0.1, 0.1,
				QPen(Qt::blue, 0.1))->setZValue(1);
	}
	for (int i = 10; i <= 100; i += 10)
	{
		qt_init_view_graphics_scene_->addEllipse(-i / 2, -i / 2, i, i,
				QPen(Qt::gray, 0.2, Qt::DotLine))->setZValue(0);
	}
	qt_init_view_graphics_scene_->addLine(0, 0, 0, -5,
			QPen(Qt::red, 0.2, Qt::DotLine))->setZValue(0);
	qt_init_view_graphics_scene_->addLine(0, 0, -5, 0,
			QPen(Qt::green, 0.2, Qt::DotLine))->setZValue(0);
}

void InitTrackerView::getInitState(QVector<VehicleState> &initState)
{
	int statenum = qt_line_states_.size();
	initState.resize(statenum);
	for (int i = 0; i < statenum; i++)
	{
		initState[i].x = -qt_line_states_[i]->line().p1().y();
		initState[i].y = -qt_line_states_[i]->line().p1().x();
		initState[i].theta = atan2(
				qt_line_states_[i]->line().p1().x() - qt_line_states_[i]->line().p2().x(),
				qt_line_states_[i]->line().p1().y() - qt_line_states_[i]->line().p2().y());
		initState[i].wl = 1.5;
		initState[i].wr = 1.5;
		initState[i].lf = 2.5;
		initState[i].lb = 2.5;
		initState[i].a = 0;
		initState[i].v = 10;
		initState[i].k = 0;
		initState[i].omega = 0;
	}
}

void InitTrackerView::mousePressEvent(QMouseEvent * event)
{
	std::cout << "mousePressEvent" << std::endl;
	switch (event->button())
	{
	case Qt::LeftButton:
		if (clicked_flag_)
		{
			qt_click_point_2 = this->mapToScene(event->pos());
			qt_line_pose_ = qt_init_view_graphics_scene_->addLine(qt_click_point_1.x(),
					qt_click_point_1.y(), qt_click_point_2.x(),
					qt_click_point_2.y(), QPen(Qt::red, 0.2));
			qt_line_pose_->setZValue(2);
			qt_line_states_.push_back(qt_line_pose_);
			qt_init_view_graphics_scene_->removeItem(qt_clicked_point_1);
			delete qt_clicked_point_1;
			clicked_flag_ = 0;
		}
		else
		{
			qt_click_point_1 = this->mapToScene(event->pos());
			qt_clicked_point_1 = qt_init_view_graphics_scene_->addEllipse(qt_click_point_1.x() - 0.05, qt_click_point_1.y() - 0.05,
					0.1, 0.1, QPen(Qt::red, 0.2));
			qt_clicked_point_1->setZValue(2);
			clicked_flag_ = 1;
		}
		break;
	case Qt::RightButton:
		if (clicked_flag_)
		{
			qt_init_view_graphics_scene_->removeItem(qt_clicked_point_1);
			delete qt_clicked_point_1;
			clicked_flag_ = 0;
		}
		else
		{
			QGraphicsLineItem * item = (QGraphicsLineItem *) (this->itemAt(
					event->pos()));
			if (qt_line_states_.contains(item))
			{
				qt_init_view_graphics_scene_->removeItem(item);
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
	if (ctrl_key_pressed_)
	{
		if (event->delta() > 0)
		{
			sx *= 1.1;
			sy *= 1.1;
			this->scale(1.1, 1.1);
		}
		else
		{
			sx *= 0.9;
			sy *= 0.9;
			this->scale(0.9, 0.9);
		}
	}
	else
	{
		QGraphicsView::wheelEvent(event);
	}
}

void InitTrackerView::keyPressEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key_Control:
		ctrl_key_pressed_ = 1;
		break;
	default:
		QGraphicsView::keyPressEvent(event);
		break;
	}
}

void InitTrackerView::keyReleaseEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key_Control:
		ctrl_key_pressed_ = 0;
		break;
	default:
		QGraphicsView::keyReleaseEvent(event);
		break;
	}
}

UpdateTrackerView::UpdateTrackerView(QWidget *parent) :
		QGraphicsView(parent)
{
	qt_view_tracker_scene_ = new QGraphicsScene;
	qt_view_tracker_scene_->setSceneRect(-5000, -5000, 10000, 10000);
	this->setScene(qt_view_tracker_scene_);
	sx = sy = 15;
	this->scale(sx, sy);
}

//function to handle final tracking result stored in trackerresultmap [vehicleID, tracking result]
void UpdateTrackerView::slotUpdateTrackerFinish(LaserScan scan, QMap<int, TrackerResultContainer> trackerresultmap)
{
	qt_view_tracker_scene_->clear();

	QTransform transform;
	transform.rotateRadians(-scan.theta);
	transform.translate(-scan.y, -scan.x);
	transform.scale(sx, sy);
	setTransform(transform);

	centerOn(0, 0);

	double density = 2 * PI / scan.beamnum;
	for (int i = 0; i < scan.beamnum; i++)
	{
		double theta = i * density - PI;
		double x = scan.length[i] * cos(theta);
		double y = scan.length[i] * sin(theta);
		qt_view_tracker_scene_->addEllipse(-y - 0.05, -x - 0.05, 0.1, 0.1,
				QPen(Qt::blue, 0.1))->setZValue(1);
	}
	for (int i = 10; i <= 100; i += 10)
	{
		qt_view_tracker_scene_->addEllipse(-i / 2, -i / 2, i, i,
				QPen(Qt::gray, 0.2, Qt::DotLine))->setZValue(0);
	}
	qt_view_tracker_scene_->addLine(0, 0, 0, -5,
			QPen(Qt::red, 0.2, Qt::DotLine))->setZValue(0);
	qt_view_tracker_scene_->addLine(0, 0, -5, 0,
			QPen(Qt::green, 0.2, Qt::DotLine))->setZValue(0);

	QList<int> curidlist = pathmap.keys();
	QList<int> trackidlist = trackerresultmap.keys();

	//for loop to get all tracking results
	jsk_recognition_msgs::BoundingBoxArray out_boxes;
	out_boxes.header = boxes_header_;
	for (int i = 0; i < trackidlist.size(); i++)
	{
		//get one tracking result stored in trackerresult
		TrackerResultContainer trackerresult = trackerresultmap[trackidlist[i]];
		//for loop to draw rectangle of vehicle. the corner is represented as (cx[i],cy[i]) (0<=i<=3)
		for (int j = 0; j < 4; j++)
		{
			qt_view_tracker_scene_->addLine(-trackerresult.estimate.cy[j],
					-trackerresult.estimate.cx[j],
					-trackerresult.estimate.cy[(j + 1) % 4],
					-trackerresult.estimate.cx[(j + 1) % 4],
					QPen(Qt::red, 0.1, Qt::DotLine));
		}
		qt_view_tracker_scene_->addLine(-trackerresult.estimate.cy[0],
				-trackerresult.estimate.cx[0], -trackerresult.estimate.cy[2],
				-trackerresult.estimate.cx[2], QPen(Qt::red, 0.1, Qt::DotLine));
		qt_view_tracker_scene_->addLine(-trackerresult.estimate.cy[1],
				-trackerresult.estimate.cx[1], -trackerresult.estimate.cy[3],
				-trackerresult.estimate.cx[3], QPen(Qt::red, 0.1, Qt::DotLine));

		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < trackerresult.edgepointnum[j]; k++)
			{
				int id = trackerresult.edgepointid[j][k];
				double theta = id * density - PI;
				double x = scan.length[id] * cos(theta);
				double y = scan.length[id] * sin(theta);
				qt_view_tracker_scene_->addEllipse(-y - 0.05, -x - 0.05, 0.1,
						0.1, QPen(Qt::green, 0.1))->setZValue(2);
			}
		}
		//Publishing Results to ROS
		jsk_recognition_msgs::BoundingBox bbox;
		bbox.header = boxes_header_;
		bbox.pose.position.x = trackerresult.estimate.x;
		bbox.pose.position.y = trackerresult.estimate.y;
		bbox.pose.position.z = 0;

		bbox.dimensions.x = sqrt(
				pow(trackerresult.estimate.cx[0] - trackerresult.estimate.cx[1],
						2)
						+ pow(
								trackerresult.estimate.cy[0]
										- trackerresult.estimate.cy[1], 2));
		bbox.dimensions.y = sqrt(
				pow(trackerresult.estimate.cx[1] - trackerresult.estimate.cx[2],
						2)
						+ pow(
								trackerresult.estimate.cy[1]
										- trackerresult.estimate.cy[2], 2));
		bbox.dimensions.z = 2.0;

		tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0,
				trackerresult.estimate.theta);
		tf::quaternionTFToMsg(quat, bbox.pose.orientation);

		bbox.value = 1.0;
		bbox.label = trackidlist[i];

		out_boxes.boxes.push_back(bbox);
	}
	boxes_publisher_->sendMessage(out_boxes);//punlish boxes to ROS

	for (int i = 0; i < curidlist.size(); i++)
	{
		bool flag = trackidlist.contains(curidlist[i]);
		if (!flag)
		{
			pathmap.remove(curidlist[i]);
		}
	}
}

void UpdateTrackerView::wheelEvent(QWheelEvent *event)
{
	if (ctrl_key_pressed_)
	{
		if (event->delta() > 0)
		{
			sx *= 1.1;
			sy *= 1.1;
			this->scale(1.1, 1.1);
		}
		else
		{
			sx *= 0.9;
			sy *= 0.9;
			this->scale(0.9, 0.9);
		}
	}
	else
	{
		QGraphicsView::wheelEvent(event);
	}
}

void UpdateTrackerView::keyPressEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key_Control:
		ctrl_key_pressed_ = 1;
		break;
	default:
		QGraphicsView::keyPressEvent(event);
		break;
	}
}

void UpdateTrackerView::keyReleaseEvent(QKeyEvent *event)
{
	switch (event->key())
	{
	case Qt::Key_Control:
		ctrl_key_pressed_ = 0;
		break;
	default:
		QGraphicsView::keyReleaseEvent(event);
		break;
	}
}

MainWindow::MainWindow(QWidget *parent) :
		QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	laserscan_subscriber_ = new ROSSub<sensor_msgs::LaserScanConstPtr>("/scan",
			1000, 10, this);
	//detectionsub=new ROSSub<cv_tracker_msgs::obj_label::ConstPtr>("obj_label",1000,10,this);
	boxes_subscriber_ = new ROSSub<
			jsk_recognition_msgs::BoundingBoxArray::ConstPtr>("bounding_boxes",
			1000, 10, this);
	tf_subscriber_ = new ROSTFSub("/world", "/velodyne", 10, this);
	tf_map_to_lidar_subscriber_ = new ROSTFSub("/velodyne", "/map", 10, this); // obj_pose is published into "map" frame

	boxes_publisher_ = new ROSPub<jsk_recognition_msgs::BoundingBoxArray>(
			"bounding_boxes_tracked", 10, this);

	//subscribe vehicle detection results (array of [x,y,theta])

	connect(laserscan_subscriber_, SIGNAL(receiveMessageSignal()), this,
			SLOT(slotReceive()));
	//connect(detection_subscriber_,SIGNAL(receiveMessageSignal()), this, SLOT(slotReceiveDetection()));
	connect(boxes_subscriber_, SIGNAL(receiveMessageSignal()), this,
			SLOT(slotReceiveBoxes()));
	connect(tf_subscriber_, SIGNAL(receiveTFSignal()), this,
			SLOT(slotReceiveTF()));
	connect(tf_map_to_lidar_subscriber_, SIGNAL(receiveTFSignal()), this,
			SLOT(slotReceiveTFMap2Lidar()));

	QSplitter * splitter = new QSplitter(Qt::Horizontal);
	ui->layout->addWidget(splitter);

	initview = new InitTrackerView;
	splitter->addWidget(initview);

	updateview = new UpdateTrackerView;
	splitter->addWidget(updateview);
	updateview->boxes_publisher_ = boxes_publisher_;

	vehicletracker = new RBSSPFVehicleTracker;
	connect(vehicletracker,
			SIGNAL(
					signalUpdateTrackerFinish(LaserScan,QMap<int,TrackerResultContainer>)),
			updateview,
			SLOT(
					slotUpdateTrackerFinish(LaserScan,QMap<int,TrackerResultContainer>)));

	laserscan_subscriber_->startReceiveSlot();
	//detection_subscriber_->startReceiveSlot();
	boxes_subscriber_->startReceiveSlot();
	tf_subscriber_->startReceiveSlot();
	tf_map_to_lidar_subscriber_->startReceiveSlot();
}

MainWindow::~MainWindow()
{
	laserscan_subscriber_->stopReceiveSlot();
	//detection_subscriber_->stopReceiveSlot();
	boxes_subscriber_->stopReceiveSlot();
	tf_subscriber_->stopReceiveSlot();
	tf_map_to_lidar_subscriber_->stopReceiveSlot();
	delete ui;
}

//receive laser scan
void MainWindow::slotReceive()
{
	sensor_msgs::LaserScanConstPtr msg = laserscan_subscriber_->getMessage();
	int msec = (msg->header.stamp.sec) % (24 * 60 * 60) * 1000
			+ (msg->header.stamp.nsec) / 1000000;
	QTime timestamp = QTime::fromMSecsSinceStartOfDay(msec);
	LaserScan scan;
	scan.timestamp = timestamp.msecsSinceStartOfDay();
	scan.beamnum = msg->ranges.size();
	for (int i = 0; i < scan.beamnum; i++)
	{
		scan.length[i] = msg->ranges[i];
	}
	scanlist.push_back(QPair<QTime, LaserScan>(timestamp, scan)); //"sync

	if (ui->trigger->isChecked())
	{
		slotShowScan();
	}
}

//not used, receive image detections
void MainWindow::slotReceiveDetection()
{
	cv_tracker_msgs::obj_label::ConstPtr msg = detection_subscriber_->getMessage();

	for (const auto& point : msg->reprojected_pos)
	{
		int msec = (msg->header.stamp.sec) % (24 * 60 * 60) * 1000
				+ (msg->header.stamp.nsec) / 1000000;
		QTime timestamp = QTime::fromMSecsSinceStartOfDay(msec);
		VehicleState state;
		//fill state from msg;
		// convert object position from map coordinate to sensor coordinate
		tf::Vector3 pt(point.x, point.y, point.z);
		tf::Vector3 converted = transformMap2Lidar * pt;
		state.x = converted.x();
		state.y = converted.y();

		detectionlist.push_back(QPair<QTime, VehicleState>(timestamp, state)); //"sync"
		if (ui->trigger->isChecked())
		{
			slotShowScan();
		}
	}
}

//receive bounding boxes from euclidean cluster
void MainWindow::slotReceiveBoxes()
{
	jsk_recognition_msgs::BoundingBoxArray::ConstPtr msg =
			boxes_subscriber_->getMessage();
	boxes_header_ = msg->header; //store header
	updateview->boxes_header_ = boxes_header_;
	for (const auto& box : msg->boxes)
	{
		int msec = (msg->header.stamp.sec) % (24 * 60 * 60) * 1000
				+ (msg->header.stamp.nsec) / 1000000;
		QTime timestamp = QTime::fromMSecsSinceStartOfDay(msec);
		VehicleState vstate;
		//fill state from msg;
		// convert object position from map coordinate to sensor coordinate
		tf::Vector3 pt(box.pose.position.x, box.pose.position.y,
				box.pose.position.z);
		//tf::Vector3 converted = transformMap2Lidar * pt;
		vstate.x = pt.x();
		vstate.y = pt.y();
		tf::Quaternion quat;
		tf::quaternionMsgToTF(box.pose.orientation, quat);
		vstate.theta = tf::getYaw(quat);

		detectionlist.push_back(QPair<QTime, VehicleState>(timestamp, vstate)); // "sync"
		if (ui->trigger->isChecked())
		{
			slotShowScan();
		}
	}
}

//receive transformation from velodyne to world
void MainWindow::slotReceiveTF()
{
	tf::StampedTransform tf;
	tf_subscriber_->getTF(tf);
	int msec = (tf.stamp_.sec) % (24 * 60 * 60) * 1000
			+ (tf.stamp_.nsec) / 1000000;
	QTime timestamp = QTime::fromMSecsSinceStartOfDay(msec);

	tf::Vector3 pos = tf.getOrigin();
	tf::Matrix3x3 rot = tf.getBasis();
	Eigen::Vector3d head;
	head(0) = 1;
	head(1) = 0;
	head(2) = 0;
	Eigen::Matrix3d rotmat;
	for (int i = 0; i < 3; i++)
	{
		rotmat(i, 0) = (double) (rot.getRow(i).x());
		rotmat(i, 1) = (double) (rot.getRow(i).y());
		rotmat(i, 2) = (double) (rot.getRow(i).z());
	}
	head = rotmat * head;
	EGOMOTION ego =
	{ pos.x(), pos.y(), atan2(head(1), head(0)) };

	tflist.push_back(QPair<QTime, EGOMOTION>(timestamp, ego)); //"sync"

	if (ui->trigger->isChecked())
	{
		slotShowScan();
	}
}

//tf map to lidar
void MainWindow::slotReceiveTFMap2Lidar()
{
	tf_map_to_lidar_subscriber_->getTF(transformMap2Lidar);
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
	//"synchronization" between vscan and tf
	bool flag = 1;
	while (flag && !scanlist.isEmpty() && !tflist.isEmpty())
	{
		QTime scantime = scanlist[0].first;
		QTime tftime = tflist[0].first;
		if (scantime == tftime)
		{
			flag = 0;
		}
		else if (scantime > tftime)
		{
			tflist.pop_front();
		}
		else
		{
			scanlist.pop_front();
		}
	}

	int scannum = scanlist.size();
	int tfnum = tflist.size();
	//it's better to synchronize detection with them

	if (scannum >= 1 && tfnum >= 1)
	{
		if (initflag)
		{
			//=====================================
			//fill initstate from subscribed detection topic
			//=====================================
			QVector<VehicleState> initstate;
			initview->getInitState(initstate);
			//getInitStateFromTopic(initstate);//get from detectionlist which is being received on slotReceiveXXX
			vehicletracker->addTrackerData(curscan, initstate);
		}
		curscan = scanlist[0].second;
		curscan.x = tflist[0].second.x;
		curscan.y = tflist[0].second.y;
		curscan.theta = tflist[0].second.theta;
		initview->showLaserScan(curscan);
		initflag = 1;

		scanlist.pop_front();
		//detectionlist.pop_front();
		tflist.pop_front();

		if (ui->local->isChecked())
		{
			QTransform transform;
			transform.scale(initview->sx, initview->sy);
			initview->setTransform(transform);
		}
		else
		{
			QTransform transform;
			transform.rotateRadians(-curscan.theta);
			transform.translate(-curscan.y, -curscan.x);
			transform.scale(initview->sx, initview->sy);
			initview->setTransform(transform);
		}
		initview->centerOn(0, 0);
	}
}
