#include "calibrationtoolkit.h"


CalibrationToolkitBase::CalibrationToolkitBase(QWidget * parent)
    : QWidget(parent)
{
    QHBoxLayout * layout=new QHBoxLayout;
    this->setLayout(layout);

    splitter=new QSplitter(Qt::Horizontal);
    layout->addWidget(splitter);

    calibsplitter=new QSplitter(Qt::Vertical);
    splitter->addWidget(calibsplitter);
}

void CalibrationToolkitBase::grabCalibDataSlot()
{
    if(grabCalibData())
    {
        emit calibDataGrabbedSignal();
    }
    else
    {
        emit calibDataGrabbedErrorSignal();
    }
}

void CalibrationToolkitBase::calibrateSensorSlot()
{
    if(calibrateSensor())
    {
        emit sensorCalibratedSignal();
    }
    else
    {
        emit sensorCalibratedErrorSignal();
    }
}

void CalibrationToolkitBase::loadCalibResultSlot()
{
    QString filename=QFileDialog::getOpenFileName(this,"Load Calib",QString(),"YML (*.yml)");
    if(!filename.isEmpty())
    {
        cv::FileStorage fs(filename.toStdString(),cv::FileStorage::READ);
        if(loadCalibResult(fs))
        {
            emit calibResultLoadedSignal();
        }
        else
        {
            emit calibResultLoadedErrorSignal();
        }
        fs.release();
    }
}

void CalibrationToolkitBase::saveCalibResultSlot()
{
    QString filename=QFileDialog::getSaveFileName(this,"Save Calib",QString(),"YML (*.yml)");
    if(!filename.isEmpty())
    {
        cv::FileStorage fs(filename.toStdString(),cv::FileStorage::WRITE);
        if(saveCalibResult(fs))
        {
            emit calibResultSavedSignal();
        }
        else
        {
            emit calibResultSavedErrorSignal();
        }
        fs.release();
    }
}

void CalibrationToolkitBase::setResultShow(cv::Mat result, QTableWidget *show)
{
    int i,n=result.rows;
    int j,m=result.cols;
    show->clear();
    show->setRowCount(n);
    show->setColumnCount(m);
    for(i=0;i<n;i++)
    {
        for(j=0;j<m;j++)
        {
            show->setItem(i,j,new QTableWidgetItem(QString("%1").arg(result.at<double>(i,j))));
        }
    }
}

void CalibrationToolkitBase::readResultShow(cv::Mat & result, QTableWidget *show)
{
    int i,n;
    int j,m;
    n=show->rowCount();
    m=show->columnCount();
    result=cv::Mat(n,m,CV_64F);
    for(i=0;i<n;i++)
    {
        for(j=0;j<m;j++)
        {
           result.at<double>(i,j)=show->item(i,j)->text().toDouble();
        }
    }
}

QVector<double> CalibrationToolkitBase::convertMatrix2Euler(cv::Mat mat)
{
    QVector<double> euler(3);
    euler[0]=atan2(mat.at<double>(2,1),mat.at<double>(2,2));
    euler[1]=atan2(-mat.at<double>(2,0),sqrt(mat.at<double>(2,1)*mat.at<double>(2,1)+mat.at<double>(2,2)*mat.at<double>(2,2)));
    euler[2]=atan2(mat.at<double>(1,0),mat.at<double>(0,0));
    return euler;
}

//=========================================================================

CalibrateCameraBase::CalibrateCameraBase(QWidget *parent)
    : CalibrationToolkitBase(parent)
{
    cameracalibtab=new QTabWidget;
    calibsplitter->addWidget(cameracalibtab);

    cameraextrinsicmat=cv::Mat::eye(4,4,CV_64F);
    cameraextrinsicshow=new QTableWidget;
    cameracalibtab->addTab(cameraextrinsicshow,CAMERAEXTRINSICMAT);
    setResultShow(cameraextrinsicmat,cameraextrinsicshow);

    cameramat=cv::Mat::zeros(3,3,CV_64F);
    cameramatshow=new QTableWidget;
    cameracalibtab->addTab(cameramatshow,CAMERAMAT);
    setResultShow(cameramat,cameramatshow);

    distcoeff=cv::Mat::eye(1,8,CV_64F);
    distcoeffshow=new QTableWidget;
    cameracalibtab->addTab(distcoeffshow,DISTCOEFF);
    setResultShow(distcoeff,distcoeffshow);

    imagesize=cv::Size2i(0,0);
    imagesizeshow=new QLabel(IMAGESIZE);
    cameracalibtab->addTab(imagesizeshow,IMAGESIZE);

    imagesplitter=new QSplitter(Qt::Vertical);
    splitter->addWidget(imagesplitter);

    cameraimagetab=new QTabWidget;
    imagesplitter->addWidget(cameraimagetab);

    calibimageshow=new QLabel("Image");
    QScrollArea * scrollarea=new QScrollArea;
    scrollarea->setWidget(calibimageshow);
    cameraimagetab->addTab(scrollarea,"Timestamp");

    int i;
    for (i=0; i<256; i++)
    {
        colorTable.push_back(qRgb(i,i,i));
    }
}

void CalibrateCameraBase::refreshImageSlot()
{
    if(refreshImage())
    {
        emit imageRefreshedSignal();
    }
    else
    {
        emit imageRefreshedErrorSignal();
    }
}

void CalibrateCameraBase::refreshParametersSlot()
{
    readResultShow(cameraextrinsicmat,cameraextrinsicshow);
    readResultShow(cameramat,cameramatshow);
    readResultShow(distcoeff,distcoeffshow);
}

bool CalibrateCameraBase::refreshImage()
{
    cameraimagetab->setTabText(0,cameratimestamp.toString("HH:mm:ss:zzz"));
    if(calibimage.type()==CV_8UC3)
    {
        QImage img(calibimage.data, calibimage.cols, calibimage.rows, calibimage.step, QImage::Format_RGB888);
        img=img.rgbSwapped();
        calibimageshow->setPixmap(QPixmap::fromImage(img));
        calibimageshow->resize(img.size());
    }
    else if(calibimage.type()==CV_8UC1)
    {
        QImage img(calibimage.data, calibimage.cols, calibimage.rows, calibimage.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        calibimageshow->setPixmap(QPixmap::fromImage(img));
        calibimageshow->resize(img.size());
    }
    else
    {
        calibimageshow->setText("Not Supported");
        cameraimagetab->setCurrentIndex(0);
        return 0;
    }
    imagesizeshow->setText(QString("%1 * %2").arg(imagesize.height).arg(imagesize.width));
    return 1;
}

bool CalibrateCameraBase::loadCalibResult(cv::FileStorage &fs)
{
    fs[CAMERAEXTRINSICMAT]>>cameraextrinsicmat;
    setResultShow(cameraextrinsicmat,cameraextrinsicshow);
    fs[CAMERAMAT]>>cameramat;
    setResultShow(cameramat,cameramatshow);
    fs[DISTCOEFF]>>distcoeff;
    setResultShow(distcoeff,distcoeffshow);
    fs[IMAGESIZE]>>imagesize;
    imagesizeshow->setText(QString("%1 * %2").arg(imagesize.height).arg(imagesize.width));
    return 1;
}

bool CalibrateCameraBase::saveCalibResult(cv::FileStorage &fs)
{
    fs<<CAMERAEXTRINSICMAT<<cameraextrinsicmat;
    fs<<CAMERAMAT<<cameramat;
    fs<<DISTCOEFF<<distcoeff;
    fs<<IMAGESIZE<<imagesize;
    return 1;
}

cv::Mat CalibrateCameraBase::getCameraExtrinsicMat()
{
    return cameraextrinsicmat;
}

cv::Mat CalibrateCameraBase::getCameraMat()
{
    return cameramat;
}

cv::Mat CalibrateCameraBase::getDistCoeff()
{
    return distcoeff;
}

cv::Size2i CalibrateCameraBase::getImageSize()
{
    return imagesize;
}

//=========================================================================

CalibrateCameraChessboardBase::CalibrateCameraChessboardBase(cv::Size2f patternSize, cv::Size2i patternNum, QWidget *parent)
    : CalibrateCameraBase(parent)
{
    patternsize=patternSize;
    patternnum=patternNum;

    int i,j;
    for(i=0;i<patternnum.height;i++)
    {
        for(j=0;j<patternnum.width;j++)
        {
            cv::Point3f tmpgrid3dpoint;
            tmpgrid3dpoint.x=i*patternsize.height;
            tmpgrid3dpoint.y=j*patternsize.width;
            tmpgrid3dpoint.z=0;
            grid3dpoint.push_back(tmpgrid3dpoint);
        }
    }    

    chessboardtab=new QTabWidget;
    calibsplitter->addWidget(chessboardtab);

    chessboardposeshow=new QTabWidget;
    chessboardtab->addTab(chessboardposeshow,CHESSBOARDPOSE);

    reprojectionerrorshow=new QLabel;
    chessboardtab->addTab(reprojectionerrorshow,REPROJECTIONERROR);

    calibimagesshow=new QTabWidget;
    imagesplitter->addWidget(calibimagesshow);
}

bool CalibrateCameraChessboardBase::calibrateSensor()
{
    if(calibimages.size()==0)
    {
        return 0;
    }
    cv::vector<cv::Mat> rvecs;
    cv::vector<cv::Mat> tvecs;
    cv::Size imgsize;
    imgsize.height=calibimages[0].rows;
    imgsize.width=calibimages[0].cols;
    cameramat.at<double>(0,2)=imgsize.height/2;
    cameramat.at<double>(1,2)=imgsize.width/2;
    reprojectionerror=cv::calibrateCamera(grid3dpoints,grid2dpoints,imgsize,cameramat,distcoeff,rvecs,tvecs);
    reprojectionerrorshow->setText(QString("%1").arg(reprojectionerror));
    setResultShow(cameramat,cameramatshow);
    setResultShow(distcoeff,distcoeffshow);
    int i,n=rvecs.size();
    for(i=0;i<n;i++)
    {
        cv::Mat chessboardpose=cv::Mat::eye(4,4,CV_64F);
        cv::Mat tmprmat=cv::Mat(3,3,CV_64F);
        cv::Rodrigues(rvecs[i],tmprmat);
        int j,k;
        for(j=0;j<3;j++)
        {
            for(k=0;k<3;k++)
            {
                chessboardpose.at<double>(j,k)=tmprmat.at<double>(j,k);
            }
            chessboardpose.at<double>(j,3)=tvecs[i].at<double>(j);
        }
        chessboardposes[i]=chessboardpose;
        setResultShow(chessboardpose,(QTableWidget *)(chessboardposeshow->widget(i)));
    }
    return 1;
}

bool CalibrateCameraChessboardBase::loadCalibResult(cv::FileStorage &fs)
{
    CalibrateCameraBase::loadCalibResult(fs);

    reprojectionerror=(double)fs[REPROJECTIONERROR];
    reprojectionerrorshow->setText(QString("%1").arg(reprojectionerror));

    int i,n=(int)fs[CHESSBOARDVIEWNUM];
    chessboardposes.resize(n);
    chessboardposeshow->clear();
    calibimages.resize(n);
    calibimagesshow->clear();
    grid3dpoints.resize(n);
    grid2dpoints.resize(n);
    for(i=0;i<n;i++)
    {
        fs[QString("%1_%2").arg(GRID3DPOINTS).arg(i).toStdString()]>>grid3dpoints[i];
        fs[QString("%1_%2").arg(GRID2DPOINTS).arg(i).toStdString()]>>grid2dpoints[i];

        fs[QString("%1_%2").arg(CHESSBOARDPOSE).arg(i).toStdString()]>>chessboardposes[i];
        QTableWidget * tmpchessboardposeshow=new QTableWidget;
        chessboardposeshow->addTab(tmpchessboardposeshow,QString("Chessboard_%1").arg(i));
        setResultShow(chessboardposes[i],tmpchessboardposeshow);

        fs[QString("%1_%2").arg(CHESSBOARDIMAGE).arg(i).toStdString()]>>calibimages[i];
        cv::Mat tmpimage=calibimages[i].clone();
        cv::drawChessboardCorners(tmpimage,patternnum,grid2dpoints[i],1);
        if(tmpimage.type()==CV_8UC3)
        {
            QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_RGB888);
            img=img.rgbSwapped();
            QLabel * tmpcalibimageshow=new QLabel;
            tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
            QScrollArea * scrollarea=new QScrollArea;
            scrollarea->setWidget(tmpcalibimageshow);
            calibimagesshow->addTab(scrollarea,QString("Image_%1").arg(i));
        }
        else if(tmpimage.type()==CV_8UC1)
        {
            QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_Indexed8);
            img.setColorTable(colorTable);
            QLabel * tmpcalibimageshow=new QLabel;
            tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
            QScrollArea * scrollarea=new QScrollArea;
            scrollarea->setWidget(tmpcalibimageshow);
            calibimagesshow->addTab(scrollarea,QString("Image_%1").arg(i));
        }
    }
    return 1;
}

bool CalibrateCameraChessboardBase::saveCalibResult(cv::FileStorage &fs)
{
    CalibrateCameraBase::saveCalibResult(fs);    

    fs<<REPROJECTIONERROR<<reprojectionerror;
    int chessboardnum=chessboardposes.size();
    fs<<CHESSBOARDVIEWNUM<<chessboardnum;
    int i,n=chessboardposes.size();
    for(i=0;i<n;i++)
    {
        fs<<QString("%1_%2").arg(GRID3DPOINTS).arg(i).toStdString()<<grid3dpoints[i];
        fs<<QString("%1_%2").arg(GRID2DPOINTS).arg(i).toStdString()<<grid2dpoints[i];
        fs<<QString("%1_%2").arg(CHESSBOARDPOSE).arg(i).toStdString()<<chessboardposes[i];
        fs<<QString("%1_%2").arg(CHESSBOARDIMAGE).arg(i).toStdString()<<calibimages[i];
    }
    return 1;
}

int CalibrateCameraChessboardBase::getChessboardNum()
{
    return chessboardposes.size();
}

cv::vector<cv::Mat> CalibrateCameraChessboardBase::getChessboardPoses()
{
    return chessboardposes;
}

cv::Mat CalibrateCameraChessboardBase::getChessboardPose(int id)
{
    return chessboardposes[id];
}

cv::Mat CalibrateCameraChessboardBase::getCalibImage(int id)
{
    return calibimages[id];
}

//=========================================================================

CalibrateCameraChessboardROS::CalibrateCameraChessboardROS(QString topic, u_int32_t queueSize, int interval, cv::Size2f patternSize, cv::Size2i patternNum, QWidget *parent)
    : CalibrateCameraChessboardBase(patternSize,patternNum,parent)
{
    camerasub=new ROSSub<sensor_msgs::ImageConstPtr>(topic,queueSize,interval);
    connect(camerasub,SIGNAL(receiveMessageSignal()),this,SLOT(refreshImageSlot()));
    camerasub->startReceiveSlot();
}

CalibrateCameraChessboardROS::~CalibrateCameraChessboardROS()
{
    delete camerasub;
}

bool CalibrateCameraChessboardROS::refreshImage()
{
    sensor_msgs::ImageConstPtr msg=camerasub->getMessage();
    if(msg==NULL)
    {
        return 0;
    }
    imagesize.height=msg->height;
    imagesize.width=msg->width;
    cameratimestamp=QTime::fromMSecsSinceStartOfDay((msg->header.stamp.sec%(24*60*60))*1000+msg->header.stamp.nsec/1000000);
    void * data=(void *)(msg->data.data());
    if(QString::fromStdString(msg->encoding)=="rgb8")
    {
        cv::Mat tmpimage=cv::Mat(msg->height,msg->width,CV_8UC3,data);
        cv::cvtColor(tmpimage,calibimage,CV_BGR2RGB);
    }
    else if(QString::fromStdString(msg->encoding)=="mono8")
    {
        calibimage=cv::Mat(msg->height,msg->width,CV_8UC1,data);
    }
    else
    {
        return 0;
    }
    return CalibrateCameraBase::refreshImage();
}

bool CalibrateCameraChessboardROS::grabCalibData()
{
    camerasub->stopReceiveSlot();
    cv::vector<cv::Point2f> grid2dpoint;
    bool found=cv::findChessboardCorners(calibimage,patternnum,grid2dpoint);
    if(!found)
    {
        camerasub->startReceiveSlot();
        return 0;
    }
    calibimages.push_back(calibimage.clone());
    grid3dpoints.push_back(grid3dpoint);
    grid2dpoints.push_back(grid2dpoint);

    chessboardposes.push_back(cv::Mat::eye(4,4,CV_64F));
    QTableWidget * tmpchessboardtable=new QTableWidget;
    chessboardposeshow->addTab(tmpchessboardtable,QString("Chessboard_%1").arg(calibimages.size()-1));

    cv::Mat tmpimage=calibimages.back().clone();
    cv::drawChessboardCorners(tmpimage,patternnum,grid2dpoint,1);
    if(tmpimage.type()==CV_8UC3)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_RGB888);
        img=img.rgbSwapped();
        QLabel * tmpcalibimageshow=new QLabel;
        tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
        QScrollArea * scrollarea=new QScrollArea;
        scrollarea->setWidget(tmpcalibimageshow);
        calibimagesshow->addTab(scrollarea,QString("Image_%1").arg(calibimages.size()-1));
        calibimagesshow->setCurrentWidget(scrollarea);
    }
    else if(tmpimage.type()==CV_8UC1)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        QLabel * tmpcalibimageshow=new QLabel;
        tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
        QScrollArea * scrollarea=new QScrollArea;
        scrollarea->setWidget(tmpcalibimageshow);
        calibimagesshow->addTab(scrollarea,QString("Image_%1").arg(calibimages.size()-1));
        calibimagesshow->setCurrentWidget(scrollarea);
    }
    camerasub->startReceiveSlot();
    return 1;
}

//=========================================================================

CalibrateCameraVelodyneChessboardBase::CalibrateCameraVelodyneChessboardBase(float maxRange, cv::Size2f patternSize, cv::Size2i patternNum, QWidget *parent)
    : CalibrateCameraChessboardBase(patternSize,patternNum,parent)
{
    maxrange=maxRange;

    calibvelodynetab=new QTabWidget;
    calibsplitter->addWidget(calibvelodynetab);

    calibvelodynepointstab=new QTabWidget;
    calibvelodynetab->addTab(calibvelodynepointstab,"Chessboard Points");

    calibvelodynenormalstab=new QTabWidget;
    calibvelodynetab->addTab(calibvelodynenormalstab,"Chessboard Normals");

    calibrationerrorshow=new QLabel;
    calibvelodynetab->addTab(calibrationerrorshow,CALIBRATIONERROR);

    velodynesplitter=new QSplitter(Qt::Vertical);
    splitter->addWidget(velodynesplitter);

    velodynetab=new QTabWidget;
    velodynesplitter->addWidget(velodynetab);

    calibvelodyneviewer=new GLViewer;
    velodynetab->addTab(calibvelodyneviewer,"Timestamp");

    calibvelodyneviewer->makeCurrent();
    calibvelodynedisplaylist=glGenLists(1);
    calibvelodyneviewer->addDisplayList(calibvelodynedisplaylist);
    calibvelodyneviewer->update();

    calibvelodynesshow=new QTabWidget;
    velodynesplitter->addWidget(calibvelodynesshow);
}

void CalibrateCameraVelodyneChessboardBase::refreshVelodyneSlot()
{
    if(refreshVelodyne())
    {
        emit velodyneRefreshedSignal();
    }
    else
    {
        emit velodyneRefreshedErrorSignal();
    }
}

void CalibrateCameraVelodyneChessboardBase::extractionResultSlot(pcl::PointCloud<pcl::PointXYZI>::Ptr extraction, cv::Mat normal, int id)
{
    calibvelodynespoints[id]=extraction;
    calibvelodynesnormals[id]=normal;
    QTableWidget * pointstable=(QTableWidget *)(calibvelodynepointstab->widget(id));
    QTableWidget * normalstable=(QTableWidget *)(calibvelodynenormalstab->widget(id));
    pointstable->clear();
    normalstable->clear();
    if(extraction!=NULL)
    {
        pointstable->setRowCount(extraction->points.size());
        pointstable->setColumnCount(4);
        int i,n=extraction->points.size();
        for(i=0;i<n;i++)
        {
            pointstable->setItem(i,0,new QTableWidgetItem(QString("%1").arg(extraction->points[i].x)));
            pointstable->setItem(i,1,new QTableWidgetItem(QString("%1").arg(extraction->points[i].y)));
            pointstable->setItem(i,2,new QTableWidgetItem(QString("%1").arg(extraction->points[i].z)));
            pointstable->setItem(i,3,new QTableWidgetItem(QString("%1").arg(extraction->points[i].intensity)));
        }
        setResultShow(calibvelodynesnormals[id],normalstable);
    }
    calibvelodynepointstab->setCurrentIndex(id);
    calibvelodynenormalstab->setCurrentIndex(id);
}

void CalibrateCameraVelodyneChessboardBase::projectVelodynePointsSlot()
{
    cv::Mat invR=cameraextrinsicmat(cv::Rect(0,0,3,3)).t();
    cv::Mat invT=-invR*(cameraextrinsicmat(cv::Rect(3,0,1,3)));
    int i,n=calibvelodynespoints.size();
    for(i=0;i<n;i++)
    {
        cv::Mat tmpimage=calibimages[i].clone();
        int j,m=calibvelodynespoints[i]->points.size();
        cv::Mat camerapoints(m,3,CV_64F);
        for(j=0;j<m;j++)
        {
            camerapoints.at<double>(j,0)=double(calibvelodynespoints[i]->points[j].x);
            camerapoints.at<double>(j,1)=double(calibvelodynespoints[i]->points[j].y);
            camerapoints.at<double>(j,2)=double(calibvelodynespoints[i]->points[j].z);
        }
        camerapoints=camerapoints*invR.t()+cv::Mat::ones(m,1,CV_64F)*invT.t();
        cv::vector<cv::Point2d> planepoints;
        planepoints.resize(m);
        for(j=0;j<m;j++)
        {
            double tmpx=camerapoints.at<double>(j,0)/camerapoints.at<double>(j,2);
            double tmpy=camerapoints.at<double>(j,1)/camerapoints.at<double>(j,2);
            double r2=tmpx*tmpx+tmpy*tmpy;
            double tmpdist=1+distcoeff.at<double>(0)*r2+distcoeff.at<double>(1)*r2*r2+distcoeff.at<double>(4)*r2*r2*r2;
            planepoints[j].x=tmpx*tmpdist+2*distcoeff.at<double>(2)*tmpx*tmpy+distcoeff.at<double>(3)*(r2+2*tmpx*tmpx);
            planepoints[j].y=tmpy*tmpdist+distcoeff.at<double>(2)*(r2+2*tmpy*tmpy)+2*distcoeff.at<double>(3)*tmpx*tmpy;
            planepoints[j].x=cameramat.at<double>(0,0)*planepoints[j].x+cameramat.at<double>(0,2);
            planepoints[j].y=cameramat.at<double>(1,1)*planepoints[j].y+cameramat.at<double>(1,2);
            cv::circle(tmpimage,planepoints[j],2,cv::Scalar(0,0,255));
        }
        if(tmpimage.type()==CV_8UC3)
        {
            QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_RGB888);
            img=img.rgbSwapped();
            QScrollArea * scrollarea=(QScrollArea *)(calibimagesshow->widget(i));
            QLabel * tmpcalibimageshow=(QLabel *)(scrollarea->widget());
            tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
        }
        else if(tmpimage.type()==CV_8UC1)
        {
            QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_Indexed8);
            img.setColorTable(colorTable);
            QScrollArea * scrollarea=(QScrollArea *)(calibimagesshow->widget(i));
            QLabel * tmpcalibimageshow=(QLabel *)(scrollarea->widget());
            tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
        }
    }
}

bool CalibrateCameraVelodyneChessboardBase::refreshVelodyne()
{
    velodynetab->setTabText(0,velodynetimestamp.toString("HH:mm:ss:zzz"));

    calibvelodyneviewer->makeCurrent();
    int i,n=calibvelodyne->height*calibvelodyne->width;
    QVector<float> colors(3*n);
    char * data=(char *)(calibvelodyne->data.data());
    for(i=0;i<n;i++)
    {
        float * base=(float *)(data+i*calibvelodyne->point_step);
        colors[i*3+0]=base[4]/255.0;
        colors[i*3+1]=base[4]/255.0;
        colors[i*3+2]=base[4]/255.0;
    }

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3,GL_FLOAT,calibvelodyne->point_step,calibvelodyne->data.data());
    glColorPointer(3,GL_FLOAT,3*sizeof(float),colors.data());


    glNewList(calibvelodynedisplaylist,GL_COMPILE);

    glDrawArrays(GL_POINTS,0,n);

    glEndList();

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    calibvelodyneviewer->update();

    return 1;
}

double calibrationCameraVelodyneChessboardObjectiveFunc(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    CalibrateCameraVelodyneChessboardBase::CameraVelodyneCalibrationData * calibdata=reinterpret_cast<CalibrateCameraVelodyneChessboardBase::CameraVelodyneCalibrationData *>(data);

    Eigen::Matrix3d rotation;
    rotation=Eigen::AngleAxisd(x[2],Eigen::Vector3d::UnitZ())
            *Eigen::AngleAxisd(x[1],Eigen::Vector3d::UnitY())
            *Eigen::AngleAxisd(x[0],Eigen::Vector3d::UnitX());

    cv::Mat rmat(3,3,CV_64F);
    rmat.at<double>(0,0)=rotation(0,0);rmat.at<double>(0,1)=rotation(0,1);rmat.at<double>(0,2)=rotation(0,2);
    rmat.at<double>(1,0)=rotation(1,0);rmat.at<double>(1,1)=rotation(1,1);rmat.at<double>(1,2)=rotation(1,2);
    rmat.at<double>(2,0)=rotation(2,0);rmat.at<double>(2,1)=rotation(2,1);rmat.at<double>(2,2)=rotation(2,2);

    cv::Mat vmat(1,3,CV_64F);
    vmat.at<double>(0)=x[0];vmat.at<double>(1)=x[1];vmat.at<double>(2)=x[2];

    int chessboardnum=calibdata->velodynepoints.size();

    cv::Mat normals=calibdata->chessboardnormals*rmat.t();
    cv::Mat points=calibdata->chessboardpoints*rmat.t()+cv::Mat::ones(chessboardnum,1,CV_64F)*vmat;

    double error=0;
    int i,n=chessboardnum;
    int count=0;
    for(i=0;i<n;i++)
    {
        if(calibdata->velodynepoints[i].cols==1)
        {
            continue;
        }
        int pointsnum=calibdata->velodynepoints[i].rows;
        count+=pointsnum;

        cv::Mat duppoints=cv::Mat::ones(pointsnum,1,CV_64F)*points.row(i);
        cv::Mat dists=(calibdata->velodynepoints[i]-duppoints)*(normals.row(i).t());
        cv::Mat tmperror=(dists.t())*dists;
        double delta=tmperror.at<double>(0);
        error+=delta;
    }
    error/=count;
    qDebug()<<error<<x[0]<<x[1]<<x[2];
    return error;
}

double calibrationCameraVelodyneChessboardRotationalObjectiveFunc(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    CalibrateCameraVelodyneChessboardBase::CameraVelodyneCalibrationData * calibdata=reinterpret_cast<CalibrateCameraVelodyneChessboardBase::CameraVelodyneCalibrationData *>(data);

    Eigen::Matrix3d rotation;
    rotation=Eigen::AngleAxisd(x[2],Eigen::Vector3d::UnitZ())
            *Eigen::AngleAxisd(x[1],Eigen::Vector3d::UnitY())
            *Eigen::AngleAxisd(x[0],Eigen::Vector3d::UnitX());

    cv::Mat rmat(3,3,CV_64F);
    rmat.at<double>(0,0)=rotation(0,0);rmat.at<double>(0,1)=rotation(0,1);rmat.at<double>(0,2)=rotation(0,2);
    rmat.at<double>(1,0)=rotation(1,0);rmat.at<double>(1,1)=rotation(1,1);rmat.at<double>(1,2)=rotation(1,2);
    rmat.at<double>(2,0)=rotation(2,0);rmat.at<double>(2,1)=rotation(2,1);rmat.at<double>(2,2)=rotation(2,2);

    calibdata->rotationresult=rmat;

    cv::Mat normals=calibdata->chessboardnormals*rmat.t();

    double error=0;
    int i,n=calibdata->velodynepoints.size();
    int count=0;
    for(i=0;i<n;i++)
    {
        if(calibdata->velodynepoints[i].cols==1)
        {
            continue;
        }
        count++;
        cv::Mat tmpmat=normals.row(i)*calibdata->velodynenormals.row(i).t();
        double tmpd=tmpmat.at<double>(0);
        error+=1-tmpd*tmpd;
    }
    error/=count;
    return error;
}

double calibrationCameraVelodyneChessboardTranslationalObjectiveFunc(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    CalibrateCameraVelodyneChessboardBase::CameraVelodyneCalibrationData * calibdata=reinterpret_cast<CalibrateCameraVelodyneChessboardBase::CameraVelodyneCalibrationData *>(data);

    cv::Mat rmat=calibdata->rotationresult;

    cv::Mat vmat(1,3,CV_64F);
    vmat.at<double>(0)=x[0];vmat.at<double>(1)=x[1];vmat.at<double>(2)=x[2];

    int chessboardnum=calibdata->velodynepoints.size();

    cv::Mat normals=calibdata->chessboardnormals*rmat.t();
    cv::Mat points=calibdata->chessboardpoints*rmat.t()+cv::Mat::ones(chessboardnum,1,CV_64F)*vmat;

    double error=0;
    int i,n=chessboardnum;
    int count=0;
    for(i=0;i<n;i++)
    {
        if(calibdata->velodynepoints[i].cols==1)
        {
            continue;
        }
        int pointsnum=calibdata->velodynepoints[i].rows;
        count+=pointsnum;

        cv::Mat duppoints=cv::Mat::ones(pointsnum,1,CV_64F)*points.row(i);
        cv::Mat dists=(calibdata->velodynepoints[i]-duppoints)*(normals.row(i).t());
        cv::Mat tmperror=(dists.t())*dists;
        double delta=tmperror.at<double>(0);
        error+=delta;
    }
    error/=count;
    qDebug()<<error<<x[0]<<x[1]<<x[2];
    return error;
}

bool CalibrateCameraVelodyneChessboardBase::calibrateSensor()
{
    if(!CalibrateCameraChessboardBase::calibrateSensor())
    {
        return 0;
    }  

    CameraVelodyneCalibrationData calibrationdata;
    cv::Mat normal=cv::Mat(1,3,CV_64F);
    normal.at<double>(0)=0;normal.at<double>(1)=0;normal.at<double>(2)=1;
    int i,n=calibvelodynespoints.size();
    calibrationdata.chessboardnormals=cv::Mat(n,3,CV_64F);
    calibrationdata.chessboardpoints=cv::Mat(n,3,CV_64F);
    calibrationdata.velodynepoints.resize(n);
    calibrationdata.velodynenormals=cv::Mat(n,3,CV_64F);
    bool flag=0;
    cv::Mat idmat=cv::Mat::eye(3,3,CV_64F);
    for(i=0;i<n;i++)
    {
        calibrationdata.chessboardnormals.row(i)=normal*chessboardposes[i](cv::Rect(0,0,3,3)).t();
        calibrationdata.chessboardpoints.at<double>(i,0)=chessboardposes[i].at<double>(0,3);
        calibrationdata.chessboardpoints.at<double>(i,1)=chessboardposes[i].at<double>(1,3);
        calibrationdata.chessboardpoints.at<double>(i,2)=chessboardposes[i].at<double>(2,3);
        if(calibvelodynespoints[i]==NULL)
        {
            calibrationdata.velodynepoints[i]=cv::Mat(1,1,CV_64F);
            continue;
        }
        flag=1;
        int j,m=calibvelodynespoints[i]->size();
        calibrationdata.velodynepoints[i]=cv::Mat(m,3,CV_64F);
        for(j=0;j<m;j++)
        {
            calibrationdata.velodynepoints[i].at<double>(j,0)=double(calibvelodynespoints[i]->points[j].x);
            calibrationdata.velodynepoints[i].at<double>(j,1)=double(calibvelodynespoints[i]->points[j].y);
            calibrationdata.velodynepoints[i].at<double>(j,2)=double(calibvelodynespoints[i]->points[j].z);
        }
        calibrationdata.velodynenormals.at<double>(i,0)=calibvelodynesnormals[i].at<double>(0);
        calibrationdata.velodynenormals.at<double>(i,1)=calibvelodynesnormals[i].at<double>(1);
        calibrationdata.velodynenormals.at<double>(i,2)=calibvelodynesnormals[i].at<double>(2);
    }
    if(!flag)
    {
        return 0;
    }

    cv::Mat NN=calibrationdata.chessboardnormals.t()*calibrationdata.chessboardnormals;
    if(cv::determinant(NN)==0)
    {
        QMessageBox::information(this,"Error","Not enough patterns");
        return 0;
    }
    cv::Mat NM=calibrationdata.chessboardnormals.t()*calibrationdata.velodynenormals;

    cv::Mat UNR=(NN.inv()*NM).t();
    cv::Mat S,U,V;
    cv::SVD::compute(UNR,S,U,V);
    calibrationdata.rotationresult=U*V;
    //calibrationdata.rotationresult=(NN.inv()*NM).t();

    cv::Mat tmpmat=cameraextrinsicmat(cv::Rect(0,0,3,3));
    calibrationdata.rotationresult.copyTo(tmpmat);
    QVector<double> euler=convertMatrix2Euler(calibrationdata.rotationresult);
    qDebug()<<euler;

    calibrationrotationalerror=cv::norm(calibrationdata.velodynenormals-calibrationdata.chessboardnormals*calibrationdata.rotationresult.t());

    QVector<double> lb(3),ub(3);
    std::vector<double> x(3);
    nlopt::result localresult;

    lb[0]=-20;lb[1]=-20;lb[2]=-20;
    ub[0]=20;ub[1]=20;ub[2]=20;
    x[0]=cameraextrinsicmat.at<double>(0,3);
    x[1]=cameraextrinsicmat.at<double>(1,3);
    x[2]=cameraextrinsicmat.at<double>(2,3);

    nlopt::opt localopt(nlopt::LN_COBYLA,3);
    localopt.set_lower_bounds(lb.toStdVector());
    localopt.set_upper_bounds(ub.toStdVector());
    localopt.set_xtol_rel(1e-4);
    localopt.set_min_objective(calibrationCameraVelodyneChessboardTranslationalObjectiveFunc,(void *)&calibrationdata);
    localresult = localopt.optimize(x, calibrationtranslationalerror);

    cameraextrinsicmat.at<double>(0,3)=x[0];cameraextrinsicmat.at<double>(1,3)=x[1];cameraextrinsicmat.at<double>(2,3)=x[2];
    qDebug()<<x[0]<<x[1]<<x[2];

    calibrationerrorshow->setText(QString("%1, %2").arg(calibrationrotationalerror).arg(calibrationtranslationalerror));
    setResultShow(cameraextrinsicmat,cameraextrinsicshow);
    return 1;
}

bool CalibrateCameraVelodyneChessboardBase::loadCalibResult(cv::FileStorage &fs)
{
    CalibrateCameraChessboardBase::loadCalibResult(fs);

    int i,n=(int)fs[CHESSBOARDVIEWNUM];
    calibvelodynespoints.resize(n);
    calibvelodynesnormals.resize(n);
    calibvelodynepointstab->clear();
    calibvelodynenormalstab->clear();
    calibvelodynesshow->clear();
    double xradius=patternsize.height*patternnum.height/2;
    double yradius=patternsize.width*patternnum.width/2;
    double radius=xradius<yradius?xradius:yradius;
    for(i=0;i<n;i++)
    {
        cv::Mat tmpmat;
        fs[QString("%1_%2").arg(VELODYNEPOINTS).arg(i).toStdString()]>>tmpmat;

        int j,m=tmpmat.rows;
        calibvelodynespoints[i]=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        calibvelodynespoints[i]->points.resize(m);
        QTableWidget * tmpvelodynetable=new QTableWidget;
        calibvelodynepointstab->addTab(tmpvelodynetable,QString("Velodyne_%1").arg(i));
        tmpvelodynetable->setRowCount(m);
        tmpvelodynetable->setColumnCount(4);
        for(j=0;j<m;j++)
        {
            calibvelodynespoints[i]->points[j].x=tmpmat.at<float>(j,0);
            calibvelodynespoints[i]->points[j].y=tmpmat.at<float>(j,1);
            calibvelodynespoints[i]->points[j].z=tmpmat.at<float>(j,2);
            calibvelodynespoints[i]->points[j].intensity=tmpmat.at<float>(j,3);

            tmpvelodynetable->setItem(j,0,new QTableWidgetItem(QString("%1").arg(calibvelodynespoints[i]->points[j].x)));
            tmpvelodynetable->setItem(j,1,new QTableWidgetItem(QString("%1").arg(calibvelodynespoints[i]->points[j].y)));
            tmpvelodynetable->setItem(j,2,new QTableWidgetItem(QString("%1").arg(calibvelodynespoints[i]->points[j].z)));
            tmpvelodynetable->setItem(j,3,new QTableWidgetItem(QString("%1").arg(calibvelodynespoints[i]->points[j].intensity)));
        }

        cv::Mat tmpviewerpose;
        fs[QString("%1_%2").arg(VELODYNEVIEWERPOSE).arg(i).toStdString()]>>tmpviewerpose;
        Eigen::Matrix4d viewerpose;
        viewerpose(0,0)=tmpviewerpose.at<double>(0,0);viewerpose(0,1)=tmpviewerpose.at<double>(0,1);viewerpose(0,2)=tmpviewerpose.at<double>(0,2);viewerpose(0,3)=tmpviewerpose.at<double>(0,3);
        viewerpose(1,0)=tmpviewerpose.at<double>(1,0);viewerpose(1,1)=tmpviewerpose.at<double>(1,1);viewerpose(1,2)=tmpviewerpose.at<double>(1,2);viewerpose(1,3)=tmpviewerpose.at<double>(1,3);
        viewerpose(2,0)=tmpviewerpose.at<double>(2,0);viewerpose(2,1)=tmpviewerpose.at<double>(2,1);viewerpose(2,2)=tmpviewerpose.at<double>(2,2);viewerpose(2,3)=tmpviewerpose.at<double>(2,3);
        viewerpose(3,0)=tmpviewerpose.at<double>(3,0);viewerpose(3,1)=tmpviewerpose.at<double>(3,1);viewerpose(3,2)=tmpviewerpose.at<double>(3,2);viewerpose(3,3)=tmpviewerpose.at<double>(3,3);
        PlaneExtractor * planeextraction=new PlaneExtractor(calibvelodynespoints[i],i,radius);
        calibvelodynesshow->addTab(planeextraction,QString("Velodyne_%1").arg(i));
        planeextraction->setCameraPose(viewerpose);
        planeextraction->update();
        connect(planeextraction,SIGNAL(extractionResultSignal(pcl::PointCloud<pcl::PointXYZI>::Ptr,cv::Mat,int)),this,SLOT(extractionResultSlot(pcl::PointCloud<pcl::PointXYZI>::Ptr,cv::Mat,int)));

        fs[QString("%1_%2").arg(VELODYNENORMALS).arg(i).toStdString()]>>calibvelodynesnormals[i];

        QTableWidget * tmpvelodynenormaltable=new QTableWidget;
        calibvelodynenormalstab->addTab(tmpvelodynenormaltable,QString("Normal_%1").arg(i));
        setResultShow(calibvelodynesnormals[i],tmpvelodynenormaltable);
    }
    fs[QString("%1_%2").arg(CALIBRATIONERROR).arg("Rotation").toStdString()]>>calibrationrotationalerror;
    fs[QString("%1_%2").arg(CALIBRATIONERROR).arg("Translation").toStdString()]>>calibrationtranslationalerror;
    calibrationerrorshow->setText(QString("%1, %2").arg(calibrationrotationalerror).arg(calibrationtranslationalerror));
    return 1;
}

bool CalibrateCameraVelodyneChessboardBase::saveCalibResult(cv::FileStorage &fs)
{
    CalibrateCameraChessboardBase::saveCalibResult(fs);

    int i,n=calibvelodynespoints.size();
    for(i=0;i<n;i++)
    {
        int j,m=calibvelodynespoints[i]->points.size();
        cv::Mat tmpmat(m,4,CV_32F);
        for(j=0;j<m;j++)
        {
            tmpmat.at<float>(j,0)=calibvelodynespoints[i]->points[j].x;
            tmpmat.at<float>(j,1)=calibvelodynespoints[i]->points[j].y;
            tmpmat.at<float>(j,2)=calibvelodynespoints[i]->points[j].z;
            tmpmat.at<float>(j,3)=calibvelodynespoints[i]->points[j].intensity;
        }
        fs<<QString("%1_%2").arg(VELODYNEPOINTS).arg(i).toStdString()<<tmpmat;
        fs<<QString("%1_%2").arg(VELODYNENORMALS).arg(i).toStdString()<<calibvelodynesnormals[i];
        PlaneExtractor * tmpviewer=(PlaneExtractor *)(calibvelodynesshow->widget(i));
        Eigen::Matrix4d viewerpose=tmpviewer->getCameraPose();
        cv::Mat tmpviewerpose=cv::Mat::eye(4,4,CV_64F);
        tmpviewerpose.at<double>(0,0)=viewerpose(0,0);tmpviewerpose.at<double>(0,1)=viewerpose(0,1);tmpviewerpose.at<double>(0,2)=viewerpose(0,2);tmpviewerpose.at<double>(0,3)=viewerpose(0,3);
        tmpviewerpose.at<double>(1,0)=viewerpose(1,0);tmpviewerpose.at<double>(1,1)=viewerpose(1,1);tmpviewerpose.at<double>(1,2)=viewerpose(1,2);tmpviewerpose.at<double>(1,3)=viewerpose(1,3);
        tmpviewerpose.at<double>(2,0)=viewerpose(2,0);tmpviewerpose.at<double>(2,1)=viewerpose(2,1);tmpviewerpose.at<double>(2,2)=viewerpose(2,2);tmpviewerpose.at<double>(2,3)=viewerpose(2,3);
        fs<<QString("%1_%2").arg(VELODYNEVIEWERPOSE).arg(i).toStdString()<<tmpviewerpose;
    }
    fs<<QString("%1_%2").arg(CALIBRATIONERROR).arg("Rotation").toStdString()<<calibrationrotationalerror;
    fs<<QString("%1_%2").arg(CALIBRATIONERROR).arg("Translation").toStdString()<<calibrationtranslationalerror;
    return 1;
}

//=========================================================================

CalibrateCameraVelodyneChessboardROS::CalibrateCameraVelodyneChessboardROS(QString cameraTopic, u_int32_t cameraQueueSize, int cameraInterval, QString velodyneTopic, u_int32_t velodyneQueueSize, int velodyneInterval, float maxRange, cv::Size2f patternSize, cv::Size2i patternNum, QWidget *parent)
    : CalibrateCameraVelodyneChessboardBase(maxRange,patternSize,patternNum,parent)
{
    camerasub=new ROSSub<sensor_msgs::ImageConstPtr>(cameraTopic,cameraQueueSize,cameraInterval);
    connect(camerasub,SIGNAL(receiveMessageSignal()),this,SLOT(refreshImageSlot()));
    velodynesub=new ROSSub<sensor_msgs::PointCloud2ConstPtr>(velodyneTopic,velodyneQueueSize,velodyneInterval);
    connect(velodynesub,SIGNAL(receiveMessageSignal()),this,SLOT(refreshVelodyneSlot()));
    camerasub->startReceiveSlot();
    velodynesub->startReceiveSlot();
}


CalibrateCameraVelodyneChessboardROS::~CalibrateCameraVelodyneChessboardROS()
{
    delete camerasub;
    delete velodynesub;
}

bool CalibrateCameraVelodyneChessboardROS::refreshImage()
{
    sensor_msgs::ImageConstPtr msg=camerasub->getMessage();
    if(msg==NULL)
    {
        return 0;
    }
    imagesize.height=msg->height;
    imagesize.width=msg->width;
    cameratimestamp=QTime::fromMSecsSinceStartOfDay((msg->header.stamp.sec%(24*60*60))*1000+msg->header.stamp.nsec/1000000);
    void * data=(void *)(msg->data.data());
    if(QString::fromStdString(msg->encoding)=="rgb8")
    {
        cv::Mat tmpimage=cv::Mat(msg->height,msg->width,CV_8UC3,data);
        cv::cvtColor(tmpimage,calibimage,CV_BGR2RGB);
    }
    else if(QString::fromStdString(msg->encoding)=="mono8")
    {
        calibimage=cv::Mat(msg->height,msg->width,CV_8UC1,data);
    }
    else
    {
        return 0;
    }
    return CalibrateCameraBase::refreshImage();
}

bool CalibrateCameraVelodyneChessboardROS::refreshVelodyne()
{
    sensor_msgs::PointCloud2ConstPtr msg=velodynesub->getMessage();
    if(msg==NULL)
    {
        return 0;
    }
    imagesize.height=msg->height;
    imagesize.width=msg->width;
    velodynetimestamp=QTime::fromMSecsSinceStartOfDay((msg->header.stamp.sec%(24*60*60))*1000+msg->header.stamp.nsec/1000000);
    calibvelodyne=msg;
    return CalibrateCameraVelodyneChessboardBase::refreshVelodyne();
}

bool CalibrateCameraVelodyneChessboardROS::grabCalibData()
{
    camerasub->stopReceiveSlot();
    velodynesub->stopReceiveSlot();

    cv::vector<cv::Point2f> grid2dpoint;
    bool found=cv::findChessboardCorners(calibimage,patternnum,grid2dpoint);
    if(!found)
    {
        camerasub->startReceiveSlot();
        velodynesub->startReceiveSlot();
        return 0;
    }
    if(calibvelodyne==NULL)
    {
        return 0;
    }
    calibimages.push_back(calibimage.clone());
    grid3dpoints.push_back(grid3dpoint);
    grid2dpoints.push_back(grid2dpoint);

    chessboardposes.push_back(cv::Mat::eye(4,4,CV_64F));
    QTableWidget * tmpchessboardtable=new QTableWidget;
    chessboardposeshow->addTab(tmpchessboardtable,QString("Chessboard_%1").arg(calibimages.size()-1));

    cv::Mat tmpimage=calibimages.back().clone();
    cv::drawChessboardCorners(tmpimage,patternnum,grid2dpoint,1);
    if(tmpimage.type()==CV_8UC3)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_RGB888);
        img=img.rgbSwapped();
        QLabel * tmpcalibimageshow=new QLabel;
        tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
        QScrollArea * scrollarea=new QScrollArea;
        scrollarea->setWidget(tmpcalibimageshow);
        calibimagesshow->addTab(scrollarea,QString("Image_%1").arg(calibimages.size()-1));
        calibimagesshow->setCurrentWidget(scrollarea);
    }
    else if(tmpimage.type()==CV_8UC1)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        QLabel * tmpcalibimageshow=new QLabel;
        tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
        QScrollArea * scrollarea=new QScrollArea;
        scrollarea->setWidget(tmpcalibimageshow);
        calibimagesshow->addTab(scrollarea,QString("Image_%1").arg(calibimages.size()-1));
        calibimagesshow->setCurrentWidget(scrollarea);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpptr;
    calibvelodynespoints.push_back(tmpptr);
    QTableWidget * tmpvelodynetable=new QTableWidget;
    calibvelodynepointstab->addTab(tmpvelodynetable,QString("Velodyne_%1").arg(calibvelodynespoints.size()-1));

    calibvelodynesnormals.push_back(cv::Mat(1,3,CV_64F));
    QTableWidget * tmpvelodynenormaltable=new QTableWidget;
    calibvelodynenormalstab->addTab(tmpvelodynenormaltable,QString("Normal_%1").arg(calibvelodynespoints.size()-1));

    double xradius=patternsize.height*patternnum.height/2;
    double yradius=patternsize.width*patternnum.width/2;
    double radius=xradius<yradius?xradius:yradius;
    PlaneExtractor * planeextraction=new PlaneExtractor(calibvelodyne,calibvelodynespoints.size()-1,radius);
    calibvelodynesshow->addTab(planeextraction,QString("Velodyne_%1").arg(calibvelodynespoints.size()-1));
    calibvelodynesshow->setCurrentWidget(planeextraction);
    planeextraction->setCameraPose(calibvelodyneviewer->getCameraPose());
    planeextraction->update();
    connect(planeextraction,SIGNAL(extractionResultSignal(pcl::PointCloud<pcl::PointXYZI>::Ptr,cv::Mat,int)),this,SLOT(extractionResultSlot(pcl::PointCloud<pcl::PointXYZI>::Ptr,cv::Mat,int)));

    camerasub->startReceiveSlot();
    velodynesub->startReceiveSlot();
    return 1;
}
